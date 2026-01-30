bl_info = {
    "name": "Kinect Server",
    "author": "Pekka L.",
    "version": (1, 2),
    "blender": (5, 0, 0),
    "location": "View3D > Sidebar > Kinect",
    "description": "Starts a server to receive Kinect depth data and displays it as a point cloud.",
    "warning": "",
    "wiki_url": "",
    "category": "3D View",
}

import bpy
import os
import sys
import socket
import struct
import zlib
import numpy as np
import threading
import queue
import time
from collections import deque
# --- Addon Setup ---

# Global variables to manage the server thread and data queue
server_thread = None
stop_event = threading.Event()
frame_queue = queue.Queue()

# Log management
log_lock = threading.Lock()
log_messages = deque(maxlen=10)

def add_log(msg):
    """Thread-safe append a timestamped message to the log deque."""
    timestamp = time.strftime("%H:%M:%S")
    entry = f"[{timestamp}] {msg}"
    with log_lock:
        log_messages.append(entry.strip())

# FPS globals
frame_count = 0
last_fps_time = time.time()
current_fps = 0.0
# --- Point Cloud Logic (Main Thread) ---

def create_geometry_nodes_setup(obj):
    """
    Creates and configures a Geometry Nodes modifier on the given object
    for instancing points.
    """
    # Add a new Geometry Nodes modifier
    modifier = obj.modifiers.new(name="KinectGeoNodes", type='NODES')

    # --- Create the object to use as an instance ---
    instance_obj_name = "KinectInstanceDot"
    instance_obj = bpy.data.objects.get(instance_obj_name)
    if not instance_obj:
        # Create a small, low-poly icosphere to act as a "dot"
        bpy.ops.mesh.primitive_ico_sphere_add(radius=0.005, subdivisions=2)
        instance_obj = bpy.context.active_object
        instance_obj.name = instance_obj_name
        # Hide the original instance object from view and render
        instance_obj.hide_set(True)
        instance_obj.hide_render = True
        # Move to the same collection as the main object
        for coll in obj.users_collection:
            if instance_obj not in coll.objects:
                coll.objects.link(instance_obj)
        # Unlink from context collection if it's not the right one
        if bpy.context.collection not in obj.users_collection:
             bpy.context.collection.objects.unlink(instance_obj)


    # --- Create the Node Tree ---
    node_group = bpy.data.node_groups.new(name="KinectPointCloudNodes", type='GeometryNodeTree')
    modifier.node_group = node_group
    nodes = node_group.nodes

    # Clear default nodes
    for node in nodes:
        nodes.remove(node)

    # Add necessary nodes
    group_input = nodes.new(type='NodeGroupInput')
    group_input.location = (-300, 0)

    instance_on_points = nodes.new(type='GeometryNodeInstanceOnPoints')
    instance_on_points.location = (0, 0)

    object_info = nodes.new(type='GeometryNodeObjectInfo')
    object_info.location = (-300, -150)
    object_info.inputs[0].default_value = instance_obj # Set the instance object

    group_output = nodes.new(type='NodeGroupOutput')
    group_output.location = (300, 0)

    # Link the nodes
    links = node_group.links
    links.new(group_input.outputs['Geometry'], instance_on_points.inputs['Points'])
    links.new(object_info.outputs['Geometry'], instance_on_points.inputs['Instance'])
    links.new(instance_on_points.outputs['Instances'], group_output.inputs['Geometry'])


def create_or_get_point_cloud_obj():
    """Finds the point cloud object or creates a new one with GeoNodes."""
    obj_name = "KinectPointCloud"

    obj = bpy.data.objects.get(obj_name)
    if obj:
        return obj

    # Create new mesh and object
    mesh = bpy.data.meshes.new(name=obj_name)
    obj = bpy.data.objects.new(obj_name, mesh)
    bpy.context.collection.objects.link(obj)

    # Set up the Geometry Nodes modifier for instancing
    create_geometry_nodes_setup(obj)

    return obj

def update_point_cloud():
    """
    Checks the queue for a new frame and updates the point cloud mesh.
    This function runs in Blender's main thread via a timer.
    """
    global frame_count, last_fps_time, current_fps
    try:
        # Get a frame from the queue without blocking
        depth_map = frame_queue.get_nowait()
    except queue.Empty:
        # Sync logs even when no new frame so UI stays up-to-date
        sync_logs_to_scene()
        return 0.0025

    obj = create_or_get_point_cloud_obj()
    mesh = obj.data
    point_drop_amount = getattr(bpy.context.scene, "kinect_point_drop_amount", 1)

    height, width = depth_map.shape

    # --- High-Performance NumPy Vertex Generation ---

    # 1. Create a mask to filter out invalid depth points (value 2047 means "too far")
    valid_mask = depth_map < 2047

    # 2. Generate X and Y grids and filter them with the mask
    all_y, all_x = np.indices((height, width), dtype=np.float32)
    pixel_x = all_x[valid_mask]
    pixel_y = all_y[valid_mask]

    # 3. Extract only the valid depth values
    valid_depths = depth_map[valid_mask].astype(np.float32)

    # Drop every n-th point
    point_drop_amount = bpy.context.scene.kinect_point_drop_amount
    if point_drop_amount > 1:
        pixel_x = pixel_x[::point_drop_amount]
        pixel_y = pixel_y[::point_drop_amount]
        valid_depths = valid_depths[::point_drop_amount]

    # 4. Apply the physically accurate formula to all valid points at once
    # This converts the raw 11-bit depth value to meters.
    z_meters = 1.0 / (valid_depths * -0.0030711016 + 3.3309495161)

    # 5. Convert pixel coordinates to 3D space and scale for the scene
    # The scaling factor (e.g., 300.0) adjusts the overall size of the point cloud.
    scale_factor_x = getattr(bpy.context.scene, "kinect_scale_factor_x", 0.5)
    scale_factor_y = getattr(bpy.context.scene, "kinect_scale_factor_y", 0.5)
    scale_factor_z = getattr(bpy.context.scene, "kinect_scale_factor_z", 0.5)

    world_x = -(pixel_x - width / 2) * scale_factor_x

    # Flip vertically
    world_y = (height / 2 - pixel_y) * scale_factor_y
    # We apply a final scaling to the Z-axis for better visibility.
    visual_z = -np.log1p(z_meters) * (scale_factor_z  * 1000.0) / 2

    # 6. Combine the X, Y, and Z coordinates into a final vertex array
    # np.stack is a fast way to join the arrays.
    vertices = np.stack((world_x, visual_z, world_y), axis=-1)

    # Efficiently update the mesh
    mesh.clear_geometry()
    mesh.from_pydata(vertices, [], [])
    mesh.update()

    # --- FPS update ---
    frame_count += 1
    now = time.time()
    if now - last_fps_time >= 1.0:
        current_fps = frame_count / (now - last_fps_time)
        frame_count = 0
        last_fps_time = now

    # Sync logs so the UIList active index moves to latest
    sync_logs_to_scene()

    # Check again for a new frame very soon
    return 0.0025


# --- Server Logic (Background Thread) ---

def recvall(conn, n):
    """Helper function to receive n bytes from a socket."""
    data = bytearray()
    while len(data) < n:
        packet = conn.recv(n - len(data))
        if not packet:
            return None
        data.extend(packet)
    return data

def server_run(stop_event, frame_queue):
    """The main function for the server thread."""
    # Add venv path inside the thread
    venv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'venv', 'lib', 'python3.11', 'site-packages')
    if venv_path not in sys.path:
        sys.path.append(venv_path)

    # Use an absolute path for the socket file
    socket_path = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "server.sock"))

    if os.path.exists(socket_path):
        os.remove(socket_path)

    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
        s.bind(socket_path)
        s.listen()
        s.settimeout(1.0)
        add_log(f"Server listening on socket: {socket_path}")
        print(f"Server listening on socket: {socket_path}")

        while not stop_event.is_set():
            try:
                conn, addr = s.accept()
                with conn:
                    add_log("Client connected")
                    print("Client connected")
                    while not stop_event.is_set():
                        raw_data_size = recvall(conn, 4)
                        if not raw_data_size:
                            break

                        data_size = struct.unpack('!I', raw_data_size)[0]
                        compressed_data = recvall(conn, data_size)
                        if not compressed_data:
                            break

                        decompressed_data = zlib.decompress(compressed_data)
                        depth_map = np.frombuffer(decompressed_data, dtype=np.uint16).reshape((480, 640))

                        # Only keep the latest frame in the queue
                        while not frame_queue.empty():
                            try:
                                frame_queue.get_nowait()
                            except queue.Empty:
                                break
                        frame_queue.put(depth_map)

            except socket.timeout:
                continue
            except Exception as e:
                add_log(f"Server error: {e}")
                break
            finally:
                add_log("Client disconnected")

    if os.path.exists(socket_path):
        os.remove(socket_path)
    add_log("Server has stopped.")


# --- Blender Operators ---

class WM_OT_StartKinectServer(bpy.types.Operator):
    """Starts the Kinect server and the UI timer"""
    bl_idname = "wm.start_kinect_server"
    bl_label = "Start Kinect Server"

    def execute(self, context):
        global server_thread, stop_event
        if server_thread is None or not server_thread.is_alive():
            stop_event.clear()
            server_thread = threading.Thread(target=server_run, args=(stop_event, frame_queue))
            server_thread.start()

            # Register the timer to update the UI
            bpy.app.timers.register(update_point_cloud)

            add_log("Kinect server started.")
            self.report({'INFO'}, "Kinect server started.")
        else:
            add_log("Start attempted but server already running.")
            self.report({'WARNING'}, "Server is already running.")
        return {'FINISHED'}

class WM_OT_StopKinectServer(bpy.types.Operator):
    """Stops the Kinect server and the UI timer"""
    bl_idname = "wm.stop_kinect_server"
    bl_label = "Stop Kinect Server"

    def execute(self, context):
        global server_thread, stop_event

        # Unregister the timer
        if bpy.app.timers.is_registered(update_point_cloud):
            bpy.app.timers.unregister(update_point_cloud)

        if server_thread and server_thread.is_alive():
            stop_event.set()
            server_thread.join(timeout=2.0)
            server_thread = None
            add_log("Kinect server stopped.")
            self.report({'INFO'}, "Kinect server stopped.")
        else:
            add_log("Stop attempted but server was not running.")
            self.report({'WARNING'}, "Server is not running.")
        return {'FINISHED'}

# New operator to clear logs
class WM_OT_ClearKinectLogs(bpy.types.Operator):
    """Clear Kinect log messages"""
    bl_idname = "wm.clear_kinect_logs"
    bl_label = "Clear Logs"

    def execute(self, context):
        with log_lock:
            log_messages.clear()
        # clear the UI collection as well (main thread)
        context.scene.kinect_log_items.clear()
        context.scene.kinect_log_active_index = 0
        return {'FINISHED'}


# --- UI-backed log support (main-thread) ---
def sync_logs_to_scene():
    """Copy thread-safe deque into scene collection and set active index to last item."""
    try:
        scn = bpy.context.scene
    except Exception:
        return
    with log_lock:
        logs = list(log_messages)
    # ensure the collection exists on the scene (registered in register())
    scn.kinect_log_items.clear()
    for msg in logs:
        it = scn.kinect_log_items.add()
        it.message = msg
    scn.kinect_log_active_index = max(0, len(scn.kinect_log_items) - 1)


# New lightweight item holder for the UIList
class KinectLogItem(bpy.types.PropertyGroup):
    message: bpy.props.StringProperty(name="Message")

class VIEW3D_UL_KinectLogs(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        if item:
            layout.label(text=item.message)

class VIEW3D_PT_KinectServerPanel(bpy.types.Panel):
    bl_label = "Kinect Server"
    bl_idname = "VIEW3D_PT_kinect_server"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Kinect'

    def draw(self, context):
        layout = self.layout
        layout.operator(WM_OT_StartKinectServer.bl_idname, icon='PLAY')
        layout.operator(WM_OT_StopKinectServer.bl_idname, icon='PAUSE')
        layout.prop(context.scene, "kinect_point_drop_amount")
        layout.prop(context.scene, "kinect_scale_factor_x")
        layout.prop(context.scene, "kinect_scale_factor_y")
        layout.prop(context.scene, "kinect_scale_factor_z")
        layout.label(text=f"FPS: {current_fps:.1f}")  # Added

        # --- Logs box (UIList will auto-scroll to active index) ---
        box = layout.box()
        row = box.row(align=True)
        row.label(text="Logs:")
        row.operator(WM_OT_ClearKinectLogs.bl_idname, text="", icon='TRASH')
        box.template_list("VIEW3D_UL_KinectLogs", "kinect_logs", context.scene,
                          "kinect_log_items", context.scene, "kinect_log_active_index", rows=5)


# --- Registration ---

classes = (
    WM_OT_StartKinectServer,
    WM_OT_StopKinectServer,
    WM_OT_ClearKinectLogs,
    KinectLogItem,
    VIEW3D_UL_KinectLogs,
    VIEW3D_PT_KinectServerPanel,
)

def register():
    bpy.types.Scene.kinect_point_drop_amount = bpy.props.IntProperty(
        name="Drop points",
        description="Drop every n-th point from the depth map",
        default=1,
        min=1,
        max=64
    )
    bpy.types.Scene.kinect_scale_factor_x = bpy.props.FloatProperty(
        name="X Scale",
        description="Scale factor for the X axis",
        default=0.5,
        min=0.1,
        max=4.0
    )
    bpy.types.Scene.kinect_scale_factor_y = bpy.props.FloatProperty(
        name="Y Scale",
        description="Scale factor for the Y axis",
        default=0.5,
        min=0.1,
        max=4.0
    )
    bpy.types.Scene.kinect_scale_factor_z = bpy.props.FloatProperty(
        name="Z Scale",
        description="Scale factor for the Z axis",
        default=1.0,
        min=0.1,
        max=4.0
    )
    for cls in classes:
        bpy.utils.register_class(cls)

    # Scene collection + active index for the UIList
    bpy.types.Scene.kinect_log_items = bpy.props.CollectionProperty(type=KinectLogItem)
    bpy.types.Scene.kinect_log_active_index = bpy.props.IntProperty(default=0)

def unregister():
    # Ensure the server and timer are stopped when the addon is disabled
    if server_thread and server_thread.is_alive():
        stop_event.set()
        server_thread.join(timeout=2.0)
    if bpy.app.timers.is_registered(update_point_cloud):
        bpy.app.timers.unregister(update_point_cloud)

    # remove Scene properties
    if hasattr(bpy.types.Scene, "kinect_log_items"):
        del bpy.types.Scene.kinect_log_items
    if hasattr(bpy.types.Scene, "kinect_log_active_index"):
        del bpy.types.Scene.kinect_log_active_index

    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    if hasattr(bpy.types.Scene, "kinect_point_drop_amount"):
        del bpy.types.Scene.kinect_point_drop_amount
    if hasattr(bpy.types.Scene, "kinect_scale_factor_x"):
        del bpy.types.Scene.kinect_scale_factor_x
    if hasattr(bpy.types.Scene, "kinect_scale_factor_y"):
        del bpy.types.Scene.kinect_scale_factor_y
    if hasattr(bpy.types.Scene, "kinect_scale_factor_z"):
        del bpy.types.Scene.kinect_scale_factor_z