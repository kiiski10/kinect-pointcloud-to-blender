import os
import socket
import argparse
import time
import zlib
import struct
import freenect


class Kinect:
    def get_depth_frame(self):
        try:
            depth, timestamp = freenect.sync_get_depth()
            return depth, timestamp
        except TypeError:
            print("KINECT: Failed to get depth frame.")
            return None, None


def main():
    parser = argparse.ArgumentParser(
        description="Client to send kinect depth map over Unix socket."
    )
    parser.add_argument(
        "socket_path",
        type=str,
        help="Path of the socket to connect to",
    )
    args = parser.parse_args()
    socket_path = args.socket_path

    kinect = Kinect()

    while not os.path.exists(socket_path):
        print(f"Socket '{ socket_path }' does not exist. Retrying...")
        time.sleep(1)

    while True:
        # Connect to server Unix socket
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
            connected = False
            while not connected:
                try:
                    s.connect(socket_path)
                    connected = True
                except ConnectionRefusedError:
                    print(f"Connection refused by server on socket: { socket_path }. Retrying...")
                    time.sleep(1)
                except FileNotFoundError:
                    print(f"Socket not found: { socket_path }. Retrying...")
                    time.sleep(1)

            print(f"Connected to server on socket: { socket_path }")
            while True:
                depth_map, timestamp = kinect.get_depth_frame()
                try:
                    if depth_map is None:
                        print("Is something else using the Kinect? Only one application can access it at a time.")
                        return

                    # Serialize and compress the frame
                    data = zlib.compress(depth_map.tobytes())
                    # print(len(data))
                    # Send the size of the compressed data first
                    s.sendall(struct.pack('!I', len(data)) + data)
                except BrokenPipeError:
                    print("Connection closed by server. Reconnecting...")
                    time.sleep(1)
                    break


if __name__ == "__main__":
    main()