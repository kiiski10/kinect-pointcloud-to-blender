# Scripts

## `client.py`  

This is the part that communicates with the Kinect and sends the point cloud data to server script.

## `server.py`  

Blender addon that listens for the point cloud data from the client and displays it in Blender.

# Install

1. Installing of freenect python library requires libfreenect development files to be installed.
  - On Ubuntu run: `apt install libfreenect-dev`

2. Then create the virtual environment for python  
  - `python3 -m venv venv`

3. And install the requirements for the client script
  - `pip install -r client-requirements.txt`

4. Install the server.py in Blender as add-on by draging the file to blender

5. Run the client.py

6. Open the server addon in Blender by going to 3D view and pressing `N` to show the side panel.
  - Select the Kinect tab in the side panel on left side of 3D view and click `Start Kinect Server`
