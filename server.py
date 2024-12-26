import os
from flask import Flask, send_from_directory
from flask_socketio import SocketIO, emit
import ssl
import logging
import yaml

# =======================
# SERVER AND FILE STUFF
# =======================

# Initialize Flask and SocketIO
app = Flask(__name__, static_folder='client')
socket = SocketIO(app, cors_allowed_origins="*")

# Suppress Flask's default request logs
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

# SSL context configuration
context = ssl.SSLContext(ssl.PROTOCOL_TLS)
context.load_cert_chain('ssl/server.cert', keyfile='ssl/server.key')

# Serve static files
@app.route('/')
def index():
    return send_from_directory('client', 'index.html')

# Load User Parameters
with open('userParams.yaml', 'r') as file:
    data = yaml.safe_load(file)
    base_offset = data["base_offset"]
    right_hand_control = data["right_hand_control"]

print(right_hand_control)


# ======================================
# VR DATA PROCESSING AND STATE MACHINE
# ======================================

# System state variables
headset_connection = False
tracking_start_permitted = False
tracking_enabled = False
endeff_coords = None
left_coords = None
right_coords = None
left_vel = None
right_vel = None

# Connection Events
@socket.on('headset_connect')
def handle_headset_connect():
    global headset_connection
    headset_connection = True
    print("headset connected")

@socket.on('headset_disconnect')
def handle_headset_disconnect():
    global headset_connection, left_coords, right_coords, left_vel, right_vel
    headset_connection, left_coords, right_coords, left_vel, right_vel = False, None, None, None, None
    print("headset disconnected")

# Controller Events
@socket.on('leftPos')
def handle_left_position(leftPos):
    global left_coords
    if headset_connection:
        left_coords = leftPos
        if(not right_hand_control):
            tracking_start_permission_check()

@socket.on('rightPos')
def handle_right_position(rightPos):
    global right_coords
    if headset_connection:
        right_coords = rightPos
        if(right_hand_control):
            tracking_start_permission_check()

@socket.on('leftVel')
def handle_left_position(leftVel):
    global left_coords
    if headset_connection:
        left_vel = leftVel

@socket.on('rightVel')
def handle_right_position(rightVel):
    global right_coords
    if headset_connection:
        right_vel= rightVel



# Helper Functions
def logPoint(point):
    text = "x: " + str(round(point["x"],2)) + "\ty: " + str(round(point["y"],2)) + "\tz: " + str(round(point["z"],2))
    print(text)

def absNorm(point):
    return (point['x']**2 + point['y']**2 + point['z']**2) ** 0.5

def diffNorm(pointA, pointB):
    # Calculate differences
    diffX = pointA['x'] - pointB['x']
    diffY = pointA['y'] - pointB['y']
    diffZ = pointA['z'] - pointB['z']

    # Return norm
    return (diffX**2 + diffY**2 + diffZ**2) ** 0.5

# Tracking activation check
def tracking_start_permission_check():
    global tracking_start_permitted
    if (headset_connection and 
        (left_coords != None and right_coords != None and left_vel != None and right_vel != None and endeff_coords != None) and
        ((right_hand_control and absNorm(right_vel) < 0.05) or (not right_hand_control and absNorm(right_vel) < 0.05)) and
        ((right_hand_control and diffNorm(right_coords, endeff_coords) < 0.05) or (not right_hand_control and diffNorm(left_coords, endeff_coords) < 0.05))):
        tracking_start_permitted = True
    else:
        tracking_start_permitted = False

# Start the server with SSL
if __name__ == '__main__':
    print('Server running at https://localhost:3000')
    socket.run(app, host='0.0.0.0', port=3000, ssl_context=context)
