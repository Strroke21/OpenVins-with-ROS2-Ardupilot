import rclpy
from rclpy.node import Node
import time
from transformations import euler_from_quaternion
from pymavlink import mavutil
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
import sys
import csv

reset_counter = 1
jump_threshold = 0.1 # in meters, from trials and errors
jump_speed_threshold = 20 # in m/s
start_time = time.time()
rng_alt = 0
initial_roll = -1.75
initial_pitch = 1.58
initial_yaw = 1.68
cam_orient = 1 # 1=downward, 0=forward
conn_string = '/dev/ttyACM0' #'tcp:127.0.0.1:5763'
loc_coords = [0,0,0]


def normalize_roll(current_roll, initial_roll):
    # Make yaw positive for clockwise rotation (left-to-right)
    roll = -(current_roll - initial_roll)
    # Normalize to [-pi, pi]
    roll = (roll + math.pi) % (2 * math.pi) - math.pi
    return roll


def get_relative_yaw(yaw):
    global initial_yaw

    # Set initial yaw once
    if initial_yaw is None:
        initial_yaw = yaw

    # Get relative yaw: right turn = positive
    yaw = (yaw - initial_yaw)
    return yaw

def get_relative_roll(roll):
    global initial_roll
    # Set initial yaw once
    if initial_roll is None:
        initial_roll = roll

    # Get relative yaw: right turn = positive
    relative_roll = normalize_roll(roll, initial_roll)
    return relative_roll

def normalize_pitch(current_pitch, initial_pitch):
    # Make yaw positive for clockwise rotation (left-to-right)
    pitch = -(current_pitch + initial_pitch)

    # Normalize to [-pi, pi]
    pitch = (pitch + math.pi) % (2 * math.pi) - math.pi
    return pitch

def get_relative_pitch(pitch):
    global initial_pitch
    # Get relative pitch
    if initial_pitch is None:
        initial_pitch = pitch
    relative_pitch = normalize_pitch(pitch, initial_pitch)
    return relative_pitch

def set_default_home_position(vehicle, home_lat, home_lon, home_alt):
    x = 0
    y = 0
    z = 0
    q = [1, 0, 0, 0]   # w x y z

    approach_x = 0
    approach_y = 0
    approach_z = 1

    vehicle.mav.set_home_position_send(
        1,
        int(home_lat * 1e7), 
        int(home_lon * 1e7),
        int(home_alt),
        x,
        y,
        z,
        q,
        approach_x,
        approach_y,
        approach_z
    )

def vision_position_send(vehicle, x, y, z, roll, pitch, yaw):

    msg = vehicle.mav.vision_position_estimate_encode(
        int(time.time() * 1e6),
        x, y, z,
        roll, pitch, yaw  
    )
    vehicle.mav.send(msg)

def vision_speed_send(vehicle, vx, vy, vz):

    msg = vehicle.mav.vision_speed_estimate_encode(
        int(time.time() * 1e6),
        vx, vy, vz
        )
    vehicle.mav.send(msg)

def connect(connection_string, baud):

    vehicle = mavutil.mavlink_connection(connection_string,baud)
    
    return vehicle

def progress(string):
    print(string, file=sys.stdout)
    sys.stdout.flush()

def send_msg_to_gcs(vehicle,text_to_be_sent):
    # MAV_SEVERITY: 0=EMERGENCY 1=ALERT 2=CRITICAL 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG, 8=ENUM_END
    text_msg = 'D455: ' + text_to_be_sent
    vehicle.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text_msg.encode())
    progress("INFO: %s" % text_to_be_sent)

def enable_data_stream(vehicle,stream_rate):

    vehicle.wait_heartbeat()
    vehicle.mav.request_data_stream_send(
    vehicle.target_system, 
    vehicle.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    stream_rate,1)

def local_pos(vehicle):
    global loc_coords
    msg = vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=False)
    if msg is not None:
        loc_coords = [msg.x, msg.y, msg.z]
    return loc_coords  

def increment_reset_counter():
    global reset_counter
    if reset_counter >= 255:
        reset_counter = 1
    reset_counter += 1

def vision_position_delta_send(vehicle, prev_pos, prev_att, curr_pos, curr_att, dt_usec):
    # Compute delta position
    dx = curr_pos[0] - prev_pos[0]
    dy = curr_pos[1] - prev_pos[1]
    dz = curr_pos[2] - prev_pos[2]

    # Compute delta orientation (simplified)
    # Use roll-pitch-yaw difference between current and previous quaternions
    roll1 = prev_att[0]
    pitch1 = prev_att[1]
    yaw1 = prev_att[2]

    roll2 = curr_att[0]
    pitch2 = curr_att[1]
    yaw2 = curr_att[2]

    droll = roll2 - roll1
    dpitch = pitch2 - pitch1
    dyaw = yaw2 - yaw1

    # Normalize angles to [-pi, pi]
    droll = (droll + np.pi) % (2 * np.pi) - np.pi
    dpitch = (dpitch + np.pi) % (2 * np.pi) - np.pi
    dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi
    delta_magnitude = np.linalg.norm([dx,dy,dz])  # √(dx² + dy² + dz²)
    confidence = max(0.0, min(100.0, 100.0 - delta_magnitude * 100.0))
    print(f"[Confidence]: {int(confidence)}")
    # Build and send the message
    msg = vehicle.mav.vision_position_delta_encode(
        int(time.time() * 1e6),  # time_usec
        dt_usec,                 # time_delta_usec
        [droll, dpitch, dyaw],   # delta angles in radians
        [dx, dy, dz],            # delta position in meters
        int(confidence) # confidence in percentage
        )
    vehicle.mav.send(msg) #delta position and orientation update

def rotate_to_world(attitude):
    # Convert from body frame to NED/world frame
    cr = math.cos(attitude[0])
    sr = math.sin(attitude[0])
    cp = math.cos(attitude[1])
    sp = math.sin(attitude[1])
    cy = math.cos(attitude[2])
    sy = math.sin(attitude[2])

    # Rotation matrix R_body_to_world
    R = [
        [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
        [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
        [-sp,     sr * cp,                cr * cp]
    ]

    # Convert attitude to world frame
    return [
        math.atan2(R[2][1], R[2][2]),  # Roll
        math.asin(-R[2][0]),            # Pitch
        math.atan2(R[1][0], R[0][0])    # Yaw
    ]

class SlamLocalization(Node):
    def __init__(self,vehicle):
        super().__init__('localization')
        self.prev_position = None
        self.prev_vel = None
        self.counter = 0
        qos = QoSProfile(depth=0, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_subscription = self.create_subscription(Odometry,'/odomimu', self.odom_callback, qos)
        self.csv_file = open('ov_data.csv', mode='a', newline='')
        self.vehicle = vehicle
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['SLAM_X', 'SLAM_Y', 'SLAM_Z', 'loc_X', 'loc_Y', 'loc_Z'])

    def odom_callback(self, msg):
        linear_vel = msg.twist.twist.linear
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        if cam_orient == 0:
            q = [orientation.x, orientation.y, orientation.z, orientation.w]
            attitude = euler_from_quaternion(q)
            cam_x, cam_y, cam_z = position.x, -position.y, -position.z # Adjusted for forward facing camera
            cam_vx, cam_vy, cam_vz = linear_vel.x, -linear_vel.y, -linear_vel.z  # Adjusted for forward facing camera
            cam_roll = -attitude[1]
            cam_pitch = get_relative_pitch(attitude[2])
            cam_yaw = get_relative_yaw(-attitude[0])
            self.get_logger().info(f'[Orientation]: roll: {cam_roll}, pitch: {cam_pitch}, yaw: {cam_yaw}')
            self.get_logger().info(f'[SLAM]: X: {cam_x}, Y: {cam_y}, Z: {cam_z}')  
            self.get_logger().info(f'[Linear Velocity]: x: {cam_vx}, y: {cam_vy}, z: {cam_vz}')
            self.counter += 1
            current_time = time.time()
            data_hz_per_second = self.counter / (current_time - start_time)
            self.get_logger().info(f'Sending to FCU {data_hz_per_second:.2f} Hz')
            # vision_speed_send(self.vehicle, cam_vx, cam_vy, cam_vz)
            # vision_position_send(self.vehicle, cam_x, cam_y, cam_z, cam_roll, cam_pitch, cam_yaw)

        elif cam_orient == 1:

            q = [orientation.x, orientation.y, orientation.z, orientation.w]
            attitude = euler_from_quaternion(q)
            cam_x, cam_y, cam_z = position.z, -position.y, position.x
            [roll,pitch,yaw] = rotate_to_world(attitude) # Adjusted for downfacing camera
            cam_pitch = get_relative_pitch(yaw)
            cam_yaw = -pitch
            cam_roll = get_relative_roll(roll)
            cam_vx, cam_vy, cam_vz = linear_vel.z, -linear_vel.y, linear_vel.x
            self.get_logger().info(f'[Orientation]: roll: {cam_roll}, pitch: {cam_pitch}, yaw: {cam_yaw}')
            self.get_logger().info(f'[SLAM]: X: {cam_x}, Y: {cam_y}, Z: {cam_z}')  
            self.get_logger().info(f'[Linear Velocity]: x: {cam_vx}, y: {cam_vy}, z: {cam_vz}')
            self.counter += 1
            current_time = time.time()
            data_hz_per_second = self.counter / (current_time - start_time)
            self.get_logger().info(f'Sending to FCU {data_hz_per_second:.2f} Hz')
            # vision_speed_send(self.vehicle, cam_vx, cam_vy, cam_vz)
            # vision_position_send(self.vehicle, cam_x, cam_y, cam_z, cam_roll, cam_pitch, cam_yaw)
            loc = local_pos(self.vehicle)
            self.csv_writer.writerow([cam_x, cam_y, cam_z, loc[0], loc[1], loc[2]])

    def destroy_node(self):
        super().destroy_node()
        self.csv_file.close()        

def main(args=None):
    vehicle = connect(conn_string, 115200)
    enable_data_stream(vehicle, 200)
    rclpy.init(args=args)
    time.sleep(1)  
    localization = SlamLocalization(vehicle)
    rclpy.spin(localization)
    localization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

