import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

import csv

# --- RosBag Reader util -----------------------------------------------------

def readOdomData(rosbag_path):

    reader = Reader(rosbag_path) 
    # Create a typestore and get the string class.
    typestore = get_typestore(Stores.LATEST)

    with reader:
        pos = []
        orient = []

        # Iterate over messages.
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/ego_racecar/odom':
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            
                pos.append((msg.pose.pose.position.x, 
                            msg.pose.pose.position.y, 
                            msg.pose.pose.position.z))
                
                orient.append((msg.pose.pose.orientation.x,
                               msg.pose.pose.orientation.y,
                               msg.pose.pose.orientation.z,
                               msg.pose.pose.orientation.w))

    return np.array(pos), np.array(orient)


def readImuData(rosbag_path, raw=True):

    reader = Reader(rosbag_path) 
    # Create a typestore and get the string class.
    typestore = get_typestore(Stores.LATEST)

    with reader:
        theta_dot = []
        x_dot = []
        y_dot = []

        if raw == True:
            topic_name = '/sensors/imu/raw'
        else:
            topic_name = '/sensors/imu' # Not present in recorded rosbags!

        # Iterate over messages.
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic_name:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            
                theta_dot.append(msg.angular_velocity.z)
                
                x_dot.append(msg.linear_acceleration.x) 
                y_dot.append(msg.linear_acceleration.y) 

        pos_imu_dot = np.hstack((
            np.array(x_dot)[:,np.newaxis],
            np.array(y_dot)[:,np.newaxis]
        ))
        
    return np.array(theta_dot), pos_imu_dot



# --- OptiTrack CSV Reader util ------------------------------------------------
def readOptiTrackCSV(optitrack_csv_path):
    """ 
        Reads from (headerless) OptiTrack Motive CSV rigid body output.
        
        Returns:
            (pos, orient) - ndarrays, float
    """
    pos = []
    orient = []
    with open(optitrack_csv_path, newline='') as csvfile:

        csv_reader = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in csv_reader:
            orient.append((row[2], row[3], row[4], row[5]))
            pos.append((row[6], row[7], row[8]))
            
    orient = np.array(orient).astype('float')
    pos = np.array(pos).astype('float')
    return pos, orient


# --- Quaternion util ---------------------------------------------------------------
def quaternionRotPoint(p, q):
    u = q[0:3]
    s = q[3]
    return 2*(u@p)*u + (s**2 - u@u)*p + 2*s*np.cross(u,p)

def quaternionMult(q0, q1):
    x0, y0, z0, w0= q0
    x1, y1, z1, w1 = q1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

def quaternionRotPoints(posArray, q):
    u = q[0:3]
    s = q[3]
    
    # pre-calc constants depending only on fixed q
    constant = s**2 - u@u
    
    ret = np.zeros_like(posArray)
    for i in range(posArray.shape[0]):
        p = posArray[i]
        p_ =  2*(u@p)*u + constant*p + 2*s*np.cross(u,p)
        ret[i]=p_

    return ret
        
def quat2yaw(q):
    w = q[-1]
    return 2*np.arccos(w)

def getQuatAxis(q):
    v = q[0:3]
    v_ = v/np.linalg.norm(v)
    return v_

# --- Other util -------------------------------------------------------------

def angles2vects(theta):
    x = np.cos(theta)[:,np.newaxis]
    y = np.sin(theta)[:,np.newaxis]

    return np.hstack((x, y))

def deg2rad(deg):
    return deg * (2*np.pi)/(360)


# --- Measurement synchronization --------------------------------------------

def synchronizeTimeOffsets(pos_gt, theta_imu, pos_imu_dot, gt_threshold=0.0001, imu_threshold=0.005):
    """
        Finds index for point in time, where car starts driving in the two different
        measurement systems:
        
        pos_gt:      Positions from OptiTrack system to synchronize
        theta_imu:   Integrated angles from IMU to synchronize
        pos_imu_dot: Linear xy-velocities from IMU to locate drive start in
    """
    
    # Find index for first position change in pos_gt
    dummy = 1
    x_dot_gt = pos_gt[1:, 0]-pos_gt[0:-1, 0]
    idx_start_gt = np.argmax(x_dot_gt>gt_threshold) # argmax returns index of first 'True'

    # Find index for first x
    v_length = np.linalg.norm(pos_imu_dot, axis=1)
    idx_start_imu = np.argmax((v_length-v_length[0])>imu_threshold)

    return pos_gt[idx_start_gt:], theta_imu[idx_start_imu:]


def synchronizeRates(pos_gt, theta_imu, N_subsample=5):
    """ 
        Subsamples OptiTrack gt positions and IMU orientations (theta) with
        harmonic subsampling rates, in order to get synchronized measurements

        pos_gt:      OptiTrack 120Hz position array
        theta_imu:   IMU 50Hz z-rotation array
        N_subsample: Subsampling of _synchronized_ arrays

        return: pos_gt_synched, theta_imu_synched
    """
    # subsample harmonics
    pos_gt_synched = pos_gt[::12*N_subsample]     
    theta_imu_synched = theta_imu[::5*N_subsample] 

    # clip to shorter measurement series
    clip_idx = min(pos_gt_synched.shape[0], theta_imu_synched.shape[0])

    return pos_gt_synched[0:clip_idx], theta_imu_synched[0:clip_idx]


