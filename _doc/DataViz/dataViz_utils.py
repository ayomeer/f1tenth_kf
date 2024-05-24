from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg, register_types

import csv


# From https://gitlab.com/ternaris/rosbags/-/blob/master/docs/examples/register_types_files.py
def guess_msgtype(path: Path) -> str:
    """Guess message type name from path."""
    name = path.relative_to(path.parents[2]).with_suffix('')
    if 'msg' not in name.parts:
        name = name.parent / 'msg' / name.name
    return str(name)


# Create a typestore and get the string class.
typestore = get_typestore(Stores.LATEST)
add_types = {}

# Add custom Vesc message types to be able to read them from rosbags
msgDirPath = 'ROS/vesc_msgs/msg/'
msgDefs = ['VescState.msg', 
           'VescStateStamped.msg',
           'VescImu.msg',
           'VescImuStamped.msg']
msgDefPaths = [msgDirPath+msgFileName for msgFileName in msgDefs]

for pathstr in msgDefPaths:
    msgpath = Path(pathstr)
    msgdef = msgpath.read_text(encoding='utf-8')
    add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))

typestore.register(add_types)   

# --- Data reader utils -----------------------------------------------------

def readSensorsCore(rosbag_path):
    reader = Reader(rosbag_path) 
    
    with reader:
        data = []

        # Iterate over messages.
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/sensors/core':
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            
                data.append(msg.state.speed)

    return np.array(data)

def readFloatData(rosbag_path, topic_string):
    reader = Reader(rosbag_path) 

    with reader:
        t = []
        data = []

        # Iterate over messages.
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic_string:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            
                t.append(timestamp)
                data.append(msg.data)

    return np.array(t), np.array(data)

def readOdomData(rosbag_path):

    reader = Reader(rosbag_path) 
    # Create a typestore and get the string class.
    typestore = get_typestore(Stores.LATEST)

    with reader:
        pos = []
        orient = []
        z_twist = []

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
                
                z_twist.append(msg.twist.twist.angular.z)

    return np.array(pos), np.array(orient), np.array(z_twist)

def readImuData(rosbag_path, raw=True):

    reader = Reader(rosbag_path) 

    with reader:
        theta_dot = []
        x_dot = []
        y_dot = []

        if raw == True:
            topic_name = '/sensors/imu/raw'

        # Iterate over messages.
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic_name:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            
                theta_dot.append(msg.angular_velocity.z)
                
                x_dot.append(msg.linear_acceleration.x) 
                y_dot.append(msg.linear_acceleration.y) 

        lin_accel_imu = np.hstack((
            np.array(x_dot)[:,np.newaxis],
            np.array(y_dot)[:,np.newaxis]
        ))
        
    return np.array(theta_dot), lin_accel_imu

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
            
    orient = np.array(orient)[1:].astype('float') # discard 1st row: base frame
    pos = np.array(pos)[1:].astype('float')
    return pos, orient


# --- Measurement synchronization --------------------------------------------

def getTimeSynchIndices(pos_gt, lin_accel_imu, gt_threshold=0.0002, imu_threshold=0.005, plot=False):
    """
        Finds index for point in time, where car starts driving in the two different
        measurement systems:
        
        pos_gt:      Positions from OptiTrack system to synchronize
        theta_imu:   Integrated angles from IMU to synchronize
        lin_accel_imu: Linear xy-velocities from IMU to locate drive start in
    """
    
    # Find index for first position change in pos_gt
    pos_gt_dot = pos_gt[1:]-pos_gt[0:-1]
    v_length_gt = np.linalg.norm(pos_gt_dot, axis=1)
    idx_start_gt = np.argmax(v_length_gt > gt_threshold) 

    if plot == True:
        # show chosen start index for pos_gt
        plt.figure(0)
        plt.title("Finding drive start in pos_gt_dot; threashold= {}".format(gt_threshold))
        plt.plot(v_length_gt)
        plt.vlines(idx_start_gt, np.min(v_length_gt), np.max(v_length_gt))
        plt.hlines(gt_threshold, 0, v_length_gt.shape[0])
        plt.show(block=False)

    # Find index for first linear_velocity change over threashold
    v_length_imu = np.linalg.norm(lin_accel_imu, axis=1)
    v_length_imu = v_length_imu-np.mean(v_length_imu[0:10])
    idx_start_imu = np.argmax(v_length_imu > imu_threshold)

    if plot == True:
        # show chosen start index for theta_imu
        plt.figure(1)
        plt.title("Finding drive start in lin_accel_imu; threashold= {}".format(imu_threshold))
        plt.plot(v_length_imu)
        plt.vlines(idx_start_imu, np.min(v_length_imu), np.max(v_length_imu))
        plt.hlines(imu_threshold, 0, v_length_imu.shape[0])
        plt.show(block=False)

    return idx_start_gt, idx_start_imu


def synchronizeRates(pos_gt, theta_imu, N_subsample):
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


def upsample(x_lowRate, len_highRate):
    x_highRate = np.zeros(shape=len_highRate)

    for i in range(len_highRate):
        i_ = (i*len(x_lowRate)//len_highRate)
        x_highRate[i] = x_lowRate[i_]

    return x_highRate

def upsampleVariableRate(t_variable_ms, x_variable, len_highRate, dt_highRate_ms=20):
    len_varRate = len(x_variable)

    x_variable_highRate = np.zeros(len_highRate)
    i_var = 0
    i_var_list = []
    for i in np.arange(len_highRate):
        t_ms = i*dt_highRate_ms
        
        while(t_variable_ms[i_var+1] < t_ms):
            

            # stop if end of t_variable_ms array reached
            if i_var+1 >= (len_varRate-1): 
                break 
            else: 
                i_var += 1

        x_variable_highRate[i] = x_variable[i_var]
        i_var_list.append(i_var_list)

    return x_variable_highRate

# --- Stats ---------------------------------------------------------------------
def getLocalStats(signal, windowSize):
    N = len(signal)
    m = windowSize

    mean = []
    std = []
    
    for i in range(N-(m-1)): # i is starting index of window
       x = signal[i:i+m]
       mean.append(np.mean(x))
       std.append(np.std(x)) 

    return np.array(mean), np.array(std)

def removeBias(x, plot=False):
    m, s = getLocalStats(x, windowSize=5)
    idleIdx = np.argmin(s)

    if plot == True:
        plt.figure()
        plt.plot(s)
        plt.vlines(idleIdx, 0, np.max(s))

    return x-m[idleIdx]

def getDistTraveled(posArray):
    path_lengths = np.linalg.norm(posArray[1:]-posArray[:-1], axis=1)
    dist = np.sum(path_lengths)
    return dist
    

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
    z = q[2]
    
    # normalize
    mag = np.sqrt(w**2+z**2)
    w /= mag
    z /= mag

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


def getOrientXYUV(pos_odom, orient_odom, N_subsample):
    orient_dir_0 = np.array([1,0,0]) # base orientation vector to transform onto paths
    
    # get subsampled versions of odom orientations & positions for plotting
    N_subsample = 25
    orient_odom_subset = orient_odom[::N_subsample] 
    pos_odom_subset = pos_odom[::N_subsample]

    # apply orient_odom quaternion rotations to orient0
    orient_odom_dir = []
    for i in range(orient_odom_subset.shape[0]):
        orient_odom_dir.append(quaternionRotPoint(orient_dir_0, orient_odom_subset[i]))

    orient_odom_dir = np.array(orient_odom_dir)

    return pos_odom_subset, orient_odom_dir

