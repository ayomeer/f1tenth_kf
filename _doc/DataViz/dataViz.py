import numpy as np
import matplotlib.pyplot as plt

from dataViz_utils import *


# --- Config -----------------------------------------------------------------------------


rosbag_path = 'ROS/sensordata_05_circle_left'
optitrack_csv_path = 'OptiTrack/gt_OptiTrack_05_circle_left.csv'


# toggle plots
PLOT_POS_GT_RAW = 0
PLOT_POS_GT_ADJ = 1

PLOT_POS_ODOM = 0

PLOT_ODOM_ORIENTATIONS = 0
PLOT_IMU_ORIENTATIONS = 1

# --- Read Data -----------------------------------------------------------------------------

# Read Pose from RosBag
pos_odom, orient_odom = readOdomData(rosbag_path)

# Read IMU Data from RosBag
theta_imu_dot, pos_imu_dot = readImuData(rosbag_path) # message in deg/s even though doc says rad/s
theta_imu_dot = deg2rad(theta_imu_dot) 
# NOTE: pos_imu_dot in IMU reference frame

# Read OptiTrack ground truth from CSV
pos_gt, orient_gt = readOptiTrackCSV(optitrack_csv_path)


# --- Data Manipulation ----------------------------------------------------------------

# --- Postion Data ---

# Odom
# Reset Odom. if multiple measurements made without restarting driver stack
pos_odom -= pos_odom[0]

q0 = orient_odom[0]
q0[-1] = -q0[-1] # rotate the other way (back)
pos_odom = quaternionRotPoints(pos_odom, q0)


# OptiTrack
# set beginning position to origin (like odom)
gt_startFrame = 1 # frame 0 is base frame!
pos_gt -= pos_gt[gt_startFrame]


# Align  gt with odom starting orientation (odom is reference for car)
q0 = orient_gt[gt_startFrame]
q0[-1] = -q0[-1] # rotate the other way (back)
pos_gt_ = quaternionRotPoints(pos_gt, q0)

print("Aligning gt to odom with q0=",q0)
print("Rotation axis vector: ", getQuatAxis(q0))
print("q0 rotation in deg: ", quat2yaw(q0)*(360)/(2*np.pi))


# --- Orientation Data ---

# Odom
# get subsampled versions of odom orientations & positions for plotting
N_subsample = 25
orient_odom_subset = orient_odom[::N_subsample] 
pos_odom_subset = pos_odom[::N_subsample]

orient_dir_0 = np.array([1,0,0]) # starting vector pointing in x-direction

# apply orient_odom quaternion rotations to orient0
orient_odom_dir = []
for i in range(orient_odom_subset.shape[0]):
    orient_odom_dir.append(quaternionRotPoint(orient_dir_0, orient_odom_subset[i]))

orient_odom_dir = np.array(orient_odom_dir)

# IMU
# Convert angular velocity into angle by integrating
dt = 0.02
theta_imu_ = np.cumsum(theta_imu_dot*dt)


# --- Synchronize Measurements ---

# synchronize measurement start
pos_gt_, theta_imu_ = synchronizeTimeOffsets(pos_gt_, theta_imu_, pos_imu_dot)

# get synchronized subsamples of 50Hz and 120Hz measurements
pos_gt_synched, theta_imu_synched = synchronizeRates(pos_gt_, theta_imu_)




# --- Plotting -------------------------------------------------------------------------
fig = plt.figure()
ax = fig.add_subplot()

## --- Positions ---

# plot ROS system pose
if PLOT_POS_ODOM == 1:
    ax.scatter(pos_odom[:, 0], 
            pos_odom[:, 1], 
            s=0.1, c='blue',
            label='Odom') 

# plot ground truth from OptiTrack system
if PLOT_POS_GT_RAW:
    ax.scatter(pos_gt[gt_startFrame:, 0], 
            pos_gt[gt_startFrame:, 1],
            alpha=0.15, 
            s=0.1, c='black',
            label='OptiTrack GT raw')

if PLOT_POS_GT_ADJ == 1:
    ax.scatter(pos_gt_[gt_startFrame:, 0], 
            pos_gt_[gt_startFrame:, 1], 
            s=0.1, c='orange',
            label='OptiTrack GT adjusted')

## --- Orientations ---

# Odom orientations
if PLOT_ODOM_ORIENTATIONS == 1:
    ax.quiver(pos_odom_subset[:,0], pos_odom_subset[:,1], 
            orient_odom_dir[:,0], orient_odom_dir[:,1],
            color='blue', scale=20, 
            label='Odom orientation')

# IMU orientations
if PLOT_IMU_ORIENTATIONS == 1:  
    theta_imu_dir = angles2vects(theta_imu_synched)        
    ax.quiver(pos_gt_synched[:,0], pos_gt_synched[:,1], 
        theta_imu_dir[:,0], theta_imu_dir[:,1],
        color='blue', scale=20, 
        label='IMU orientation')

ax.legend()
ax.grid(True)
ax.axis('equal')
plt.show(block = False)

dummy = 1