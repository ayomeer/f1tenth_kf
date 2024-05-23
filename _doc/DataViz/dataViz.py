import numpy as np
import matplotlib.pyplot as plt

from dataViz_utils import *
from kf_odom import Odom

# --- Config -----------------------------------------------------------------------------
# Measurement data paths
rosbag_path = 'ROS/sensordata_05_path'
optitrack_csv_path = 'OptiTrack/gt_OptiTrack_05_path.csv'

# Toggle plots
PLOT_POS_GT_RAW = 0
PLOT_POS_GT_ADJ = 1
PLOT_POS_ODOM   = 1
PLOT_POS_PYODOM = 1

PLOT_ODOM_ORIENTATIONS   = 0
PLOT_PYODOM_ORIENTATIONS = 1
PLOT_IMU_ORIENTATIONS    = 0

PLOT_STARTPOINT_DETECTION = 0

# Data adjustment params
ADV_IMU_ORIENT_NUDGE = 5
SYNCHED_SUBSAMPLE    = 2


# Time constants
dt = 0.02
NANO_TO_MILLI = 1/1000000

# --- Read Data -----------------------------------------------------------------------------
# Read Pose from RosBag
pos_odom, orient_odom, theta_odom_dot = readOdomData(rosbag_path)
theta_odom = np.cumsum(theta_odom_dot*dt)

# Read IMU Data from RosBag
theta_imu_dot, lin_accel_imu = readImuData(rosbag_path) # message in deg/s even though doc says rad/s
theta_imu_dot = deg2rad(theta_imu_dot) 
# NOTE: lin_accel_imu in IMU reference frame

# Read Servo & Motor commands from RosBag
t_servo, command_servo = readFloatData(rosbag_path, '/commands/servo/position')
t_rpm, command_rpm = readFloatData(rosbag_path, '/commands/motor/speed')

t_servo_ms = (t_servo-t_servo[0]) * NANO_TO_MILLI 
t_rpm_ms = (t_rpm-t_rpm[0]) * NANO_TO_MILLI

dt_servo = t_servo - np.insert(t_servo[:-1], 0, t_servo[0])
dt_rpm = t_rpm - np.insert(t_rpm[:-1], 0, t_rpm[0])


len_highRate = len(theta_imu_dot)

command_servo_upsampled = upsampleVariableRate(t_servo_ms, 
                                               command_servo,
                                               len_highRate)

command_rpm_upsampled = upsampleVariableRate(t_rpm_ms, 
                                             command_rpm,
                                             len_highRate)


# Read OptiTrack ground truth from CSV
pos_gt, orient_gt = readOptiTrackCSV(optitrack_csv_path)


# --- Generating Data for own Odometry -------------------------------------------------
pyOdom = Odom()
pos_pyOdom, theta_pyOdom = pyOdom.run(command_servo_upsampled, 
                                      command_rpm_upsampled)

# --- Data Manipulation ----------------------------------------------------------------
# --- Postion Data ---
# Odom
pos_odom -= pos_odom[0] # Reset Odom 
                        # (in case multiple measurements made without restarting driver stack)
q0 = orient_odom[0]
q0[-1] = -q0[-1] # rotate the other way (back)
pos_odom = quaternionRotPoints(pos_odom, q0)


# OptiTrack
# set beginning position to origin (like odom)
pos_gt -= pos_gt[0]


# Align  gt with odom starting orientation (odom is reference for car)
q0 = orient_gt[0]
q0[-1] = -q0[-1] # rotate the other way (back)
pos_gt_aligned = quaternionRotPoints(pos_gt, q0)

print("--- Coord system matching -------------------------------------------- ")
print("Aligning gt to odom with q0= \t",q0)
print("Rotation axis unit vector: \t", getQuatAxis(q0))
print("q0 rotation in deg: \t\t", round(quat2yaw(q0)*(360)/(2*np.pi),5))

# --- Orientation Data ---


# Odom
# get subsampled versions of odom orientations & positions for plotting
N_subsample = 25
pos_odom_subset = pos_odom[::N_subsample]
orient_odom_subset = orient_odom[::N_subsample] 

pos_pyOdom_subset = pos_pyOdom[::N_subsample]
theta_pyOdom_subset = theta_pyOdom[::N_subsample] 


# get unit dir vectors 
orient_dir_0 = np.array([1,0,0]) # base orientation vector to transform onto paths
orient_odom_dir = []
for i in range(orient_odom_subset.shape[0]):
    orient_odom_dir.append(quaternionRotPoint(orient_dir_0, orient_odom_subset[i]))

orient_odom_dir = np.array(orient_odom_dir)

orient_pyOdom_dir = angles2vects(theta_pyOdom_subset)


# IMU
# Subtract bias 
theta_imu_dot_unbiased = removeBias(theta_imu_dot)

# Convert angular velocity into angle by integrating
theta_imu_ = np.cumsum(theta_imu_dot_unbiased*dt)


# --- Synchronize Measurements ---
# synchronize measurement start
idx_start_gt, idx_start_imu = getTimeSynchIndices(pos_gt_aligned, lin_accel_imu, plot=PLOT_STARTPOINT_DETECTION)

# get synchronized subsamples of 50Hz and 120Hz measurements
ADV_IMU_ORIENT_NUDGE = 5 # adjust timing offset between imu and gt offset after automatic synch
pos_gt_synched, theta_imu_synched = synchronizeRates(pos_gt_aligned[idx_start_gt: ], 
                                                     theta_imu_[idx_start_imu+ADV_IMU_ORIENT_NUDGE: ],
                                                     N_subsample=SYNCHED_SUBSAMPLE)



# --- Print some stats -----------------------------------------------------------------
print("--- Data Stats -------------------------------------------------------")
print("Distance travelled according to gt (aligned): \t", round(getDistTraveled(pos_gt_aligned),5),"m")
print("Distance travelled according to odom: \t\t", round(getDistTraveled(pos_odom),5),"m")
print("Distance travelled according to pyOdom: \t", round(getDistTraveled(pos_pyOdom),5),"m")



# --- Plotting -------------------------------------------------------------------------
fig = plt.figure()
ax = fig.add_subplot()

## --- Positions ---
# plot ROS system pose
if PLOT_POS_ODOM == 1:
    ax.scatter( pos_odom[:, 0], 
                pos_odom[:, 1], 
                s=0.1, c='blue',
                label='Odom') 

# plot ground truth from OptiTrack system
if PLOT_POS_GT_RAW:
    ax.scatter( pos_gt[:, 0], 
                pos_gt[:, 1],
                alpha=0.15, 
                s=0.1, c='black',
                label='pos gt raw')

if PLOT_POS_GT_ADJ == 1:
    ax.scatter( pos_gt_aligned[:, 0], 
                pos_gt_aligned[:, 1], 
                s=0.1, c='orange',
                label='pos gt adjusted')
    
    # highlight time synch sample in pos data
    ax.scatter( pos_gt_aligned[idx_start_gt, 0], 
                pos_gt_aligned[idx_start_gt, 1], 
                marker='+', s=30, c='red',
                label='Time Synch Sample',
                zorder=20)
    
    # highlight synchronization samples
    ax.scatter( pos_gt_synched[:, 0], 
                pos_gt_synched[:, 1], 
                s=10, c='black',
                label='Synch Samples',
                zorder=10)
    
if PLOT_POS_PYODOM == True:
    ax.scatter( pos_pyOdom[:, 0], 
                pos_pyOdom[:, 1],
                alpha=1, 
                s=0.1, c='red',
                label='pos pyOdom')

## --- Orientations ---
# Odom orientations
if PLOT_ODOM_ORIENTATIONS == 1:
    ax.quiver(  pos_odom_subset[:,0], pos_odom_subset[:,1], 
                orient_odom_dir[:,0], orient_odom_dir[:,1],
                color='blue', 
                label='Odom orientation')

# pyOdom orientations
if PLOT_PYODOM_ORIENTATIONS == 1:
    ax.quiver(  pos_pyOdom_subset[:,0], pos_pyOdom_subset[:,1], 
                orient_pyOdom_dir[:,0], orient_pyOdom_dir[:,1],
                color='red', 
                label='pyOdom orientation')


# IMU orientations
if PLOT_IMU_ORIENTATIONS == 1:  
    theta_imu_dir = angles2vects(theta_imu_synched)        
    ax.quiver(  pos_gt_synched[:, 0], pos_gt_synched[:, 1], 
                theta_imu_dir[:, 0], theta_imu_dir[:, 1],
                color='black', 
                label='IMU orientation')



ax.legend()
ax.grid(True)
ax.axis('equal')
plt.show(block = True)

dummy = 1

