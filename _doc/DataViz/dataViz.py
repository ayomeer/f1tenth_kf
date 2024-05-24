import numpy as np
import matplotlib.pyplot as plt

from dataViz_utils import *
from kf_odom import Odom

# --- Config -----------------------------------------------------------------------------
# Measurement data paths
rosbag_path = 'ROS/sensordata_06_path' 
optitrack_csv_path = 'OptiTrack/gt_OptiTrack_06_path.csv'

# Toggle plots
PLOT_POS_GT_RAW = 0
PLOT_POS_GT_ADJ = 1
PLOT_POS_ODOM   = 1
PLOT_POS_PYODOM = 1

# Orientation quiver plots
PLOT_ODOM_ORIENTATIONS   = 0
PLOT_PYODOM_ORIENTATIONS = 0
PLOT_IMU_ORIENTATIONS    = 0

# Debug plots
PLOT_STARTPOINT_DETECTION = 0

# Show index next to scatter dots
ANNOTATE_POS_IDX = 0
SCATTER_SCALE = 1 + ANNOTATE_POS_IDX

PLOT_SYNCH_DOTS = 0
SYNCH_DOTS_SCALE = 10

# Data adjustment params
ADV_IMU_ORIENT_NUDGE = 5
SYNCHED_SUBSAMPLE    = 5

ADV_IMU_ORIENT_NUDGE = 5 # adjust timing offset between imu and gt offset


synchMult = 5
N_subsample = synchMult * 5

# Time constants
dt = 0.02
NANO_TO_MILLI = 1/1000000

# --- Read Data -----------------------------------------------------------------------------
# Read Pose from RosBag
pos_odom, orient_odom, theta_odom_dot = readOdomData(rosbag_path)
theta_odom = np.cumsum(theta_odom_dot*dt)

# Read IMU Data from RosBag ('/sensors/imu/raw')
theta_imu_dot, lin_accel_imu = readImuData(rosbag_path) # message in deg/s even though doc says rad/s
theta_imu_dot = deg2rad(theta_imu_dot) 
# NOTE: lin_accel_imu in IMU reference frame

# clip for countering delay
theta_imu_dot = theta_imu_dot[ADV_IMU_ORIENT_NUDGE:]

# Read motor speed measurement
rpm_sens = readSensorsCore(rosbag_path)
t_servo_sens, servo_sens = readFloatData(rosbag_path, '/sensors/servo_position_command')
t_servo_ms = (t_servo_sens-t_servo_sens[0]) * NANO_TO_MILLI 



# clip high rate signals to minLength (in case rosbag recording halted inbetween topics):
highRateData = [pos_odom, orient_odom, theta_odom_dot, # from /ego_racecar/odom (50Hz)
                theta_imu_dot, lin_accel_imu,          # from /sensors/imu/raw  (50Hz)
                rpm_sens]                              # from                   (50Hz)

minLength = min(len(arr) for arr in highRateData)

pos_odom = pos_odom[:minLength]
orient_odom = orient_odom[:minLength]
theta_odom_dot = theta_odom_dot[:minLength]

theta_imu_dot = theta_imu_dot[:minLength]
lin_accel_imu = lin_accel_imu[:minLength]
rpm_sens = rpm_sens[:minLength]


# Read OptiTrack ground truth from CSV
pos_gt, orient_gt = readOptiTrackCSV(optitrack_csv_path)




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
# IMU
# Subtract bias 
theta_imu_dot_unbiased = removeBias(theta_imu_dot)

# Convert angular velocity into angle by integrating
theta_imu = np.cumsum(theta_imu_dot_unbiased*dt)


# --- Synchronize IMU & Ground Truth Data ---
# synchronize measurement start

idx_start_gt, idx_start_imu = getTimeSynchIndices(pos_gt_aligned, lin_accel_imu, plot=PLOT_STARTPOINT_DETECTION)

theta_imu = theta_imu[idx_start_imu+ADV_IMU_ORIENT_NUDGE: ]
pos_gt_aligned = pos_gt_aligned[idx_start_gt: ]

# get synchronized subsamples of 50Hz and 120Hz measurements
pos_gt_synched, theta_imu_synched = synchronizeRates(pos_gt_aligned, 
                                                     theta_imu,
                                                     N_subsample=SYNCHED_SUBSAMPLE)


# --- Generating Data for own Odometry -------------------------------------------------
# Align command data w/ variable update rate to fixed rate imu data
len_fixedRate = len(theta_imu_dot) # May need to adjust by adding '-1' 
servo_upsampled = upsampleVariableRate(t_servo_ms, 
                                       servo_sens,
                                       len_fixedRate)

pyOdom = Odom()
state = pyOdom.run(servo_upsampled, 
                   rpm_sens,
                   theta_imu_dot_unbiased)

pos_pyOdom, theta_pyOdom = state[0], state[1]


# --- Print some stats -----------------------------------------------------------------
print("--- Data Stats -------------------------------------------------------")
print("Distance travelled according to gt (aligned): \t", round(getDistTraveled(pos_gt_aligned),5),"m")
print("Distance travelled according to odom: \t\t", round(getDistTraveled(pos_odom),5),"m")
print("Distance travelled according to pyOdom: \t", round(getDistTraveled(pos_pyOdom),5),"m")



# --- Plotting -------------------------------------------------------------------------
fig = plt.figure()
ax = fig.add_subplot()

## --- Positions ---
# plot ground truth from OptiTrack system
if PLOT_POS_GT_RAW:
    ax.scatter( pos_gt[:, 0], 
                pos_gt[:, 1],
                alpha=0.15, 
                s=SCATTER_SCALE, c='black',
                label='pos gt raw')

if PLOT_POS_GT_ADJ == 1:
    ax.scatter( pos_gt_aligned[:, 0], 
                pos_gt_aligned[:, 1], 
                s=SCATTER_SCALE, c='orange',
                label='pos gt OptiTrack')
    
    # highlight time synch sample in pos data
    ax.scatter( pos_gt_aligned[idx_start_gt, 0], 
                pos_gt_aligned[idx_start_gt, 1], 
                marker='+', s=30, c='red',
                label='Time Synch Sample',
                zorder=20)
    
    if PLOT_SYNCH_DOTS == 1:
        #highlight synchronization samples
        ax.scatter( pos_gt_aligned[::12, 0], 
                    pos_gt_aligned[::12, 1], 
                    s=SYNCH_DOTS_SCALE, c='orange',
                    label='Synch Samples',
                    zorder=10)

# plot ROS system pos
if PLOT_POS_ODOM == 1:
    ax.scatter( pos_odom[:, 0], 
                pos_odom[:, 1], 
                s=SCATTER_SCALE, c='blue',
                label='pos Odom') 
    
    if PLOT_SYNCH_DOTS == 1:
        ax.scatter( pos_odom[::5, 0], 
                    pos_odom[::5, 1], 
                    s=SYNCH_DOTS_SCALE, c='blue',
                    label='Synch Samples',
                    zorder=10)

    if ANNOTATE_POS_IDX == 1:
        for i in range(pos_odom.shape[0]):
            ax.annotate(i, (pos_odom[i,0], pos_odom[i,1]))

# plot pyOdom pos
if PLOT_POS_PYODOM == True:
    ax.scatter( pos_pyOdom[:, 0], 
                pos_pyOdom[:, 1],
                alpha=1, 
                s=SCATTER_SCALE, c='red',
                label='pos pyOdom')
    
    if PLOT_SYNCH_DOTS == 1:
        #highlight synchronization samples
        ax.scatter( pos_pyOdom[::5, 0], 
                    pos_pyOdom[::5, 1], 
                    s=SYNCH_DOTS_SCALE, c='red',
                    label='Synch Samples',
                    zorder=10)
    
    if ANNOTATE_POS_IDX == 1:
        for i in range(pos_pyOdom.shape[0]):
            ax.annotate(i, (pos_pyOdom[i,0], pos_pyOdom[i,1]))


## --- Orientations ---
# get subsampled versions of odom orientations & positions for plotting
pos_odom_subset = pos_odom[::N_subsample]
orient_odom_subset = orient_odom[::N_subsample] 

pos_pyOdom_subset = pos_pyOdom[::N_subsample]
theta_pyOdom_subset = theta_pyOdom[::N_subsample] 

# get unit dir vectors for quiver plot
orient_dir_0 = np.array([1,0,0]) # base orientation vector to transform onto paths
orient_odom_dir = []
for i in range(orient_odom_subset.shape[0]):
    orient_odom_dir.append(quaternionRotPoint(orient_dir_0, orient_odom_subset[i]))

orient_odom_dir = np.array(orient_odom_dir)

orient_pyOdom_dir = angles2vects(theta_pyOdom_subset)

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
plt.show(block = False)
dummy = 1
