import numpy as np
import matplotlib.pyplot as plt

from dataViz_utils import readTopic, readOptiTrackCSV, quaternionRot, quaternionRotAlign


# --- Config -----------------------------------------------------------------------------

rosbag_path = 'ROS/sensordata_03'
optitrack_csv_path = 'OptiTrack/gt_optiTrack_03.csv'


# toggle plots
PLOT_POS_GT_RAW = 0

PLOT_ODOM_ORIENTATIONS = 1
PLOT_IMU_ORIENTATIONS = 1

# --- Read Data -----------------------------------------------------------------------------

# Read Pose from RosBag
pos_odom, orient_odom = readTopic(rosbag_path, '/ego_racecar/odom')

# Read IMU Data from RosBag
#TODO: Read IMU Data

# Read OptiTrack ground truth from CSV
pos_gt, orient_gt = readOptiTrackCSV(optitrack_csv_path)


# --- Data Manipulation ----------------------------------------------------------------

# --- Postion Data ---

# Find when vehicle was put on the ground and zero starting point
# plt.plot(range(pos_gt.shape[0]), pos_gt[:, -1])
# plt.show()
# gt_startFrame = 2541 # sensordata_01
gt_startFrame = 1 # frame 0 is base frame!

pos_gt -= pos_gt[gt_startFrame]

# Align  gt with odom starting orientation (odom is reference for car)
q0 = orient_gt[gt_startFrame]
q0[-1] = -q0[-1] # rotate the other way (bacl)

pos_gt_ = quaternionRotAlign(pos_gt, q0)


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
    orient_odom_dir.append(quaternionRot(orient_dir_0, orient_odom_subset[i]))

orient_odom_dir = np.array(orient_odom_dir)

# IMU
# TODO: Get orientations from cumsum of accelerations along z-dir

# --- Plotting -------------------------------------------------------------------------
fig = plt.figure()
ax = fig.add_subplot()

## --- Positions ---

# plot ROS system pose
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


if PLOT_IMU_ORIENTATIONS == 1:
    # TODO: Plot IMU orientations similarly to odom ^
    pass

ax.legend()
ax.grid(True)
ax.axis('equal')
plt.show()

dummy = 1