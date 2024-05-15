import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

import csv

# --- RosBag Reader util -----------------------------------------------------

def readTopic(rosbag_path, topic):

    reader = Reader(rosbag_path) 
    # Create a typestore and get the string class.
    typestore = get_typestore(Stores.LATEST)

    with reader:
        pos = []
        orient = []

        # Iterate over messages.
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topic:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            
                pos.append((msg.pose.pose.position.x, 
                            msg.pose.pose.position.y, 
                            msg.pose.pose.position.z))
                
                orient.append((msg.pose.pose.orientation.x,
                               msg.pose.pose.orientation.y,
                               msg.pose.pose.orientation.z,
                               msg.pose.pose.orientation.w))

    return np.array(pos), np.array(orient)

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
def quaternionRot(p, q):
    u = q[0:3]
    s = q[3]
    return 2*(u@p)*u + (s**2 - u@u)*p + 2*s*np.cross(u,p)

def quaternionRotAlign(posArray, q):
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
        

        