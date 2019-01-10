#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

X = []
Y = []
Z = []

def odometryCb(msg):
    
    global X
    X.append(msg.pose.pose.position.x)

    global Y
    Y.append(msg.pose.pose.position.y)

    global Z
    Z.append(msg.pose.pose.position.z)
 
    #print msg.pose.pose.position.x
    #print msg.pose.pose.position.y
    #print msg.pose.pose.position.z
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    

def Plotting():
    
    print X
    fig = plt.figure()
    #ax = fig.gca(projection='3d')
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_wireframe(X, Y, Z, label='parametric curve')
    plt.show()


def Storing():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('Plotting', anonymous=True)

    rospy.Subscriber("/mavros/local_position/odom", Odometry, odometryCb)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    Storing()
    Plotting()


