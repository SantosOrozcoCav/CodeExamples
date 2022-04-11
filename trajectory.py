#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

ros_rate = 10
samp = 100
k = 0
f = 0.15 #Hz
w = 2*np.pi*f
#t = np.linspace(0,10,samp*ros_rate)
#X = 0.4*np.cos(w*t)
#Y = 0.4*np.sin(w*t)

t = np.linspace(0,60,samp*ros_rate)
X = np.zeros(len(t))
Y = np.zeros(len(t))
Z = np.zeros(len(t))

for k in range(len(t)):
	X[k] = ( 1*np.sqrt(2)*np.cos(f*t[k]) )/( pow(np.sin(f*t[k]),2)+1 )
	Y[k] = ( 1*np.sqrt(2)*np.cos(f*t[k])*np.sin(f*t[k]) )/( pow(np.sin(f*t[k]),2)+1 )
	Z[k] = 1+0.025*t[k]
dX = np.concatenate( ([0],np.diff(X)) )
dY = np.concatenate( ([0],np.diff(Y)) )
dZ = np.concatenate( ([0],np.diff(Z)) ) 
ddX = np.concatenate( ([0],np.diff(dX)) )
ddY = np.concatenate( ([0],np.diff(dY)) )
ddZ = np.concatenate( ([0],np.diff(dZ)) )

k = 0
pub = rospy.Publisher('/references', Float32MultiArray, queue_size=10)
rospy.init_node('trajectory', anonymous=True)
r = rospy.Rate(ros_rate)
print("1: Regulation\n2: Regulation + orientation\n3: Tracking")
sw = float(input("\n"))
while not rospy.is_shutdown():
	if sw == 1:
		print("Target point:")
		x = float(input("x: "))
		y = float(input("y: "))
		z = float(input("z: "))
		ref = Float32MultiArray()
		take_off = 1.0
		ref.data = np.array([x,y,z,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,take_off])
	elif sw == 2:
		print("Target point:")
		x = float(input("x: "))
		y = float(input("y: "))
		z = float(input("z: "))
		psi = float(input("psi: "))
		ref = Float32MultiArray()
		take_off = 1.0
		ref.data = np.array([x,y,z,0,0,psi,0,0,0,0,0,0,0,0,0,0,0,0,take_off])
	else:
		ref = Float32MultiArray()
		take_off = 1.0
		print(str(k)+"--"+str(X[k])+"--"+str(Y[k])+"--"+str(Z[k]))
		ref.data = np.array([X[k],Y[k],Z[k],0,0,0,dX[k],dY[k],dZ[k],0,0,0,ddX[k],ddY[k],ddZ[k],0,0,0,take_off])
		k = k+1
	pub.publish(ref)
	r.sleep()
