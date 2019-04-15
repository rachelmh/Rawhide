
#! /usr/bin/env python

import rospy
import math
from std_msgs import bool
import time

from RobotRaconteur.Client import *

def pushed1callback(data):
	poirot1=data
def pushed2callback(data):
	poirot2=data
def pushed3callback(data):
	captain1=data
def pushed4callback(data):
	captain2=data
	#pdb.set_trace()
	#DA_Move_To_Position(poirot,captain,[0, -2.5, 0, -1.9, 0, 0, 3.3],[0, -2.5, 0, -1.9, 0,0,3.3],.2)
def main():

	#NEUTRAL POSITION {'right_j6': 3.3119794921875, 'right_j5': 0.566505859375, 'right_j4': 0.0049091796875, 'right_j3': 2.177044921875, 'right_j2': -0.00241796875, 'right_j1': -1.178189453125, 'right_j0': 0.001283203125}
	print("Lets get it on")

	poirot1=0
	poirot2=0
	captain1=0
	captain2=0
	rospy.Subscriber("/pushed1" , bool, pushed1callback)
	rospy.Subscriber("/pushed2" , bool, pushed2callback)
	rospy.Subscriber("/pushed3" ,bool, pushed3callback)
	rospy.Subscriber("/pushed4" , bool, pushed4callback)

	while True:
		print(poirot1,poirot2,captain1,captain2)


if __name__ == '__main__':
	print('running main')
	main()
