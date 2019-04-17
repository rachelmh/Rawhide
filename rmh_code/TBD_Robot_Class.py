
#! /usr/bin/env python
import pdb
import rospy
import argparse
import numpy

import math
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time

from RobotRaconteur.Client import *

#THIS CODE DOESN"T HAVE ADJUSTABLE PLAYBACK FUNCTIONALITY YET
class RobotArm_TBD():


	def __init__(self, filename):

		#self.robot=RRN.ConnectService('tcp://localhost:45179/SawyerRMHServer/Sawyer')
		self.robot=RRN.ConnectService(filename)
		#self.captain=RRN.ConnectService('tcp://localhost:42413/SawyerRMHServer/Sawyer')

		self.robot.controlmode=0
		self.robot.setControlMode(self.robot.controlmode)
		self.robot.setPositionModeSpeed(.2)

		#self.captain.setControlMode(0)
		self.currpos=self.robot.joint_positions
		self.rate = .05  # Hz
		self.recrate=.01
		# functions beginning with get_ are functions that should be called in the main program. gets a value of a function, regardless of what the function is
		#other functions can be edited without fear of messing up the overall structure. Similar to changing the definition of a function. as long as
		#the changed function returns the same number and types of variables, the get_ function should still work
		self.countdown=50
		self.recordflag=0
		self.jointrecordedposition=[]
		self._done = False
		self.doneflag=False
		self.image_path=''



	def get_current_joint_positions(self):

		return self.robot.joint_positions

	def get_current_joint_velocity(self):
		return self.robot.joint_velocities

	def get_current_joint_torques(self):
		return self.robot.joint_torques

	def get_ee_pos(self):
		return self.robot.endeffector_positions

	def get_ee_orientation(self):
		return self.robot.endeffector_orientations

	def get_ee_twist(self):
		return self.robot.endeffector_twists

	def get_ee_wrench(self):
		return self.robot.endeffector_wrenches

	def get_ee_vel(self):
		return self.robot.endeffector_velocity
	def test(self):
		return self.get_current_joint_positions()

	def Move_To_Position(self,positions):
		#whichsawyer.setControlMode(0)
		pos=numpy.array(positions)
		curr_pos=numpy.array(self.get_current_joint_positions())
		curr_ee_vel=numpy.array(self.get_ee_vel())
		curr_max_vel=max(abs(numpy.array(self.get_current_joint_velocity())))
		vel_threshold=0.001
		prev_max=0
		max_counter=0
		ctr_ctr=0
		if self.robot.controlmode != 0:
			self.SetControlMode(0)
			self.SetPositionModeSpeed(.2)


		self.Set_Position( positions)
		while max_counter<100 :# and ctr_ctr<100:
			#print('looping')
			curr_pos=numpy.array(self.get_current_joint_positions())
			curr_max=max(abs(curr_pos-pos))
			vel_max=max(abs(curr_ee_vel))
			print('looping',curr_max,vel_max)
			#if vel_max < vel_threshold:
			# position threshold from sawyer settings 0.0087
			if (curr_max) < .0087:
				if curr_max_vel < .08:
					max_counter=max_counter+1


			#prev_max_ctr=max_counter

			#if prev_max_ctr==max_counter:
				#ctr_ctr=ctr_ctr+1



			print(max_counter,curr_max,curr_max_vel)
		print('break')


	def Set_Position(self,positions,speed):
		#whichsawyer.setControlMode(0)
		if self.robot.controlmode != 0:
			print('change')
			self.robot.setControlMode(0)
		self.robot.setPositionModeSpeed(speed)
		self.robot.setJointCommand('right', positions)

	def Set_Velocity(self,velocities):
		#whichsawyer.setControlMode(0)
		if self.robot.controlmode != 1:
			self.robot.setControlMode(1)

		self.robot.setJointCommand('right', velocities)

	def Set_Torques(self,torques):
		#whichsawyer.setControlMode(0)

		if self.robot.controlmode != 2:
			self.robot.setControlMode(2)
		self.robot.setJointCommand('right', torques)

	def SpaceNavJoyCallback(self):
		#roslaunch spacenav_node classic.launch
		spacenavdata=self.robot.SpaceNavigatorJoy

        #I think these have to be self. variables due to the nature of callback. data is only accessed in the callback

		self.Jxl_raw=spacenavdata[0]
		self.Jyl_raw=spacenavdata[1]
		self.Jzl_raw=spacenavdata[2]

		self.Jxa_raw=spacenavdata[3]
		self.Jya_raw=spacenavdata[4]
		self.Jza_raw=spacenavdata[5]
		self.leftbutton=spacenavdata[6]
		self.rightbutton=spacenavdata[7]

	def get_raw_SpaceNav(self):
		#shouldn't have to touch!
		self.robot.SpaceNavJoyCallback()
		(Jxel, Jyel, Jzel,Jxea, Jyea, Jzea)= (self.Jxl_raw,self.Jyl_raw,self.Jzl_raw,self.Jxa_raw,self.Jya_raw,self.Jza_raw)#self.ForceSensorCallback()
		#print("Jxel, Jyel, Jzel,Jxea, Jyea, Jzea",Jxel, Jyel, Jzel,Jxea, Jyea, Jzea)
		return (Jxel, Jyel, Jzel,Jxea, Jyea, Jzea)

	def get_inverse_Jacobian(self):
		#shouldn't have to touch!
		invJac1=numpy.asarray(self.robot.pseudoinverse_Jacobian)
		#print(invJac1)
		invJac=invJac1.reshape((7,6))
		#print(invJac)
		return invJac

	def SetControlMode(self,Mode):
		self.robot.controlmode=Mode
		self.robot.setControlMode(Mode)

	def SetPositionModeSpeed(self,speed):
		self.robot.setPositionModeSpeed(speed)
