
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





def DA_Move_To_Position(robot1,robot2,positions1,positions2,speed):
	#whichsawyer.setControlMode(0)
	despos1=numpy.array(positions1)
	despos2=numpy.array(positions2)
	curr_pos1=numpy.array(robot1.get_current_joint_positions())
	curr_pos2=numpy.array(robot2.get_current_joint_positions())
	curr_ee_vel1=numpy.array(robot1.get_ee_vel())
	curr_ee_vel2=numpy.array(robot2.get_ee_vel())
	curr_max_vel1=max(abs(numpy.array(robot1.get_current_joint_velocity())))
	curr_max_vel2=max(abs(numpy.array(robot2.get_current_joint_velocity())))

	vel_threshold=0.001
	prev_max=0
	max_counter=0
	ctr_ctr=0


	robot1.SetControlMode(0)
	robot2.SetControlMode(0)
	robot1.robot.setPositionModeSpeed(speed)
	robot2.robot.setPositionModeSpeed(speed)


	robot1.Set_Position(positions1,speed)
	robot2.Set_Position(positions2,speed)
	start=time.time()
	while max_counter<100 :# and ctr_ctr<100:
		#print('looping')

		curr_pos1=numpy.array(robot1.get_current_joint_positions())
		curr_pos2=numpy.array(robot2.get_current_joint_positions())
		curr_ee_vel1=numpy.array(robot1.get_ee_vel())
		curr_ee_vel2=numpy.array(robot2.get_ee_vel())


		#curr_pos1=numpy.array(self.get_current_joint_positions(self.robot))
		curr_max1=max(abs(curr_pos1-despos1))
		curr_max2=max(abs(curr_pos2-despos2))
		vel_max1=max(abs(curr_ee_vel1))
		vel_max2=max(abs(curr_ee_vel2))
		print('looping','time: ',time.time()-start,'ctr: ', max_counter,curr_max1,vel_max1, curr_max2, vel_max2)
		#if vel_max < vel_threshold:
		# position threshold from sawyer settings 0.0087
		# if (curr_max1) < .0087 and (curr_max1) < .0087 :
		if (curr_max1) < .08 and (curr_max2) < .08 :
			if vel_max1 < .08 and vel_max2 < .08:
				max_counter=max_counter+1



		if time.time()-start > 7  and (curr_max1) < .02 and (curr_max2) < .02 and vel_max1 < .08 and vel_max2 < .08:
			max_counter=100
			print('timeout')


def time_stamp(start_time):
	ts=time.time() - start_time
	return ts


def record_sequence(robot1,robot2):
	xflag=0
	scrollflag=0
	fileiter=0
	filename_list=[]
	repeatflag=1
	noinput=1
	button=0
	while repeatflag==1 :
		robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/startrecording.png')
		robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/startrecording.png')
		print('to start recording sequence, press Red Button')
		while robot1.robot.PB1==0 and robot1.robot.PB3==0:
			time.sleep(.01)
		robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/stoprecord.png')
		robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/stoprecord.png')
		fileName = ("/home/rachel/rawhide/rmh_code/TBD_logs/logdemo_%d.txt" % (fileiter))

		record(robot1,robot2,fileName)

		robot1.SetControlMode(0)
		robot1.SetControlMode(0)
		#import pdb;
		#pdb.set_trace()
		robot1.currpos=robot1.get_current_joint_positions()
		robot1.currpos=robot1.get_current_joint_positions()
		filename_list.append(fileName)
		fileiter=fileiter+1

		robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/recordanother.png')
		robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/recordanother.png')

		print('get button')
		#button=xorscroll(robot1)
		button=redorblue(robot1,robot2)
		print('gotbutton')
		time.sleep(1)
		if button=='blue':
			repeatflag=0
		time.sleep(.2)



	robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/donerecordingsequence.png')
	robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/donerecordingsequence.png')
	print('DoneRecordingSequence')
	time.sleep(.1)
	return filename_list

	#done recording a sequence, now check if need to redo sequence


def record(robot1,robot2,filename):
	"""
	Records the current joint positions to a csv file if outputFilename was
	provided at construction this function will record the latest set of
	joint angles in a csv format.

	If a file exists, the function will overwrite existing file.
    """

	recrate=.1#.003
	time.sleep(1)
	robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/3.png')
	robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/3.png')
	time.sleep(1)
	robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/2.png')
	robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/2.png')
	time.sleep(1)
	robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/1.png')
	robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/1.png')
	time.sleep(1)

	print('recording now!')

	robot1.currpos=robot1.get_current_joint_positions()
	robot2.currpose=robot2.get_current_joint_positions()
	grippos=[robot1.robot.gripperPos,robot2.robot.gripperPos]
	angles_right1=numpy.concatenate((robot1.currpos, robot2.currpose))
	angles_right_start=numpy.concatenate((angles_right1,grippos))
	robot1.SetControlMode(0)
	robot2.SetControlMode(0)
	checkcuffbutton(robot1,robot2)
	robot1.doneflag=False
	if filename:

		with open(filename, 'w') as f:
			start_time=time.time()


			#print(angles_right,grippos)
			temp_str =  '\n'
			f.write("%f," % (time_stamp(start_time),))
			#f.write(','.join([str(x) for x in angles_right]) +  temp_str)
			f.write(','.join([str(x) for x in angles_right_start]) +  temp_str)
			#robot1.SetControlMode(0)
			# robot1.SetControlMode(2)
			# robot2.SetControlMode(2)


			while not robot1.doneflag:
				checkcuffbutton(robot1,robot2)
				robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/stoprecord.png')
				robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/stoprecord.png')
				print('vels',robot1.get_current_joint_velocity())
				#robot1.Set_Torques([0,0,0,0,0,0,0])
				#robot2.Set_Torques([0,0,0,0,0,0,0])
				#print('recording',robot1.doneflag)
				temp_str =  '\n'
				angles_rob1 = robot1.get_current_joint_positions()
				angles_rob2 = robot2.get_current_joint_positions()
				vels_rob1=robot1.get_current_joint_velocity()
				vels_rob2=robot2.get_current_joint_velocity()
				grippos=[robot1.robot.gripperPos,robot2.robot.gripperPos]

				angles_right1=numpy.concatenate((angles_rob1, angles_rob2))
				angles_right=numpy.concatenate((angles_right1,grippos))

				vels_right1=numpy.concatenate((vels_rob1, vels_rob2))
				vels_right=numpy.concatenate((vels_right1,grippos))
				#print(angles_right,grippos)
				f.write("%f," % (time_stamp(start_time),))
				#f.write(','.join([str(x) for x in angles_right]) +  temp_str)
				f.write(','.join([str(x) for x in vels_right]) +  temp_str)
				#print(','.join([str(x) for x in angles_right]) +  temp_str)
				# f.write(','.join([str(x) for x in angles_right]) + ',' + temp_str)
				if robot1.robot.PB1==1 or robot1.robot.PB3==1:
					robot1.currpos=robot1.get_current_joint_positions()
					robot2.currpose=robot2.get_current_joint_positions()
					grippos=[robot1.robot.gripperPos,robot2.robot.gripperPos]
					angles_right1=numpy.concatenate((robot1.currpos, robot2.currpose))
					angles_right_start=numpy.concatenate((angles_right1,grippos))
					f.write("%f," % (time_stamp(start_time),))
					f.write(','.join([str(x) for x in angles_right]) +  temp_str)
					robot1.doneflag=True

				time.sleep(recrate)



			robot1.doneflag=False
			print("done!")
			robot1.currpos=robot1.get_current_joint_positions()
			robot2.currpos=robot2.get_current_joint_positions()

		# print('setting control mode to 0')
			robot1.SetControlMode(0)
			robot2.SetControlMode(0)
			time.sleep(.1)
	else:
		print("error with filename")




def playback_sequence(robot1,robot2,filename_list):
	#lines.extend([1,2,3,4,5])
	button='None'
	robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/beginplayback.png')
	robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/beginplayback.png')
	print('press red button to  begin playback')
	while not robot1.robot.PB1 and robot1.robot.PB3:
		time.sleep(.1)

	# import pdb;
	# pdb.set_trace()

	for name in filename_list:
		playback(robot1,robot2,name)
		robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/makeadjustments.png')
		robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/makeadjustments.png')
		#button=xorscroll(robot1)
		button=redorblue(robot1,robot2)
		if button == 'red':

			editplaybackfile(robot1,robot2,name)




	print('alldone playback sequence function')




def playback(robot1,robot2,filename):

	with open(filename, 'r') as f:
		lines = f.readlines()
	jointgoal=[]
	currjointgoal=[]
	currtime=0
	#lines[-1] to get timestamp
	startpos=numpy.array(lines[0].rstrip().split(','))
	startjointgoal=startpos.astype(numpy.float)
	endpos=numpy.array(lines[-1].rstrip().split(','))
	endjointgoal=endpos.astype(numpy.float)
	print('moving to starting pos')
	robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/playbackmoving.png')
	robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/playbackmoving.png')

	DA_Move_To_Position(robot1,robot2,startjointgoal[1:8],startjointgoal[8:15],.2)

	robot1.robot.setGRIPpos(startjointgoal[15])
	robot2.robot.setGRIPpos(startjointgoal[16])

	#for rev playback,extract timestamp of reversed(lines)[0], this is offset time.
	#so then in the forloop, need to edit currjointgoal[0] = abs(currjointgoal[0]-offsettime).
	#rev playback will be to go from A2 to A, if big changes.
	#should see how it goes with small changes, just for demo.
	#using collision checking + discrete 0.5 s waypoints may also help. cause then you can just change waypoints
	#but this is an issue for after the demo, pending nothing big goes wrong with the simplest implementation


	for l in lines:  #in reversed(lines) for reverse playback?
		#pdb.set_trace()
		currtime=time.time()
		nums=numpy.array(l.rstrip().split(','))

		currjointgoal=nums.astype(numpy.float)
		jointgoal.append(currjointgoal)


	jointgoal.remove
	time.sleep(0.5)
	robot1.robot.setControlMode(0)
	robot2.robot.setControlMode(0)
	robot1.robot.setPositionModeSpeed(1)
	robot2.robot.setPositionModeSpeed(1)



	starttime=time.time()

	#remove start and last joint pos from jointgoal
	del jointgoal[0]
	del jointgoal[-1]
	robot1.robot.setControlMode(1)
	robot2.robot.setControlMode(1)
	for goal in jointgoal:

		robot1.robot.setJointCommand( 'right',goal[1:8])
		robot2.robot.setJointCommand('right', goal[8:15])
		print('setgrippos')
		robot1.robot.setGRIPpos(goal[15])
		robot2.robot.setGRIPpos(goal[16])
		while (abs(time.time() - starttime) <goal[0]):

			time.sleep(0.001)

	robot1.robot.setControlMode(0)
	robot2.robot.setControlMode(0)
	DA_Move_To_Position(robot1,robot2,endjointgoal[1:8],endjointgoal[8:15],.2)
	robot1.robot.setPositionModeSpeed(1)
	robot2.robot.setPositionModeSpeed(1)
	robot1.currpos=robot1.get_current_joint_positions()
	robot2.currpos=robot2.get_current_joint_positions()
	robot1.robot.setJointCommand('right',robot1.currpos)
	robot2.robot.setJointCommand('right',robot2.currpos)


def editplaybackfile(robot1,robot2,filename):
	robot1.robot.setPositionModeSpeed(1)
	robot2.robot.setPositionModeSpeed(1)
	print('reading file')
	with open(filename, 'r') as f:
		lines = f.readlines()
	jointgoal=[]
	currjointgoal=[]
	currtime=0
	#lines[-1] to get timestamp
	startpos=numpy.array(lines[0].rstrip().split(','))
	endpos=numpy.array(lines[-1].rstrip().split(','))
	startjointgoal=startpos.astype(numpy.float)
	endjointgoal=endpos.astype(numpy.float)
	with open(filename, 'w') as f:
		for i in range(len(lines)-1):
			f.write(lines[i])
	pdb.set_trace()
	print('moving to starting pos')
	DA_Move_To_Position(robot1,robot2,endjointgoal[1:8],endjointgoal[8:15],.2)
	robot1.robot.setGRIPpos(endjointgoal[15])
	robot2.robot.setGRIPpos(endjointgoal[16])
	endingtime=endjointgoal[0]
	print('endingtime',endingtime)
	time.sleep(4)
	print('opening file')
	with open(filename, 'a') as f:
		recrate=.003
		time.sleep(1)
		robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/3.png')
		time.sleep(1)
		robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/2.png')
		time.sleep(1)
		robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/1.png')
		time.sleep(1)

		print('recording now!')
		robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/stoprecord.png')
		# robot1.currpos=robot1.get_current_joint_positions()
		# robot2.currpose=robot2.get_current_joint_positions()
		robot1.SetControlMode(0)
		robot2.SetControlMode(0)
		robot1.robot.setPositionModeSpeed(.2)
		robot2.robot.setPositionModeSpeed(.2)
		checkcuffbutton(robot1,robot2)
		robot1.doneflag=False
		temp_str =  '\n'
		angles_rob1 = robot1.get_current_joint_positions()
		angles_rob2 = robot2.get_current_joint_positions()
		angles_right1=numpy.concatenate((angles_rob1, angles_rob2))
		vels_rob1=robot1.get_current_joint_velocity()
		vels_rob2=robot2.get_current_joint_velocity()
		vels_right1=numpy.concatenate((vels_rob1, vels_rob2))
		grippos=[robot1.robot.gripperPos,robot2.robot.gripperPos]
		vels_right=numpy.concatenate((vels_right1,grippos))

		angles_right=numpy.concatenate((angles_right1,grippos))
		start_time=time.time()
		#f.write("%f," % (time_stamp(start_time)+endingtime,))
		#f.write(','.join([str(x) for x in angles_right]) +  temp_str)
		#start_time=time.time()
		#robot1.SetControlMode(0)
		# robot1.SetControlMode(2)
		# robot2.SetControlMode(2)


		while not robot1.doneflag:
			checkcuffbutton(robot1,robot2)
			temp_str =  '\n'
			angles_rob1 = robot1.get_current_joint_positions()
			angles_rob2 = robot2.get_current_joint_positions()
			angles_right1=numpy.concatenate((angles_rob1, angles_rob2))
			grippos=[robot1.robot.gripperPos,robot2.robot.gripperPos]

			#angles_right1=numpy.concatenate((angles_rob1, angles_rob2))
			angles_right=numpy.concatenate((angles_right1,grippos))
			#print(angles_right)
			#print(time_stamp(start_time+endingtime), time_stamp(start_time))
			vels_rob1=robot1.get_current_joint_velocity()
			vels_rob2=robot2.get_current_joint_velocity()
			vels_right1=numpy.concatenate((vels_rob1, vels_rob2))
			vels_right=numpy.concatenate((vels_right1,grippos))
			f.write("%f," % (time_stamp(start_time)+endingtime,))
			f.write(','.join([str(x) for x in vels_right]) +  temp_str)
			#print(','.join([str(x) for x in angles_right]) +  temp_str)
			# f.write(','.join([str(x) for x in angles_right]) + ',' + temp_str)
			if robot1.robot.PB1==1 or robot1.robot.PB3==1:
				temp_str =  '\n'
				angles_rob1 = robot1.get_current_joint_positions()
				angles_rob2 = robot2.get_current_joint_positions()
				angles_right1=numpy.concatenate((angles_rob1, angles_rob2))
				vels_rob1=robot1.get_current_joint_velocity()
				vels_rob2=robot2.get_current_joint_velocity()
				vels_right1=numpy.concatenate((vels_rob1, vels_rob2))
				vels_right=numpy.concatenate((vels_right1,grippos))
				grippos=[robot1.robot.gripperPos,robot2.robot.gripperPos]

				angles_right=numpy.concatenate((angles_right1,grippos))

				f.write("%f," % (time_stamp(start_time)+endingtime,))
				f.write(','.join([str(x) for x in angles_right]) +  temp_str)
				robot1.doneflag=True

			time.sleep(recrate)




		print("done editing!")
		robot1.SetControlMode(0)
		robot2.SetControlMode(0)
		robot1.robot.setPositionModeSpeed(.2)
		robot2.robot.setPositionModeSpeed(.2)
		time.sleep(.2)





def checkcuffbutton(robot1,robot2):

	if robot1.robot.CuffButton==1 or robot2.robot.CuffButton==1:
		robot1.currpos=robot1.get_current_joint_positions()
		robot2.currpos=robot2.get_current_joint_positions()
		robot1.SetControlMode(0)
		robot2.SetControlMode(0)
		robot1.robot.setPositionModeSpeed(.2)
		robot2.robot.setPositionModeSpeed(.2)
		#print("cuffbutton")
	else:
		robot1.Set_Position(robot1.currpos,10)
		robot2.Set_Position(robot2.currpos,10)
		#print("nocuffbutton")


def check_button_press(previous_button_state, button_to_check):
	#print('cbp',previous_button_state,type(previous_button_state),button_to_check,type(button_to_check))
	if previous_button_state==0:
		if button_to_check==1:
			print('return 1')
			return 1
	else: return 2

def xorscroll(robot1):
	xflag=0
	scrollflag=0
	time.sleep(.25)
	while xflag==0 and scrollflag==0:
		scrollflag=robot1.robot.OkWheelButton
		xflag=robot1.robot.XButton
		if xflag==1:
			return('x')
		if scrollflag==1:
			return('scroll')
		time.sleep(.01)
def redorblue(robot1,robot2):
	redflag=0
	blueflag=0
	time.sleep(.25)
	while redflag==0 and blueflag==0:
		#print(robot1.robot.PB3,robot1.robot.PB1,robot1.robot.PB2,robot1.robot.PB4)
		redflag1=robot1.robot.PB3
		redflag2=robot1.robot.PB1
		blueflag1=robot1.robot.PB2
		blueflag2=robot1.robot.PB4
		if redflag1== 1 or redflag2 == 1:
			redflag=1

		if blueflag1== 1 or blueflag2 == 1:
			blueflag=1

		if redflag==1:
			return('red')
		if blueflag==1:
			return('blue')
		time.sleep(.01)

def move_to_wire_ready_pos(robot1,robot2):
	DA_Move_To_Position(robot1,robot2,[0, -1.18, 0, 2.6, 0, 0, 3.3],[0, -1.18, 0, 2.6, 0,0,3.3],.2)
	DA_Move_To_Position(robot1,robot2,[0, -2, 0, 0, 0, 2.5, 3.3],[0, -2, 0, 0, 0,2.5,3.3],.2)
	DA_Move_To_Position(robot1,robot2,[0, -2.5, 0, -1.9, 0, 0, 3.3],[0, -2.5, 0, -1.9, 0,0,3.3],.2)

def grab_wire_ret_to_neut(robot1,robot2):
	move_to_wire_ready_pos(robot1,robot2)
	#xorscroll(robot1)
	robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/grabwire.png')
	robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/grabwire.png')
	redorblue(robot1,robot2)
	DA_Move_To_Position(robot1,robot2,[0, -2, 0, 0, 0, 2.5, 3.3],[0, -2, 0, 0, 0,2.5,3.3],.2)
	DA_Move_To_Position(robot1,robot2,[0, -1.18, 0, 2.6, 0, 0, 3.3],[0, -1.18, 0, 2.6, 0,0,3.3],.2)



	#pdb.set_trace()
	#DA_Move_To_Position(poirot,captain,[0, -2.5, 0, -1.9, 0, 0, 3.3],[0, -2.5, 0, -1.9, 0,0,3.3],.2)
def main():

	#NEUTRAL POSITION {'right_j6': 3.3119794921875, 'right_j5': 0.566505859375, 'right_j4': 0.0049091796875, 'right_j3': 2.177044921875, 'right_j2': -0.00241796875, 'right_j1': -1.178189453125, 'right_j0': 0.001283203125}
	print("Lets get it on")
	poirot=RobotArm_TBD('tcp://localhost:40369/SawyerRMHServer/Sawyer')
	captain=RobotArm_TBD('tcp://localhost:33363/SawyerRMHServer/Sawyer')


	poirot.robot.setPositionModeSpeed(.2)
	captain.robot.setPositionModeSpeed(.2)
	prevr1=0
	prevr2=0
	prevb1=0
	prevb2=0
	prevwheel=0
	prevx=0
	fileiter=0
	filename_list=[]
	playback_flag=0
	done_recording_flag=0
	totallydoneflag=0
	wire_ready_pos= [3, -1.18, 0, 2.6, 0, 0, 3.3]

		#print('button',poirot.robot.PB1,poirot.robot.PB2)

	#DA_Move_To_Position(poirot,captain,[0,.1,.1,0,0,0,0],[0,.1,.1,0,0,0,0])
	#DA_Move_To_Position(poirot,captain,[0, 0, 0, 2.2, 0,'tcp://localhost:35473/SawyerRMHServer/Sawyer' -.5, 3.3],[0, 0, 0, 2.2, 0,-.5,3.3])
	poirot.robot.setGRIPpos(0)
	#pdb.set_trace()

	time.sleep(.5)
	print('move to small pos')
	DA_Move_To_Position(poirot,captain,[0, -1.18, 0, 2.6, 0, 0, 3.3],[0, -1.18, 0, 2.6, 0,0,3.3],.2)
	print('get wire')
	time.sleep(.5)

	#grab_wire_ret_to_neut(poirot,captain)




	#DA_Move_To_Position(poirot,captain,wire_ready_pos,wire_ready_pos,.2)


	#pdb.set_trace()

	#DA_Move_To_Position(poirot,captain,[0, -1.18, 0, 2.2, 0, -.5, 3.3],[0, -1.18, 0, 2.2, 0,-.5,3.3],.2)
	#DA_Move_To_Position(poirot,captain,[.5, -1.18, 0, 2.2, 0, -.5, 3.3],[0, -1.18, 0, 2.2, 0,-.5,3.3])

	time.sleep(1)

	poirot.currpos=poirot.get_current_joint_positions()
	captain.currpos=captain.get_current_joint_positions()
	wpress=0
	xpress=0
	rpress=0
	bpress=0
	button='None'


	# poirot.robot.setDIOCommand('port_sink_0', 1)
	# captain.robot.setDIOCommand('port_sink_0', 1)
	time.sleep(1)
	while not rospy.is_shutdown():

		curr_wheel=poirot.robot.OkWheelButton
		curr_x=poirot.robot.XButton
		curr_red1=poirot.robot.PB1
		curr_red2=poirot.robot.PB3
		curr_blue1=poirot.robot.PB2
		curr_blue2=poirot.robot.PB4

		checkcuffbutton(poirot,captain)

		r1press=check_button_press(prevwheel,poirot.robot.PB1)
		r2press=check_button_press(prevwheel,poirot.robot.PB3)
		b1press=check_button_press(prevwheel,poirot.robot.PB2)
		b2press=check_button_press(prevwheel,poirot.robot.PB4)

		# wpress=check_button_press(prevwheel,poirot.robot.OkWheelButton)
		# xpress=check_button_press(prevx,poirot.robot.XButton)

		#print('buttpress',r1press,r2press,b1press,b2press)

		if done_recording_flag==0:
			poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/torecord.png')
			captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/torecord.png')
			print('to record, press the red button, to quit press the blue button Waiting')
			#
			#
			# r1press=check_button_press(prevr1,poirot.robot.PB1)
			# r2press=check_button_press(prevr2,poirot.robot.PB3)
			# b1press=check_button_press(prevb1,poirot.robot.PB2)
			# b2press=check_button_press(prevb2,poirot.robot.PB4)
			#
			# if r1press==1 or r2press==1:
			# 	rpress=1
			# if b1press==1 or b2press==1:
			# 	bpress=1
			button=redorblue(poirot,captain)
			#print('button',button)
			if button=='red':
				#print('red press')
				rpress=1

			else:
				playback_flag=1
				done_recording_flag=1
			#wpress=check_button_press(prevwheel,poirot.robot.OkWheelButton)

			if rpress==1:# wpress==1:
				print(1)

				time.sleep(1)
				filename_list=record_sequence(poirot,captain)
				poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/keep_redo.png')
				captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/keep_redo.png')
				print('done recording!')
				print('To keep the sequence, press red button. To redo sequence press blue button')
				time.sleep(.2)
				#button=xorscroll(poirot)
				button=redorblue(poirot,captain)
				print('button1',button)
				#time.sleep(1)
				if button=='red':
					done_recording_flag=1
			else:
				pass




		if done_recording_flag==1:
			if playback_flag==0:


					poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/torecord_toplayback.png')
					captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/torecord_toplayback.png')
					print('to record, press scrollwheel. To playback press x button')
					#button=xorscroll(poirot)
					button=redorblue(poirot,captain)
					if button=='red':
						done_recording_flag= 0
					if button == 'blue':
						print('playback here!')
						playback_sequence(poirot,captain,filename_list)
						time.sleep(.1)
						poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/playbackagain.png')
						captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/playbackagain.png')
						#button=xorscroll(poirot)
						button=redorblue(poirot,captain)
						if button=='red':
							print("playback again!")
						if button == 'blue':
							playback_flag=1
							print("Done playing back")



			if playback_flag==1:

				poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/doneexecuting.png')
				captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/doneexecuting.png')
				#DA_Move_To_Position(poirot,captain,[0, -1.18, 0, 2.2, 0, -.5, 3.3],[0, -1.18, 0, 2.2, 0,-.5,3.3],.2)
				DA_Move_To_Position(poirot,captain,[0, -1.18, 0, 2.6, 0, 0, 3.3],[0, -1.18, 0, 2.6, 0,0,3.3],.2)
				print('totally done!')

				#writefilename list to file
				fileName = ("/home/rachel/rawhide/rmh_code/TBD_logs/filenamelist.txt")

				with open(fileName, 'w') as f:
					f.write(','.join([str(x) for x in filename_list]))

				while not rospy.is_shutdown():

					time.sleep(0.1)



		prevwheel=curr_wheel
		prevx=curr_x
		prevr1=curr_red1
		prevr2=curr_red2
		prevb1=curr_blue1
		prevb2=curr_blue2
		t1=time.time()

		while (abs(time.time() - t1) <poirot.rate):

			time.sleep(0.001)

	# poirot.robot.setDIOCommand('port_sink_0', 0)
	# captain.robot.setDIOCommand('port_sink_0', 0)
	RRN.disconnect(poirot)
	RRN.disconnect(captain)

if __name__ == '__main__':
	print('running main')
	main()
