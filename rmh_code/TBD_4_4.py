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
from TBD_Robot_Class import RobotArm_TBD

def checkcuffbutton(robot1,robot2):

	if robot1.robot.CuffButton==1 or robot2.robot.CuffButton==1:

		robot1.SetControlMode(0)
		robot2.SetControlMode(0)
		robot1.robot.setPositionModeSpeed(.2)
		robot2.robot.setPositionModeSpeed(.2)
		#print("cuffbutton")
	else:
		robot1.Set_Position(robot1.get_current_joint_positions(),10)
		robot2.Set_Position(robot2.get_current_joint_positions(),10)
		#print("nocuffbutton")

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

def set_disp_image(robot1,robot2,imagename):
	#'/home/rachel/rawhide/rmh_code/TBD_images/startrecording.png'
	robot1.robot.setDisplayImage(imagename)
	robot2.robot.setDisplayImage(imagename)

def scan_wire():
    wireID= input('give me a wire id ')
    return wireID

def grasp_wire():
	pass
def filecheck(filename):
	try:

		open(filename,"r")
		print('fc file exists')
		return 1
	except:
		print('fc file does not exist')
		return 0


def get_playback_filename_list(filenamelisttxt):

	with open(filenamelisttxt ,'r') as f:
		lines = f.readlines()
	filename_list=numpy.array(lines[0].rstrip().split(','))
	return filename_list


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
	print('playback: ', filename)
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

	robot1.robot.setJointCommand('right',robot1.get_current_joint_positions())
	robot2.robot.setJointCommand('right',robot2.get_current_joint_positions())


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

def move_from_wire_ret_to_neut(robot1,robot2):
	# move_to_wire_ready_pos(robot1,robot2)
	# #xorscroll(robot1)
	# robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/grabwire.png')
	# robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/grabwire.png')
	#redorblue(robot1,robot2)
	DA_Move_To_Position(robot1,robot2,[0, -2, 0, 0, 0, 2.5, 3.3],[0, -2, 0, 0, 0,2.5,3.3],.2)
	DA_Move_To_Position(robot1,robot2,[0, -1.18, 0, 2.6, 0, 0, 3.3],[0, -1.18, 0, 2.6, 0,0,3.3],.2)


def record_sequence(robot1,robot2,wireid):
	xflag=0
	scrollflag=0
	fileiter=0
	graspiter=0
	filename_list=[]
	repeatflag=1
	noinput=1
	button=0
	while repeatflag==1 :
		robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/startrecording.png')
		robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/startrecording.png')
		print('to start recording sequence, press any Button')
		redorblue(robot1,robot2)
		# while robot1.robot.PB1==0 and robot1.robot.PB3==0:
		# 	time.sleep(.01)
		# robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/stoprecord.png')
		# robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/stoprecord.png')
		fileName = ("/home/rachel/rawhide/rmh_code/TBD_logs/logdemo%d_%d.txt" % (wireid,fileiter))

		record(robot1,robot2,fileName)

		robot1.SetControlMode(0)
		robot1.SetControlMode(0)

		filename_list.append(fileName)
		fileiter=fileiter+1

		robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/recordanother.png')
		robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/recordanother.png')
		print( 'record more? red yes, blue no')
		time.sleep(.5)
		button=redorblue(robot1,robot2)
		time.sleep(.5)
		if button=='red':
			robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/regrasp_or_record.png')
			robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/regrasp_or_record.png')
			print('regrasp wire or record another pose? red regrasp, blue record another')
			time.sleep(.5)
			button=redorblue(robot1,robot2)
			time.sleep(.5)
			if button=='red':
				print('regrasp')
				fileName = ("/home/rachel/rawhide/rmh_code/TBD_logs/loggrasp%d_%d.txt" % (wireid,graspiter))
				filename_list.append(fileName)
				#grasp_wire()
				regrasp_record(robot1,robot2,fileName)
				graspiter=graspiter+1
				robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/donegrasping.png')
				robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/donegrasping.png')
				print('done grasping,record another? red yes, blue no')
				button=redorblue(robot1,robot2)
				if button =='red':
					pass
				elif button =='blue':
					repeatflag=0
			elif button =='blue':
				print('record another')

		elif button=='blue':
			repeatflag=0
			robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/donerecordingsequence.png')
			robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/donerecordingsequence.png')
			print('DoneRecordingSequence')
			time.sleep(.5)


		robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/recordanother.png')
		robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/recordanother.png')


	fileName = ("/home/rachel/rawhide/rmh_code/TBD_logs/filenamelist_%d.txt" % (wireid))


	with open(fileName, 'w') as f:
		f.write(','.join([str(x) for x in filename_list]))

	return fileName

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
	robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/stoprecord.png')
	robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/stoprecord.png')


	grippos=[robot1.robot.gripperPos,robot2.robot.gripperPos]
	angles_right1=numpy.concatenate((robot1.get_current_joint_positions(), robot2.get_current_joint_positions()))
	angles_right_start=numpy.concatenate((angles_right1,grippos))
	robot1.SetControlMode(0)
	robot2.SetControlMode(0)
	#pdb.set_trace()
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
				# robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/stoprecord.png')
				# robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/stoprecord.png')
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

					grippos=[robot1.robot.gripperPos,robot2.robot.gripperPos]
					angles_right1=numpy.concatenate((robot1.get_current_joint_positions(), robot2.get_current_joint_positions()))
					angles_right_start=numpy.concatenate((angles_right1,grippos))
					f.write("%f," % (time_stamp(start_time),))
					f.write(','.join([str(x) for x in angles_right]) +  temp_str)
					robot1.doneflag=True

				time.sleep(recrate)



			robot1.doneflag=False
			print("done!")


		# print('setting control mode to 0')
			robot1.SetControlMode(0)
			robot2.SetControlMode(0)
			time.sleep(.1)
	else:
		print("error with filename")

def regrasp_record(robot1,robot2,filename):
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
	robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/grabwire.png')
	robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/grabwire.png')
	robot1.SetControlMode(0)
	robot2.SetControlMode(0)


	grippos=[robot1.robot.gripperPos,robot2.robot.gripperPos]
	angles_right1=numpy.concatenate((robot1.get_current_joint_positions(), robot2.get_current_joint_positions()))
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
				# robot1.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/stoprecord.png')
				# robot2.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/stoprecord.png')
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

					grippos=[robot1.robot.gripperPos,robot2.robot.gripperPos]
					angles_right1=numpy.concatenate((robot1.get_current_joint_positions(), robot2.get_current_joint_positions()))
					angles_right_start=numpy.concatenate((angles_right1,grippos))
					f.write("%f," % (time_stamp(start_time),))
					f.write(','.join([str(x) for x in angles_right]) +  temp_str)
					robot1.doneflag=True

				time.sleep(recrate)



			robot1.doneflag=False
			print("done!")


		# print('setting control mode to 0')
			robot1.SetControlMode(0)
			robot2.SetControlMode(0)
			time.sleep(.1)
	else:
		print("error with filename")


def main():

	#NEUTRAL POSITION {'right_j6': 3.3119794921875, 'right_j5': 0.566505859375, 'right_j4': 0.0049091796875, 'right_j3': 2.177044921875, 'right_j2': -0.00241796875, 'right_j1': -1.178189453125, 'right_j0': 0.001283203125}
	print("Lets get it on")
	poirot=RobotArm_TBD('tcp://localhost:40369/SawyerRMHServer/Sawyer')
	captain=RobotArm_TBD('tcp://localhost:33363/SawyerRMHServer/Sawyer')
	poirot.robot.setPositionModeSpeed(.2)
	captain.robot.setPositionModeSpeed(.2)
	poirot.robot.setGRIPpos(0)
	captain.robot.setGRIPpos(0)
	poirot.robot.setGRIPpos(250)
	captain.robot.setGRIPpos(250)
	poirot.robot.setGRIPpos(0)
	captain.robot.setGRIPpos(0)
	#DA_Move_To_Position(poirot,captain,[0, -1.18, 0, 2.6, 0, 0, 3.3],[0, -1.18, 0, 2.6, 0,0,3.3],.2)
	wire_ready_pos= [3, -1.18, 0, 2.6, 0, 0, 3.3]
	poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/startingup.png')
	captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/startingup.png')
	DA_Move_To_Position(poirot,captain,[0, -1.18, 0, 2.6, 0, 0, 3.3],[0, -1.18, 0, 2.6, 0,0,3.3],.2)
	quitflag=0

	while quitflag==0:
		poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/wirestation.png')
		captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/wirestation.png')
		move_to_wire_ready_pos(poirot,captain)
		#move_from_wire_ret_to_neut(poirot,captain)
		print('scan wire')
		poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/wire_id.png')
		captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/wire_id.png')
		wireid=scan_wire()
		print('wire id is: ',wireid)
		fileNameList = ("/home/rachel/rawhide/rmh_code/TBD_logs/filenamelist_%d.txt" % (wireid))
		file_exist=filecheck(fileNameList)
		poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/grabwire.png')
		captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/grabwire.png')
		print('grab wire with sawyer buttons, press red or blue to continue')
		redorblue(poirot,captain)
		if file_exist==1:
			print('file exists')
			poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/fileexists.png')
			captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/fileexists.png')
			time.sleep(1)
			filename_list= get_playback_filename_list(fileNameList)
			#print('playback? red yes, blue no')
			playback_flag=0
			while playback_flag==0:
				poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/playback_ry_bn.png')
				captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/playback_ry_bn.png')
				print('playback? red yes, blue no')
				time.sleep(.5)
				button=redorblue(poirot,captain)
				if button=='red': #playback
					print('playback sequence func')
					move_from_wire_ret_to_neut(poirot,captain)
					playback_sequence(poirot,captain,filename_list)
					poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/getnewwire.png')
					captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/getnewwire.png')
					print('get new wire?')
					time.sleep(.5)
					button=redorblue(poirot,captain)
					if button=='red':  #get new wire
						playback_flag=1
					elif button =='blue': #do not get new wire
						poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/quit.png')
						captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/quit.png')
						print('quit? red yes, blue no')
						time.sleep(.5)
						button=redorblue(poirot,captain)
						if button=='red':  #quit
							print('quitting')
							poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/quitting.png')
							captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/quitting.png')
							playback_flag=1
							quitflag=1

						elif button =='blue': #do not quit
							print('blue do not quit')
							time.sleep(.5)
							print(playback_flag,quitflag)


					#playback_flag=1
				elif button =='blue': #do not playback
					poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/getnewwire.png')
					captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/getnewwire.png')
					print('get new wire?')
					time.sleep(.5)
					button=redorblue(poirot,captain)
					if button=='red':  #get new wire
						playback_flag=1
					elif button =='blue': #do not get new wire
						poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/quit.png')
						captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/quit.png')
						print('quit? red yes, blue no')
						time.sleep(.5)
						button=redorblue(poirot,captain)
						if button=='red':  #quit
							poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/quitting.png')
							captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/quitting.png')
							print('quitting')
							playback_flag=1
							quitflag=1
						elif button =='blue': #do not quit
							print('blue do not quit')
							time.sleep(.5)
							print(playback_flag,quitflag)
		if file_exist==0:
			poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/file_doesnotexist.png')
			captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/file_doesnotexist.png')
			print('file does not exist')
			time.sleep(0.5)
			poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/recordnewfile.png')
			captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/recordnewfile.png')
			print('record new file? red yes, blue no')
			time.sleep(.5)
			button=redorblue(poirot,captain)
			if button=='red':  #record new file
				poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/recordsequence.png')
				captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/recordsequence.png')
				print('record sequence')
				time.sleep(.5)
				move_from_wire_ret_to_neut(poirot,captain)
				#playback_sequence(poirot,captain,filename_list)
				#DA_Move_To_Position(poirot,captain,[0, -1.18, 0, 2.6, 0, 0, 3.3],[0, -1.18, 0, 2.6, 0,0,3.3],.2)
				record_sequence(poirot,captain,wireid)
				#DA_Move_To_Position(poirot,captain,[0, -1.18, 0, 2.6, 0, 0, 3.3],[0, -1.18, 0, 2.6, 0,0,3.3],.2)
				poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/getnewwire.png')
				captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/getnewwire.png')
				print('Get Next Wire? Red yes, Blue No')
				time.sleep(.5)
				button=redorblue(poirot,captain)
				if button=='red':
					pass
				elif button =='blue':
					poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/quitting.png')
					captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/quitting.png')
					button=redorblue(poirot,captain)
					time.sleep(.5)
					print('quitting')

					quitflag=1


			elif button =='blue': #DO NOT record new file
				pass
				# print('get new wire? red yes, blue no')
				# button=redorblue(poirot,captain)
				# if button=='red':
				# 	playback_flag=1
				# elif button =='blue':















	DA_Move_To_Position(poirot,captain,[0, -1.18, 0, 2.6, 0, 0, 3.3],[0, -1.18, 0, 2.6, 0,0,3.3],.2)
	print('QUIT FINAL')
	poirot.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/alldone.png')
	captain.robot.setDisplayImage('/home/rachel/rawhide/rmh_code/TBD_images/alldone.png')
	#RRN.disconnect(poirot)
	#RRN.disconnect(captain)


if __name__ == '__main__':
	print('running main')
	main()
