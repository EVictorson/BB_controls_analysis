#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
import numpy as np

def calc_rise_time(data, time, ref):
	y = data
	high_val = max(ref)
	low_val = min(ref)
	t0 =[]
	t10 = []
	t90 = []
	ten_percent = False
	rise_time = []
	start = False
	ninety_percent = False

	for i in range(len(data[:-1])):
		if((data[i] <= low_val +3 and data[i] >= low_val-3) and data[i+1] > low_val+3 and start == False):
			t0.append(time[i])
			start = True

		if(data[i] >= 0.1*high_val and start == True and ten_percent == False):
			t10.append(time[i])
			ten_percent = True

		if(data[i] >= 0.9*high_val and start == True and ten_percent == True):
			t90.append(time[i])
			start = False
			ten_percent = False
			ninety_percent = True


	for i in range(len(t10)):
		if((i < len(t90)) and (i < len(t10))):
			rise_time.append(t90[i] - t10[i])
	


	return rise_time, t10, t90	 


def calc_settling_time(data, time, ref):
	high_val = max(ref)
	low_val = min(ref)
	y = data	

	for i in y:
                if(y[i] == low_val and y[i+1] > low_val):
                        t0.append(t[i])
                        start = true
 

def calc_steady_state_error(data, time, ref):
	high_val = max(ref)
	low_val = min(ref)
	y = data
	tend = []
	ess = []

	for i in y[:-1]:
		if(ref[i] == high_val and ref[i+1] < high_val):
			tend.append(time[i])

	for t in tend:
		error = y[t] - ref[t]
		ess.append(error)

	return ess
	
	

def calc_overshoot(data, time, ref):
	high_val = max(ref)
	y = data
	overshoot = []
	t0 = []
	tend = []

	for i in y[:-1]:
                if(ref[i] == low_val  and ref[i+1] > low_val):
                        t0.append(time[i])
		if(ref[i] == high_val and ref[i+1] < high_val):
			tend.append(time[i])
	for i in t0:
		maxima = max(y[t0[i]:tend[i]])
		overshoot.append(maxima)

	return overshoot
			
def unloop_time(time):
	overflow_at = 1999969 # overflow at this many us
	us_per_tick = 30.5175
	num_overflows = -1
	unfucked_time = []
	unfucked_time.append(time[0])

	for t in range(len(time[1::])):
		if time[t-1] > time[t]:
			num_overflows += 1
		temp_time = (num_overflows * overflow_at) + time[t]
		print(temp_time, time[t])
		unfucked_time.append(temp_time)
			
	return unfucked_time
		


if __name__ == "__main__":


	bag_name = 'eyes_step_resp_p2_gains.bag'
	ref = []
	vel = []
	pos = []
	time = [] 
	i = 0
	unfucked_time = []

	bag = rosbag.Bag(bag_name)
        for topic, msg, t in bag.read_messages():

		if topic == '/pid_tuning/act_vel':
			vel.append(msg.data)
		if topic == '/pid_tuning/act_time':
			time.append(msg.data)
		if topic == '/pid_tuning/act_pos':
			pos.append(msg.data)
		if topic == '/pid_tuning/des':
			ref.append(msg.data)
#	print('time: {}'.format(time))

	unfucked_time = unloop_time(time) 
#	print('unfucked time: {}'.format(unfucked_time))

	rise_time,t10, t90 = calc_rise_time(pos, unfucked_time, ref)
	mean_rise_time = np.mean(rise_time)

	print('rise time: {}'.format(rise_time))
	print('mean rise time: {}'.format(mean_rise_time))
	print(len(unfucked_time))
	print(len(pos))

        len_diff = len(unfucked_time) - len(pos)

        # make sure input and output lengths are the same
        if len(unfucked_time) > len(pos):
                x = unfucked_time[:-len_diff]
                y = pos
        if len(pos) > len(unfucked_time):
                x = unfucked_time
                y = pos[:-len_diff]

	plt.plot(x, y, 'r')
	for t in t10:
		plt.axvline(x = t)
	for t in t90:
		plt.axvline(x = t) 

	plt.show()
#	print('len unfucked time: {}'.format(len(unfucked_time)))
#	print('len time: {}'.format(len(time)))
#	print('len vel: {}'.format(len(vel)))
#	print('len pos: {}'.format(len(pos)))
#	print('len ref: {}'.format(len(ref)))
#	print(unfucked_time)

	#x = /pid_tuning/act_time
	#y = /pid_tuning/act_vel
	#ref = /pid_tuning/des

#	low_val = min(ref)
#	high_val = max(ref)

#	overshoot = calc_overshoot(y,x,ref)
#	mean_overshoot = np.mean(overshoot)

#	ess = calc_steady_state_error(y,x,ref)
#	mean_ess = np.mean(ess)
#
#	print("Rise time: {}").format(rise_time)
#	print("Mean rise time: {}").format(mean_rise_time)
#	print("Overshoot: {}").format(overshoot)
#	print("Mean overshoot: {}").format(mean_overshoot)
#	print("Steady state error: {}").format(ess)
#	print("Mean steady state error: {}").format(mean_ess)

#	plt.plot(x,y, 'r')
#	plt.plot(x,ref, 'b')
#	plt.xlabel('Time (s)')	
#	plt.ylabel('Counts or counts/sec')
#	plt.title('Step response')
#	plt.show()
