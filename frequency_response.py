#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
import numpy as np
import scipy.fftpack
from scipy import signal
from scipy.signal import kaiserord, lfilter, firwin, freqz, argrelextrema


def get_phase_margin_lower_bound(y_arr):
	MT = np.amax(y_arr)
	PMlb = 2*np.arcsin(1/(2*MT))
	print('Phase margin lower bound: {}'.format(PMlb))
	return PMlb

def get_gain_margin_lower_bound(y_arr):
	MT = np.amax(y_arr)   # supremum of complementary sensitivity function
	GMlb = 1 + 1/MT
	print('Gain margin lower bound: {}'.format(GMlb))	    
	return GMlb, MT

def get_local_extrema(array):
	arr = np.array(array)
	maxima_locs = argrelextrema(arr, np.greater)
	minima_locs = argrelextrema(arr, np.less)

	peakidx = signal.find_peaks_cwt(arr, np.arange(100,200))
	print('peakidxs: {}'.format(peakidx))
	print(arr[peakidx])

	print(maxima_locs[0])
	return maxima_locs[0], minima_locs[0]

def get_phase(input_maxima_locs, output_maxima_locs, ref, pos, time):
	
	freq = []
	phi = []

	len_input = len(input_maxima_locs)
	len_output = len(output_maxima_locs)
	
	if len_input > len_output:
		max_array = output_maxima_locs
	else:
		max_array = input_maxima_locs	



	for i in range(len(max_array[:len(max_array)-1])):
		dt = float(time[input_maxima_locs[i+1]] - time[input_maxima_locs[i]])/1000000
		print('input maxima i :{}'.format(input_maxima_locs[i]))
		print('dt :{}'.format(1/dt))
		freq.append(1/dt)
		delay = float(time[input_maxima_locs[i]]/1000000 - time[output_maxima_locs[i]]/1000000)
		phi.append((delay/dt))


	print(len(time), len(ref))
	print(len(time), len(pos))

	# debug plot
	for i in range(len(input_maxima_locs)):
		plt.plot(time[input_maxima_locs[i]], ref[input_maxima_locs[i]],'-ob')
	plt.plot(time[0:len(ref)], ref,'b')
	for i in range(len(output_maxima_locs)):
		plt.plot(time[output_maxima_locs[i]], pos[output_maxima_locs[i]], '-or') 
	plt.plot(time, pos,'r')
	plt.show()	

	print(freq)
	print(phi)

	plt.plot(freq, phi)
	plt.title('Bode Phase Plot')
	plt.xlabel('Frequency (hz)')
	plt.ylabel('Phase (deg)')
	plt.show()

	return freq, phi
		

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
                #print(temp_time, time[t])
                unfucked_time.append(temp_time)

        return unfucked_time


def FIR_filter(x, cutoff_hz):
	sample_rate = 200.0
	nsamples = len(x)
	nyq_rate = sample_rate / 2.0
	width = 5.0 / nyq_rate	

	# The desired attenuation in the stop band, in dB.
	ripple_db = 60.0
	# Compute the order and Kaiser parameter for the FIR filter.
	N, beta = kaiserord(ripple_db, width)

	# The cutoff frequency of the filter.
	#cutoff_hz = 10.0

	# Use firwin with a Kaiser window to create a lowpass FIR filter.
	taps = firwin(N, cutoff_hz/nyq_rate, window=('kaiser', beta))

	# Use lfilter to filter x with the FIR filter.
	filtered_x = lfilter(taps, 1.0, x)		
	
	return filtered_x

def find_bandwidth(polyfit):
	#idx = (np.abs(array-0.7071)).argmin()
#	idx = array.index(min(np.abs(array-0.7071).argmin()))

	x = np.linspace(0.01,20,10000)
	y = polyfit(x)
	idx = (np.abs(y-0.707)).argmin()
	val = y[idx]
	fval = x[idx]
	get_phase_margin_lower_bound(y)
	get_gain_margin_lower_bound(y)

	return idx, val, fval

def fix_len_to_shortest(ref, pos):
        len_diff = len(ref) - len(pos)
	if len(ref) > len(pos):
                x = ref[:-len_diff]
                y = pos
        if len(pos) > len(ref):
                x = ref
                y = pos[:-len_diff]
	return x, y

def fix_len_to_static(nonstatic_arr, static_arr):
	len_nonstatic = len(nonstatic_arr)
	len_static = len(static_arr)
	len_diff = len(nonstatic_arr) - len(static_arr)
	if len_diff > 0:
		nonstatic_arr = nonstatic_arr[:-len_diff]
	else:
		for i in range(len_diff):
			nonstatic_arr.append(nonstatic_arr[len_nonstatic])

	return nonstatic_arr, static_arr

def bode(time, pos, ref):
	dt = 1/200   # sampling rate (200 hz)

	# make sure input and output lengths are the same

	x,y = fix_len_to_shortest(ref, pos)
	time, y = fix_len_to_static(time, y)	

	n = len(y)
	#print('y len: {}'.format(len(y)))
	#print('x len: {}'.format(len(x)))
	# frequency domain system output

	y = FIR_filter(y, 10)
	x = FIR_filter(x, 10)

	Y = np.fft.fft(y)
	Yfreq = np.fft.fftfreq(len(y))
	np.fft.fftshift(Yfreq)

	# Number of samplepoints
	N = len(y)
	# sampling rate
	Fs = 200.0
	# sample spacing
	Ts = 1.0 / Fs	

	T = N/Fs
	k = scipy.arange(N)
	frq = k/T
	frq = frq[range(N/2)]

	# FREQUENCY RESOLUTION:
	# length of signal = Tmax
	# number of points = n
	# dt = Tmax / n
	# max freq = 1 / dt
	# dF = 1 / Tmax
	# therefore, the longer the recording, the better the frequency resolution

	Y = scipy.fft(y)/n # fft computing and normalization
 	Y = Y[range(n/2)]
	X = scipy.fft(x)/n
	X = X[range(n/2)] 

	#X = FIR_filter(X, 90)
	#Y = FIR_filter(Y, 90)

	plt.plot(frq,abs(Y),'-or') # plotting the spectrum
	plt.plot(frq, abs(X), '-ob')
	plt.xlabel('Freq (Hz)')
	plt.ylabel('|Y(freq)|')
	plt.xlim([0,10])
	plt.title('Input and Output Frequency Content')
	plt.xlabel('Frequency (hz)')
	plt.ylabel('Magnitude')
	plt.show()

        H = np.divide(Y, X)
        #H = FIR_filter(H,40)
	#H_trimmed = np.array([])	
	#frq_trimmed = np.array([])
	H_trimmed = []
	frq_trimmed = []


	for i in range(len(frq)):
		#print(frq[i])
		if frq[i] <= 5.0:
			#np.append(frq_trimmed, [frq[i]])
			#np.append(H_trimmed, [H[i]])
			frq_trimmed.append(frq[i])
			H_trimmed.append(H[i])

	#print(frq_trimmed)
	z = np.polyfit(frq_trimmed, np.abs(H_trimmed), 7)
	poly = np.poly1d(z)
	#print(poly)
	idx, val, fval = find_bandwidth(poly)
	#print(idx, val, fval)
	bandwidth = fval
	print('Bandwidth: {}'.format(bandwidth))

	plt.figure(1)
        plt.loglog(frq, np.abs(H), '-ob')
	plt.xlim([0,10])
	plt.ylim([-100,100])
	plt.axhline(y=0.707, color='c')
	plt.grid()
        plt.title('Bode Magnitude Plot')
	plt.xlabel('Frequency (Hz)')
	plt.ylabel('Magnitude')
	plt.plot(frq_trimmed, poly(frq_trimmed), 'r')
	plt.text(0.1, 10, 'Bandwidth = {}'.format(bandwidth))
	plt.show()



	########### BODE PHASE PLOT
	input_maxima, input_minima = get_local_extrema(ref)
	print('len input maxima: {}'.format(len(input_maxima)))
	print(input_maxima)
	output_maxima, output_minima = get_local_extrema(pos)
	get_phase(input_maxima, output_maxima,x, y, time)

	# in order to get the phase plot I will have to collect data at discrete
	# frequencies, and for each calculate the phase shift relative to the reference
	# 
	##### LETS DO IT THE ABOVE WAY AND THEN TAKE RATIOS ####
	#### LOOKS LIKE I NEED TO MAKE A FILTER FIRST ######

def time_response(pos, ref, time):
	hz = 200
	x1 = np.linspace(0, len(pos), len(pos))/hz
	x2 = np.linspace(0, len(ref), len(ref))/hz
	plt.plot(x1, pos)
	plt.plot(x2, ref, 'r')
	plt.title('Time Domain Frequency Response')
	plt.xlabel('Time (s)')
	plt.ylabel('Magnitude (ticks)')
	plt.show() 

if __name__ == "__main__":

        bag_name = 'pan_freq_resp_9_2_0_01_2_075_rad.bag'
#	bag_name = 'joint_states
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

        unfucked_time = unloop_time(time)

	bode(unfucked_time, pos, ref)
	
	pos = FIR_filter(pos,20)
	ref = FIR_filter(ref,20)
	#time_response(pos,ref, unfucked_time)
