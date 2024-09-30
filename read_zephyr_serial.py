
import serial
import pandas as pd
import matplotlib.pyplot as plt
from numpy.fft import fft, ifft
from scipy.fftpack import fftfreq
import numpy as np
import math
from time import perf_counter
from datetime import datetime
import csv
torque_samples = 3998


def fft_filter(signal):
    # sampling rate
    sr = 1/1000
    # sampling interval
    t = np.arange(0,10.0,sr)

    x = signal

    # FFT the signal
    sig_fft = fft(x)
    # copy the FFT results
    sig_fft_filtered = sig_fft.copy()

    # obtain the frequencies using scipy function
    freq = fftfreq(len(x), d=1./1000)

    # define the cut-off frequency
    cut_off_low =  0.0
    cut_off_high = 1.0

    sig_fft_filtered[np.abs(freq) > cut_off_high] = 0
    sig_fft_filtered[np.abs(freq) < cut_off_low] = 0


    # get the filtered signal in time domain
    filtered = ifft(sig_fft_filtered)

    return filtered

def fft_filter_set(signal, low_cutoff, high_cutoff, sampling):

    # sampling rate
    sr = sampling
    scale = 1
    fft_threshold_scale= 99
    # sampling interval
    t = np.arange(0,10.00,sr)


    x = signal
    # FFT the signal
    sig_fft = fft(x)
    # copy the FFT results
    sig_fft_filtered = sig_fft.copy()

    # obtain the frequencies using scipy function
    freq = fftfreq(len(x), d=1./1000)

    sig_fft_filtered[np.abs(freq) > high_cutoff] = 0
    sig_fft_filtered[np.abs(freq) < low_cutoff] = 0

    ## For magnitude threshold filtering
    # percentile_threshold = np.percentile(sig_fft_filtered, fft_threshold_scale)
    # sig_fft_filtered = np.where(sig_fft_filtered >= percentile_threshold, sig_fft_filtered, 0)

    # get the filtered signal in time domain
    filtered = ifft(sig_fft_filtered) 

    return filtered


def calc_accel(pos_data, sampling):
    dt = sampling
    t = np.arange(0,10.0,dt)
    
    velocity_data = np.diff(pos_data) / dt

    # Calculate acceleration (second derivative of position)
    acceleration_data = np.diff(velocity_data) / dt
    filtered_accel = fft_filter_set(acceleration_data,0.0, 1.0, dt)
    # return acceleration_data
    return filtered_accel

def read_accel(serial):
        data = []
        ref = []
        t= [ ]
        data_received = 0
        error = 0
        # sampling_rate = 1/750
        # max = 5000
        sampling_rate = 1/1000
        max = 10000
        
        while data_received < max:
            dat_str = serial.read_until(b'\n'); 
            # print(dat_str)
            dat_f = list(map(float,dat_str.split()))
            ref.append(dat_f[0] * (2*math.pi))  ## Ref acceleration is in turns/s/s, this converts it to rad/s/s
            data.append(dat_f[1]*(math.pi/180))  ## output position in deg comes in, here it is converted to rad and then used for accel calculation
            
            error = error + abs((dat_f[0] * (2*math.pi))-(dat_f[1]*(math.pi/180)))
            # ref.append(dat_f[0])   ## Ref acceleration is in turns/s/s, this converts it to rad/s/s
            # data.append(dat_f[1])  ## output position in deg comes in, here it is converted to rad and then used for accel calculation
            t.append(data_received*sampling_rate)
            print("Getting data\n", data_received,dat_f[0] * (2*math.pi), dat_f[1]*(math.pi/180) )

            data_received+=1
        

        average_error = error/max

        filtered_data = fft_filter_set(data,0.0, 1.2, sampling_rate) 
        filtered_acceleration = calc_accel(filtered_data, sampling_rate)

        accel = calc_accel(data, sampling_rate)
        for i in range(len(t[:-2])):
             print(ref[i],accel[i],filtered_acceleration[i])

        plt.plot(t,ref,'r-', t[:-2],filtered_acceleration,'b-')
        # plt.plot(t,data,'k-', t[:-2],accel,'g-') # unfiltered accelerations

        plt.ylim(-10, 10)

        # plt.plot(t,ref,'r-', t,data,'b-')
        plt.ylabel('Acceleration [rad/s/s]')
        plt.xlabel('Time [sec]')
        plt.title("Input vs Output Acceleration, 05ms Walking Profile | error = {:.4f}".format(average_error))
        plt.legend(['Input Acceleration', 'Output Acceleration']) 


        plt.show()

def read_radians(serial):
        data = []
        ref = []
        t= [ ]
        data_received = 0
        error = 0
        # sampling_rate = 1/750
        # max = 5000
        sampling_rate = 1/100
        max = 1000
        
        while data_received < max:
            dat_str = serial.read_until(b'\n'); 
            # print(dat_str)
            dat_f = list(map(float,dat_str.split()))
            ref.append(dat_f[0]*(math.pi/180))  
            data.append(dat_f[1]*(math.pi/180)) 
            t.append(data_received*sampling_rate)
            print("Getting data\n", data_received,dat_f[0] * (180/math.pi), dat_f[1]*(180/math.pi) )
            data_received+=1
        
        # filtered_data = fft_filter_set(data,0.0, 1.2, sampling_rate) 

        # for i in range(len(t[:-2])):
        #      print(ref[i],accel[i],filtered_data[i])

        plt.plot(t,ref,'r-', t,data,'b-')
        # plt.plot(t,data,'k-', t[:-2],accel,'g-') # unfiltered accelerations

        plt.ylim(-0.010, 0.010)

        # plt.plot(t,ref,'r-', t,data,'b-')
        plt.ylabel('Angle [rad]')
        plt.xlabel('Time [sec]')
        plt.title("Input vs Output Angle (sin)")
        plt.legend(['Input Angle', 'Output Angle']) 
 

        plt.show()

def read_current(serial):
        data = []
        ref = []
        t= [ ]
        data_received = 0
        error = 0
        # sampling_rate = 1/750
        # max = 5000
        sampling_rate = 1/1000
        max = 10000
        
        while data_received < max:
            dat_str = serial.read_until(b'\n'); 
            # Collect current data 
            dat_f = list(map(float,dat_str[1:].split()))
            ref.append(dat_f[0])
            data.append(dat_f[1])
            t.append(data_received * sampling_rate)
            print("Getting current data\n", data_received,dat_f[0] , dat_f[1])
            data_received+=1
        

        plt.plot(t,ref,'r-', t,data,'b-')
        # plt.plot(t,data,'k-', t[:-2],accel,'g-') # unfiltered accelerations

        # plt.ylim(-0.010, 0.010)

        # plt.plot(t,ref,'r-', t,data,'b-')
        plt.ylabel('Current [A]')
        plt.xlabel('Time [sec]')
        plt.title("Input vs Output Current (step)")
        plt.legend(['Input Angle', 'Output Angle']) 
        plt.show()

def read_both(serial):
        data_current = []
        ref_current = []        
        data_angle = []
        ref_angle = []
        t_current = []
        t_angle = []
        data_received_current = 0
        data_received_angle = 0
        sampling_rate_current = 1/1000
        max_current = 60000
        sampling_rate_angle = 1/100
        max_angle = 6000
        # sampling_rate_angle = 1/500
        # max_angle = 1000
        
        while data_received_current < max_current or data_received_angle < max_angle:
            dat_str = serial.read_until(b'\n').decode('utf-8').strip() 
            if dat_str.startswith('s'):
                 continue # do nothing
            elif dat_str.startswith('c') and data_received_current < max_current:
                # Collect current data 
                dat_f = list(map(float,dat_str[1:].split()))
                ref_current.append(dat_f[0])
                data_current.append(dat_f[1])
                t_current.append(data_received_current * sampling_rate_current)
                print("Getting current data\n", data_received_current,dat_f[0] , dat_f[1])
                data_received_current+=1

            elif dat_str.startswith('a') and data_received_angle < max_angle:
                # Collect angle data    
                dat_f = list(map(float,dat_str[1:].split()))
                ref_angle.append(dat_f[0])
                data_angle.append(dat_f[1])
                t_angle.append(data_received_angle * sampling_rate_angle)
                print("Getting angle data\n", data_received_angle,dat_f[0], dat_f[1])
                data_received_angle+=1

        plt.subplot(1,2,1)
        plt.plot(t_current,ref_current,'r-', t_current,data_current,'b-')
        plt.ylabel('Current [A]')
        plt.xlabel('Time [sec]')
        plt.title("Input vs Output Current")
        plt.legend(['Input Current', 'Output Current']) 
        # plt.ylim(-0.010,0.010)

        plt.subplot(1,2,2)
        plt.plot(t_angle,ref_angle, 'r-',t_angle, data_angle,'b-')
        plt.ylabel('Angle [rad]')
        plt.xlabel('Time [sec]')
        plt.title("Input vs Output Angle")
        plt.legend(['Input Angle', 'Output Angle']) 
        # plt.ylim(-0.010, 0.010)
        plt.show()
     
        plt.show()


def read_IMU(serial):
        imu = []
        steps = []
        time_between_steps = []
        t1= [ ]
        t2= [ ]
        sampling_rate = 1/610
        data_received1 = 0
        data_received2= 0
        time1 = perf_counter()
   
        while data_received2 < 6100:
            dat_str = serial.read_until(b'\n').strip().decode('utf-8')  # get the data as a string, ints separated by spaces
            if dat_str.startswith('s') :
                # Collect step data 
                dat_f = list(map(float,dat_str[1:].split()))
                time_between_steps.append(dat_f[0])
                steps.append((data_received2 + 1) * sampling_rate) # add time of step
                print("Getting step data\n")
                

            elif dat_str.startswith('m'):
                # Collect imu data    
                dat_f = list(map(float,dat_str[1:].split()))
                imu.append(dat_f[0])
                print("Getting imu data\n",dat_f[0])
                data_received2+=1
                t2.append(data_received2 * sampling_rate)
        time2 = perf_counter()
        average = np.average(time_between_steps)
        speed = 3.6




        # plt.subplot(1,2,1)
        # plt.plot(t1,cor,'r-')
        # plt.ylabel('Correlation [A]')
        # plt.xlabel('Time [sec]')
        # plt.title("Correlation from FIR Filter (Step Detection)")

        # plt.subplot(1,2,2)
        plt.plot(t2,imu,'b-')
        for x in steps:
            plt.axvline(x=x, color='r', linestyle='--')
        plt.ylabel('Acceleration [m/s/s]')
        plt.xlabel('Time [sec]')
        plt.title(f"IMU Data | Treadmill Speed: {speed:.1f} mph Avg Cadence: {average:.3f}step/sec")
        plt.show()
     


def main():
    ser = serial.Serial("/dev/tty.usbmodem101", timeout=3.0)

    has_quit = False
    # menu loop
    
    while not has_quit:
        
        # try: 
            # read_IMU(ser)
            # read_accel(ser)
            # read_radians(ser)
            # read_current(ser)
            read_both(ser)
         
        #     break
        # except:
        #     print("error open serial port")
        #     exit()



main()