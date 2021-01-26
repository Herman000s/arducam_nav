import time
import serial
import json
import math
import struct
import numpy as np
import pisense
import cProfile, pstats

from datetime import datetime
#import sense_file
#from sense_hat import SenseHat

#sense = SenseHat()
#sense_obj = sense_file.OwnSenseHAT()
hat = pisense.SenseHAT()
port = serial.Serial("/dev/serial0", baudrate=115200, timeout=0)
execute = False

print("About to start program execution!")
#sense.show_message("Waking up!", scroll_speed=0.02)

while True:
	response = port.read(1).decode()
	print("Response from serial: ", response)

	if response == "s":
		print("Start received!")
#		sense.show_message("Starting!", scroll_speed=0.01)
		port.write(b'y')
#		port.close()
#		time.sleep(1)
		execute = True
#		port = serial.Serial("/dev/serial0", baudrate=115200, timeout=None)
		print("Beginning data transfer..")
		break
	elif response == "q":
		print("sw1 not pressed, terminating program..")
#		sense.show_message("Terminating..", scroll_speed=0.02)
		execute = False
		break
	else:
		print("Waiting for host..")
		time.sleep(0.1)

end_of_data = float('inf')
mergedlist = []
elapsed = 0.0
counter = 0
sum = 0.0
#sense_obj.read_raw_data()
get_day_stamp = time.time() - 86400

profiler = cProfile.Profile()
profiler.enable()
hat.imu._interval = 0.003
print("IMU interval: ", hat.imu._interval)
while execute:
	start = time.time()
	resp = port.read(1).decode()
	if resp == "d":
		vals = hat.imu.read()
		time_stamp = time.time() - get_day_stamp
		#mergedlist.append([vals.compass.x, vals.compass.y, vals.compass.z, vals.accel.x, vals.accel.y, vals.accel.z, vals.gyro.x, vals.gyro.y, vals.gyro.z])
		#list_to_bytes = np.array(mergedlist, dtype=np.float16).tobytes()
		#port.write(list_to_bytes)
		#mergedlist = []
		#bytes_list = bytearray(struct.pack("f",vals.compass.x))
		port.write(struct.pack("fffffffffff", time_stamp, vals.compass.x, vals.compass.y, vals.compass.z, vals.accel.x, vals.accel.y, vals.accel.z, vals.gyro.x, vals.gyro.y, vals.gyro.z, end_of_data))
		#port.write(struct.pack("f",vals.compass.y))
		#port.write(struct.pack("f",vals.compass.z))
		#port.write(struct.pack("f",end_of_data))
		counter += 1
		end = time.time()
		elapsed = end - start
		sum += elapsed
	elif resp == "e":
#		port.reset_input_buffer()
#		port.reset_output_buffer()
#		port.flush()
		if counter > 0:
			print("Round counter: ", counter)
			print("Average loop round: ", (sum / counter))
		print("Data collection has ended for now..")
#		sense.show_message("Paused!", scroll_speed=0.01)
		while True:
			resp = port.read(1).decode()
			if resp == "q":
				print("Terminating program execution..")
#				sense.show_message("Terminating..", scroll_speed=0.02)
				execute = False
				break
			elif resp == "c":
#				print("Continuing program execution!")
#				sense.show_message("Continuing!", scroll_speed=0.02)
				counter = 0
				sum = 0
				break
			else:
				continue
	else:
		time.sleep(0.0001)
#	elif resp == "d":
#		mergedlist.append(time.time())

#		sense_obj.read_raw_data()

		# Get magnetometer (values in microTeslas)
#		compass_raw = sense_obj.get_compass_raw()
#		print("Compass raw vals: ", compass_raw)
		#compass_raw_vals = list(compass_raw.values())
#		mergedlist.extend(compass_raw)
#		print("Compass raw values: ", compass_raw)

		#Get gyroscope (angle of axis in degrees)
#		gyro_raw = sense_obj.get_gyroscope_raw()
		#gyro_vals = list(gyro_raw.values())
#		mergedlist.extend(gyro_raw)
#		print("Gyro values: ", gyro_raw)

		# Get accelerometer (angle of axis in degrees)
#		accel_raw = sense_obj.get_accelerometer_raw()
		#accel_vals = list(accel_raw.values())
#		mergedlist.extend(accel_raw)

#		vals = hat.imu.read()
#		mergedlist.append([vals.compass.x, vals.compass.y, vals.compass.z, vals.accel.x, vals.accel.y, vals.accel.z, vals.gyro.x, vals.gyro.y, vals.gyro.z])

#		mergedlist.append(end_of_data)
#		print("Acceleration values: ", accel_raw)
#		print("Combined values list: ", mergedlist)

#		list_to_bytes = np.array(mergedlist, dtype=np.float16).tobytes()

#		port.write(list_to_bytes)
#		mergedlist = []
#		time.sleep(0.1)
#		counter += 1
#		end = time.time()
#		elapsed = end - start
#		sum += elapsed

#sense.show_message("Closing serial..", scroll_speed=0.02)
port.close()

profiler.disable()
stats = pstats.Stats(profiler).sort_stats('tottime')
stats.dump_stats('/home/pi/rpi_profile.prof')
stats.print_stats()
