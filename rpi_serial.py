import serial
import json
import numpy as np
import globals
import logging
import time
import sys
import struct
import cProfile, pstats
from datetime import datetime

# Serial abbreviations -> s = start (checking if RPi has woken up), y = yes (indication that RPi has received start command), e = end (ending data collection for now), d = done (send more data) 

profiler = cProfile.Profile()

class Rpicomms:

	def setup(self):
		globals.port = serial.Serial("/dev/ttyPS1", baudrate=115200, timeout=5.0)
		self.local_round_counter = 0


	def start_comms(self):
		while True:
			globals.port.write(b's')
			resp = globals.port.read(1).decode()

			if resp == "y":
				print("Communication with rpi has started!")
				break
			else:
				print("Waiting for rpi to wake up..")
#		self.port.close()


	def collect_data(self):
		counter = 0
		resp = None
		rpi_vals = np.zeros(shape=(1,11))

#		print("port in wait in thread: ", globals.port.in_waiting)
#		print("port out wait in thread: ", globals.port.out_waiting)
		globals.port.reset_input_buffer()
		globals.port.reset_output_buffer()
		globals.port.flush()

#		logging.info('Starting RPi data collection!')
		ts = datetime.now()
		#with open(globals.rpi_file_path, 'a') as f:
		globals.rpidata.update({"Timestamp round%d"%globals.round_counter: [], "RPi data round%d"%globals.round_counter: []})
		globals.rpidata["Timestamp round%d"%globals.round_counter].append(str(ts))
		#	json.dump(globals.rpidata, f, indent=4)
		
		globals.rpi_val_list = []

		profiler.enable()
		while globals.running:
			globals.port.write(b'd')
#			i = 0
#			while i < 10: 
			byte_resp = globals.port.read(44)

			if len(byte_resp) > 0:
				if globals.rpi_lock == False:
					globals.rpi_val_list.append(byte_resp)
					#num_of_floats = "%df" % (len(byte_resp) // 4)
					#globals.rpi_val_list.append(struct.unpack(num_of_floats, byte_resp))
					#resp = struct.unpack(num_of_floats, byte_resp)
				else:
					continue
			
			#resp = struct.unpack("f", byte_resp)
				#rpi_vals = np.frombuffer(resp, dtype=np.float16)			
			#globals.rpidata["RPi data round%d"%globals.round_counter].append(resp)
				#globals.rpidata["RPi data round%d"%globals.round_counter].append(str(rpi_vals))
			#	i += 1 

#				if len(rpi_vals) > 0:
#					with open(globals.rpi_file_path, 'w') as file:
#						globals.rpidata["RPi data round%d"%globals.round_counter].append({"RPi time": str(rpi_vals[0])})
#						globals.rpidata["RPi data round%d"%globals.round_counter].append({"Magnetometer (in microTeslas)": {"x": str(rpi_vals[2]), "y": str(rpi_vals[3]), "z": str(rpi_vals[1])}})
#						globals.rpidata["RPi data round%d"%globals.round_counter].append({"Gyro (in rad/s)": {"x": str(rpi_vals[5]), "y": str(rpi_vals[6]), "z": str(rpi_vals[4])}})
#						globals.rpidata["RPi data round%d"%globals.round_counter].append({"Acceleration (in Gs)": {"x": str(rpi_vals[8]), "y": str(rpi_vals[9]), "z": str(rpi_vals[7])}})
#						globals.rpidata["RPi data round%d"%globals.round_counter].append({"End of dataset": str(rpi_vals[10])})
#						json.dump(globals.rpidata, file, indent=4)
#				else:
#					time.sleep(0.1)
			counter += 1
#				time.sleep(0.1)

		globals.port.write(b'e')
		print("Number of rounds: ", counter)
		print("Closing serial comms..")
		print("Maximum system size: ", sys.maxsize)
		print("Memorysize of rpidata variable after round: " +str(sys.getsizeof(globals.rpidata)) + str(sys.getsizeof(globals.rpidata["RPi data round%d"%globals.round_counter])))

#		with open(globals.rpi_file_path, 'a') as file:
#			json.dump(globals.rpidata, file, indent=4)

#		globals.rpidata = {}
		logging.info('Stopping RPi data collection..')

		profiler.disable()
		stats = pstats.Stats(profiler).sort_stats('tottime')
		stats.dump_stats('/home/root/profiler_data/mz_rpi_profiler.prof')

