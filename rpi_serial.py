import serial
import json
import numpy as np
import pickle
import globals
from datetime import datetime

# Serial abbreviations -> s = start, e = end, y = yes 

class Rpicomms:

	def setup(self):
		self.port = serial.Serial("/dev/ttyPS1", baudrate=9600, timeout=2.0)
		self.local_round_counter = 0


	def start_comms(self):
		while True:
			self.port.write(b's')
			resp = self.port.read(1).decode()

			if resp == "y":
				print("Communication with rpi has started!")
				break
			else:
				print("Waiting for rpi to wake up..")
		self.port.close()


	def collect_data(self):
		port = serial.Serial("/dev/ttyPS1", baudrate=9600, timeout=2.0)
		ts = datetime.now()
		with open(globals.rpi_file_path, 'w') as f:
			globals.rpidata.update({"Timestamp round%d"%globals.round_counter: [], "RPi data round%d"%globals.round_counter: []})
			globals.rpidata["Timestamp round%d"%globals.round_counter].append(str(ts))
			json.dump(globals.rpidata, f, indent=4)

		while globals.running:
			resp = port.read(88)
			print("data length received from rpi (bytes): ", len(resp))
			print("decoded rpi data: ", resp)
			print("rpi data type: ", type(resp))

			rpi_vals = np.frombuffer(resp, dtype=np.float64)
			print("data length: ", len(rpi_vals))
 
			if len(rpi_vals) > 0:
				with open(globals.rpi_file_path, 'w') as file:
					globals.rpidata["RPi data round%d"%globals.round_counter].append({"RPi time": rpi_vals[0]})
					globals.rpidata["RPi data round%d"%globals.round_counter].append({"Magnetometer": {"x": rpi_vals[1], "y": rpi_vals[2], "z": rpi_vals[3]}})
					globals.rpidata["RPi data round%d"%globals.round_counter].append({"Gyro": {"pitch": rpi_vals[4], "roll": rpi_vals[5], "yaw": rpi_vals[6]}})
					globals.rpidata["RPi data round%d"%globals.round_counter].append({"Acceleration": {"pitch": rpi_vals[7], "roll": rpi_vals[8], "yaw": rpi_vals[9]}})
					globals.rpidata["RPi data round%d"%globals.round_counter].append({"End of dataset": rpi_vals[10]})
					json.dump(globals.rpidata, file, indent=4)
			print("Decoded: ", rpi_vals)

		port.write(b'e')
		print("Closing serial comms..")
		port.close()
