
import sys
import time
import cv2
import numpy as np
import json
import cProfile, pstats
#from memory_profiler import profile
sys.path.append('/home/root/ArduCAM_USB_Camera_Shield/RaspberryPi/Python/Streaming_demo')
from ImageConvert import *

import logging
from datetime import datetime
import quaternion
sys.path.append('/home/root/hw_visnav')
from visnav.algo.model import Camera
from visnav.algo.odometry import VisualOdometry, Pose
import globals

import faulthandler
faulthandler.enable()

profiler = cProfile.Profile()

class Visualodo:

	def odo_setup(self):
		print("Starting setup process..")
		logging.info('Starting setup process..')
		self.cam = get_cam()

		params = {
			'use_ba': True,
			'threaded_ba': True,
			'max_keyframes': 8,
			'max_ba_keyframes': 8,
			'ba_interval': 4,
			'max_ba_fun_eval': 20,
			'asteroid': False,
		}
#

		# Vanhat alla
#		params = {
#			'min_keypoint_dist': 10,
#			'min_inliers': 12,
#			'min_2d2d_inliers': 24,
#			'keypoint_algo': VisualOdometry.KEYPOINT_SHI_TOMASI,
#			'max_ba_fun_eval': 10,
#		}
		#logging.basicConfig(filename=globals.visnav_log_file_path, format='%(message)s', level=logging.DEBUG)
		self.odo = VisualOdometry(self.cam,self.cam.width/2,verbose=0,pause=False,
						use_scale_correction=False,est_cam_pose=False,**params)
#		self.odo = VisualOdometry(self.cam, self.cam.width/4, verbose = 0, pause = False,
#						use_scale_correction = False, est_cam_pose = False, **params)


	def terminate_processes(self):
		print("Terminating visnav processes!")
		self.odo.quit()


	def collect_sensor_data(self):
		self.img_data_meta = {"Image time": str(globals.image_time_read),
				"Pixel clock (pixel/s)": globals.read_pixclk,
				"Row-time": globals.read_rowtime,
				"Dark current level": globals.read_delta_dk,
				"AE integration time (rows)": globals.read_ae_coarse,
				"AE integration time (seconds)": globals.read_ae_coarse_sec,
				"AE mean": globals.read_ae_mean,
				"Analog/Digital current gains": globals.read_current_gains,
				"AE analog gain": globals.read_ana_gain,
				"AE digital gain": globals.read_dig_gain,
				"Temperature approximation (deg celcius)": globals.read_temp_approx}

#	@profile
	def process_img(self):
#		self.odo_setup()	# Called once from main

		print("Starting image processing..")
		logging.info('Starting image processing..')
		totalframe = 0
		
		cam_q = quaternion.one
		cam_obj_v = [0, 0, 0]
		cam_obj_q = quaternion.one
		prior = Pose(cam_obj_v, cam_obj_q, np.ones((3,)) * 0.1, np.ones((3,)) * 0.01)
		res = None

		ts = datetime.now()
#		with open(globals.log_file_path, 'a') as f:
		globals.logdata.update({"Timestamp round%d"%globals.round_counter: [], "Image metadata round%d"%globals.round_counter: [], "USB errors round%d"%globals.round_counter: globals.usb_error_counter})
		globals.logdata["Timestamp round%d"%globals.round_counter].append(str(ts))
#			json.dump(globals.logdata, f, indent=4)

		profiler.enable()
		while globals.running:
			if globals.read_flag == False:
				time.sleep(0.01)
			else:
				rpi_vals = globals.rpi_val_list.copy()
				globals.rpi_val_list.clear()
				globals.rpi_lock = False
				image = convert_image(globals.data_read, globals.cfg_read, globals.color_mode)

				process_time_start = time.time()
				res = self.odo.process(image[2:-2, :], globals.image_time_read, prior, cam_q)
				process_time_end = time.time()
				process_time = process_time_end - process_time_start
				print("Process results (res): ", res)
				print("Process results (res[0]): ", res[0])

				#print("test%d"%totalframe)
				self.save_img(image, totalframe, res, process_time, rpi_vals)
				totalframe += 1
				if res and res[0] and res[0].post:
					print("Process results (res[0].post): ", res[0].post)
					prior = res[0].post
						
				globals.read_flag = False

		print("Size of logdata after round in bytes: ", str(globals.logdata.__sizeof__()))
		with open(globals.log_file_path, 'a') as file:
			json.dump(globals.logdata, file, indent=4)
		globals.logdata = {}
		
#		with open(globals.rpi_file_path, 'a') as file:
#			json.dump(globals.rpidata, file, indent=4)

		print("Closing image processing..")
		logging.info('Closing image processing..')

		profiler.disable()
		stats = pstats.Stats(profiler).sort_stats('tottime')
		stats.dump_stats('/home/root/profiler_data/visnav_profiler.prof')
		stats.print_stats()

	#@profile
	def save_img(self, image, totalframe, res, process_time, rpi_vals):
		cv2.imwrite("/home/root/images/image%d.jpg"%totalframe, image)

		#with open(globals.rpi_file_path, 'w') as file:
			#globals.rpidata["RPi data round%d"%globals.round_counter].append(resp)
		#globals.rpidata["RPi data round%d"%globals.round_counter].append({"Image number %d"%totalframe: rpi_vals})
		file_obj = open(globals.rpi_alt_file_path, 'ab')
		file_obj.writelines(rpi_vals)
		file_obj.write(b'\n')
		file_obj.close()
		#	json.dump(globals.rpidata, file, indent=4)

		
		self.collect_sensor_data()
		with open("/home/root/meta/meta%d.json"%totalframe, 'w') as json_file:
			json.dump(self.img_data_meta, json_file, indent=4)

		
		img_data_log = {"Image number": totalframe,
				"Pose estimate": "None" if res[0] == None else {"location": str(res[0].post)},
				"Process time": process_time
				}
		img_data_log.update(self.img_data_meta)


#		with open(globals.log_file_path, 'w') as file:
		globals.logdata["Image metadata round%d"%globals.round_counter].append(img_data_log)
		globals.logdata["USB errors round%d"%globals.round_counter] = globals.usb_error_counter
#			json.dump(globals.logdata, file, indent=4)


def get_cam():
	common_kwargs_worst = {
		'sensor_size': (1280 * 0.00375, 960 * 0.00375),
		'quantum_eff': 0.30,
		'px_saturation_e': 2200,
		'lambda_min': 350e-9, 'lambda_eff': 580e-9, 'lambda_max': 800e-9,
		'dark_noise_mu': 40, 'dark_noise_sd': 6.32, 'readout_noise_sd': 15,
		'emp_coef': 1,
		'exclusion_angle_x': 55,
		'exclusion_angle_y': 90,
	}
	common_kwargs_best = dict(common_kwargs_worst)
	common_kwargs_best.update({
		'quantum_eff': 0.4,
		'px_saturation_e': 3500,
		'dark_noise_mu': 25, 'dark_noise_sd': 5, 'readout_noise_sd': 5,
	})

	common_kwargs = common_kwargs_best

	return Camera(
		1280,
		960,
		43.6,
		33.4,
		f_stop=5,
		point_spread_fn = 0.50,
		scattering_coef = 2e-10,
		dist_coefs=[-3.79489919e-01, 2.55784821e-01, 9.52433459e-04, 1.27543923e-04, -2.74301340e-01],
		cam_mx=np.array([[1.60665503e+03, 0.00000000e+00, 6.12522544e+02],
				[0.00000000e+00, 1.60572265e+03, 4.57510418e+02],
				[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]),
		**common_kwargs
	)

	# Vanhat alla
#	return Camera(
#		1280,
#		960,
#		35.5,
#		33.5,
#		f_stop = 5,
#		point_spread_fn = 0.50,
#		scattering_coef = 2e-10,
#		**common_kwargs
#	)

