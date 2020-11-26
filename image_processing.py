
import sys
import time
import cv2
import numpy as np
import json
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

class Visualodo:

	def odo_setup(self):
		print("Starting setup process..")
		logging.info('Starting setup process..')
		self.cam = get_cam()
		params = {
			'min_keypoint_dist': 10,
			'min_inliers': 12,
			'min_2d2d_inliers': 24,
			'keypoint_algo': VisualOdometry.KEYPOINT_SHI_TOMASI,
			'max_ba_fun_eval': 10,
		}
		#logging.basicConfig(filename=globals.visnav_log_file_path, format='%(message)s', level=logging.DEBUG)
		self.odo = VisualOdometry(self.cam, self.cam.width/4, verbose = 0, pause = False,
						use_scale_correction = False, est_cam_pose = False, **params)


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


	def process_img(self):
		#self.odo_setup()	# Called once from main

		print("Starting image processing..")
		logging.info('Starting image processing..')
		totalframe = 0
		cam_q = quaternion.one
		cam_obj_v = [0, 0, 0]
		cam_obj_q = quaternion.one
		prior = Pose(cam_obj_v, cam_obj_q, np.ones((3,)) * 0.1, np.ones((3,)) * 0.01)
		res = None

		ts = datetime.now()
		with open(globals.log_file_path, 'w') as f:
			globals.logdata.update({"Timestamp round%d"%globals.round_counter: [], "Image metadata round%d"%globals.round_counter: [], "USB errors round%d"%globals.round_counter: globals.usb_error_counter})
			globals.logdata["Timestamp round%d"%globals.round_counter].append(str(ts))
			json.dump(globals.logdata, f, indent=4)

		while globals.running:
			if globals.read_flag == False:
				time.sleep(0.01)
			else:
				image = convert_image(globals.data_read, globals.cfg_read, globals.color_mode)

				process_time_start = time.time()
				res = self.odo.process(image[2:-2, :], globals.image_time_read, prior, cam_q)
				process_time_end = time.time()
				process_time = process_time_end - process_time_start
				#print("Process results: ", res)

				#print("test%d"%totalframe)
				self.save_img(image, totalframe, res, process_time)
				totalframe += 1
				if res[0] is not None:
					prior = res[0]
						
				globals.read_flag = False

		print("Closing image processing..")
		logging.info('Closing image processing..')


	def save_img(self, image, totalframe, res, process_time):
		cv2.imwrite("/home/root/images/image%d.jpg"%totalframe, image)

		self.collect_sensor_data()
		with open("/home/root/meta/meta%d.json"%totalframe, 'w') as json_file:
			json.dump(self.img_data_meta, json_file, indent=4)

		img_data_log = {"Image number": totalframe,
				"Pose estimate": "None" if res[0] == None else {"location": str(res[0].loc), "quaternion": str(res[0].quat)},
				"Process time": process_time
				}
		img_data_log.update(self.img_data_meta)

		with open(globals.log_file_path, 'w') as file:
			globals.logdata["Image metadata round%d"%globals.round_counter].append(img_data_log)
			globals.logdata["USB errors round%d"%globals.round_counter] = globals.usb_error_counter
			json.dump(globals.logdata, file, indent=4)

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
		35.5,
		33.5,
		f_stop = 5,
		point_spread_fn = 0.50,
		scattering_coef = 2e-10,
		**common_kwargs
	)

