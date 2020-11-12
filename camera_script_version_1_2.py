import sys
import os, re, os.path
import time
import cv2
import threading
import numpy as np
import signal
import json
sys.path.append('/home/root/ArduCAM_USB_Camera_Shield/RaspberryPi/Python/Streaming_demo')
from ImageConvert import *
import arducam_config_parser
import ArducamSDK

import tempfile
import math
from datetime import datetime
import quaternion
sys.path.append('/home/root/hw_visnav')
from visnav.algo.model import Camera
from visnav.algo.odometry import VisualOdometry, Pose
from visnav.algo import tools
from periphery import GPIO

global cfg, handle, running, Width, Height, save_flag, color_mode, save_raw, ret_val_read, data_read, cfg_read, read_flag, gpio_led_out, gpio_led_value, gpio_sw1_in, delete_old_imgs, usb_error_counter
running = True
read_flag = False
save_flag = False
save_raw = False
cfg = {}
handle = {}
delete_old_imgs = False

class camera:

	def configure_board(self, config):
		global handle
		ArducamSDK.Py_ArduCam_setboardConfig(handle, config.params[0], \
			config.params[1], config.params[2], config.params[3], \
			config.params[4:config.params_length])

	pass

	def init_camera(self, fileName):
		global cfg, handle, Width, Height, color_mode, save_raw
		config = arducam_config_parser.LoadConfigFile(fileName)

		camera_parameter = config.camera_param.getdict()
		Width = camera_parameter["WIDTH"]
		Height = camera_parameter["HEIGHT"]

		BitWidth = camera_parameter["BIT_WIDTH"]
		ByteLength = 1
		if BitWidth > 8 and BitWidth <= 16:
			ByteLength = 2
			save_raw = True
		FmtMode = camera_parameter["FORMAT"][0]
		color_mode = camera_parameter["FORMAT"][1]
		print("color mode: ", color_mode)

		I2CMode = camera_parameter["I2C_MODE"]
		I2cAddr = camera_parameter["I2C_ADDR"]
		TransLvl = camera_parameter["TRANS_LVL"]

		cfg = {"u32CameraType": 0x00,
			"u32Width": Width,
			"u32Height": Height,
			"usbType": 0,
			"u8PixelBytes": ByteLength,
			"u16Vid": 0,
			"u32Size": 0,
			"u8PixelBits": BitWidth,
			"u32I2cAddr": I2cAddr,
			"emI2cMode": I2CMode,
			"emImageFmtMode": FmtMode,
			"u32TransLvl": TransLvl
			}

		ret, handle, rtn_cfg = ArducamSDK.Py_ArduCam_autoopen(cfg)

		if ret == 0:
			usb_version = rtn_cfg['usbType']
			configs = config.configs
			configs_length = config.configs_length

			for i in range(configs_length):
				type = configs[i].type

				if((type >> 16) & 0xFF) != 0 and ((type >> 16) & 0xFF) != usb_version:
					continue
				if type & 0xFFFF == arducam_config_parser.CONFIG_TYPE_REG:
					ArducamSDK.Py_ArduCam_writeSensorReg(handle, configs[i].params[0], configs[i].params[1])
				elif type & 0xFFFF == arducam_config_parser.CONFIG_TYPE_DELAY:
					time.sleep(float(configs[i].params[0])/1000)
				elif type & 0xFFFF == arducam_config_parser.CONFIG_TYPE_VRCMD:
					self.configure_board(configs[i])

			rtn_val, datas = ArducamSDK.Py_ArduCam_readUserData(handle, 0x400-16, 16)
			print("Serial: %c%c%c%c-%c%c%c%c-%c%c%c%c"%(datas[0], datas[1], datas[2], datas[3],
									datas[4], datas[5], datas[6], datas[7],
									datas[8], datas[9],datas[10], datas[11]))

			return True
		else:
			print("open fail, rtn_val = ", ret)

	pass


	def read_image(self):
		global handle, running, Width, Height, save_flag, cfg, color_mode, save_raw, ret_val_read, data_read, cfg_read, read_flag, gpio_led_out, gpio_led_value, gpio_sw_in, delete_old_imgs, usb_error_counter
		global COLOR_BayerGB2BGR, COLOR_BayerRG2BGR, COLOR_BayerGR2BGR, COLOR_BayerBG2BGR

		usb_error_counter = 0
		count = 0
		time0 = time.time()
		time1 = time.time()
		data = {}

		if not os.path.exists("/home/root/images"):
			os.makedirs("/home/root/images")

		if not os.path.exists("/home/root/meta"):
			os.makedirs("/home/root/meta")
		
		start_time = time.time()
		seconds = 60
		image_counter = 0

		rtn_val_c = ArducamSDK.Py_ArduCam_beginCaptureImage(handle)
		print("Beginning image capturing!")

		if rtn_val_c != 0:
			print("Error in beginning capture, rtn_val = ", rtn_val_c)
			ArducamSDK.Py_ArduCam_endCaptureImage(handle)
			running = False
			return
		else:
			print("Image capturing started, rtn_val = ", rtn_val_c)

		while running:
			current_time = time.time()
			elapsed_time = current_time - start_time

			rtn_val_c = ArducamSDK.Py_ArduCam_captureImage(handle)

			if rtn_val_c > 255:
				print("Error in capturing images, rtn_val_c = ", rtn_val_c)
				usb_error_counter += 1
				if rtn_val_c == ArducamSDK.USB_CAMERA_USB_TASK_ERROR:
					ArducamSDK.Py_ArduCam_endCaptureImage(handle)
					running = False
					break
#			time.sleep(0.01)

			if ArducamSDK.Py_ArduCam_availableImage(handle) > 0 and rtn_val_c == 1:
				rtn_val, data, rtn_cfg = ArducamSDK.Py_ArduCam_readImage(handle)
				datasize = rtn_cfg['u32Size']
				
				if rtn_val != 0 or datasize == 0:
					ArducamSDK.Py_ArduCam_del(handle)
					print("Read data fail!")
					continue

				if read_flag == False:
					read_flag = True
					ret_val_read = rtn_val
					data_read = data
					cfg_read = rtn_cfg
					print("Image to be processed: ", image_counter)

				time1 = time.time()
				
				if time1 - time0 >= 1:
					print("%s %d %s\n"%("fps:", count, "/s"))
					count = 0
					time0 = time1
					gpio_led_value = not gpio_led_value
					gpio_led_out.write(gpio_led_value)

				count += 1
				
				ArducamSDK.Py_ArduCam_del(handle)

				image_counter += 1

				if elapsed_time > seconds:
					ArducamSDK.Py_ArduCam_endCaptureImage(handle)
					print("Waiting button press for 20 sec before terminating program!")
					start_time_bPress = time.time()
					wait_time = 20
					gpio_led_out.write(True)

					while True:
						current_wait_time = time.time()
						elapsed_wait_time = current_wait_time - start_time_bPress
						
						gpio_sw_value = gpio_sw_in.read()
						if gpio_sw_value:
							delete_old_imgs = True
							print("sw1 pressed!")
							time.sleep(1.5)
							flush_err = ArducamSDK.Py_ArduCam_flush(handle)
							image_counter = 0
							usb_error_counter = 0
							#print("Arducam flush state: ", flush_err)

							start_time = time.time()
							current_time = time.time()
							time0 = time.time()
							time1 = time.time()
							count = 0

							rtn_val_c = ArducamSDK.Py_ArduCam_beginCaptureImage(handle)
							print("Beginning image capturing!")

							if rtn_val_c != 0:
								print("Error in beginning capture, rtn_val = ", rtn_val_c)
								ArducamSDK.Py_ArduCam_endCaptureImage(handle)
								running = False
								return
							else:
								print("Image capturing started, rtn_val = ", rtn_val_c)
							break
			
						if elapsed_wait_time > wait_time:
							print("Closing image read...")
							running = False
							break
			else:
				time.sleep(0.002)

		print("Closing image capture...")

		
class visualOdo:

	def odo_setup(self):
		print("Starting setup process..")
		self.cam = get_cam()
		params = {
			'min_keypoint_dist': 10,
			'min_inliers': 12,
			'min_2d2d_inliers': 24,
		}
		self.odo = VisualOdometry(self.cam, self.cam.width/4, verbose = 0, pause = False,
						use_scale_correction = False, est_cam_pose = False, **params)

	def process_img(self):
		global ret_val_read, data_read, cfg_read, color_mode, running, read_flag, delete_old_imgs, usb_error_counter
		self.odo_setup()
		print("Starting image processing..")
		totalFrame = 0

		cam_q = quaternion.one
		orig_time = datetime.strptime('2020-10-01 12:00:00', '%Y-%m-%d %H:%M:%S').timestamp()
		print("orig time: ", orig_time)
		cam_obj_v = [0, 0, 0]
		cam_obj_q = quaternion.one
		time_increment = 60
		time_new = datetime.fromtimestamp(orig_time + time_increment)
		prior = Pose(cam_obj_v, cam_obj_q, np.ones((3,)) * 0.1, np.ones((3,)) * 0.01)
		
		image_path = "/home/root/images"
		if os.path.getsize(image_path) != 0:
			print("Removing old images..")
			for root, dirs, files in os.walk(image_path):
				for file in files:
					os.remove(os.path.join(root, file))

		metadata_path = "/home/root/meta/metadata.json"

		if os.path.exists(metadata_path):
			print("Deleting old metadata file!")
			os.remove(metadata_path)
	
		if not os.path.exists(metadata_path):
			print("Metadata file does not exist, creating..")
			open(metadata_path, "a")
		
		round_counter = 0
		metadata = {"Timestamp round%d"%round_counter: [],
				"Image metadata round%d"%round_counter: [],
				"USB DATA LEN ERROR round%d"%round_counter: usb_error_counter}

		while running:

			if delete_old_imgs == False:

				if read_flag == False:
					time.sleep(0.01)
				else:
					image = convert_image(data_read, cfg_read, color_mode)
					res = self.odo.process(image, time_new, prior, cam_q)
				
					time_increment = time_increment + 60
					time_new = datetime.fromtimestamp(orig_time + time_increment)
					print("Process results: ", res)

					print("test%d"%totalFrame)
					self.save_img(image, totalFrame, metadata_path, metadata, round_counter)
					totalFrame += 1
					if res[0] is not None and 0:
						prior = res[0]

					read_flag = False

			else:
				delete_old_imgs = False
				print("Clearing old image data..")
				for root, dirs, files in os.walk(image_path):
					for file in files:
						os.remove(os.path.join(root, file))

				image = None
				totalFrame = 0
				time_increment = 60
				orig_time = datetime.strptime('2020-10-01 12:00:00', '%Y-%m-%d %H:%M:%S').timestamp()
				time_new = datetime.fromtimestamp(orig_time + time_increment)
				ts = datetime.now().timestamp()
				round_counter += 1
				with open(metadata_path, 'w') as f:
					metadata.update({"Timestamp round%d"%round_counter: [], "Image metadata round%d"%round_counter: [], "USB DATA LEN ERROR round%d"%round_counter: usb_error_counter})
					metadata["Timestamp round%d"%round_counter].append(ts)
					json.dump(metadata, f, indent=4)

				prior = Pose(cam_obj_v, cam_obj_q, np.ones((3,)) * 0.1, np.ones((3,)) * 0.01)
		
		print("Closing image processing..")

	def save_img(self, image, totalFrame, metadata_path, metadata, round_counter):
		cv2.imwrite("/home/root/images/image%d.jpg"%totalFrame, image)

		#temp_value_hex = ArducamSDK.Py_ArduCam_readSensorReg(handle, 0x30B2)
		#print("Temp. hex-value: ", temp_value_hex)
		#tempsens_calib1 = ArducamSDK.Py_ArduCam_readSensorReg(handle, 0x30CC)
		#print("calib1: ", tempsens_calib1)
		#tempsens_calib2 = ArducamSDK.Py_ArduCam_readSensorReg(handle, 0x30C8)
		#print("calib2: ", tempsens_calib2)

		delta_dk_level = ArducamSDK.Py_ArduCam_readSensorReg(handle, 0x3188)
		ae_coarse_integtime = ArducamSDK.Py_ArduCam_readSensorReg(handle, 0x3164)
		ae_mean = ArducamSDK.Py_ArduCam_readSensorReg(handle, 0x3152)
		ae_current_gains = ArducamSDK.Py_ArduCam_readSensorReg(handle, 0x312A)
		
		img_data = {"Image number": totalFrame,
				"Dark current level": delta_dk_level,
				"AE integration time": ae_coarse_integtime,
				"AE mean": ae_mean,
				"Analog/Digital current gains": ae_current_gains}

		if os.path.getsize(metadata_path) == 0:
			with open(metadata_path, 'w') as json_file:
				time_stamp = datetime.now().timestamp()
				metadata["Timestamp round%d"%round_counter].append(time_stamp)
				metadata["Image metadata round%d"%round_counter].append(img_data)
				metadata["USB DATA LEN ERROR round%d"%round_counter] = usb_error_counter
				json.dump(metadata, json_file, indent=4)
		else:
			with open(metadata_path, 'w') as file:
				metadata["Image metadata round%d"%round_counter].append(img_data)
				metadata["USB DATA LEN ERROR round%d"%round_counter] = usb_error_counter
				json.dump(metadata, file, indent=4)

def get_cam():
	common_kwargs_worst = {
		'sensor_size': (1280 * 0.00375, 964 * 0.00375),
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
		964,
		35.5,
		33.5,
		f_stop = 5,
		point_spread_fn = 0.50,
		scattering_coef = 2e-10,
		**common_kwargs
	)

def write_json(metadata, filename='metadata.json'):
	with open('metadata.json', 'w') as f:
		json.dump(metadata, f, indent=4)

def sigint_handler(signum, frame):
	global running, handle
	running = False
	exit()

signal.signal(signal.SIGINT, sigint_handler)
signal.signal(signal.SIGTERM, sigint_handler)

if __name__ == "__main__":

	config_file_name = "/home/root/ArduCAM_USB_Camera_Shield/Config/USB2.0_UC-391_Rev.D/DVP/AR0135/AR0135_MONO_custom.cfg"

	if not os.path.exists(config_file_name):
		print("Config file does not exist!")
		exit()

	camera_obj = camera()
	odo_obj = visualOdo()

	gpio_led_out = GPIO(953, "out")
	gpio_led_value = False

	gpio_sw_in = GPIO(957, "in")

	camera_obj.init_camera(config_file_name)
	ArducamSDK.Py_ArduCam_setMode(handle, ArducamSDK.CONTINUOUS_MODE)

	rt = threading.Thread(target = camera_obj.read_image)
	pt = threading.Thread(target = odo_obj.process_img)

	rt.start()
	pt.start()

	rt.join()
	pt.join()

	gpio_led_out.write(False)
	gpio_led_out.close()
	gpio_sw_in.close()

	rtn_val = ArducamSDK.Py_ArduCam_close(handle)
	print("About to close program execution..")

	if rtn_val == 0:
		print("Device close success!")
	else:
		print("Device close fail!")

