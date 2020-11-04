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

global cfg, handle, running, Width, Height, save_flag, color_mode, save_raw, ret_val_read, data_read, cfg_read, read_flag, gpio_led_out, gpio_led_value, gpio_sw1_in, new_round_flag, delete_old_imgs
running = True
read_flag = False
save_flag = False
save_raw = False
cfg = {}
handle = {}
new_round_flag = False
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


	def take_image(self):
		global handle, running, new_round_flag

		rtn_val = ArducamSDK.Py_ArduCam_beginCaptureImage(handle)
		print("Beginning image capturing!")
		
		if rtn_val != 0:
			print("Error in beginning capture, rtn_val = ", rtn_val)
			running = False
			return
		else:
			print("Image capturing started, rtn_val = ", rtn_val)

		while running:
			if new_round_flag == False:
				rtn_val = ArducamSDK.Py_ArduCam_captureImage(handle)
			
				if rtn_val > 255:
					print("Error in capturing images, rtn_val = ", rtn_val)
				
					if rtn_val == ArducamSDK.USB_CAMERA_USB_TASK_ERROR:
						break
				#time.sleep(0.01)
			else:
				continue

		running = False
		ArducamSDK.Py_ArduCam_endCaptureImage(handle)
		print("Closing image capture!")


	def read_image(self):
		global handle, running, Width, Height, save_flag, cfg, color_mode, save_raw, ret_val_read, data_read, cfg_read, read_flag, gpio_led_out, gpio_led_value, gpio_sw_in, new_round_flag, delete_old_imgs
		global COLOR_BayerGB2BGR, COLOR_BayerRG2BGR, COLOR_BayerGR2BGR, COLOR_BayerBG2BGR

		count = 0
		time0 = time.time()
		time1 = time.time()
		data = {}

		if not os.path.exists("/home/root/images"):
			os.makedirs("/home/root/images")
		
		start_time = time.time()
		seconds = 20

		while running:

			current_time = time.time()
			elapsed_time = current_time - start_time

			display_time = time.time()

			if ArducamSDK.Py_ArduCam_availableImage(handle) > 0:
				rtn_val, data, rtn_cfg = ArducamSDK.Py_ArduCam_readImage(handle)
				datasize = rtn_cfg['u32Size']
				
				if rtn_val != 0 or datasize == 0:
					ArducamSDK.Py_ArduCam_del(handle)
					print("Read data fail!")
					continue

				#image = convert_image(data, rtn_cfg, color_mode)

				time1 = time.time()
				
				if time1 - time0 >= 1:
					print("%s %d %s\n"%("fps:", count, "/s"))
					count = 0
					time0 = time1
					gpio_led_value = not gpio_led_value
					gpio_led_out.write(gpio_led_value)

				count += 1
				
				ArducamSDK.Py_ArduCam_del(handle)
	
				if read_flag == False:
					read_flag = True
					ret_val_read = rtn_val
					data_read = data
					cfg_read = rtn_cfg

				if elapsed_time > seconds:
					new_round_flag = True
					print("Waiting button press for 10 sec before terminating program!")
					start_time_bPress = time.time()
					wait_time = 10
					gpio_led_out.write(True)
					while True:
						current_wait_time = time.time()
						elapsed_wait_time = current_wait_time - start_time_bPress
						
						gpio_sw_value = gpio_sw_in.read()
						if gpio_sw_value:
							print("sw1 pressed!")
							time.sleep(1)
							flush_err = ArducamSDK.Py_ArduCam_flush(handle)
							print("Arducam flush state: ", flush_err)
							start_time = time.time()
							current_time = time.time()
							time0 = time.time()
							time1 = time.time()
							count = 0
							delete_old_imgs = True
							new_round_flag = False
							#time.sleep(0.001)
							break
			
						if elapsed_wait_time > wait_time:
							
							print("Closing image read...")
							running = False
							break
			else:
				#time.sleep(0.001)
				continue
				#print("Image not available!")

	def sigint_handler(signum, frame):
		global running, handle
		running = False
		exit()

	signal.signal(signal.SIGINT, sigint_handler)
	signal.signal(signal.SIGTERM, sigint_handler)

		
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
		global ret_val_read, data_read, cfg_read, color_mode, running, read_flag, new_round_flag, delete_old_imgs
		self.odo_setup()
		print("Starting image processing..")
		totalFrame = 0
		cam_q = quaternion.one
		orig_time = datetime.strptime('2020-10-01 12:00:00', '%Y-%m-%d %H:%M:%S').timestamp()
		cam_obj_v = [0, 0, 0]
		cam_obj_q = quaternion.one
		time_increment = 60
		time_new = datetime.fromtimestamp(orig_time + time_increment)
		prior = Pose(cam_obj_v, cam_obj_q, np.ones((3,)) * 0.1, np.ones((3,)) * 0.01)
		
		while running:
			
			if delete_old_imgs == False:
				if read_flag == False:
					continue
				else:
					image = convert_image(data_read, cfg_read, color_mode)

					#prior = Pose(cam_obj_v, cam_obj_q, np.ones((3,)) * 0.1, np.ones((3,)) * 0.01)
					res = self.odo.process(image, time_new, prior, cam_q)
				
					time_increment = time_increment + 60
					time_new = datetime.fromtimestamp(orig_time + time_increment)
					print("Process results: ", res)

					print("test%d"%totalFrame)
					self.save_img(image, totalFrame)
					totalFrame += 1
					#time.sleep(0.01)
					read_flag = False

			else:
				print("Clearing old data..")
				my_image_path = "/home/root/images"
				for root, dirs, files in os.walk(my_image_path):
					for file in files:
						os.remove(os.path.join(root, file))

				totalFrame = 0
				time_increment = 60
				time_new = datetime.fromtimestamp(orig_time + time_increment)
				delete_old_imgs = False
		
		print("Closing image processing..")

	def save_img(self, image, totalFrame):
		cv2.imwrite("/home/root/images/image%d.jpg"%totalFrame, image)
		cv2.waitKey(10)

def get_cam():
	common_kwargs_worst = {
		'sensor_size': (1280 * 0.0022, 960 * 0.0022),
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
		7.7,
		7.309,
		f_stop = 5,
		point_spread_fn = 0.50,
		scattering_coef = 2e-10,
		**common_kwargs
	)

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

	#if camera().init_camera(config_file_name):
	camera_obj.init_camera(config_file_name)
	ArducamSDK.Py_ArduCam_setMode(handle, ArducamSDK.CONTINUOUS_MODE)

	ct = threading.Thread(target = camera_obj.take_image)
	rt = threading.Thread(target = camera_obj.read_image)
	pt = threading.Thread(target = odo_obj.process_img)

	ct.start()
	rt.start()
	pt.start()

	ct.join()
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

