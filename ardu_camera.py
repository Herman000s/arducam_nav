
import sys
import time
sys.path.append('/home/root/ArduCAM_USB_Camera_Shield/RaspberryPi/Python/Streaming_demo')
from ImageConvert import *
import arducam_config_parser
import ArducamSDK
import globals
from datetime import datetime
import logging
import faulthandler
import cProfile, pstats

faulthandler.enable()

profiler = cProfile.Profile()

class Cam:

	def configure_board(self, config):
		ArducamSDK.Py_ArduCam_setboardConfig(globals.handle, config.params[0], \
			config.params[1], config.params[2], config.params[3], \
			config.params[4:config.params_length])

	pass

	def init_camera(self, filename):
		config = arducam_config_parser.LoadConfigFile(filename)

		camera_parameter = config.camera_param.getdict()
		width = camera_parameter["WIDTH"]
		height = camera_parameter["HEIGHT"]

		bitwidth = camera_parameter["BIT_WIDTH"]
		bytelength = 1
		if bitwidth > 8 and bitwidth <= 16:
			bytelength = 2
			globals.save_raw = True
		fmtmode = camera_parameter["FORMAT"][0]
		globals.color_mode = camera_parameter["FORMAT"][1]
		print("color mode: ", globals.color_mode)

		i2cmode = camera_parameter["I2C_MODE"]
		i2caddr = camera_parameter["I2C_ADDR"]
		translvl = camera_parameter["TRANS_LVL"]

		cfg = {"u32CameraType": 0x00,
			"u32Width": width,
			"u32Height": height,
			"usbType": 0,
			"u8PixelBytes": bytelength,
			"u16Vid": 0,
			"u32Size": 0,
			"u8PixelBits": bitwidth,
			"u32I2cAddr": i2caddr,
			"emI2cMode": i2cmode,
			"emImageFmtMode": fmtmode,
			"u32TransLvl": translvl
			}

		ret, globals.handle, rtn_cfg = ArducamSDK.Py_ArduCam_autoopen(cfg)

		if ret == 0:
			usb_version = rtn_cfg['usbType']
			configs = config.configs
			configs_length = config.configs_length

			for i in range(configs_length):
				type = configs[i].type

				if((type >> 16) & 0xFF) != 0 and ((type >> 16) & 0xFF) != usb_version:
					continue
				if type & 0xFFFF == arducam_config_parser.CONFIG_TYPE_REG:
					ArducamSDK.Py_ArduCam_writeSensorReg(globals.handle, configs[i].params[0], configs[i].params[1])
				elif type & 0xFFFF == arducam_config_parser.CONFIG_TYPE_DELAY:
					time.sleep(float(configs[i].params[0])/1000)
				elif type & 0xFFFF == arducam_config_parser.CONFIG_TYPE_VRCMD:
					self.configure_board(configs[i])

			rtn_val, datas = ArducamSDK.Py_ArduCam_readUserData(globals.handle, 0x400-16, 16)
			print("Serial: %c%c%c%c-%c%c%c%c-%c%c%c%c"%(datas[0], datas[1], datas[2], datas[3],
									datas[4], datas[5], datas[6], datas[7],
									datas[8], datas[9],datas[10], datas[11]))

			return True
		else:
			print("open fail, rtn_val = ", ret)

	pass

	def get_sensor_vals(self):
		temp_value_hex = ArducamSDK.Py_ArduCam_readSensorReg(globals.handle, 0x30B2)
		self.temperature_approx = (self.slope * temp_value_hex[1]) + self.T0
		
		self.delta_dk_level = ArducamSDK.Py_ArduCam_readSensorReg(globals.handle, 0x3188)
		self.ae_coarse_integtime = ArducamSDK.Py_ArduCam_readSensorReg(globals.handle, 0x3164)
		self.ae_coarse_integtime_in_sec = self.ae_coarse_integtime[1] * globals.read_rowtime
		self.ae_mean = ArducamSDK.Py_ArduCam_readSensorReg(globals.handle, 0x3152)
		self.ae_current_gains = ArducamSDK.Py_ArduCam_readSensorReg(globals.handle, 0x312A)
		self.ae_ana_gain = 2**((0b1100000000 & self.ae_current_gains[1]) >> 8)
		self.ae_dig_gain = (0b11111111 & self.ae_current_gains[1])/32


	def read_image(self):
		globals.usb_error_counter = 0
		count = 0
		time0 = time.time()
		time1 = time.time()
		data = {}

		# Pixelclock estimate (static values taken from file AR0135_MONO_custom.cfg)
		
		pll_multiplier = 18	# (M, 0x0012)
		pre_pll_clk_div = 1	# (N, 0x0001)
		vt_sys_clk_div = 1	# (P1, 0x0001)
		vt_pix_clk_div = 16	# (P2, 0x0010)
		extclk = 33330000	# (MZ input clock, 33.33 MHz, estimation since not sure if this is the actual value used by extclock)

		globals.read_pixclk = (extclk * pll_multiplier) / (pre_pll_clk_div * vt_sys_clk_div * vt_pix_clk_div)	# pixel/sec

		line_length_pck = 5500	# (0x157C, static value taken from above file)
		globals.read_rowtime = line_length_pck / globals.read_pixclk

		# Sensor temperature approximation (calibration values taken from AR0134 developer guide, page 37)

		calib1 = 466	# (R0x30C6 = 0x01D2 = 466 = 70 deg celcius)
		calib2 = 445	# (R0x30C8 = 0x01BD = 445 = 55 deg celcius)

		self.slope = (70 - 55) / (calib1 - calib2)

		self.T0 = 55 - (self.slope * calib2)	# Intercept

		start_time = time.time()
		seconds = 60
		image_counter = 0

		rtn_val_c = ArducamSDK.Py_ArduCam_beginCaptureImage(globals.handle)
		print("Beginning image capturing!")
		logging.info('Beginning image capturing..')

		if rtn_val_c != 0:
			print("Error in beginning capture, rtn_val = ", rtn_val_c)
			logging.info('Could not begin capturing..')
			ArducamSDK.Py_ArduCam_endCaptureImage(globals.handle)
			globals.running = False
			return
		else:
			print("Image capturing started, rtn_val = ", rtn_val_c)
			logging.info('Image capturing started!')

		profiler.enable()
		while globals.running:
			current_time = time.time()
			elapsed_time = current_time - start_time

			display_time = time.time()

			rtn_val_c = ArducamSDK.Py_ArduCam_captureImage(globals.handle)

			if rtn_val_c > 255:
				print("Error in capturing images, rtn_val_c = ", rtn_val_c)
				globals.usb_error_counter += 1
				if rtn_val_c == ArducamSDK.USB_CAMERA_USB_TASK_ERROR:
					ArducamSDK.Py_ArduCam_endCaptureImage(globals.handle)
					globals.running = False
					break

			if ArducamSDK.Py_ArduCam_availableImage(globals.handle) > 0:
				rtn_val, data, rtn_cfg = ArducamSDK.Py_ArduCam_readImage(globals.handle)
				
				image_time = datetime.now()
				self.get_sensor_vals()
				
				datasize = rtn_cfg['u32Size']

				if rtn_val != 0 or datasize == 0:
					ArducamSDK.Py_ArduCam_del(globals.handle)
					print("Read data fail!")
					continue

				if globals.read_flag == False:
					globals.read_flag = True
					globals.rpi_lock = True
					globals.ret_val_read = rtn_val
					globals.data_read = data
					globals.cfg_read = rtn_cfg

					globals.image_time_read = image_time

					globals.read_delta_dk = self.delta_dk_level
					globals.read_ae_coarse = self.ae_coarse_integtime
					globals.read_ae_coarse_sec = self.ae_coarse_integtime_in_sec
					globals.read_ae_mean = self.ae_mean
					globals.read_current_gains = self.ae_current_gains
					globals.read_ana_gain = self.ae_ana_gain
					globals.read_dig_gain = self.ae_dig_gain
					globals.read_temp_approx = self.temperature_approx

					#print("Read image to be processed: ", image_counter)

				time1 = time.time()
				if time1 - time0 >= 1:
					#print("%s %d %s\n"%("fps:", count, "/s"))
					count = 0
					time0 = time1
					globals.gpio_led_value = not globals.gpio_led_value
					globals.gpio_led_out.write(globals.gpio_led_value)

				count += 1
				ArducamSDK.Py_ArduCam_del(globals.handle)
				image_counter += 1

				if elapsed_time > seconds:
					ArducamSDK.Py_ArduCam_endCaptureImage(globals.handle)
					print("Closing image read..")
					globals.running = False
			else:
				time.sleep(0.002)

		print("Closing image capture...")
		logging.info('Closing image capturing..')

		profiler.disable()
		stats = pstats.Stats(profiler).sort_stats('tottime')
		stats.dump_stats('/home/root/profiler_data/ardu_profiler.prof')
