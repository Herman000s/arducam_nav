# File that contains all global variables

def initialize():
	# Camera setup / program state
	global handle, running, color_mode, save_raw, usb_error_counter, round_counter

	# Image handle / image read 
	global ret_val_read, data_read, cfg_read, image_time_read, read_flag

	# HW led
	global gpio_led_out, gpio_led_value

	# log dictionary
	global logdata

	# camera sensor values
	global read_delta_dk, read_ae_coarse, read_ae_coarse_sec, read_ae_mean, read_current_gains, read_ana_gain, read_dig_gain, read_pixclk, read_rowtime, read_temp_approx

	# files and folders
	global image_path, metadata_path, log_path, log_file_path, visnav_log_file_path

	image_path = "/home/root/images"
	metadata_path = "/home/root/meta"
	log_path = "/home/root/log"
	log_file_path = "/home/root/log/log.json"
	visnav_log_file_path = "/home/root/log/visnav_msg.log"

	logdata = {}

	round_counter = 0
	usb_error_counter = 0
	ret_val_read = None
	data_read = None
	cfg_read = None
	image_time_read = None
	running = True
	read_flag = False
	save_raw = False
	handle = {}
