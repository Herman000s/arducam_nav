
import sys
import os, re, os.path
import time
import threading
import serial
import signal
sys.path.append('/home/root/ArduCAM_USB_Camera_Shield/RaspberryPi/Python/Streaming_demo')
import ArducamSDK
from periphery import GPIO

import globals
import ardu_camera
import image_processing
import rpi_serial
import logging

import faulthandler

faulthandler.enable()

def sigint_handler(signum, frame):
	globals.running = False
	exit()

signal.signal(signal.SIGABRT, sigint_handler)
signal.signal(signal.SIGINT, sigint_handler)
signal.signal(signal.SIGTERM, sigint_handler)


def start_threads(rpi_obj, camera_obj, odo_obj):
	ct = threading.Thread(target = rpi_obj.collect_data)
	rt = threading.Thread(target = camera_obj.read_image)
	pt = threading.Thread(target = odo_obj.process_img)

	ct.start()
	rt.start()
	pt.start()
	#print("Number of threads after start: ", threading.activeCount())

	ct.join()
	rt.join()
	pt.join()


def port_setup(message):
	port = serial.Serial("/dev/ttyPS1", baudrate=9600, timeout=1.0)
	port.write(message.encode())
	port.close()


def check_folders():
	if not os.path.exists(globals.image_path):
		os.makedirs(globals.image_path)

	if not os.path.exists(globals.log_path):
		os.makedirs(globals.log_path)

	if not os.path.exists(globals.metadata_path):
		os.makedirs(globals.metadata_path)


def remove_old_files():
	if os.path.getsize(globals.image_path) != 0:
		print("Removing old images..")
		for root, dirs, files in os.walk(globals.image_path):
			for file in files:
				os.remove(os.path.join(root, file))

	if os.path.getsize(globals.metadata_path) != 0:
		print("Removing old metadata..")
		for root, dirs, files in os.walk(globals.metadata_path):
			for file in files:
				os.remove(os.path.join(root, file))

	if os.path.exists(globals.log_file_path):
		print("Removing old log file..")
		os.remove(globals.log_file_path)

	if not os.path.exists(globals.log_file_path):
		print("Creating new log file!")
		open(globals.log_file_path, "a")

	if os.path.exists(globals.rpi_file_path):
		print("Removing old rpi data file..")
		os.remove(globals.rpi_file_path)

	if not os.path.exists(globals.rpi_file_path):
		print("Creating new rpi data file!")
		open(globals.rpi_file_path, "a")

	if os.path.exists(globals.visnav_log_file_path):
		print("Removing old visnav message log..")
		os.remove(globals.visnav_log_file_path)

	if not os.path.exists(globals.visnav_log_file_path):
		print("Creating visnav message log!")
		open(globals.visnav_log_file_path, "a")


def clear_round_based_data():
	print("Clearing round-based images..")
	for root, dirs, files in os.walk(globals.image_path):
		for file in files:
			os.remove(os.path.join(root, file))

	print("Clearing round-based metadata..")
	for root, dirs, files, in os.walk(globals.metadata_path):
		for file in files:
			os.remove(os.path.join(root, file))


if __name__ == "__main__":

	globals.initialize()
	config_file_name = "/home/root/ArduCAM_USB_Camera_Shield/Config/USB2.0_UC-391_Rev.D/DVP/AR0135/AR0135_MONO_custom.cfg"

	if not os.path.exists(config_file_name):
		print("Config file does not exist!")
		exit()

	check_folders()
	remove_old_files()

	camera_obj = ardu_camera.Cam()
	odo_obj = image_processing.Visualodo()
	rpi_obj = rpi_serial.Rpicomms()

	globals.gpio_led_out = GPIO(953, "out")
	globals.gpio_led_value = False
	gpio_sw_in = GPIO(957, "in")

	logging.basicConfig(filename=globals.visnav_log_file_path, format='%(message)s', level=logging.DEBUG)

	logging.info('Initializing visual odometry setup..')
	odo_obj.odo_setup()
	logging.info('Initializing arducam setup..')
	camera_obj.init_camera(config_file_name)
	ArducamSDK.Py_ArduCam_setMode(globals.handle, ArducamSDK.CONTINUOUS_MODE)

#	rpi_obj.setup()
#	rpi_obj.start_comms()
	run_threads = False

	start_time = time.time()
	wait_user_bpress = 60
	start_main = False
	
	print("Waiting for button press to start program execution!")
	globals.gpio_led_out.write(True)

	while True:
		wait_time = time.time()
		elapsed_time = wait_time - start_time
		gpio_sw_value = gpio_sw_in.read()
	
		if gpio_sw_value:
			print("sw1 pressed, starting main program execution!")
			time.sleep(1.5)
			globals.gpio_led_out.write(False)
			rpi_obj.setup()
			rpi_obj.start_comms()
			run_threads = True
			start_main = True
			break

		if elapsed_time > wait_user_bpress:
			print("Failed to press sw1, user did not start main program, terminating..")
			message = "q"
			port_setup(message)
			globals.running = False
			rtn_val = ArducamSDK.Py_ArduCam_close(globals.handle)
			logging.info('Failed to press sw1, closing program execution..')

			if rtn_val == 0:
				print("Device close success!")
				logging.info('Camera closed successfully!')
			else:
				print("Device close fail!")
				logging.info('Could not close camera!')

			start_main = False
			break
			

	start_time = None
	wait_time = 20

	while start_main:

		if run_threads:
			print("Starting threads..")
			logging.info('Starting threads..')
			start_threads(rpi_obj, camera_obj, odo_obj)
			globals.gpio_led_out.write(True)
			rtn_val = ArducamSDK.Py_ArduCam_close(globals.handle)
			print("About to close program execution..")
			logging.info('About to close program execution..')

			if rtn_val == 0:
				print("Device close success!")
				logging.info('Camera closed successfully!')
			else:
				print("Device close fail!")
				logging.info('Could not close camera!')

			print("Waiting for bpress in main loop..")
			logging.info('Waiting for button press in main loop..')
			start_time = time.time()
			run_threads = False
		else:
			current_wait_time = time.time()
			elapsed_wait_time = current_wait_time - start_time
			gpio_sw_value = gpio_sw_in.read()

			if gpio_sw_value:
				print("sw1 pressed")
				logging.info('sw1 pressed!')
				time.sleep(1.5)
				clear_round_based_data()
				camera_obj.init_camera(config_file_name)
				ArducamSDK.Py_ArduCam_setMode(globals.handle, ArducamSDK.CONTINUOUS_MODE)
				message = "c"
				port_setup(message)
				globals.round_counter += 1
				logging.info('')
				logging.info('Starting new round..')
				logging.info('')
				globals.running = True
				run_threads = True
				continue

			if elapsed_wait_time > wait_time:
				message = "q"
				port_setup(message)
				print("Closing main loop..")
				logging.info('Terminating program..')
				break


	globals.gpio_led_out.write(False)
	globals.gpio_led_out.close()
	gpio_sw_in.close()

	#os.system("shutdown -h now")

