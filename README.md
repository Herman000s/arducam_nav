# arducam_navigation

camera_script.py still under development!
Currently experiences occasional USB_DATA_LEN_ERRORs when calling self.odo.process() -> if calls to self.odo.process() is prevented then 
program runs fine with only couple of errors occurring after switch 1 (re-run program) is pressed.

Camera script version 1.2:

Code structure changed from 3 threads to 2 threads to prevent/reduce possible race conditions between threads:
 - Image capture and image read now in the same thread.

Camera parameters are updated to match with lens used with AR0135.

Old images are now deleted from images-folder, if exists, during startup.

Added functionality to create meta-folder if not already found on the target board.
Added functionality to remove old/create new metadata.json file during startup:
 - Even though images are removed each time when new round is started (program re-run with switch sw1),
   this is not the case with metadata -> new data is appended to the existing metadata.json file!
  
Metadata that is being collected currently:
 - Timestamp for a given round.
 - sensory metadata for a given round, including:
   - Image number.
   - Dark current levels.
   - Auto Exposure mean value.
   - Auto Exposure coarse integration time.
   - Analog/digital current gains.
 - Number of USB_CAMERA_DATA_LEN_ERRORs occurred for a given round -> need to change the name to some general usb error and read the corresponding error value.
 
 TODO:
 - Add ret[0] (pose estimate) value of each image returned from the process() function to metadata.
 - Calculate and add camera sensor temperature estimate to metadata.
 - Something else that needs to be added to metadata?
 - To prevent possible filesystem/data corruptions, add secure shutdown procedure when program terminates!

Camera .cfg version 1.2:

To reduce/prevent occasional USB_CAMERA_DATA_LEN_ERRORs from occuring (caused by race conditions between threads after program is re-run??),
values of registers 0x3030 (PLL_MULTIPLIER) and 0x300C (LINE_LENGTH_PCK) were modified:
 - Register 0x300C value changed from 0x06C2 (decimal 1730) to 0x157C (decimal 5500) -> drop in fps from 30 to 10.
 - Register 0x3030 value changed from 0x0022 (decimal 34) to 0x0012 (decimal 18) -> drop in fps from 10 to 5.
 
Note that if the value of register 0x3030 is reduced too much, the camera "freezes" with error value of 65318 (0xFF26, USB_CAMERA_TIMEOUT_ERROR) -> to recover from
this scenario, camera needed to be first powered off and the back on.
 - Values that were used at the time when this error occurred was 0x157C for register 0x300C and 0x000C for register 0x3030!

With above setup, usb data errors have not been recorded during several program re-runs (except 1 usb data error that always occurs when program starts!). 
Note that even when register value of 0x3030 is icreased and errors at some program re-run start to appear, images can still be processed at a relatively good pace
due to the lower fps caused by the value in register 0x300C.
