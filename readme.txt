
# AR4-MK2 Software Version 4.3.1
With tweaks to get working on linux. Tested on Ubuntu 22.04.

## Disclaimer
The code in this repository is not my work, it is by Chris Annin.
- https://github.com/Chris-Annin
- https://www.anninrobotics.com

The original source code can be found at https://drive.google.com/file/d/1VJFVFyBguJn73MbkKiAyP-Gg6n7ONtdd/view?usp=sharing

## Installation instructions

Using Source Code
	- review source code video https://youtu.be/2VGkgCKXVc0
	1. Install Python 3.10.7 
	2. Open CMD window and navigate to the folder python was installed in.
	3. Load each of the following Python modules:
		python -m pip install pyserial
		python -m pip install inputs
		python -m pip install numpy
		python -m pip install ttkthemes
		python -m pip install opencv-python
		python -m pip install Pillow
		python -m pip install auto-py-to-exe
		python -m pip install matplotlib
		python -m pip install pygrabber
		python -m pip install datetime
		python -m pip install pathlib

LINUX ONLY (ubuntu)
	4.
		Install python3-pil.imagetk (`sudo apt-get install python3-pil.imagetk`)
		Ensure user is in `dialout` group for access to USB Serial (`usermod -a -G dialout $USER`). You may need to log out and then back in for group change to be detected.

	5. Run the program with `python3 AR4.py`
	6. Under the 'Config Settings' tab, enter the full path to your Teensy and Arduino's USB serial device - e.g. `/dev/ttyACM0`. 

WINDOWS ONLY
	4. To convert ARCS.py to an EXE file python open CMD window and navigate to the folder python was installed in.
		and then install program: -m pip install auto-py-to-exe.
		Next execute the execute py to exe program by typing python -m auto_py_to_exe in cmd window.
		Use this program to create the exe files. Copy all .ico, .gif and program files from source code folder into your new exe folder otherwise exe will not work.
