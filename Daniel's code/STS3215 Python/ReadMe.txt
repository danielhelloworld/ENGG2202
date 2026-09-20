Hi guys, This is Kelvin. Hope you are doing well with your miniprojects.

If you are working on the STS3215 projects and got stuck, you may refer to this folder.
You can either run the code inside VS code or directly in the terminal just like what we did in the workshops


Here is a guide of what the files inside this folder does:

"ENGG_1101_STS3215" is the main folder that stores all the python scripts and library packages
	Inside the folder there are 3 important python scripts:
	
	1. "change_id.py" in which you have to input the orginal and new ID:

		SCS_ID = 1            # SCServo ID : 1
		NEW_ID = 2             #Change the Servo ID

	2. "ENGG1101_2servo_write.py" where I already helped you setup ID = 1 and ID = 2 for the two servos you need
		what you need to do is just run the code and input the positions for servo 1 and servo 2
		(For your projects you would need to modify the input part to some snsor vales - ToF or camea)

	3. "read_write.py" - this is the original python code that reads and write the position for a single STS3215 servo. 
		you can take some reference from it and modify using the help of AI

Also, one IMPORTANT thing:
For each of the python scripts I mentioned above, you would need to change the BAUDRATE as well as the DEVICENAME (which is the communication port on the raspberry pi)

You would have to change these lines of code:(Set Baudrate to 115200 and check the raspberry pi & servo driver communication port using the "dmesg" command in terminal)  

	BAUDRATE   = 115200           # Must match the ESP32's serial forwarding baudrate
	# Check Raspberry Pi serial port with "dmesg" command and look for the port connectng to cp210x
	DEVICENAME = '/dev/ttyUSB1'   # Change to your port (e.g., '/dev/ttyUSB0', 'COM3')  


*** If you have any futher questions, feel free to come to the helpdesks on tuesdays and fridays from 1400 to 1700. You could also email me and ryan for more helpdesk sessions or help ***

P.S. Ryan's email: ruoyu@hku.hk ; My email: ckchancd@hku.hk

