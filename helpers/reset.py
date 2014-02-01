import serial

ser = serial.Serial("/dev/tty.usbmodemfa131", 1200)
ser.open()
ser.close()
