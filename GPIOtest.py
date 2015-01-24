import Adafruit_BBIO.GPIO as GPIO

GPIO.setup("P8_11", GPIO.IN)
GPIO.cleanup()
#GPIO.add_event_detect("P9_10", GPIO.RISING)

while True:
    if GPIO.input("P8_11"):
        print GPIO.input("P8_11")
    else:
        print "LOW"

