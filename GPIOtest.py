import Adafruit_BBIO.GPIO as GPIO

GPIO.setup("P8_07", GPIO.IN)
GPIO.cleanup()
#GPIO.add_event_detect("P9_10", GPIO.RISING)

while True:
    if GPIO.input("P8_07"):
        "HIGH"
    else:
        print "LOW"

