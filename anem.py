import Adafruit_BBIO.GPio as GPIO
import time

t = time.time()
highCount = 0   # number of high voltages / sec (HZ)
GPIO.setup("P8_07", GPIO.IN)


while(1):
  if time.time() - t >= 1:
      print highCount
      highCount = 0
      t = time.time()

  elif(GPIO.input("P8_07")):
      	highCount += 1 