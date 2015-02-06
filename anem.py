import Adafruit_BBIO.GPIO as GPIO
import time

'''
def onRisingEdge(channel):
   print("rising edge!")
   #print("time between leading edge: %f miliseconds.", t - time.time())


GPIO.setup("P8_07", GPIO.IN)
GPIO.add_event_detect("P8_07", GPIO.RISING, callback = onRisingEdge)
while(1):
    t = time.time()
    #GPIO.wait_for_edge("P8_07", GPIO.RISING)
    time.sleep(20)
   ''' 

  

GPIO.setup("P8_07", GPIO.IN)

while(1):
  if GPIO.input("P8_07"):
    print("High")
  else:
    print("Low")