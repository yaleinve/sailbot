import Adafruit_BBIO.ADC as ADC
import time
ADC.setup()

t = 0

while(1): 
  t = time.time()
  value = ADC.read("AIN0")

  value = ADC.read("AIN0")
  # print value
  # continue

  epsilon = .006
  
  if(abs(value - 0.077) < epsilon):
    print("West : ~ 270 degrees bearing")
  elif(abs(value - 0.131) < epsilon):
    print("North West : ~ 315 degrees bearing")
  elif(abs(value - 0.189) < epsilon):
    print("West North West : ~ 292.5 degrees bearing")
  elif(abs(value - 0.231) < epsilon):
    print("North : ~ 0 degrees bearing")
  elif(abs(value - 0.311) < epsilon):
    print("North North West : ~ 337.5 degrees bearing")
  elif(abs(value - 0.381) < epsilon):
    print("South  West : ~ 225 degrees bearing")
  elif(abs(value - 0.411) < epsilon):
    print("West South West : ~ 247.5 degrees bearing")
  elif(abs(value - 0.546) < epsilon):
    print("North East : ~ 45 degrees bearing")
  elif(abs(value - 0.601) < epsilon):
    print("North North East : ~ 22.5 degrees bearing")
  elif(abs(value - 0.716) < epsilon):
    print("South : ~ 180 degrees bearing")
  elif(abs(value - 0.758) < epsilon):
    print("South South West : ~ 202.5 degrees bearing")
  elif(abs(value - 0.818) < epsilon):
    print("South East : ~ 145 degrees bearing")
  elif(abs(value - 0.875) < epsilon):
    print("South South East : ~ 167.5 degrees bearing")
  elif(abs(value - 0.908) < epsilon):
    print("East : ~ 90 degrees bearing")
  elif(abs(value - 0.917) < epsilon):
    print("East North East : ~ 67.5  degrees bearing")
  elif(abs(value - 0.935) < epsilon):
    print("East South East : ~ 112.5 degrees bearing")
  print time.time() - t
