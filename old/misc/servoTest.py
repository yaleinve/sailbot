import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.PWM as PWM
import time

servo_pin = "P8_13"
ADC.setup()
flag = True;
dutyCycle = 3; #ranges from 3 to 14.5
PWM.start(servo_pin, dutyCycle, 50) #args = pin, dutyCycle, frequency, pwmPolarity
numBins = 8
running_avg = [0] * numBins


def rotate(l,n):
    return l[-n:] + l[:-n]

while True:
	ADC.read("P9_40")
	input = ADC.read("AIN1")
	# print "Input: %f" %(input)
	output_end = 12
	output_start = 3
	input_end = .20
	input_start = .10
	output  =  (input - input_start) / (input_end - input_start) * (output_end - output_start) + output_start
	running_avg[4] = output
	output = reduce(lambda x, y: x + y, running_avg) / len(running_avg)
	running_avg = rotate(running_avg,-1)
	PWM.set_duty_cycle("P9_14", output)
	print "%.2f" % float(sum(running_avg)/len(running_avg))
        time.sleep(.1)
