import Adafruit_BBIO.PWM as PWM

servo_pin = "P8_13"
duty_min = 3
duty_max = 14.5
duty_span = duty_max - duty_min

PWM.start(servo_pin, (100 - duty_min), 60.0, 1) #args = pin, dutyCycle, frequency, pwmPolarity

def rotate(angle):
  print(angle)
  duty = 100 - ((angle / 180) * duty_span + duty_min)
  print(duty)
  PWM.set_duty_cycle(servo_pin, duty)

while True:
	angle = float(raw_input("enter an angle: "))
	rotate(angle)