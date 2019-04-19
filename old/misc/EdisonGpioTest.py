import mraa

AUTONOMOUS_SELECT_PIN = int(raw_input("Enter a pin number: "))
value = int(raw_input("Enter a value to write: either 0 or 1: "))

print ("writing a value of " + str(value) + " to pin " + str(AUTONOMOUS_SELECT_PIN))

auto_select_pin = mraa.Gpio(AUTONOMOUS_SELECT_PIN)
auto_select_pin.dir(mraa.DIR_OUT)
auto_select_pin.write(value)

