#!/bin/bash
rostopic pub /servo_pos servo_control/ServoPos '{main_angle: '$1', jib_angle: '$2', rudder_angle: '$3' }'

