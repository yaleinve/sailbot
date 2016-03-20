#!/bin/bash
echo "Arguments in order <main> <jib> <rudder>, in degrees"
rostopic pub /servo_pos servo_control/ServoPos '{main_angle: '$1', jib_angle: '$2', rudder_angle: '$3' }'

