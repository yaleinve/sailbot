#!/bin/bash
rostopic pub /sails_rudder_pos sails_rudder/SailsRudderPos "{mainPos: $1, jibPos: $2, rudderPos: $3}"

