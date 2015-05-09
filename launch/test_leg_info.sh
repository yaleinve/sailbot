#!/bin/bash
rostopic pub /leg_info captain/LegInfo '{begin_lat: 0.0, begin_long: 0.0, end_lat: -1.0, end_long: -1.0, xte_min: -99, xte_max: 99, leg_course: 45.0}'
