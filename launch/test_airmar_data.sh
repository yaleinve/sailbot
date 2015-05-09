#!/bin/bash
rostopic pub /airmar_data navigator/AirmarData '{: 'RoundAndReturn', angle: 180, gps_lat1: 2, gps_long1: 1, xte_min: -99, xte_max: 99}'
