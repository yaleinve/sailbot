#!/bin/bash
rostopic pub /competition_info captain/CompetitionInfo '{comp_mode: 'RoundAndReturn', angle: 180, gps_lat1: 2, gps_long1: 1, xte_min: -99, xte_max: 99}'
