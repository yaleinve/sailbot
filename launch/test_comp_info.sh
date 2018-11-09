#!/bin/bash
# Tries to sail to the CEID
rostopic pub /competition_info captain/CompetitionInfo '{comp_mode: 'SailToPoint', gps_lat1: 41.312586, gps_long1: -72.924949, xte_min: -5, xte_max: 5}'
