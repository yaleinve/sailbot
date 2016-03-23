#!/bin/bash
rostopic pub /competition_info captain/CompetitionInfo '{comp_mode: 'MaintainHeading', angle: 90.0, gps_lat1: 41.312586, gps_long1: -72.924949, xte_min: -5, xte_max: 5}'
