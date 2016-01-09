#!/bin/bash
rostopic pub /airmar_data airmar/AirmarData '{lat: '$1', long: '$2'}'
