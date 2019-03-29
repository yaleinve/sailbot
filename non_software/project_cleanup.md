Project cleanup


#### These packages seem to do what they say:
  I'm assuming 'airmar', 'captain', and 'tactics' all do something. They're all interdependent, too. 'airman_smoother' is new and therefore good as well.
  - 'servo_control' subscribes to messages on '/sails_rudder_pos' and does the work of actually rotating the sails and rudder. It would be nice and probably more roslike to split the sails and rudder into separate packages, but right now they don't have separate needs and this is fine.
  - 'speed_calculator' maybe doesn't do *exactly* what it says; it seems to be more concerned with calculating XTE. But that's worthwhile and used throughout the project.
  - sails_bagger seems to take various data and write it to a CSV file. I'm not sure how much we use that, but it could potentially be useful.


#### These packages might be eligible for deletion:
  - 'navigation' publishes to the '/target_course' topic, which is not read anywhere.
  - 'windvane' publishes to '/heading', which is not read anywhere. It's based on a hardware component "ADC" which seems like it might be radio-related (it's also referenced in radio_control_pongo). 
  - 'trim' has one runnable, 'trim_basic_runner', which publishes to '/sail_pos'. No node cares about '/sail_pos' and as far as I can tell the entire package is dead. That's worrisome, because it seems like it's supposed to control the trim of our sails under certain wind directions, and the code itself seems modern (it's in python, it uses the airmar, etc.)
  - 'vectornav' seems like it should be important, but (a) it's not our code, and (b) none of the channels it publishes to are referenced outside of the package.


#### These packages need to be cleaned up in some way:

  - tactics_runner is just bad code. One in every five lines is a `global` declaration, and it's hard to follow whether it's actually doing what it says it does. Also, our tactics are currently based totally on our cross track error (so, we only tack if we're close to the bounds.) That's a *problem*, because if the wind shifts while we're in the middle of the channel, then we'll be in irons.
  - radio_control_pongo is probably doing what it's supposed to, but there are some problems: it initializes the node and creates a new publisher every 1/5 second, and the channel it publishes to, '/radio', is actually read as 'radio' (note the namespace difference).
  - servo_manual_pub looks fine, but it's sort of misleading that it has a package all of its own. Also, not sure if we use it.


#### I have no idea on these packages:
  - common_msgs seems to be an imported package of many types of messages used frequently in ros projects. Having the whole thing in the project is unnecessary and unweildly, and maybe also significantly lengthens the build process.
  - gps_common says it "translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM." That seems like a worthy thing to do, but it publishes to the "odom" channel, which nobody seems to care about. It's also written in C++, which is not a great sign for its age. It listens to...
  - libnmea_navsat_driver, which publishes to 'fix', 'vel', and 'time_reference'. At least one of them is remapped ('fix' -> 'NavSatFix'), and I don't know enough about ros to look for that.
  - geometry and geometry_experimental seem to be doing the same thing, but they're dense libraries and I have no idea what the thing that they're doing actually is.

