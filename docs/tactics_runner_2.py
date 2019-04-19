import math


angle_table = { 0:   0     ,
                10:  0     ,  350: 0    ,  
                20:  0     ,  340: 0    ,
                30:  0     ,  330: 0    ,
                40:  0     ,  320: 0    ,
                50:  0     ,  310: 0    ,
                60:  1.2   ,  300: 1.2  ,
                70:  1.4   ,  290: 1.4  ,
                80:  1.6   ,  280: 1.6  ,
                90:  1.7   ,  270: 1.7  ,
                100: 1.75  ,  260: 1.75 ,
                110: 1.8   ,  250: 1.8  ,
                120: 1.8   ,  240: 1.8  ,
                130: 1.8   ,  230: 1.8  ,
                140: 1.8   ,  220: 1.8  ,
                150: 1.75  ,  210: 1.75 , 
                160: 1.6   ,  200: 1.6  , 
                170: 1.55  ,  190: 1.55 ,
                              180: 1.45 }
                    

abs_self_head = 0
abs_wind_head = 0
abs_targ_head = 0

# Controls how much we tack. 
# It's (hopefully) proportional to our acceptable XTE.
hysteresis = 0.5


def best_direction(abs_self_head, abs_wind_head, abs_targ_head, distance_to_target):

  best_head = abs_self_head
  best_vel = 0

  for i in range(0, 360, 10):

    vel = potential_VMG(i - abs_wind_head, i - abs_targ_head)
    if abs(i - abs_self_head) > 20: vel *= hysteresis

    if vel > best_vel:
      vel = best_vel
      best_head = i

  return best_head


def potential_VMG(angle_to_wind, angle_to_targ):
  vmg = math.cos(angle_to_targ*math.pi/180.0) * angle_table[round(angle_to_wind, -1)] 

