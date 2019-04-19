
Get rid of delay between tacks





# When do we tack?

1) Headed out of bounds (either side)
2) Point is within a tack of the target
3) Point is far on the other side of the wind

```python

# The minimum angle we're able to sail into the wind. 
# About 50 degrees at the moment.
float upwind_threshold = math.pi * 0.28; 


# The amount on the other side of the wind the target has to be to trigger a tack. 
# About 25 degrees at the moment.
float target_threshold = math.pi * 0.14; 

if(beating):

  # Wind on the starboard side (right)
  if(rel_wind_bearing < 180):
    # First case: The wind has shifted so that the target is on the other side of the wind,
    # by at least target_threshold degrees. (It might still be in the wind, though.) Tack.
    if(abs(rel_target_bearing) > abs(rel_wind_bearing) + target_threshold):
      set_bearing(bearing + rel_wind_bearing + upwind_threshold)

    # Second case: We're on the same side as the target, and it's not in the wind. Sail towards it.
    if(abs(rel_target_bearking) < abs(rel_wind_bearing) - upwind_threshold):
      set_bearing

    if()
    # Third case: We're outside our XTE. Tack appropriately.
    else if(xte < xte_min) 





  # First case: The wind has shifted so that we're no longer on a useful path.
    if(rel_wind_bearing < 180): # Tack to starboard (right / clockwise)

      
    else: # Tack to port (left / counterclockwise)
      set_bearing(bearing + rel_wind_bearing - upwind_threshold)

  # Second case: We've gone outside our allowed XTE and it's time for a scheduled tack.


  # tack!
```