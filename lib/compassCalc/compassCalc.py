#!/usr/bin/env python

#Returns the shortest signed difference between two compass headings.
#Examples: compass_diff(359.0,2.0) = 3.0, compass_diff(2.0,359.0) = -3.0
#Breaks ties by turning to the right 
#Copied from tactics_runner.py
#Assumes head1 and head2 are both within [0,360]
#tested and working 4/17/15
def compass_diff(head1,head2):
  d = head2-head1   #raw difference
  if d >= 0.0:  #head2 is on same compass 'rotation' as head1 and ahead of head1 in a CW sense
    if d <= 180.0:  #head1 is just a little behind head2
      return d
    else:
      return d - 360 #head1 is very far behind head2, so easier to go CCW (must return negative!)
  else:  #d < 0.0, head1 is ahead of head 2 on same 'rotation'
    if d >= -180.0:  #head1 is only a little ahead of head2
      return d 
    else:
      return d + 360 #shorter to go CW to get there, so must be positive


