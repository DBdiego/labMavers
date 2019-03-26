Here it is lads!

If you replace these files in paparazzi and everything goes as planned, the paparazzi console 
should print the target coordinates on the picture, the estimated distance to the way point and the 
required rotation (in rad??) to have the target in the center.

The code is a bit messy still, because it's late and I'm tired ...

To Nickhil: The code does approx the same as yours, but I optimised it such that the process of going
			over all pixels is only performed ones instead of three times. except that nothing changed I think

I did not implement the waypoint assignment in paparazzi, so it's purely print statements in the console. 


