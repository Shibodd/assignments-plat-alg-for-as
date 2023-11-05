# Report

First of all, results are not deterministic because ROS doesn't guarantee that nodes receive ALL messages (as in a FIFO) - it instead guarantees that nodes receive the most recent one.
So the exact values of RMSE are worthless, but they may be coarsely compared.

## Tuning the stochastic noise component

Setting the stochastic noise component to 0 results in the prediction "derailing" off the trajectory and losing the vehicle.
This cannot be fixed even by using absurdly precise sensors (variance = 1e-12).

Exagerating it to extremely high values (1e9) produces an absurd amount of jittering (to the point of rendering plotter.py's output apparently randomic), but in the real-time view it still manages to follow the trajectory.

High values like 1e3, 1e4 have comparable RMSEs.

Values between 1 and 1e2 have comparable RMSEs, too.

## Tuning the sensor variances with only one sensor "enabled"
Increasing sensor variances to absurd values (1e9) is effectively equivalent to disabling the sensor, so i'll write it like that.

Disabling the LiDAR, we can see that the RMSE nearly doubled w.r.t the default parameters, and it's apparent how the tracking has difficulties during turns. Decreasing it to absurd values (1e-9) shows very high precision when the vehicle is moving towards the sensor, but very poor precision when the vehicle is performing turns.
Increasing the heading variance to 1e9 results in the tracker being at the correct distance from the sensor but in a "random" direction.
Restoring the heading variance and increasing the radial velocity variance to 1e9 results in a slightly higher RMSE, but no particular quirk can be observed.
Increasing the range variance to 1e9, instead, makes the tracker "jump" away and then slowly catch back up with the vehicle. This is due to the high initial variance which, if lowered, removes the "jump" effect.
It is interesting to see that after restoring the original RADAR variances, decreasing the initial variance does NOT affect the results in any way.


Disabling the RADAR, we have a slightly higher RMSE. Additionally, you can see spikes in the trajectory when the prediction step overshoots the position measured by the LiDAR. Attempting to increase the LiDAR variance to smooth it out makes the tracker slightly overshoot the turns and increases the RMSE. Decreasing it to absurd values (1e-9) makes the LiDAR noise very apparent, creating spikes around the track. On the contrary of the RADAR, this effect is homogeneous all over the track, not just in turns.
Here too decreasing the initial variance does not affect the results.


## Tuning the sensor variances in pairs
Decreasing all sensor variances, we see that for values below 1e-9 we start getting practically random spikes, of which reason i can't think is anything other than numerical instability.

More sane values like 1e-5 just increase the noise and double the RMSE w.r.t the default values.

Other combinations don't really seem to be of any interest - the ideal parameters seem to be the parameters that have the lowest RMSE when using each sensor indipendently.
