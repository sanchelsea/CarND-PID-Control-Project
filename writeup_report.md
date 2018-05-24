**PID Controller** 

---

**PID Controller Project**

The purpose of this project was to "build a PID controller and tune the PID hyperparameters by applying the general processing flow as described in the lessons," and to "test your solution on the simulator!" The simulator provides cross-track error (CTE), speed, and steering angle data via local websocket. 

The goals / steps of this project are the following:
* Build a PID controller to steer a simulated car around a virtual track. 
* Tune the PID hyperparameters using manual tuning, twiddle, SGD or a combination.
* Test your solution on the simulator!
* Validate the vehicle is able to drive successfully around the track without leaving the road. Try to see how fast you get the vehicle to SAFELY go!
* Summarize the reflection with a written report



### Results / Reflection
A video of the simulated car driving around the track can be found [here.](https://github.com/sanchelsea/CarND-PID-Control-Project/blob/master/self_driving_car_nanodegree_program%205_24_2018%201_29_51%20AM.mp4)

#### Components of PID

* The "P" or the Proportional component refers to how much the car will steer in proportion(opposite) to the CTE. CTE refers to the car's distance from the lane center. If the car is to the left of the road then you want to steer right and vice versa. The higher the CTE, it would require a high steering angle. If the P coefficient is set too high then the car oscillates around the center of the lane as it will constantly overshoot and correct itself again. This is especially a problem when driving at higher speeds as the car might leave the track. If the coefficent is too low then it might go off track when turning in the curves.

* The "D" or the Differential component helps counteract the oscillation problem caused by P. This component represents the rate of change in CTE. If the CTE is changing quickly, usually observed in the curves or if the car is veering off the middle of the track drastically, we would need a bigger steering angle to correct the car's position. Here the D and the P component will add up. But during the corrective action of the car moving towards the middle, the D component will be negative and negate the effect of the P component. This avoids large steering angle and there by preventing the car from overshooting the middle of the lane. Low D coefficent wont help counter oscillations. 

* The "I" or the Integral component counteracts the bias such as steering drifts which prevents the PD controller from reaching the middle line. This component sums up the CTEs up to that point. If the car has drifted to the left of the road for a while, this component will help bring it back to the center. High I coefficent tends to cause oscillations as it increases the steering value. Low coefficent will not be able to remove the bias.


#### Hyperparameter Tuning

The hyperparameters were tuned manually. I did try to use Twiddle to compare the performance but I wasnt successful in getting a good consistant set of values from twiddle.
I started manual tuning by using just the P term. I started with 1 which resulted in high oscillations and eventually the car crashed when the speed increased. I then continued to lower the P term, to reduce the oscillations. Using very low P values would resulted in the car going off the track in the curves. 
I soon realised that I wont be able to drive the car around the track at a speed of 30MPH using just the P term. So I reduced my throttle in order to set the inital value of P term to 0.2. The goal was to make sure the P component was high enough to correct the car in the curves.
With the P term set to 0.2, I started tuning the H term starting with 1 and increasing it till the oscillation motion was eliminated. Again this was very subjective to the throttle value used. In order to drive successfully around the track at a speed of ~30MPH, I used a value of 3.0 for the H component.
I left the I term to 0 as it is a simulator and I assumed there wont be any steering drifts.

Once I was able to complete successful laps around the track at 30MPH, I started to increase the throttle value to achieve higher speeds.
I soon observed the car to oscillate and crash at higher speed. So I started playing around with both P and H terms. Increasing H term to reduce oscillations and decresing P in some proportions in order for a smoother ride.
I also implemented a braking system based on the CTE value.i.e reducing the throttle if the CTE exceeds 0.9. 

So with hyperparameters of 0.15(P), 0.0(I), 5.5(H), I was able to drive around the track smoothly at 60MPH.