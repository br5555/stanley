<h1>Stanley algorithm package</h1>
<br>
<p><li>The driver has been tested with ROS Kinetic on Ubuntu 16.04 64-bit.</li>
<br>
<p>The team of Stanford  implemented the velocity and steering controllers
separately for its vehicle “Stanley” at the DARPA Grand Challenge. The velocity
control is implemented as a PI controller, while the steering angle is computed by a
nonlinear feedback function of the cross-track error, which is known as the “Stanley
method.” The performance of this controller degrades seriously at higher speeds.</p>

