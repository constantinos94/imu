# imu
Inertia measuring units tracking algorithm
IMU DEMO



The Sensor Fusion and Tracking Toolbox™ in MATLAB enable you to track
orientation, position, pose, and trajectory of an object. An object's
orientation describes its rotation relative to some coordinate system,
sometimes called a parent coordinate system, in three dimensions.



Types of Sensors



For orientation estimation, three types of sensors are commonly used:
accelerometers, gyroscopes and magnetometers. Accelerometers measure proper
acceleration. Gyroscopes measure angular velocity. Magnetometers measure the
local magnetic field. Different algorithms are used to fuse different
combinations of sensors to estimate orientation.



More sensors on an IMU result in a more robust orientation estimation.
The sensor data can be cross-validated, and the information the sensors convey
is orthogonal.



Compensating for Hard Iron Distortions



Fusion algorithms use magnetometer readings which need to compensate for
magnetic distortions such as hard iron distortion. Hard iron distortions are
produced by materials which create a magnetic field, resulting in shifting the
origin on the response surface. These distortions can be corrected by
subtracting the correction values from the magnetometer readings for each axis.
In order to find the correction values, the following must be done:



1. Rotate the sensor from 0 to 360 degree around each axis.



2. Obtain the minimum and maximum magnetometer readings.



3. Average the minimum and maximum readings to get the correction values
for each axis.



Algorithms

ecompass –– Fuse
accelerometer and magnetometer readings



The ecompass function
fuses accelerometer and magnetometer data. This is a memoryless algorithm that
requires no parameter tuning, but the algorithm is highly susceptible to sensor
noise.



imufilter ––
Fuse accelerometer and gyroscope readings



The following objects estimate orientation using either an error-state
Kalman filter or a complementary filter. The error-state Kalman filter is the
standard estimation filter and allows for many different aspects of the system
to be tuned using the corresponding noise parameters. The complementary filter
can be used as a substitute for systems with memory constraints and has minimal
tuneable parameters, which allows for easier configuration at the cost of finer
tuning.



ahrsfilter ––
Fuse accelerometer, gyroscope, and magnetometer readings



An attitude and heading reference system (AHRS) consist of a 9-axis
system that uses an accelerometer, gyroscope, and magnetometer to compute
orientation. The ahrsfilter and complementaryFilter System objects™ combine the best
of the previous algorithms to produce a smoothly changing estimate of the
device orientation, while correctly estimating the direction of north.
The complementaryFilter uses the same complementary filter
algorithm as before, with an extra step to include the magnetometer and improve
the orientation estimate.



Position
estimation



 



 



 



 





Integration of the
gyroscope measurements provides information about the orientation of the
sensor. After subtraction of the earth’s gravity, double integration of the
accelerometer measurements provides information about the sensor’s position. To
be able to subtract the earth’s gravity, the orientation of the sensor needs to
be known. Hence, the estimation of the sensor’s position and orientation are
inherently linked when it comes to inertial sensors. The process of integrating
the measurements from inertial sensors to obtain position and orientation
information often called dead-reckoning.



If the initial pose would
be known, and if perfect models for the inertial sensor measurements would
exist, the process illustrated in the schematic would lead to perfect pose
estimates. In practice, however, the inertial measurements are noisy and
biased. Because of this, the integration steps from angular velocity to rotation
and from acceleration to position introduce integration drift. The use of
Madgwick algorithm ha



An attempt to correct the
drift was achieved using a 4th order Butterworth filter. Tuning at the
cut off frequencies between 1 and 10 Hz the accumulating error of positional
data has been significantly corrected. The data does not drift away from the
origin, retaining the original position it had when started moving.
