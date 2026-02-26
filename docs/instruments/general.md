# Instruments and Manipulation topics

## Robotic Arm

### 1. Absolute encoders
Each joint of the robotic arm in equipped with absolute encoders, in particular the AS5600, which provides reading of the angle ot the joint.
This sensors work with the Hall effect, so they are contactless. The reading from the sensors goes to a microcontroller, and the sensors gives the reading in I2C protocol.
On the Hardware side, is important to provide stable readings from the sensors, especially for autonomous operation, and to make that is necessary to avoid Electromagnetic Interference (EMI) from other components on the rover, and  to avoid mechanical movements on the sensors.
Right now, i think the absolute encoders are the most important sensors on the robotic arm, because they provide the position of the arm. Is really important to give to the cables a good path, in order to avoid limits on the mechanical movement of the arm.

### 2. PH-sensors
This sensor is necessary just for one task, the Astro-Bio. For this sensor we have to work just on the calibration, in order to have precise readings.
The readings are actually pretty good right now, but is always possible to improve them.
From the mechanical side, we need phalanges made only for this sensor.
Every time the sensor is used, we have to clean with demineralized water, and we must be very careful to not damage the sensor, because is very fragile.
To measure the ph, we have to insert the sensor in the soil, and wait some time to have a stable reading.

### 3. Pressure sensors
They are in general four, two for each phalanx. The idea is to use them in a ”switch configuration”, so when the pressure is higher than a certain threshold, we can assume that the phalanx is in contact with a surface. This should be very good for autonomous operation, because with the camera
there is a blind spot, so these sensors may be really important to add knowledge about the grip we have (example, I am taking the object only with the top of the phalanx, so I need to move the arm a little bit forward to have a better grip).
The code is really simple, and there is also a part for reducing noise.
On the mechanical side, we have to be sure that the sensors are well fixed, and that they do not move when the phalanx is moving, because we tested that if the sensor moves, the readings could be over the threshold just for the movement, and not because of a real contact with a surface.

### 4. Load cell
We have three load cells on the rover, one on the drill, one on the surface, and one on the robotic arm.
We use them to weigh the samples that we take. The sensors is made of an ”iron bar”, the actual load cell, and a chip, the HX711, which is necessary to read the signal from the load cell. We use a library called HX711 for the reading, we just need to calibrate the sensor with known weights (already done with precise readings).
On the mechanical side, we have to be sure that the load cell is in a position in which could read the weight correctly, so we have to avoid that other forces act on the load cell (example, if the load cell is not perfectly horizontal, the weight will create a torque on the load cell, and the reading will be wrong).

### 5. Cameras
An other essential sensor for the robotic arm are the cameras. We have two cameras on the End-Effector, one is a real sense camera, and the other is a
PCB camera, and they are obviously necessary for the visual feedback of the arm, in both manual and autonomous operation.
We also have other cameras on the arm, one on the elbow, one on the forearm, but they are obviously less important.

### 6. IMU
Is right now on the project but never used. Is situated in the End-Effector. Is a sensor able to read the acceleration and the angular velocity of the End-Effector, so is a six degrees of freedom sensor.
We thought to use it to remove the absolute encoders for the last two joints, because the IMU does not have extra cables, cables that are a problem in the movement of the wrist joint.
We should try it right now, to see if the readings are good enough to replace the absolute encoders.

### 7. End stop
The easiest sensors on the robotic arm is the end stop. It’s just a switch that gives us information if the End-effector is opened at the maximum.
We use it because every time the rover is turned-on, we have to calibrate the position of the End-Effector, so we move it until the end stop is pressed, and we set that position as ”fully opened”.
Is important so we don’t have to be careful with the position of the End-Effector when we turn on the rover.

### 8. Incremental encoder
Is the encoder that reads the width of the gripper. We start to max width every time we turn-on the rover, an the we close the phalanges knowing the number of step necessary to close them.
Here the problem is the position and stability of the encoder, but we are already solving with a small PCB.

## Drill and Surface

### 1. Load cell
The same I explained for the robotic arm. One is for the surface, and one is for the drill.
The one in the drill is way more complicated to mount, because we have to be sure that the load cell reads only the weight of the sample, and not other forces. In this system we are working on now type of weigh sensors, but we are waiting for the new load cells to arrive.

### 2. Cameras
Here we have just the PCB cameras, and we use them to look at the soil while drilling, to have a visual feedback of the process.

### 3. Incremental encoders inside the motor
To have a control system, we use the Incremental encoders inside the motors. We have a dual loop control, one a PI, and the other a PD. They are inside the motor, so we need just to connect the cables for the readings.