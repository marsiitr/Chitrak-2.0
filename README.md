# Chitrak-2.0

![image1](https://github.com/Dhruv1064/Chitrak-2.0/blob/master/images%20and%20videos/images/testing%20on%20ground.jpg)

## **Abstract-:**

Chitrak is aimed to be a dynamically stable quadruped robot. With the hopes that it will be able to serve as a robotic cat to accompany soldiers in terrain too rough for conventional vehicles and provide aid to people during natural calamities when conventional vehicles prove to be less usable. Chitrak uses four legs for movement, allowing it to move across surfaces that would defeat wheels.
<br/>

## **Motivation:**

* The current robots being used are very expensive.
* The runtime of the current robots is very less. And on a single charge they cannot be deployed for practical use by the military. 
* Also, when standing, these robot consume energy which can be avoided to increase the runtime.
* Package delivery during Natural disasters can be faster and efficient. 
* Soldiers will be able to carry more food & ammunition during combat missions.
<br/>

## **Mechanical Aspect of the Design:**
As we are designing a fully actuated quadruped so in our first design basically we have used two motors for actuating each joint(hip and knee)of a leg. In the beginning, our main aim was to build quadruped which is both cost effective and energy efficient. So, for that purpose we used a worm geared motor which has a self locking property because of worm gear which can only be able to rotate using worm and so it cannot be back-driven. So it's main advantage will be it does not need to be operating to work against gravity in static position.

<br/>

Next placing both the motors at the hip is self beneficiary as it reduces the amount of torque hip motor has to apply to lift the legs, so  we have mounted both motors on hip. Also for power transmission for motor to  actuate knee joint we have used a timing belt pulley system (as it will have less weight and  it effectively transfers the load). Now on seeing the actual leg motion during walking we see that the upper leg is moving in the direction of gravity while lower leg against it mostly. Therefore we required larger torque to move the lower leg than upper by calculating the approximate torques we have reached on conclusion of using a 1:4 gear ratio belt pulley system so as to increase torque on knee joints

<br/>

Further we have used aluminium links for constructing it ,some 3d printed parts for joints, mountings and 3d printed pulleys. There are two parallel links that are connected through nuts and bolts to provide more stiffness and strength to the structure for upper and lower leg. Rotary encoders are used as a sensors to provide feedback about the position of legs they are mounted on hip joints coupled to each motor. For end effector we used a broad padded plate in order to provide more surface area for better traction. The two front legs are joined using 6mm threaded rod, same is for back legs and these two parts are joined with alumininum channel to construct a body on which whole electronics system is being placed.

<br/>

The problem with this design was that the Belt-Pulley mechanism failed while testing the quadruped on ground. The timing belt was slipping on the upper pulley due to load on the leg during the gait cycle. This slipping changed the angle between the upper and lower leg to an arbitrary value and the whole cycle would get disrupted. 

<br/>

In the current design, we used a lead screw mechanism to drive the lower leg. In it, an 8-cm long lead screw is placed parallel to the upper leg (thigh) and a link is attached to its nut which drives the lower leg. So, the linear motion of the nut on lead screw gets converted to rotational motion of lower leg about the knee joint. Since lead screw mechanism is self locking upto very large amount of load, it can bear the impact loading during the gait cycle and overcomes the problem faced in the previous design.

<br/>

Upper leg in actuated by the same worm-geared motor. Lead Screw is powered by a 900RPM DC geared motor to provide sufficient speed for driving the lower leg. This motor is mounted on the upper leg by a 3D printed part, 'Motor Mount' (solidworks file) containing cavities for the parallel links. Knee joint is also made with 3D printed part, 'Revolute Joint', which contains link spaces and cavities for two radial ball bearings on either side for holding the shaft. A machined Aluminium part is used to couple the lower leg to knee shaft. A Rotary Encoder is also coupled with the Knee Shaft for feedback of the angle between upper and lower leg.

<br/>

## **Electronics Aspect of the Design:**

Chitrak is powered by 4 LiPo batteries(1 for each leg) driving 8 Worm-Gear 500 RPM motors. Each leg has 2 DOF with two actuators (one for the hip joint, and one for each knee joint), for a total of 8 motors. For a single leg, there are 2 active legs joint. The knee joint is rotated by the actuator mounted at hip joint through the belt. Each actuator unit consists of a rotary encoder attached to it for angle measurement.

<br/>

In our first prototype of one leg, we first simulated the motion of the leg on Matlab. By taking the trajectory of the end-effector to be a semi-ellipse (which is the approximate trajectory of natural leg motion), we applied inverse kinematics on the end-effector by considering an X-Y reference frame on the hip joint and varied the joint angles accordingly. For smooth motion, we used Spline for trajectory planning taking the acceleration to be zero. On physical implementation, the leg worked as expected with the end-effector following an ellipse and leg mimicking the motion similar to that of a four-legged animal.

<br/>

For the trajectory, for self-coordinated movement of the four legs, we are using the method of. 2D inverted pendulum method for alternate legs. It consists of the center of mass (CoM) and massless telescopic leg. We measure the inclination of the pendulum θ from vertical, positive for counterclockwise rotation. This yields us the horizontal dynamics of the CoM. The kick force f = Mg/ cos θ always balance with the gravity acting on the point mass. Time for transfer and support leg exchange movement is easily obtained by solving the differential equations of orbital energy and force. Solving these equations helps us to generate the walking pattern generation according to the corresponding gait cycle.

<br/>

For coordinating motion of all three legs, one leg is lifted from the ground at a time and rest of legs are moving in straight line. Thus a straight line has to be broken into 3 parts, and each leg traverses in a different part of straight line while another leg is completing elliptical motion.

<br/>

## **Estimated Budget:**

The Design of Kitty is made with the main focus on lightweight, high endurance and low cost. Most of the components used in the bot are easily accessible. The current estimated cost of Kitty is around 53000 INR. The cost can be significantly reduced when the bot is manufactured commercially on a large scale. It is cheap, easy to reproduce, robust, and safe to handle. This makes it an excellent tool for research of multi-segment legs in quadruped robots.

---------------------------------------------------------
|Component 		          |	Cost(INR)       |	Total	      |
|-----------------------|-----------------|-------------|
|Worm Geared Motor (x8)	|	1552 	          |	12417	      |
|LiPo Battery(x5) 	    |	1309 	          |	6545	      |
|Rotary Encoder(x8) 	  |	1590 	          |	12720	      |
|Arduino Uno(x5) 	      |	425 	          | 2125        |
|Cytron Motor Driver(x4)|	1890 	          |	7560	      |
|PCB (x1) 		          |	1000 	          |	1000	      |
|MPU6050 (x2) 		      |	150 	          |	300	        |
|Ultrasonic Sensor (x2) |	197 	          |	394	        |
|Jumper Wires       		|	5 	            |	150	        |
|Square Channels (x20ft)|	20 	            |	400	        |
|Aluminum Links (x12ft) |	20 	            |	320	        |
|Chain & Sprocket (x4) 	|	400 	          |	1600      	|
|3D Printed Parts (x4) 	|	1500 	          |	6000      	|
|Miscellaneous 	      	|	800 	          |	1000	      |
|Total 			            |		              |	52531       |
<br/>

## **Applications:**
* Travel in rough terrains where conventional vehicles have low mobility.
* Carry payloads for soldiers in Combat operations for long duration.
* Provide aid to soldiers during combat operations without loss of human lives.
* Deploy food and medicines to people stuck in inaccessible areas during natural disasters.
<br/>

## **Limitations:**

* Obstacle detection has not been included.
* IMU has not been implemented for self balancing in rough terrain.
* Currently movement of robot is restricted to 1 dimension only.
<br/>

## **Future Goals:**

* Increase Runtime of the Robot after a single charge.
* Obtain Pace, Gallop and Canter Gait cycle for faster movement.
* Increase the Payload Capacity of the Robot.
* Provide 3 DOF to each leg providing more flexibility to the robot.
* Using Computer Vision for Obstacle detection, human recognition and corresponding Path planning.
* Provide 3 DOF to each leg providing more flexibility to the robot.   
<br/>

## **Team Members:**

* Avdesh Ranwa
* Dhruv Sehgal
* Harshil Patel
* Ujjwal Baranwal 
* Yawan Gupta
<br/>

## **Mentros:**

* Divyansh Gupta
* Nitin Yadav
* Prashant Kumar
* Vishal Singh
<br/>

## **References:**
---------------------------------------------------------------------------------
|Name				                    |	Author				                              	|
|-------------------------------|-----------------------------------------------|
|Research of Mammal BionicQuadrupedRobots 	    |Yibin Li, Bin Li, Jiuhong Ruan, and Xuewen Rong Quadruped Walking Robots at Tokyo Institute of	Technology Design, Analysis, and Gait Control Methods SHIGEO HIROSE, YASUSHI FUKUDA, KAN YONEDA, AKIHIKO NAGAKUBO, HIDEYUKI TSUKAGOSHI, KEISUKE ARIKAWA, GEN ENDO, TAKAHIRO DOI, AND RYUICHI HODOSHIMA|
|Kinematic Modeling of Quadruped Robot		      |Smita A. Ganjare, V S Narwane & Ujwal Deole	  |
|Inverse Kinematic Analysis Of A QuadrupedRobot 	  |Muhammed Arif Sen, Veli Bakırcıoğlu, Mete Kalyoncu|
|Introduction to Robotics	      |John J. Craig		                              |
|Robot Dynamics and Control	    |Mark W. Spong, M. Vidyasagar, Seth Hutchinson  |
