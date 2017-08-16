## Code Features

* Stabilizing drones.
* Giving commands to the drones.
* Make group of drones work together.

## How to start

<details><summary> Hardware </summary>

 1. Arduino uno.
 2. 6x optitrack flex 3 cameras
 3. syma x5c
 4. Pc
 
</details>

----------


 <details><summary> Software </summary>

 1. Optitrack motive
 2. Visual Studio

</details>

----------


<details><summary> Motive setup </summary>

 1. Open motive.
 2. Calibrate the cameras using Calibration tools.
 3. Select Rigidbodys.
 4. Set data stream to loop back in Data Stream View
</details>


----------

<details><summary> Visual Studio </summary>

 1. Open the project.
 2. Set the COM port of your arduinos in hover_vicon class by changing the value robot[i].OpenSerial(value).
 3. Set the mission for each drone using robot[i].SetMission() in hover_vicon.
 4. Hit Run.

</details>

## Code explanation
**Robot Class**

* Each Robot have his own mission which was set in hover_vicon.cpp in main function using `SetMission()`.


* `OpenSerial()` Sets the controlling Arduino COM port for the drone.
* `setIsMainRobot()` setting master or slave, setting as drone as slave will make it follow the other drone that was set using `setMainRobot()`
