# Robot control panel

Centre for Autonomous Systems of UTS requires a remote robot control panel for an experimental robot. 
The experimental robot has wheels to move, an arm to fetch objects, and a camera on head to perceive the environment and scan objects. 

This control panel provides a user interface which built by Qt framework. The communication between robots and control panel established by Robot Operating System (ROS). 

Robot moving functions are implemented by MoveIt planning framework, scan function is achieved by OpenCV. Besides, Rviz builds a real-time virtual robot for remotely monitor robot postures and movements. 

The main window of the control panel:
<p align="center">
  <img src="examplepics\Main.jpg" alt="Main" style="width:40%; " />
</p>
The window of Rviz simulation:
<p align="center">
  <img src="examplepics\Rviz.jpg" alt="Rviz" style="width:40%; " />
</p>  

<p>Due to the confidantial requirement, the perception function is displayed in a Gazebo simulation environment.
<p>The perception window of the control panel:
<p align="center">
  <img src="examplepics\Scan.jpg" alt="Scan" style="width:40%; " />
</p>  
The environment in Gazebo simulation:
<p align="center">
  <img src="examplepics\Environment.jpg" alt="Environment" style="width:70%;" />  
</p>
