# LegoMindstormsEV3
Observer-based reference-following feedback controller for the Lego Mindstorms EV3 Gyroboy Segway robot, developed in MATLAB Simulink.

All files have been saved in MATLAB 2019a, and also exported to 2018a and 2017a formats.

This repository contains the project as it will appear in *"LQG controller for the LEGO MINDSTORMS EV3 Gyroboy segway robot."* by Timothy H. Hughes, Gareth H. Willetts and Jakub A. Kryczka. Submitted to the 2020 International Federation of Automatic Control World Congress.

Refer to the file **setup_guide.pdf** for further information on how to setup the Gyroboy robot itself. Each subfolder contains the necessary files for one iteration of the controller, as noted in the aforementioned paper. In order of appearance in the paper:

*	**LQR_NPC** – the basic LQR controller.
*	**PP_NPC** – the basic pole placement controller.
*	**LQG_NPC** – LQG controller without integral action.

Additional designs not featuring in the paper are as follows:
* **LQR_IA_NPC** – LQR controller without gyroscopic sensor filtering but supplemented with integral action to compensate for drift in the gyroscopic sensor.
*	**OPP_NPC** – LQR controller supplemented with an observer designed using pole placement.
*	**LQG_IA_NPC** – LQG controller without gyroscopic sensor filtering but supplemented with integral action to compensate for drift in the gyroscopic sensor.
*	**LQG_IA_WPC** – LQG controller with a pre-compensator to smooth the reference signal supplied to Gyroboy.

Alongside these controllers, the relevant files used to deduce the motor constants can be found in **MOTOR_TEST** and the files used to test the sensors can be found in **SENSOR_TEST**.

The files for supplying reference signals to the Segway robot are found in the master folder – *KeyboardControl.slx* allows for the arrows of the keyboard to be used to supply signals to the robot, while *GamepadAnalog.slx* accomplishes the same task with an Xbox One controller. These files will work with all of the above controllers.

Videos demonstrating each controller can be found in the corresponding folders. Additionally, there is a video of Gyroboy being controlled by an Xbox Controller in the master folder. The short video *gyroboy.mp4* also shows the Segway robot in action.
