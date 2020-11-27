# VelocityPlanning_DMP_FL
Analysis of a trajectory planning algorithm, with velocity characterization on the basis of the path geometry. The reference velocity is computed through Fuzzy Logic, while the execution is performed with the use of Dynamical Movement Primitives.

This thesis work uses a Fuzzy Logic library available at: https://github.com/carmelgafa/ml_from_scratch/tree/master/fuzzy_inference/fuzzy_system

While the re-adapted DMP framework is based on: https://github.com/studywolf/pydmps/tree/master/pydmps

The proposed work is based on the attached master thesis (PDF), which uses a physical setup composed of a Franka Emika Panda robot and a Makita DCG180. Both the STL files for 3D printing the flange and the Arduino code are available here. 

To run the simulation:
  
  1 - Load the code on the Arduino board and fix in the code the Serial communication (baud: 115200, COM port depends on the workstation used)
  
  2 - Place the robot in the same position of the offset (this avoid an initial discontinuity, since the robot trajectory starts in the origin)

  3 - Choose one path in the "path_generation" folder and run the corresponding code (this will generate a .npz file which is the reference path in XY coordinates)
  
  4 - Run "MAIN_standard.py" (this code requires the download of the Fuzzy Logic library from the above source).
    
If everything is working, the robot should move along the taught trajectory with a variable speed as function of the path geometry. The Arduino should command the gun trigger with a PWM technique. 
