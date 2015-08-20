
This module sends through a YARP port the desired pose in the operational space as it evolves in time.
The desired pose is specified by a vector of 7 components, of which the first 3 gives the translation part
in [m], whereas the last 4 accounts for the orientation that has to be given in axis/angle notation. 

Launch:
"./txTarget --from <path_to_conf>/txP2P.ini"
for the generation of point-to-point desired motion.

Launch:
"./txTarget --from <path_to_conf>/txLemniscate.ini"
for the generation of the lemniscate.

Then type:
"yarp connect /<txTargetStemName>/xd:o /<controllerStemName>/<inputTargetPort>"
in order to connect the output port of the transmitter with the input port of the controller.

Typically you'll be required to type the following:
- "yarp connect /txTarget/xd:o /iKinArmCtrl/right_arm/xd:i"
- "yarp connect /txTarget/xd:o /viteCtrl/target:i"
- "yarp connect /txTarget/xd:o /kdlctrl/xd:i"


Configuration parameters:
- name     <string>: the tag defining the YARP ports stem-name
- execTime <double>: the overall execution time for the trajectory stored in the data file [s]
- ox       <double>: the x-coordinate of the new origin where to translate the desired trajectory [m]
- oy       <double>: the y-coordinate of the new origin where to translate the desired trajectory [m]
- oz       <double>: the z-coordinate of the new origin where to translate the desired trajectory [m]
- gain     <double>: the gain factor that multiplies the trajectory (only the translational part) stored in the data file
- dataFile <string>: name of the file that contains the trajectory's data


