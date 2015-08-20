
Once the viteCtrl is running it requires a proportional controller in order to pass to it the
computed reference joints configuration that have to be translated in velocity commands.
When controlling the robot the P controller task is accomplished by the velocityControl module
located within the iCub repository; on the other hand, in simulation mode the simVelCtrl module
replaces velocityControl.

Launch:
"./simVelCtrl --from <path_to_conf>/simVelCtrl.ini"

Then type:
"yarp connect /viteCtrl/vc_command:o /simVelCtrl/qd:i"
in order to connect the output port of viteCtrl with the input port of the simulated P controller.


Output ports are:
/simVelCtrl/v:o - outputs the velocity profiles [deg/s]
/simVelCtrl/q:o - outputs the joints configuration as result of the integration of the velocities [deg]


Notes:
Both the resulting velocities and the joints are augmented with the torso values (kept constantly equal to 0)
so that the joints configuration ouput can be directly sent to the arm displayer iKinArmView.


Configuration parameters:
- name     <string>: the tag defining the P controller YARP port stem-name
- period   <int>   : the period in milliseconds of the P controller
- joints   <int>   : the number of controlled joints (should be 7)
[joint-dependent sections]
- min      <double>: the minimum value of the joint angle [deg]
- max      <double>: the maximum value of the joint angle [deg]
- q0       <double>: the initial value of the joint angle [deg]
- Kp       <double>: the proportional gain
- max_vel  <double>: the maximum velocity magnitude delivered by the controller [deg/s]



