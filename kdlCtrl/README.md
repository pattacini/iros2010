
The kdlCtrl module implements the traditional Damped Least-Squares (DLS) for the inversion of the
kinematics, coupled through the gradient projection method (GPM) to a secondary task that accounts for
the joints angles limits. Details about the use of the DLS algorithm with the GPM can be found in 
reference [15] within the paper, whereas the specific gradient law designed to avoid joints bounds
is described in [16].
Moreover, a simple algorithm is employed to modify at run-time the value of the DLS damping factor:
iteration by iteration, its value is increased/decreased by given gains depending on the ratio between
the current and the previous distance from the target. If the DLS is behaving well moving towards the target,
this ratio is smaller than 1 and we can decrease the damping factor; conversely, if the ratio is greater
than 1, we increase the damping factor. Of course the damping factor is saturated on a specified interval.
At each step the minimum singular value of the Jacobian is also computed and in case it is smaller than a 
given threshold, then the damping factor is suddenly brought to the maximum value.

Launch:
"./kdlCtrl --from <path_to_conf>/kdlCtrl.ini"

Input ports are:
/kdlCtrl/xd:i - accepts the target as a 7-components vector: [x,y,z,ox,oy,oz,theta]; the translational part is
                given in [m], the rotational part in axis/angle representation

Output ports are:
/kdlCtrl/v:o - outputs the velocity profiles [deg/s]
/kdlCtrl/q:o - outputs the joints configuration as result of the integration of the velocities [deg]
/kdlCtrl/x:o - outputs the current end-effector pose: [x,y,z,ox,oy,oz,theta]


Notes:
Both the resulting velocities and the joints are augmented with the torso values (kept constantly equal to 0)
so that the joints configuration ouput can be directly sent to the arm displayer iKinArmView.


Configuration parameters:
- name          <string>: the tag defining the controller YARP port stem-name
- period        <int>   : the period in milliseconds of the P controller
- joints        <int>   : the number of controlled joints (should be 7)
- simulation    <string>: it can be on/off
- robot         <string>: the robot name to connect to when simulation is off
- lambda_min    <double>: the minimum value for the damping factor
- lambda_max    <double>: the maximum value for the damping factor
- lambda_dec    <double>: the decreasing gain for the damping factor
- lambda_inc    <double>: the increasing gain for the damping factor
- sv_thres      <double>: the minimum allowable singular value under which to keep the
                          damping factor equal to lambda_max
- gpm           <string>: it can be on/off: enable/disable the secondary task
- K             <double>: the gain for the secondary task (ref. [16])
- safeAreaRatio <double>: the joint range where the influence of the secondary task is off (ref. [16])
[joint-dependent sections]
- min      <double>: the minimum value of the joint angle [deg]
- max      <double>: the maximum value of the joint angle [deg]
- q0       <double>: the initial value of the joint angle [deg]



