
The viteCtrl module is a verbatim copy of the Hersch's code already available in the iCub public repository
and whose documentation page can be accessed here: http://eris.liralab.it/iCub/dox/html/group__icub__reaching__module.html .
Differently from the original code this version accepts the desired pose in the iCub standard convention, i.e. the 
root reference frame is attached to the waist of the robot; moreover, the orientation is expressed relying on the
axis/angle notation in order to harmonize the representation as done with other controllers.

Launch in this order (to be used with robot only):
"./velocityControl --robot icub --part right_arm --period 10"
"./viteCtrl --file <path_to_conf>/viteCtrl.ini"

The input port /viteCtrl/target:i now accepts the target as a 7-components vector: [x,y,z,ox,oy,oz,theta]; the translational
part is given in [m], the rotational part in axis/angle representation.

In simulation mode you will be required to launch the simVelCtrl module that replaces the velocityControl;
so please have a look at its documentation.



