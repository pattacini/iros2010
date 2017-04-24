
### INTRODUCTION
The code distributed here has been employed as part of the paper entitled "[_An Experimental Evaluation of a Novel Minimum-Jerk Cartesian Controller for Humanoid Robots_](https://doi.org/10.1109/IROS.2010.5650851)" in order to carry out a testing session among different Cartesian controllers.

In summary, the list of the tested controllers is:

#### iKinArmCtrl
It implements the minimum-jerk Cartesian controller as contribution to the community given by the authors. It is provided directly with the iCub software, hence it is not embedded with the zip, and requires the iKin library for the forward/inverse kinematics computation as well as the IpOpt package for the nonlinear optimization.

###### IMPORTANT NOTE
Currently, `iKinArmCtrl` has been declared **superseded** since a new highly configurable Cartesian interface came out after this work. However, the code of the now obsolete controller is still available under this repository path: iCub/contrib/src/superseded/iKinArmCtrl.

#### viteCtrl
It is a verbatim copy of the Hersch's module already included within the iCub software (it is called `lasaBodySchema`) with the small addition of code snippets that enable the module to accept inputs according to the standard convention of the [iCub kinematics](http://wiki.icub.org/wiki/ICubForwardKinematics).

#### kdlCtrl
It is a traditional implementation of a Cartesian controller resorting to the Damped Least-Squares algorithm coupled with a secondary task that accounts for the joints angles bounds through the gradient projection method. It employs KDL, the kinematic part of the Orocos package, a public domain tool that collects a number of features for kinematics computations that are already contained within iKin; therefore, the adoption of KDL was meant for comparison purposes.

### DEPENDENCIES
Before compiling the code you are required to install [YARP and iCub software](http://wiki.icub.org/wiki/Manual#Six._Software.2C_Installing_YARP_and_iCub).

Further dependencies are:

#### iKinArmCtrl
- [IpOpt](http://wiki.icub.org/wiki/Installing_IPOPT).
Moreover, the controller documentation is available at: http://wiki.icub.org/iCub/contrib/dox/html/group__iKinArmCtrl.html.

#### kdlCtrl
- KDL/Orocos: http://www.orocos.org/kdl together with Eigen2: http://eigen.tuxfamily.org/index.php?title=Main_Page.

### COMPILATION
The compilation makes use of the cmake tool.

### TESTED OS
- `iKinArmCtrl` : Windows and Linux
- `viteCtrl`    : Linux
- `kdlCtrl`     : Linux


### NOTES
The controllers can be launched also without the iCub, i.e. in simulation mode, displaying the results of virtual commands through the [iKinArmView](https://github.com/pattacini/icub-contrib/tree/master/matlabViewers/src/iKinArmView) GUI developed in MATLAB.

Users are supposed to be familiar with [Yarp](http://wiki.icub.org/yarp).
