# morus_uav_gazebo
Repository for developing UAV simulator based on Gazebo.

## Running the MPC simulation
```sh
  $ roscore &
  $ roslaunch morus_gazebo morus_multirotor_empty_world_ctl_mpc.launch MPC_control_run:=true
```

* With this launch file, following will be ran: 
    * _Gazebo_ simulation, 
    * height controller, 
    * attitude controller 
    * _rqt_gui_ for posting the references

In _rqt_gui_ post desired height and angles on topics:
* "/morus/euler_ref" (angles for roll-x and pitch-y)
* "/morus/pos_ref" (reference for height-z)

After the desired values are set, run:
```sh
  $ rosservice call gazebo/unpause_physics
```
to start the simulation.

## Saving plots
There is a save file called _process_bag.sh_, position in the required directory and call
```sh
  $ ./process_bag.sh "name_of_the_directory_to_save"
```

inside the .sh script define the topics wanted to save and plot later in Matlab or desired software

## Changing the optimization problem
The file in the "morus_control/include/morus_control/solve.h" needs to be changed with the newer version generated from CVXGEN software
https://cvxgen.com/docs/index.html
to support c++ compiler with the code at the beggining:
```sh
 #define SOLVER_H

 // enable the C++ compiler to compile C code
 #ifdef __cplusplus
 extern "C" {
 #endif

 /* Uncomment the next line to remove all library dependencies. */
```
and at the end:
```sh
// enable the C++ compiler to compile C code
#ifdef __cplusplus
}
#endif
```
After that change the begginings of files _ldl.c_, _matrix_support.c_, _solver.c_ and _util.c_ to include the right solver header

## Troubleshooting
If the _gazebo_gui_ fails to launch, run in another terminal:
```sh
  $ gzclient
```

If the _gazebo_ fails to launch, run in another terminal:
```sh
  $ killall gazebo gzgui gzserver gzclient
```
and restart the launch file

If nothing else works, run:
```sh
  $ rosclean purge
```
and restart everything.

--------

Contact
-------
Luka Pevec luka.pevec@fer.hr