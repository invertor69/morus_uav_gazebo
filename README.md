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