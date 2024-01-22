This package provides the service to use amcl on Toyota HSRB robot.

---------------------------------------------------
For simulation:

Use the following command to make the scripts executable:
chmod +x src/scripts/loc_service_sim.py
chmod +x src/scripts/localization_amcl2.py

The following script will run the lauch file launch/amcl_hsrb_sim.launch to launch a ROS node that provides the localization service:
rosrun hsrb_amcl loc_service_sim.py

The map file is provided by tmc_wrs_gazebo_world package.

Call the service to start the localization process:
rosservice call /localization

---------------------------------------------------
For real robot:

Use the following command to make the scripts executable:
chmod +x src/scripts/loc_service_realworld.py
chmod +x src/scripts/localization_amcl.py

The following script will run the lauch file launch/amcl_hsrb_realworld.launch launch a node that provides the localization service:
rosrun hsrb_amcl loc_service_realworld.py

The map file should be put in hsrb_amcl/launch/robot-map folder (Not provided by the package).

Call the service to start the localization process:
rosservice call /localization


------------------------------------------------------------------
Notes:

When it print "finish" in the terminal, the localization is done. Then you can kill the node in terminal. Otherwise, the model will keep shaking because of the localization.


-----------------------------------------------------------------
EXPLAIN ABOUT THE PROTOTYPE LAUNCH FILE: amcl_hsrb.launch

1.Launch the map_server node to publish the map.yaml as "/amcl/map", 
if your map path is different, you can change the args.

2.Launch the move_base node, so that we can call clear_costmap later.

3.Launch the amcl node. The Subscribe topic and amcl parameter are assigned here.

4.Launch localization_amcl.py. In localization_amcl.py, "global_localization" will be called firstly, 
and when loc_acc is lower than 0.05, it will stop and call "clear_costmaps"

5.Launch hsrb_move.py to let robot rotate. I try to implement this part in localization_amcl.py,
but in simulation there are something wrong, so I separated the ratating parts in a new python file.
I give a sufficient value for localization to converge. If it can not converge, you can also use ihsrb or change the value.