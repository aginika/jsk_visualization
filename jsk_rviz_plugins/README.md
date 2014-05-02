jsk_rviz_plugins
=====================

jsk_rviz_plugins is extended plugins for rviz.

#Displays
##AmbientSound
###What Is This
##FootStep
###What Is This
##NormalDisplay
###What Is This

This display subscribe sensor_msgs::PointCloud2 and display the normals.
The subscribed pointcloud is assumed to have the fields x, y, z, normal_x, normal_y, normal_z(pcl::XYZRGBNormal or pcl::PointNormal).

###Sample

Plug the depth sensor which can be launched by openni.launch and run the below command

```
roslaunch jsk_rviz_plugins normal_display.launch
```

and launch the rviz

```
rosrun rviz rviz -d `rospack find jsk_rviz_plugins`/launch/rviz/normal_display.rviz
```

You can see like below display.

##PolygonArray
###What Is This

#Panels
##SelectPointCloudPublishAction
###What Is This

This will publish points which is selected by the "Select" button. Be careful to select only pointcloud.

##CancelAction
###What Is This
##PublishTopic
###What Is This
