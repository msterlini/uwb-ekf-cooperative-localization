# Plotjuggler settings

Contains the settings file for the third party node "plotjuggler" used to better plot data from the topics both from bag file and in real time.
Importing one of these layout files will allow the user to plot with ease. It is also possible to manually import ROS bagfiles. 

- `track_robot4.xml`: Importing this layout file will make the software subscribe to the localization and ground truth topics of robot4 in order to see in real time the real state variables and the estimated ones
- `target_estimate.xml`: Plots x,y and z target's coordinate estimate for each robot involved
- `localization_all.xml`: Plots the x,y estimation of each one of the robots involved