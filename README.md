# Paparazzi - Struck + KCF

This repository contains code used and developed during the master thesis entitled: "Tracking and Following a Moving Person Onboard a Pocket Drone”.

Used operative system: Ubuntu 14.04.5 LTS

The airframes, flight plans and modules used within paparazzi UAV. The modules are to be used in the following pipeline, first comes the crop of the image(video_crop) to select the area that is of interest to the tracker, then the module for the tracker (KCF or Struck, for the pocket tracker, the stereo vision board sends the detection parameters directly to the drone) and lastly the module to draw the detections in the frames. After the vision modules, the control modules selfie_drone_altitude_controller, selfie_drone_waypoint_generator, selfie_drone_yaw_controller and selfie_drone_guided_mode_control for the Bebop and update_detection for the pocket drone are used to calculate the references which are then sent to the internal control loops of paparazzi.
