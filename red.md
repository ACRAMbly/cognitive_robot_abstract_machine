To record new data with red

1. Start the real Orbbec camera driver.(ros2 launch iai_tracy_bringup tracy_ros2.launch.py)
2. workon red
3. command for running sensor data recording analysis engine. it is a variation of original storage.py analysis engine described in robokudo documnetation for the recording: ros2 run robokudo_ros main _ae=storage_red
4. when recorded enough data ctr+c

To use the recorded ros2 bag data 

important: 
* please, Do not start the real Orbbec driver and recorded ros2 bag simultaneously
* when using recorded ros2 bag data do not need to change CrDescriptorFactory 

1. ros2 bag play 
    --loop
2. workon robokudo
3. run required analysis engine for experimentation any time on perception with real sensor data and without real tracy orbecc camera

