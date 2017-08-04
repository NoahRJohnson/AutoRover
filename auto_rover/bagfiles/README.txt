http://www.gpsvisualizer.com/

takes .csv files as input

We launch the appropriate state_estimation script, then use rosbag record -O "bagname.bag" /gps/filtered to record the filtered output in the UTM frame. Then we use the bag2csv python script to produce the .csv files which go into the visualizer.

