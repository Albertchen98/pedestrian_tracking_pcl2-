#boolean if pedestrian is visible in current frame (=1) or is occluded/disappeared in current frame (=0)
bool pedestrian_visible

# randomly given ID of pedestrian
int16 id

# coordinates of the centroid (Median)
float32 x
float32 y

#coordinates of the bounding box center point
geometry_msgs/Point bb_center

#size of bounding box
geometry_msgs/Point bb_size

#time stamp of the Point Cloud
int64 seconds
int64 nanoseconds

# velocity of each pedestrian
geometry_msgs/Point direction

# uncertainty of each pedestrian
float32 size
