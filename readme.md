## ATLAS: Large-Scale Mapping with Multiple Maps



### User Manual

#### Phase 0: Prepare the environment

1. Download *hdl_graph_slam* and *hdl_localization* into *catkin_ws/src*
2. Override sources in *hdl_localization* with files in this repo. 
3. Configure dependencies and run *catkin_make*.

#### Phase 1: extract data from rosbag file

Denote DIR as the main directory of the data file.

1. Use hdl_graph_slam to generate a point cloud map from a rosbag file with the save_map service, saved as e.g. *DIR/X1.pcd*, in which X is the series name and 1 is an integer index.
2. Use the *hdl_graph_slam/dump* service to dump the odometry data to the directory *X1*.
3. Parse the dump file via *DIR/UCB_ros/parse/dump_parse.cpp*, output in *DIR/X1_rel.txt*
4. Extract the GPS topic (Type: NavSat/Fix) to a text file with rostopic echo -b, and parse it with *DIR/UCB_ros/parse/gps.cpp*, output in *DIR/X1_gps.txt*

Now for each rosbag file of map Xi, we have the triple *DIR/{Xi.pcd, Xi_rel.txt, Xi_gps.txt}*. We assume i=1,2,3,4,5. (The indices must start from 1 and be continuous)

#### Phase 2: Calibrate maps to the UTM coordinate

1. Run in MATLAB: 
   `pipeline('X', 5)`
   to output in directories *DIR/maps_unified* and *DIR/tiles*, containing calibrated maps and their spatial occupancy data respectively.
2. Create a directory *DIR/trees*; compile and run the C++ sources in *DIR/tree_generation*, enter
   `X 5`
   then the compact data structure of spatial occupancy is stored in *DIR/trees*.
3. Create a text file *DIR/trees/trees.txt* with content
   `5 X1.tree X2.tree X3.tree X4.tree X5.tree`
   and *DIR/maps_unified/maps.txt* with
   `X1/pc.pcd X2/pc.pcd X3/pc.pcd X4/pc.pcd X5/pc.pcd`

Now the *ATLAS* data structure is completed.

#### Phase 3: adapt the launch file to apply the atlas

1. Open the *catkin_ws/src/hdl_localization/launch/localize_atlas.launch*.
2. modify the parameters *"globalmap_pcd", "globalmap_utm", "atlas_trees_dir", "atlas_trees_files", "atlas_map_dir", "atlas_map_files"* in the **globalmap_server_nodelet**, and *"globalmap_utm"* in the **hdl_localization_nodelet** accordingly.