# pointcloud2_to_costmap

#### HOW TO INSTALL
首先用CMake工具构建myLibrary，然后用catkin工具构建msg_to_costmap，所需执行命令如下：
~ cd myLibrary/build
~ cmake ..
~ make
~ cd ../
~ cd ../
~ cd msg_to_costmap/catkin_ws
~ catkin_make

#### HOW TO RUN

要运行节点以可视化，首先需要运行pointcloud2_to_costmap node，并运行costmap2d module和rviz，然后运行msg_to_pointcloud2节点处理数据并将数据发布为主题。

pointcloud2_to_costmap node：
roslaunch pointcloud2_to_costmap pointcloud2_to_costmap.launch 

costmap_2d module + rviz：
roslaunch pointcloud2_to_costmap costmap.launch

msgpack_to_pointcloud2:
rosrun msg_to_pointcloud2 msg_to_pointcloud2


