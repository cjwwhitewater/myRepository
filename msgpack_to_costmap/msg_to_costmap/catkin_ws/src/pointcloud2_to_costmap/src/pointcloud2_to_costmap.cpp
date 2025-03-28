// .h
#include "pointcloud2_to_costmap.h"
#include <tf/transform_listener.h>

//................................................CONFIG......................................................

void PointCloudConverter::read_config(ros::NodeHandle *nh) {
    try
    {
        if (!nh->hasParam(param_name)) {
            throw std::runtime_error("Error! Could not find param \'"+ param_name + "\' in config.yaml.");
        }
        else {
            XmlRpc::XmlRpcValue typeList;
            try {
                bool print_conf = false;
                nh->getParam("/print", print_conf);
                nh->getParam("/types", typeList);
                for (int i = 0; i < typeList.size(); ++i) {
                    Config tmp;
                    tmp.name = static_cast<std::string>(typeList[i]["name"]);
                    tmp.value = static_cast<int>(typeList[i]["value"]);
                    tmp.group = static_cast<std::string>(typeList[i]["group"]);
                    config.push_back(tmp);
                }
                print_config(print_conf);
            }
            catch (const XmlRpc::XmlRpcException &e) {
                std::cout << "Error! XmlRpcException: probably you have to write integer number ('0' or '25')." << std::endl;
            } 
        }
    }
    catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
    } 
}

void PointCloudConverter::print_config(bool print_conf) {
    if (print_conf) {
        std::cout << "\n______" << std::endl;
        std::cout << "CONFIG\n" << std::endl;
        for (int i = 0; i < config.size(); ++i) {
            std::cout << "name: " << config[i].name << std::endl;
            std::cout << "value: " << config[i].value << std::endl;
            std::cout << "group: " << config[i].group << std::endl;
            std::cout <<  std::endl;
        }
    }
}


//............................................PointCloudType..................................................

// Convert 4 bytes to Float32
float PointCloudType::Convert(unsigned int (&c) [4]) {
    UStuff b;
    for (size_t i = 0; i < 4; ++i) {
            b.c[i] = *(c+i);
    }
    return b.f;
}

void PointCloudType::send_msg(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // 打印原始消息的数据大小等信息
    ROS_INFO("Received msg data size: %lu", msg->data.size());
    unsigned int width = 0;
    _msg.data.clear();
    _msg.header = msg->header;
    _msg.height = msg->height;
    _msg.fields = msg->fields;
    _msg.is_bigendian = msg->is_bigendian;
    _msg.point_step = msg->point_step;
    _msg.row_step = msg->row_step;
    _msg.is_dense = msg->is_dense;

    // 假设costmap的分辨率和尺寸
    const float costmap_resolution = 0.1;  
    const int costmap_width = 150;  
    const int costmap_height = 150;  
    std::vector<int> costmap(costmap_width * costmap_height, 0);  // 初始化costmap

    // 清除旧的占用信息
    std::fill(costmap.begin(), costmap.end(), 0);

    for(int i = 0; i < (msg->data.size()/16); ++i) { 
        // 获取点云的3D坐标
        float x, y, z;
        x = *(float*)(&msg->data[16*i]);
        y = *(float*)(&msg->data[16*i + 4]);
        z = *(float*)(&msg->data[16*i + 8]);

        // 将3D坐标投影到2D网格
        int grid_x = static_cast<int>(x / costmap_resolution);
        int grid_y = static_cast<int>(y / costmap_resolution);

        // 检查网格坐标是否在costmap范围内
        if (grid_x >= 0 && grid_x < costmap_width && grid_y >= 0 && grid_y < costmap_height) {
            costmap[grid_y * costmap_width + grid_x] = 1;  // 标记该网格被占用
        }

        for (int k = 0; k < 16; ++k) { 
            _msg.data.push_back(msg->data[16*i+k]);
        }
        width++;
    }
    _msg.width = width;

    // 打印要发布消息的数据大小等信息
    ROS_INFO("Publish msg data size: %lu", _msg.data.size());
    pub.publish(_msg);

    // 保存占用网格数据到文件
    // std::ofstream outfile("~/code/组内/outdata/costmap_occupancy.txt");
    // if (outfile.is_open()) {
    //     for (int y = 0; y < costmap_height; ++y) {
    //         for (int x = 0; x < costmap_width; ++x) {
    //             outfile << costmap[y * costmap_width + x];
    //             if (x < costmap_width - 1) {
    //                 outfile << " ";
    //             }
    //         }
    //         outfile << std::endl;
    //     }
    //     outfile.close();
    //     std::cout << "write success!" << std::endl;
    //     ROS_INFO("Occupancy grid data saved to costmap_occupancy.txt");
    // } else {
    //     ROS_ERROR("Unable to open file for writing occupancy grid data");
    // }
}
//..........................................PointCloudConverter...............................................

void PointCloudConverter::Callback(const sensor_msgs::PointCloud2ConstPtr& msg) {    
    for (auto &type : types){
        type.send_msg(msg);
    }
}

void PointCloudConverter::create_types(ros::NodeHandle *nh){
    for (auto &config_ : config){
        bool push = true;
        if (types.size() > 0){
            for (auto &type_ : types){
                if (type_.group == config_.group){
                    push = false;
                }
            }
            if (push){
                PointCloudType tmp = PointCloudType(nh, config_.group);
                types.push_back(tmp);
            }
        }
        else{
            PointCloudType tmp = PointCloudType(nh, config_.group);
            types.push_back(tmp);
        }
    }
    for (auto &config_ : config){
        for (auto &type_ : types){
            if (config_.group == type_.group){
                type_.values.push_back(config_.value);
            }
        }
    }
}

void PointCloudConverter::print_types(){
    std::cout << "\n______" << std::endl;
    std::cout << "GROUPS\n" << std::endl;
    for (auto &type : types){
        std::cout << "type->group: " << type.group << std::endl;
        for (auto &value : type.values){
            std::cout << "value: " << value << std::endl;
        }
        std::cout <<  std::endl; 
    }
}

//.................................................Main.......................................................

int main(int argc, char **argv) {
    ros::init(argc, argv, "pointcloud2_to_costmap");
    ros::NodeHandle nh;
    PointCloudConverter pc = PointCloudConverter(&nh);
    ros::spin();
    return 0;
}