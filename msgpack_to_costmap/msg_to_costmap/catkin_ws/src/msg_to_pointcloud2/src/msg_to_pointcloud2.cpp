#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "msgpack.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <exception>
#include <dirent.h>
#include <regex>
#include <algorithm>
#include <signal.h>
#include <sstream>

// 定义 PointXYZ 结构体
struct PointXYZ {
    float x, y, z;
};

// 读取文件到缓冲区
std::vector<byte> readFileToBuffer(const std::string& filePath) {
    std::ifstream file(filePath, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filePath);
    }

    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<byte> buffer(size);
    if (!file.read(reinterpret_cast<char*>(buffer.data()), size)) {
        throw std::runtime_error("Failed to read file: " + filePath);
    }

    return buffer;
}

// 解析 msgpack 文件
std::vector<std::vector<PointXYZ>> parseMsg(const std::string& fileDir) {
    try {
        // 构建 .msgpack 文件的路径
        std::string filePath = fileDir + "/clusters.msg";
        std::vector<byte> buffer = readFileToBuffer(filePath);
        std::cout << "Read success! Directory: " << fileDir << std::endl;

        // 使用用户自建 msgpack 库定义 unpacker
        msgpack::Unpacker unpacker(buffer.data(), buffer.size());

        // 解包外层的 vector
        uint32_t outer_size = unpacker.start_array();

        std::vector<std::vector<PointXYZ>> point_clouds;

        for (uint32_t i = 0; i < outer_size; ++i) {
            // 解包内层的 vector
            uint32_t inner_size = unpacker.start_array();

            std::vector<PointXYZ> points;

            for (uint32_t j = 0; j < inner_size; ++j) {
                PointXYZ point;

                // 解包 3 个 float 值
                for (int k = 0; k < 3; ++k) {
                    int type = unpacker.peek();
                    if (type != MSGPACK_FLOAT) {
                        throw std::runtime_error("Unexpected data type in msgpack file");
                    }

                    float value;
                    unpacker >> value;
                    switch (k) {
                    case 0:
                        point.x = value;
                        break;
                    case 1:
                        point.y = value;
                        break;
                    case 2:
                        point.z = value;
                        break;
                    default:
                        break;
                    }
                }
                points.push_back(point);
            }
            point_clouds.push_back(points);
        }

        return point_clouds;
    }
    catch (const std::exception& e) {
        std::cerr << "Error in " << fileDir << ": " << e.what() << "\n";
        return std::vector<std::vector<PointXYZ>> {};
    }
}

// 读取 votedata.csv 文件
std::vector<int> readVoteData(const std::string& voteDir, size_t numClusters) {
    std::vector<int> voteData(numClusters, 2); // 初始化所有簇的属性为 2
    std::string filePath = voteDir + "/votedata.csv";
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filePath);
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string clusterIdStr, attributeStr;
        std::getline(iss, clusterIdStr, ',');
        std::getline(iss, attributeStr, ',');
        int clusterId = std::stoi(clusterIdStr);
        int attribute = std::stoi(attributeStr);
        if (clusterId < static_cast<int>(numClusters)) {
            voteData[clusterId] = attribute;
        }
    }

    return voteData;
}

// 根据属性分离簇
std::vector<std::vector<std::vector<PointXYZ>>> separateClustersByAttribute(const std::vector<std::vector<PointXYZ>>& points, const std::vector<int>& voteData) {
    std::vector<std::vector<std::vector<PointXYZ>>> separatedPoints(3);
    for (size_t i = 0; i < points.size(); ++i) {
        int attribute = voteData[i];
        separatedPoints[attribute].push_back(points[i]);
    }
    return separatedPoints;
}

// 将二维 vector 数组转换为 PointCloud2 消息
sensor_msgs::PointCloud2 convertToPointCloud2(const std::vector<std::vector<PointXYZ>>& points_2d_vector) {
    // 创建 PCL 点云对象
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (const auto& cluster : points_2d_vector) {
        // 遍历簇中的每个点
        for (const auto& point : cluster) {
            pcl::PointXYZ p;
            p.x = point.z;
            p.y = -point.x;
            p.z = -point.y;
            cloud.points.push_back(p);//通过直接修改x、y、z的对应关系调整坐标系
        }
    }
    cloud.width = static_cast<uint32_t>(cloud.points.size());
    cloud.height = 1;
    cloud.is_dense = true;

    // 将 PCL 点云转换为 ROS 的 PointCloud2 消息
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "map";

    return cloud_msg;
}

// 信号处理函数
void sigintHandler(int sig) {
    ros::requestShutdown();
}

// 自定义排序函数，从目录名中提取数字并按数字大小排序
bool compareDirNames(const std::string& a, const std::string& b) {
    std::string prefix = "frame_";
    size_t posA = a.find(prefix);
    size_t posB = b.find(prefix);
    if (posA != std::string::npos && posB != std::string::npos) {
        int numA = std::stoi(a.substr(posA + prefix.length()));
        int numB = std::stoi(b.substr(posB + prefix.length()));
        return numA < numB;
    }
    return a < b;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_publisher");
    ros::NodeHandle nh;
    std::cout << "rosinit success" << std::endl;

    // 注册信号处理函数
    signal(SIGINT, sigintHandler);

    // 创建 3 个发布者，分别发布不同属性的 PointCloud2 消息
    ros::Publisher pub0 = nh.advertise<sensor_msgs::PointCloud2>("static_obstacle", 1);
    ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2>("dynamic_obstacle", 1);
    ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2>("uncertain_obstacle", 1);

    ros::Rate rate(1);

    std::string baseDir = "/home/cjw/组内/data/refinedClusters/seq_0";
    std::string voteDir = "/home/cjw/组内/data/vote";
    std::string searchPattern = "frame_.*";
    std::regex pattern(searchPattern);

    DIR* dir = opendir(baseDir.c_str());
    DIR* votedir = opendir(voteDir.c_str());
    if (dir == nullptr) {
        std::cerr << "Could not open directory: " << baseDir << std::endl;
        return 1;
    }
    std::cout << "opendir success" << std::endl;
    if (votedir == nullptr) {
        std::cerr << "Could not open directory: " << voteDir << std::endl;
        return 1;
    }
    std::cout << "openvotedir success" << std::endl;

    std::vector<std::string> dirNames;
    std::vector<std::string> voteNames;

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string dirName = entry->d_name;
        if (entry->d_type == DT_DIR && std::regex_search(dirName, pattern) && dirName != "." && dirName != "..") {
            dirNames.push_back(dirName);
        }
    }
    closedir(dir);

    struct dirent* voteentry;
    while ((voteentry = readdir(votedir)) != nullptr) {
        std::string voteName = voteentry->d_name;
        if (voteentry->d_type == DT_DIR && std::regex_search(voteName, pattern) && voteName != "." && voteName != "..") {
            voteNames.push_back(voteName);
        }
    }
    closedir(votedir);

    // 使用自定义排序函数进行排序
    std::sort(dirNames.begin(), dirNames.end(), compareDirNames);
    std::sort(voteNames.begin(), voteNames.end(), compareDirNames);

    for (size_t i = 0; i < dirNames.size(); ++i) {
        std::vector<std::vector<PointXYZ>> points;
        std::string msgDir = baseDir + "/" + dirNames[i];
        std::string msgPath = msgDir + "/clusters.msg";
        std::string voteDirPath = voteDir + "/" + voteNames[i];

        std::cout << "Checking file: " << msgPath << std::endl;
        std::ifstream file(msgPath);
        if (file.good()) {
            try {
                points = parseMsg(msgDir);
                std::vector<int> voteData = readVoteData(voteDirPath, points.size());
                std::vector<std::vector<std::vector<PointXYZ>>> separatedPoints = separateClustersByAttribute(points, voteData);

                for (int j = 0; j < 3; ++j) {
                    sensor_msgs::PointCloud2 cloud_msg = convertToPointCloud2(separatedPoints[j]);
                    switch (j) {
                    case 0:
                        pub0.publish(cloud_msg);
                        break;
                    case 1:
                        pub1.publish(cloud_msg);
                        break;
                    case 2:
                        pub2.publish(cloud_msg);
                        break;
                    }
                }

                ros::spinOnce();
                rate.sleep();
            }
            catch (const std::runtime_error& e) {
                std::cerr << "Error: " << e.what() << std::endl;
            }
        }
    }

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}