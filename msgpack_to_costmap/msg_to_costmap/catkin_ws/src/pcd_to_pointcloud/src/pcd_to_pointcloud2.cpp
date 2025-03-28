#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <dirent.h>
#include <iostream>
#include <algorithm>
#include <sstream>
using namespace std;

// 结构体用于存储文件名及对应的序号，方便后续排序
struct FileInfo {
    string filePath;
    int frameNumber;
};

// 比较函数，用于按照frameNumber对FileInfo结构体进行排序
bool compareFileInfo(const FileInfo& a, const FileInfo& b) {
    return a.frameNumber < b.frameNumber;
}

// 函数用于获取目录下所有符合条件的文件路径，并按frame序号排序（假设文件名格式固定为类似/home/cjw/组内/data/refinedClusters/seq_0/frame_0/refinedPointCloud.pcd）
vector<string> getPcdFilePaths(const string& directoryPath) {
    vector<FileInfo> fileInfos;
    DIR* dir;
    struct dirent* ent;
    if ((dir = opendir(directoryPath.c_str()))!= NULL) {
        while ((ent = readdir(dir))!= NULL) {
            string fileName = ent->d_name;
            if (fileName.find("frame_") == 0) {
                // 提取frame_后面的数字序号
                string numberStr = fileName.substr(6, fileName.length() - 6);  // 截取frame_后面的数字部分，假设文件名格式固定
                int frameNumber;
                stringstream ss(numberStr);
                ss >> frameNumber;

                // 拼接完整的文件路径，按照实际的文件名格式进行拼接
                string filePath = directoryPath + "/" + fileName + "/refinedPointCloud.pcd";
                FileInfo fileInfo = {filePath, frameNumber};
                fileInfos.push_back(fileInfo);
            }
        }
        closedir(dir);

        // 对fileInfos按照frameNumber进行排序
        sort(fileInfos.begin(), fileInfos.end(), compareFileInfo);

        // 将排序后的文件路径提取到最终的返回向量中
        vector<string> sortedFilePaths;
        for (const auto& fileInfo : fileInfos) {
            sortedFilePaths.push_back(fileInfo.filePath);
        }
        return sortedFilePaths;
    }
    return vector<string>();
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "pcd_to_pointcloud2_publisher");
    ros::NodeHandle nh;
    cout << "init successed" << endl;


    std::string bag_file_path = "/home/cjw/组内/data/output/saved_pointcloud2.bag";
    // 创建一个发布器，发布到指定话题，这里话题名为"point_cloud_topic"，队列大小为1
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_topic", 1);

    // 获取所有pcd文件的路径列表
    vector<string> pcdFilePaths = getPcdFilePaths("/home/cjw/组内/data/refinedClusters/seq_0");
    // for (size_t i = 0; i < pcdFilePaths.size(); ++i) {
    //     cout << "File path " << i << ": " << pcdFilePaths[i] << endl;
    // }
    int fileIndex = 0;

    // 设置发布频率，这里为1Hz
    ros::Rate loop_rate(1);

    // 创建rosbag对象用于写入
    rosbag::Bag bag(bag_file_path, rosbag::BagMode::Write);

    // 循环发布消息，只要ROS节点处于运行状态
    while (ros::ok()) {
        if (fileIndex < pcdFilePaths.size()) {
            // 创建一个PCL点云对象指针，这里以包含XYZ坐标的点云类型为例，可根据实际需求更换类型（如包含颜色信息的点云类型等）
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

            // 读取当前索引对应的PCD文件，将文件内容加载到点云对象中，如果读取失败则输出错误信息并继续下一个文件
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdFilePaths[fileIndex].c_str(), *cloud) == -1) {
                ROS_ERROR("Could not read PCD file: %s", pcdFilePaths[fileIndex].c_str());
                fileIndex++;
                continue;
            }

            // 创建一个sensor_msgs::PointCloud2类型的消息对象
            sensor_msgs::PointCloud2 output;

            // 将PCL点云对象转换为sensor_msgs::PointCloud2消息，这里使用pcl_conversions库中的函数来进行转换
            pcl::toROSMsg(*cloud, output);

            // 设置消息的坐标系，根据实际情况修改
            output.header.frame_id = "base_scan";

            // 设置消息的时间戳，这里使用当前时间
            output.header.stamp = ros::Time::now();

            // 发布转换后的PointCloud2消息
            pub.publish(output);

            // 将转换后的PointCloud2消息写入rosbag文件
            bag.write("point_cloud_topic", ros::Time::now(), output);

            fileIndex++;
        }

        // 处理ROS回调函数（例如接收其他节点的消息等）
        ros::spinOnce();

        // 按照设定的频率休眠，以控制发布频率
        loop_rate.sleep();
    }

    // 关闭rosbag文件，完成保存
    bag.close();

    return 0;
}