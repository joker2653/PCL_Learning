以下是在Ubuntu ROS2环境中使用PCL库进行点云构建、保存、读取及可视化的完整C++代码实现。

## 一、环境准备
首先安装所需依赖：

```bash
sudo apt install ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-pcl-conversions libpcl-dev pcl-tools
```
其中 ${ROS_DISTRO} 替换为你的ROS2版本，如 humble 或 galactic。

## 二、创建功能包
```bash
cd ~/ros2_ws/src
ros2 pkg create pcl_demo --build-type ament_cmake --dependencies rclcpp std_msgs sensor_msgs pcl_ros pcl_conversions
```
## 三、CMakeLists.txt 配置
```cmake
cmake_minimum_required(VERSION 3.8)
project(pcl_demo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_demo_node src/pcl_demo_node.cpp)
ament_target_dependencies(pcl_demo_node
  rclcpp
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
)
target_link_libraries(pcl_demo_node ${PCL_LIBRARIES})

install(TARGETS pcl_demo_node DESTINATION lib/${PROJECT_NAME})
ament_package()
```
## 四、package.xml 配置

```xml
<?xml version="1.0"?>
<package format="3">
  <name>pcl_demo</name>
  <version>0.0.1</version>
  <description>PCL point cloud demo for ROS2</description>
  <maintainer email="user@example.com">user</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>pcl_ros</depend>
  <depend>pcl_conversions</depend>
  <depend>PCL</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```
## 五、完整节点代码
在 src/pcl_demo_node.cpp 中编写以下代码：

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/random.h>

#include <thread>
#include <chrono>

class PCLDemoNode : public rclcpp::Node
{
public:
    PCLDemoNode() : Node("pcl_demo_node")
    {
        RCLCPP_INFO(this->get_logger(), "PCL Demo Node Started");

        // 1. 构建点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = createPointCloud();

        // 2. 保存点云为PCD文件
        savePointCloud(cloud, "demo_cloud.pcd");

        // 3. 读取点云文件
        pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_cloud = loadPointCloud("demo_cloud.pcd");

        // 4. 可视化点云（在独立线程中运行）
        std::thread viewer_thread(&PCLDemoNode::visualizePointCloud, this, loaded_cloud);

        // 5. 发布点云到ROS2话题（可选）
        publishPointCloud(loaded_cloud);

        viewer_thread.join();
    }

private:
    /**
     * 1. 构建点云 - 生成一个包含随机点和规则形状的点云
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr createPointCloud()
    {
        RCLCPP_INFO(this->get_logger(), "Creating point cloud...");

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 设置点云尺寸
        cloud->width = 10000;
        cloud->height = 1;
        cloud->is_dense = false;
        cloud->resize(cloud->width * cloud->height);

        // 使用随机数生成器
        pcl::common::CloudRandomGenerator<pcl::PointXYZ> random_gen;
        random_gen.setSeed(42);

        // 生成随机点云（分布在单位球体内）
        for (size_t i = 0; i < cloud->size(); ++i)
        {
            // 随机点
            float x = (rand() % 2000 - 1000) / 100.0f;
            float y = (rand() % 2000 - 1000) / 100.0f;
            float z = (rand() % 2000 - 1000) / 100.0f;

            // 在球面上添加一些规则点，形成球形结构
            if (i < 500)
            {
                float theta = 2.0f * M_PI * i / 500;
                float phi = M_PI * i / 500;
                cloud->points[i].x = 5.0f * sin(phi) * cos(theta);
                cloud->points[i].y = 5.0f * sin(phi) * sin(theta);
                cloud->points[i].z = 5.0f * cos(phi);
            }
            else
            {
                cloud->points[i].x = x;
                cloud->points[i].y = y;
                cloud->points[i].z = z;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Created point cloud with %zu points", cloud->size());
        return cloud;
    }

    /**
     * 2. 保存点云为PCD文件
     */
    void savePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& filename)
    {
        RCLCPP_INFO(this->get_logger(), "Saving point cloud to %s...", filename.c_str());

        // 保存为二进制格式（更紧凑），也可使用 pcl::io::savePCDFileASCII
        int result = pcl::io::savePCDFileBinary(filename, *cloud);

        if (result == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Point cloud saved successfully to %s", filename.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud to %s", filename.c_str());
        }
    }

    /**
     * 3. 读取PCD文件
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloud(const std::string& filename)
    {
        RCLCPP_INFO(this->get_logger(), "Loading point cloud from %s...", filename.c_str());

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        int result = pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud);

        if (result == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Loaded point cloud with %zu points", cloud->size());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load point cloud from %s", filename.c_str());
            // 返回空点云
        }

        return cloud;
    }

    /**
     * 4. 可视化点云 - 使用PCL CloudViewer
     */
    void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        if (cloud->empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot visualize empty point cloud");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Visualizing point cloud...");
        RCLCPP_INFO(this->get_logger(), "Press 'q' in the viewer window to exit");

        // 创建PCL可视化器
        pcl::visualization::CloudViewer viewer("PCL Point Cloud Viewer");

        // 显示点云
        viewer.showCloud(cloud);

        // 保持窗口打开直到用户按 'q' 退出
        while (!viewer.wasStopped())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        RCLCPP_INFO(this->get_logger(), "Viewer closed");
    }

    /**
     * 5. 发布点云到ROS2话题（使用pcl::toROSMsg转换）
     */
    void publishPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        // 创建发布者
        auto publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_demo_cloud", 10);

        // 将PCL点云转换为ROS2消息格式
        sensor_msgs::msg::PointCloud2 ros_cloud;
        pcl::toROSMsg(*cloud, ros_cloud);

        // 设置坐标系ID和时间戳
        ros_cloud.header.frame_id = "map";
        ros_cloud.header.stamp = this->now();

        // 发布点云
        publisher->publish(ros_cloud);
        RCLCPP_INFO(this->get_logger(), "Published point cloud to topic /pcl_demo_cloud");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCLDemoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
## 六、编译与运行

```bash
cd ~/ros2_ws
colcon build --packages-select pcl_demo
source install/setup.bash
ros2 run pcl_demo pcl_demo_node
```
##七、使用Rviz2可视化

在另一个终端中启动Rviz2查看发布的点云：

```bash
rviz2
```
在Rviz2中：

点击左下角 `Add`按钮

选择 `By topic`选项卡

选择 `/pcl_demo_cloud` 话题

在 `Global Options` 中将 `Fixed Frame` 设置为 `map`

在 `PointCloud2` 显示选项中将 `Style` 改为 `Points` 以便更好查看

## 八、使用命令行工具查看PCD文件

```bash
# 使用pcl_viewer工具查看PCD文件
pcl_viewer demo_cloud.pcd

# 查看PCD文件信息
pcl_pcd2png demo_cloud.pcd output.png
```
关键API说明
功能	函数	说明
ROS→PCL转换	pcl::fromROSMsg()	将sensor_msgs::PointCloud2转为pcl::PointCloud<T>
PCL→ROS转换	pcl::toROSMsg()	将pcl::PointCloud<T>转为sensor_msgs::PointCloud2
保存PCD	pcl::io::savePCDFileBinary()	二进制格式保存
读取PCD	pcl::io::loadPCDFile()	从文件加载点云
可视化	pcl::visualization::CloudViewer	PCL内置可视化工具

## 常见问题
可视化段错误：部分PCL版本（如1.12.1）在ROS2中可视化可能出现段错误，可尝试升级PCL版本或使用Rviz2替代。

点云属性丢失：保存PCD时默认只保留XYZ坐标，如需保留强度(intensity)、时间(time)等信息，需要使用对应的点类型如pcl::PointXYZI。


