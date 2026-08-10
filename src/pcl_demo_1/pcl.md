以下是在Ubuntu ROS2环境中使用PCL库进行点云构建、保存、读取及可视化的完整C++代码实现。

## 一、环境准备
首先安装所需依赖：

```bash
sudo apt install ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-pcl-conversions libpcl-dev pcl-tools
```
其中 ${ROS_DISTRO} 替换为你的ROS2版本，如 humble 或 galactic。

## 二、创建功能包
```bash
cd ~/test_pcl/src
ros2 pkg create pcl_demo_1 --build-type ament_cmake --dependencies rclcpp std_msgs sensor_msgs pcl_ros pcl_conversions
```
## 三、CMakeLists.txt 配置
```cmake
cmake_minimum_required(VERSION 3.8)
project(pcl_demo_1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
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

add_executable(pcl_demo_1_node src/pcl_demo1_node.cpp)
ament_target_dependencies(pcl_demo_1_node 
  rclcpp 
  std_msgs 
  sensor_msgs 
  pcl_ros 
  pcl_conversions)

target_link_libraries(pcl_demo_1_node ${PCL_LIBRARIES})

install(TARGETS
  pcl_demo_1_node
  DESTINATION lib/${PROJECT_NAME}) 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```
## 四、package.xml 配置

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>pcl_demo_1</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="2653439973@qq.com">sy</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>pcl_ros</depend>
  <depend>pcl_conversions</depend>
  <depend>PCL</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

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

#include <chrono>
#include <memory>
#include <cstdlib>   // for rand()

typedef pcl::PointXYZ PointT;

class PCLDemo1Node : public rclcpp::Node
{
public:
    PCLDemo1Node() : Node("pcl_demo1_node")
    {
        RCLCPP_INFO(this->get_logger(), "PCL Demo Node Started");

        // 1. 构建点云（保存为成员变量，供定时器反复使用）
        cloud_ = createPointCloud();

        // 2. 保存点云为 PCD 文件
        savePointCloud(cloud_, "demo1_cloud.pcd");

        // 3. 读取点云文件（演示）
        pcl::PointCloud<PointT>::Ptr cloud_from_file = loadPointCloud("demo1_cloud.pcd");
        // 这里我们仍使用 cloud_，但也可改用 cloud_from_file

        // 4. 创建发布者
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_demo1/point_cloud", 10);

        // 5. 立即发布一次，让已经连接的订阅者收到
        publishPointCloud(cloud_);

        // 6. 创建定时器，每 1 秒发布一次，确保后启动的 rviz2 也能收到
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                publishPointCloud(cloud_);
            }
        );

        RCLCPP_INFO(this->get_logger(), "Node is spinning. Use rviz2 to visualize /pcl_demo1/point_cloud");
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    pcl::PointCloud<PointT>::Ptr cloud_;   // 保存点云供定时器使用

    // 1. 构建点云：随机点 + 球面规则点
    pcl::PointCloud<PointT>::Ptr createPointCloud()
    {
        RCLCPP_INFO(this->get_logger(), "Creating point cloud...");
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

        cloud->width = 10000;
        cloud->height = 1;
        cloud->is_dense = false;
        cloud->points.resize(cloud->width * cloud->height);

        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            float x = (rand() % 2000 - 1000) / 100.0f;
            float y = (rand() % 2000 - 1000) / 100.0f;
            float z = (rand() % 2000 - 1000) / 100.0f;

            // 前 500 个点构成球面
            if (i < 500)
            {
                float theta = 2.0f * M_PI * i / 500.0f;
                float phi = M_PI * i / 500.0f;
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

        RCLCPP_INFO(this->get_logger(), "Point cloud created with %zu points.", cloud->points.size());
        return cloud;
    }

    // 2. 保存点云为 PCD 文件（二进制格式）
    void savePointCloud(const pcl::PointCloud<PointT>::Ptr& cloud, const std::string& filename)
    {
        RCLCPP_INFO(this->get_logger(), "Saving point cloud to %s...", filename.c_str());
        int result = pcl::io::savePCDFileBinary(filename, *cloud);
        if (result == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Point cloud saved successfully to %s.", filename.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud to %s.", filename.c_str());
        }
    }

    // 3. 读取点云文件
    pcl::PointCloud<PointT>::Ptr loadPointCloud(const std::string& filename)
    {
        RCLCPP_INFO(this->get_logger(), "Loading point cloud from %s...", filename.c_str());
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        int result = pcl::io::loadPCDFile<PointT>(filename, *cloud);
        if (result == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Point cloud loaded successfully from %s with %zu points.", filename.c_str(), cloud->points.size());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load point cloud from %s.", filename.c_str());
        }
        return cloud;
    }

    // 4. 发布点云到 ROS2 话题
    void publishPointCloud(const pcl::PointCloud<PointT>::Ptr& cloud)
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = "map";
        cloud_msg.header.stamp = this->now();
        publisher_->publish(cloud_msg);
        // 为减少日志输出，可注释掉下面一行
        // RCLCPP_INFO(this->get_logger(), "Point cloud published.");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCLDemo1Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
## 六、编译与运行

```bash
cd ~/test_pcl
colcon build --packages-select pcl_demo_1
source install/setup.bash
ros2 run pcl_demo_1 pcl_demo1_node
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


