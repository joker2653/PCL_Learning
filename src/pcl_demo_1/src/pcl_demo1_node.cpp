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