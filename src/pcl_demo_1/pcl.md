以下是在Ubuntu ROS2环境中使用PCL库进行点云构建、保存、读取及可视化的完整C++代码实现。



## 一、编译与运行

```bash
cd ~/test_pcl
colcon build --packages-select pcl_demo_1
source install/setup.bash
ros2 run pcl_demo_1 pcl_demo1_node
```
##二、使用Rviz2可视化

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

## 三、使用命令行工具查看PCD文件

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


