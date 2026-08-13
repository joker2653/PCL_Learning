// ============================================================================
// 文件名: pcl_demo1_node.cpp
// 描述: ROS2 节点，演示 PCL 点云的创建、保存、读取、发布，以及多种滤波、
//       降采样、Kd‑tree、Octree（八叉树）和关键点提取（SIFT, Harris, ISS, AGAST）的应用。
//       共发布 1 个原始点云 + 10 个滤波结果 + 4 个降采样结果 + 4 个关键点 = 19 个话题。
//       所有搜索结果（邻居索引、距离、变化点、压缩比等）打印在终端。
// 作者: Auto Generated
// 日期: 2026-08-13
// ============================================================================

// -------------------- ROS2 核心与消息头文件 --------------------
#include <rclcpp/rclcpp.hpp>                  // ROS2 节点、日志、定时器
#include <sensor_msgs/msg/point_cloud2.hpp>   // ROS2 标准点云消息类型

// -------------------- PCL 基础头文件 --------------------
#include <pcl/point_cloud.h>                  // pcl::PointCloud 类
#include <pcl/point_types.h>                  // pcl::PointXYZ 等点类型
#include <pcl_conversions/pcl_conversions.h>  // PCL ↔ ROS2 消息转换
#include <pcl/io/pcd_io.h>                    // PCD 文件读写（二进制/ASCII）

// -------------------- 滤波相关 (10 种) --------------------
#include <pcl/filters/passthrough.h>               // 直通滤波
#include <pcl/filters/conditional_removal.h>       // 条件滤波
#include <pcl/filters/statistical_outlier_removal.h> // 统计滤波
#include <pcl/filters/radius_outlier_removal.h>    // 半径滤波
#include <pcl/filters/voxel_grid.h>                // 体素滤波（可作降采样）
#include <pcl/filters/approximate_voxel_grid.h>    // 近似体素滤波
#include <pcl/filters/project_inliers.h>           // 投影滤波
#include <pcl/filters/crop_box.h>                  // 裁剪盒滤波
#include <pcl/filters/crop_hull.h>                 // 裁剪凸包滤波
#include <pcl/filters/frustum_culling.h>           // 视锥裁剪

// -------------------- 降采样相关 (4 种) --------------------
#include <pcl/filters/uniform_sampling.h>          // 均匀采样（保留原始点）
#include <pcl/filters/random_sample.h>             // 随机采样

// -------------------- Kd‑tree 相关 --------------------
#include <pcl/kdtree/kdtree_flann.h>               // FLANN 实现的 Kd‑tree

// -------------------- Octree 相关 --------------------
#include <pcl/octree/octree_search.h>              // 八叉树搜索（体素内/K近邻/半径）
#include <pcl/octree/octree_pointcloud_changedetector.h> // 空间变化检测
#include <pcl/compression/octree_pointcloud_compression.h> // 点云压缩/解压（命名空间 pcl::io）

// -------------------- 关键点提取相关 (4 种) --------------------
#include <pcl/keypoints/sift_keypoint.h>           // SIFT 关键点
#include <pcl/keypoints/harris_3d.h>               // Harris3D 关键点
#include <pcl/keypoints/iss_3d.h>                  // ISS 关键点
#include <pcl/keypoints/agast_2d.h>                // AGAST 2D 关键点

// -------------------- 辅助头文件 --------------------
#include <pcl/ModelCoefficients.h>                 // 模型系数（用于投影）
#include <pcl/surface/convex_hull.h>               // 凸包计算（用于裁剪凸包）
#include <pcl/features/normal_3d.h>                // 法线计算（ISS 和 Harris 可能需要）

#include <chrono>     // std::chrono 用于定时器
#include <memory>     // std::shared_ptr
#include <cstdlib>    // rand() 随机数
#include <string>     // std::string
#include <vector>     // std::vector
#include <iostream>   // std::cout（调试时可用）

// 为方便，将点类型定义为 pcl::PointXYZ（仅包含 x, y, z）
typedef pcl::PointXYZ PointT;

// ============================================================================
// 节点类：PCLDemo1Node
// 继承自 rclcpp::Node，实现所有功能。
// ============================================================================
class PCLDemo1Node : public rclcpp::Node
{
public:
    // 构造函数：初始化节点，执行所有处理流程
    PCLDemo1Node() : Node("pcl_demo1_node")
    {
        RCLCPP_INFO(this->get_logger(), "=== PCL 综合演示节点启动 ===");

        // ---------- 步骤1：创建原始点云 ----------
        // 包含 500 个球面规则点 + 9500 个随机点，共 10000 点
        original_cloud_ = createPointCloud();
        // 保存原始点云为 PCD 文件（二进制格式，节省空间）
        savePointCloud(*original_cloud_, "original_cloud.pcd");

        // 为了演示变化检测，创建一份带有噪声和新增点的副本
        pcl::PointCloud<PointT>::Ptr noisy_cloud = createNoisyCloud(original_cloud_);

        // ---------- 步骤2：应用 10 种滤波，获得滤波后的点云 ----------
        // 2.1 直通滤波：沿 Z 轴保留 [-2.0, 2.0] 范围内的点
        filtered_pass_through_   = filterPassThrough(original_cloud_, "z", -2.0f, 2.0f);
        // 2.2 条件滤波：保留 X>0 且 Y>0 且 Z>0 的点
        filtered_conditional_    = filterConditional(original_cloud_);
        // 2.3 统计滤波：基于邻域统计特性剔除离群点（meanK=50, stddev=1.0）
        filtered_statistical_    = filterStatisticalOutlier(original_cloud_, 50, 1.0);
        // 2.4 半径滤波：剔除半径 0.5 内邻居数少于 10 的点
        filtered_radius_         = filterRadiusOutlier(original_cloud_, 0.5f, 10);
        // 2.5 体素滤波：体素大小为 0.1，用质心代表体素内点（兼具降采样效果）
        filtered_voxel_          = filterVoxel(original_cloud_, 0.1f);
        // 2.6 近似体素滤波：体素大小 0.1，快速近似版本（不计算质心）
        filtered_approx_voxel_   = filterApproxVoxel(original_cloud_, 0.1f);
        // 2.7 投影滤波：将所有点投影到 z=0 平面（XOY 平面）
        filtered_projection_     = filterProjection(original_cloud_);
        // 2.8 裁剪盒滤波：裁剪出 [-3,3]×[-3,3]×[-3,3] 范围内的点
        filtered_crop_box_       = filterCropBox(original_cloud_);
        // 2.9 裁剪凸包滤波：使用原始点云构建凸包，保留凸包内部的点
        filtered_crop_hull_      = filterCropHull(original_cloud_);
        // 2.10 视锥裁剪：模拟相机视锥，保留视锥内的点
        filtered_frustum_        = filterFrustumCulling(original_cloud_);

        // 保存所有滤波结果到 PCD 文件，便于离线查看
        savePointCloud(*filtered_pass_through_,     "filtered_pass_through.pcd");
        savePointCloud(*filtered_conditional_,      "filtered_conditional.pcd");
        savePointCloud(*filtered_statistical_,      "filtered_statistical.pcd");
        savePointCloud(*filtered_radius_,           "filtered_radius.pcd");
        savePointCloud(*filtered_voxel_,            "filtered_voxel.pcd");
        savePointCloud(*filtered_approx_voxel_,     "filtered_approx_voxel.pcd");
        savePointCloud(*filtered_projection_,       "filtered_projection.pcd");
        savePointCloud(*filtered_crop_box_,         "filtered_crop_box.pcd");
        savePointCloud(*filtered_crop_hull_,        "filtered_crop_hull.pcd");
        savePointCloud(*filtered_frustum_,          "filtered_frustum.pcd");

        // ---------- 步骤3：应用 4 种降采样，获得降采样后的点云 ----------
        // 3.1 体素降采样：体素大小 0.2，用质心代表体素内点
        downsampled_voxel_        = downsampleVoxel(original_cloud_, 0.2f);
        // 3.2 近似体素降采样：体素大小 0.2，快速近似
        downsampled_approx_voxel_ = downsampleApproxVoxel(original_cloud_, 0.2f);
        // 3.3 均匀采样：体素大小 0.2，取最接近体素中心的原始点
        downsampled_uniform_      = downsampleUniform(original_cloud_, 0.2f);
        // 3.4 随机采样：随机保留 2000 个点
        downsampled_random_       = downsampleRandom(original_cloud_, 2000);

        // 保存所有降采样结果
        savePointCloud(*downsampled_voxel_,        "downsampled_voxel.pcd");
        savePointCloud(*downsampled_approx_voxel_, "downsampled_approx_voxel.pcd");
        savePointCloud(*downsampled_uniform_,      "downsampled_uniform.pcd");
        savePointCloud(*downsampled_random_,       "downsampled_random.pcd");

        // ---------- 步骤4：应用 4 种关键点提取 ----------
        // 注意：SIFT 需要输入点云包含强度信息，因此先将 PointXYZ 转换为 PointXYZI
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzI = convertToXYZI(original_cloud_);
        // 4.1 SIFT 关键点提取（输出为 PointWithScale，包含尺度和响应值）
        keypoints_sift_ = extractSIFTKeypoints(cloud_xyzI);
        // 4.2 Harris3D 关键点提取（输出为 PointXYZI，强度表示响应值）
        keypoints_harris_ = extractHarrisKeypoints(original_cloud_);
        // 4.3 ISS 关键点提取（输出为 PointXYZ）
        keypoints_iss_ = extractISSKeypoints(original_cloud_);
        // 4.4 AGAST 2D 关键点提取（输出为 PointUV，图像坐标）
        keypoints_agast_ = extractAGASTKeypoints(original_cloud_);

        // 保存关键点结果
        savePointCloud(*keypoints_sift_,   "keypoints_sift.pcd");
        savePointCloud(*keypoints_harris_, "keypoints_harris.pcd");
        savePointCloud(*keypoints_iss_,    "keypoints_iss.pcd");
        savePointCloud(*keypoints_agast_,  "keypoints_agast.pcd");

        // ---------- 步骤5：应用 Kd‑tree 搜索 ----------
        performKdTreeSearch(original_cloud_);

        // ---------- 步骤6：应用 Octree（八叉树） ----------
        // 6.1 Octree 搜索：体素内搜索、K近邻、半径搜索
        performOctreeSearch(original_cloud_);
        // 6.2 Octree 空间变化检测：比较原始点云和带噪声点云，找出新增点
        performOctreeChangeDetection(original_cloud_, noisy_cloud);
        // 6.3 Octree 点云压缩：编码/解码，打印压缩比并保存解压结果
        performOctreeCompression(original_cloud_);

        // ---------- 步骤7：创建 ROS2 发布者（共 19 个话题） ----------
        // 原始点云
        pub_original_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_demo1/original", 10);

        // 10 种滤波结果
        pub_pass_through_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_demo1/pass_through", 10);
        pub_conditional_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_demo1/conditional", 10);
        pub_statistical_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_demo1/statistical", 10);
        pub_radius_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_demo1/radius", 10);
        pub_voxel_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_demo1/voxel", 10);
        pub_approx_voxel_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_demo1/approx_voxel", 10);
        pub_projection_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_demo1/projection", 10);
        pub_crop_box_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_demo1/crop_box", 10);
        pub_crop_hull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_demo1/crop_hull", 10);
        pub_frustum_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_demo1/frustum", 10);

        // 4 种降采样结果
        pub_down_voxel_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_demo1/down_voxel", 10);
        pub_down_approx_voxel_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_demo1/down_approx_voxel", 10);
        pub_down_uniform_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_demo1/down_uniform", 10);
        pub_down_random_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_demo1/down_random", 10);

        // 4 个关键点结果
        pub_keypoints_sift_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_demo1/keypoints_sift", 10);
        pub_keypoints_harris_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_demo1/keypoints_harris", 10);
        pub_keypoints_iss_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_demo1/keypoints_iss", 10);
        pub_keypoints_agast_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_demo1/keypoints_agast", 10);

        // ---------- 步骤8：立即发布一次所有点云，并创建定时器每秒重复发布 ----------
        publishAllClouds();
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() { publishAllClouds(); }
        );

        // 打印提示信息
        RCLCPP_INFO(this->get_logger(), "节点就绪。共发布 19 个话题（1原始 + 10滤波 + 4降采样 + 4关键点）");
        RCLCPP_INFO(this->get_logger(), "滤波话题: /pcl_demo1/pass_through, conditional, statistical, radius, voxel, approx_voxel, projection, crop_box, crop_hull, frustum");
        RCLCPP_INFO(this->get_logger(), "降采样话题: /pcl_demo1/down_voxel, down_approx_voxel, down_uniform, down_random");
        RCLCPP_INFO(this->get_logger(), "关键点话题: /pcl_demo1/keypoints_sift, keypoints_harris, keypoints_iss, keypoints_agast");
    }

private:
    // ========================================================================
    // 成员变量：存储点云、发布者和定时器
    // ========================================================================

    // ---------- 原始点云 ----------
    pcl::PointCloud<PointT>::Ptr original_cloud_;

    // ---------- 滤波结果 (10 个) ----------
    pcl::PointCloud<PointT>::Ptr filtered_pass_through_;
    pcl::PointCloud<PointT>::Ptr filtered_conditional_;
    pcl::PointCloud<PointT>::Ptr filtered_statistical_;
    pcl::PointCloud<PointT>::Ptr filtered_radius_;
    pcl::PointCloud<PointT>::Ptr filtered_voxel_;
    pcl::PointCloud<PointT>::Ptr filtered_approx_voxel_;
    pcl::PointCloud<PointT>::Ptr filtered_projection_;
    pcl::PointCloud<PointT>::Ptr filtered_crop_box_;
    pcl::PointCloud<PointT>::Ptr filtered_crop_hull_;
    pcl::PointCloud<PointT>::Ptr filtered_frustum_;

    // ---------- 降采样结果 (4 个) ----------
    pcl::PointCloud<PointT>::Ptr downsampled_voxel_;
    pcl::PointCloud<PointT>::Ptr downsampled_approx_voxel_;
    pcl::PointCloud<PointT>::Ptr downsampled_uniform_;
    pcl::PointCloud<PointT>::Ptr downsampled_random_;

    // ---------- 关键点结果 (4 个) ----------
    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_sift_;  // SIFT 关键点（含尺度）
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_harris_;     // Harris 关键点（强度为响应值）
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_iss_;         // ISS 关键点
    pcl::PointCloud<pcl::PointUV>::Ptr keypoints_agast_;        // AGAST 关键点（2D 图像坐标）

    // ---------- ROS2 发布者 (19 个) ----------
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_original_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pass_through_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_conditional_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_statistical_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_radius_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_voxel_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_approx_voxel_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_projection_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_crop_box_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_crop_hull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_frustum_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_down_voxel_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_down_approx_voxel_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_down_uniform_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_down_random_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_keypoints_sift_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_keypoints_harris_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_keypoints_iss_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_keypoints_agast_;

    // ---------- 定时器 ----------
    rclcpp::TimerBase::SharedPtr timer_;

    // ========================================================================
    // 辅助函数：创建、保存、转换点云
    // ========================================================================

    /**
     * @brief 创建原始点云，包含 10000 个点。
     *        前 500 个点位于半径为 5 的球面上，其余为 [-10, 10] 范围内的随机点。
     * @return pcl::PointCloud<PointT>::Ptr 点云智能指针
     */
    pcl::PointCloud<PointT>::Ptr createPointCloud()
    {
        RCLCPP_INFO(this->get_logger(), "创建原始点云...");
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

        RCLCPP_INFO(this->get_logger(), "原始点云共 %zu 个点", cloud->points.size());
        return cloud;
    }

    /**
     * @brief 创建原始点云的副本，并加入噪声和新点，用于演示变化检测。
     * @param input 原始点云
     * @return 带噪声的点云（点数量可能变化）
     */
    pcl::PointCloud<PointT>::Ptr createNoisyCloud(const pcl::PointCloud<PointT>::Ptr& input)
    {
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>(*input));
        for (int i = 0; i < 200; ++i) {
            int idx = rand() % cloud->points.size();
            cloud->points[idx].x += (rand() % 100 - 50) / 100.0f;
            cloud->points[idx].y += (rand() % 100 - 50) / 100.0f;
            cloud->points[idx].z += (rand() % 100 - 50) / 100.0f;
        }
        for (int i = 0; i < 100; ++i) {
            PointT p;
            p.x = (rand() % 2000 - 1000) / 100.0f;
            p.y = (rand() % 2000 - 1000) / 100.0f;
            p.z = (rand() % 2000 - 1000) / 100.0f;
            cloud->points.push_back(p);
        }
        cloud->width = cloud->points.size();
        cloud->height = 1;
        return cloud;
    }

    /**
     * @brief 模板化的点云保存函数，支持任意点类型。
     *        参数为点云对象的 const 引用，方便类型推导。
     * @tparam PointT_ 点类型
     * @param cloud 点云对象引用
     * @param filename 文件名
     */
    template<typename PointT_>
    void savePointCloud(const pcl::PointCloud<PointT_>& cloud, const std::string& filename)
    {
        if (cloud.empty())
        {
            RCLCPP_WARN(this->get_logger(), "跳过空点云 %s", filename.c_str());
            return;
        }
        if (pcl::io::savePCDFileBinary(filename, cloud) == 0)
            RCLCPP_INFO(this->get_logger(), "保存 %s (%zu 点)", filename.c_str(), cloud.points.size());
        else
            RCLCPP_ERROR(this->get_logger(), "保存 %s 失败", filename.c_str());
    }

    /**
     * @brief 将 PointXYZ 点云转换为 PointXYZI（添加强度信息），SIFT 检测需要。
     * @param input 输入 PointXYZ 点云
     * @return 转换后的 PointXYZI 点云
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr convertToXYZI(const pcl::PointCloud<PointT>::Ptr& input)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
        output->resize(input->size());
        for (size_t i = 0; i < input->size(); ++i) {
            output->points[i].x = input->points[i].x;
            output->points[i].y = input->points[i].y;
            output->points[i].z = input->points[i].z;
            output->points[i].intensity = 0.5f; // 默认强度值
        }
        return output;
    }

    // ========================================================================
    // 滤波方法 (10 种) —— 与原始版本完全相同
    // ========================================================================

    /**
     * @brief 直通滤波：沿指定轴保留在 [min, max] 范围内的点。
     */
    pcl::PointCloud<PointT>::Ptr filterPassThrough(const pcl::PointCloud<PointT>::Ptr& input,
                                                   const std::string& field,
                                                   float min, float max)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::PassThrough<PointT> f;
        f.setInputCloud(input);
        f.setFilterFieldName(field);
        f.setFilterLimits(min, max);
        f.filter(*out);
        RCLCPP_INFO(this->get_logger(), "直通滤波 (%s in [%.1f,%.1f]) -> %zu 点",
                    field.c_str(), min, max, out->size());
        return out;
    }

    /**
     * @brief 条件滤波：保留满足所有条件（X>0 && Y>0 && Z>0）的点。
     */
    pcl::PointCloud<PointT>::Ptr filterConditional(const pcl::PointCloud<PointT>::Ptr& input)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::ConditionAnd<PointT>::Ptr cond(new pcl::ConditionAnd<PointT>);
        cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(
            new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::GT, 0.0)));
        cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(
            new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::GT, 0.0)));
        cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(
            new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::GT, 0.0)));

        pcl::ConditionalRemoval<PointT> f;
        f.setCondition(cond);
        f.setInputCloud(input);
        f.filter(*out);
        RCLCPP_INFO(this->get_logger(), "条件滤波 (X>0 && Y>0 && Z>0) -> %zu 点", out->size());
        return out;
    }

    /**
     * @brief 统计滤波：基于邻域统计特性剔除离群点。
     */
    pcl::PointCloud<PointT>::Ptr filterStatisticalOutlier(const pcl::PointCloud<PointT>::Ptr& input,
                                                          int meanK, float stddev)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::StatisticalOutlierRemoval<PointT> f;
        f.setInputCloud(input);
        f.setMeanK(meanK);
        f.setStddevMulThresh(stddev);
        f.filter(*out);
        RCLCPP_INFO(this->get_logger(), "统计滤波 (meanK=%d, stddev=%.1f) -> %zu 点",
                    meanK, stddev, out->size());
        return out;
    }

    /**
     * @brief 半径滤波：剔除指定半径内邻居数不足的点。
     */
    pcl::PointCloud<PointT>::Ptr filterRadiusOutlier(const pcl::PointCloud<PointT>::Ptr& input,
                                                     float radius, int minNeighbors)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::RadiusOutlierRemoval<PointT> f;
        f.setInputCloud(input);
        f.setRadiusSearch(radius);
        f.setMinNeighborsInRadius(minNeighbors);
        f.filter(*out);
        RCLCPP_INFO(this->get_logger(), "半径滤波 (radius=%.2f, minNeighbors=%d) -> %zu 点",
                    radius, minNeighbors, out->size());
        return out;
    }

    /**
     * @brief 体素滤波：体素网格下采样，用质心代表点。
     */
    pcl::PointCloud<PointT>::Ptr filterVoxel(const pcl::PointCloud<PointT>::Ptr& input, float leaf)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::VoxelGrid<PointT> f;
        f.setInputCloud(input);
        f.setLeafSize(leaf, leaf, leaf);
        f.filter(*out);
        RCLCPP_INFO(this->get_logger(), "体素滤波 (leaf=%.2f) -> %zu 点", leaf, out->size());
        return out;
    }

    /**
     * @brief 近似体素滤波：快速近似版本。
     */
    pcl::PointCloud<PointT>::Ptr filterApproxVoxel(const pcl::PointCloud<PointT>::Ptr& input, float leaf)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::ApproximateVoxelGrid<PointT> f;
        f.setInputCloud(input);
        f.setLeafSize(leaf, leaf, leaf);
        f.filter(*out);
        RCLCPP_INFO(this->get_logger(), "近似体素滤波 (leaf=%.2f) -> %zu 点", leaf, out->size());
        return out;
    }

    /**
     * @brief 投影滤波：将点投影到 z=0 平面。
     */
    pcl::PointCloud<PointT>::Ptr filterProjection(const pcl::PointCloud<PointT>::Ptr& input)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
        coeff->values.resize(4);
        coeff->values[0] = 0.0;
        coeff->values[1] = 0.0;
        coeff->values[2] = 1.0;
        coeff->values[3] = 0.0;

        pcl::ProjectInliers<PointT> f;
        f.setModelType(pcl::SACMODEL_PLANE);
        f.setInputCloud(input);
        f.setModelCoefficients(coeff);
        f.filter(*out);
        RCLCPP_INFO(this->get_logger(), "投影滤波 (投影到z=0) -> %zu 点", out->size());
        return out;
    }

    /**
     * @brief 裁剪盒滤波：保留 [-3,3]×[-3,3]×[-3,3] 范围内的点。
     */
    pcl::PointCloud<PointT>::Ptr filterCropBox(const pcl::PointCloud<PointT>::Ptr& input)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::CropBox<PointT> f;
        f.setInputCloud(input);
        f.setMin(Eigen::Vector4f(-3, -3, -3, 1));
        f.setMax(Eigen::Vector4f( 3,  3,  3, 1));
        f.filter(*out);
        RCLCPP_INFO(this->get_logger(), "裁剪盒滤波 ([-3,3]^3) -> %zu 点", out->size());
        return out;
    }

    /**
     * @brief 裁剪凸包滤波：利用点云构建凸包，保留凸包内部的点。
     */
    pcl::PointCloud<PointT>::Ptr filterCropHull(const pcl::PointCloud<PointT>::Ptr& input)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);

        pcl::ConvexHull<PointT> hull;
        hull.setInputCloud(input);
        std::vector<pcl::Vertices> polygons;
        pcl::PointCloud<PointT>::Ptr surface_hull(new pcl::PointCloud<PointT>);
        hull.reconstruct(*surface_hull, polygons);

        if (surface_hull->empty())
        {
            RCLCPP_WARN(this->get_logger(), "凸包构建失败，跳过裁剪凸包");
            return input;
        }

        pcl::CropHull<PointT> f;
        f.setInputCloud(input);
        f.setHullIndices(polygons);
        f.setHullCloud(surface_hull);
        f.setDim(3);
        f.filter(*out);

        RCLCPP_INFO(this->get_logger(), "裁剪凸包滤波 -> %zu 点", out->size());
        return out;
    }

    /**
     * @brief 视锥裁剪：模拟相机视锥，保留视锥内的点。
     */
    pcl::PointCloud<PointT>::Ptr filterFrustumCulling(const pcl::PointCloud<PointT>::Ptr& input)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::FrustumCulling<PointT> f;
        f.setInputCloud(input);
        f.setCameraPose(Eigen::Matrix4f::Identity());
        f.setHorizontalFOV(60.0f * M_PI / 180.0f);
        f.setVerticalFOV(45.0f * M_PI / 180.0f);
        f.setNearPlaneDistance(0.1f);
        f.setFarPlaneDistance(10.0f);
        f.filter(*out);
        RCLCPP_INFO(this->get_logger(), "视锥裁剪 -> %zu 点", out->size());
        return out;
    }

    // ========================================================================
    // 降采样方法 (4 种) —— 与原始版本完全相同
    // ========================================================================

    /**
     * @brief 体素降采样。
     */
    pcl::PointCloud<PointT>::Ptr downsampleVoxel(const pcl::PointCloud<PointT>::Ptr& input, float leaf)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::VoxelGrid<PointT> f;
        f.setInputCloud(input);
        f.setLeafSize(leaf, leaf, leaf);
        f.filter(*out);
        RCLCPP_INFO(this->get_logger(), "体素降采样 (leaf=%.2f) %zu -> %zu",
                    leaf, input->size(), out->size());
        return out;
    }

    /**
     * @brief 近似体素降采样。
     */
    pcl::PointCloud<PointT>::Ptr downsampleApproxVoxel(const pcl::PointCloud<PointT>::Ptr& input, float leaf)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::ApproximateVoxelGrid<PointT> f;
        f.setInputCloud(input);
        f.setLeafSize(leaf, leaf, leaf);
        f.filter(*out);
        RCLCPP_INFO(this->get_logger(), "近似体素降采样 (leaf=%.2f) %zu -> %zu",
                    leaf, input->size(), out->size());
        return out;
    }

    /**
     * @brief 均匀采样：保留每个体素内最接近中心的原始点。
     */
    pcl::PointCloud<PointT>::Ptr downsampleUniform(const pcl::PointCloud<PointT>::Ptr& input, float radius)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::UniformSampling<PointT> f;
        f.setInputCloud(input);
        f.setRadiusSearch(radius);
        f.filter(*out);
        RCLCPP_INFO(this->get_logger(), "均匀采样 (radius=%.2f) %zu -> %zu",
                    radius, input->size(), out->size());
        return out;
    }

    /**
     * @brief 随机采样：随机选择指定数量的点。
     */
    pcl::PointCloud<PointT>::Ptr downsampleRandom(const pcl::PointCloud<PointT>::Ptr& input, unsigned int sample)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::RandomSample<PointT> f;
        f.setInputCloud(input);
        f.setSample(sample);
        f.filter(*out);
        RCLCPP_INFO(this->get_logger(), "随机采样 (sample=%u) %zu -> %zu",
                    sample, input->size(), out->size());
        return out;
    }

    // ========================================================================
    // 关键点提取方法 (4 种)
    // ========================================================================

    /**
     * @brief SIFT 关键点提取。
     *        原理：构建尺度空间，在 DoG 金字塔中检测极值点。
     *        适用：需尺度和旋转不变性的物体识别和点云匹配。
     *        注意：输入点云需包含强度信息（PointXYZI）。
     * @param input 输入点云（PointXYZI）
     * @return SIFT 关键点（PointWithScale，包含尺度和响应值）
     */
    pcl::PointCloud<pcl::PointWithScale>::Ptr extractSIFTKeypoints(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input)
    {
        RCLCPP_INFO(this->get_logger(), "=== 提取 SIFT 关键点 ===");
        pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints(new pcl::PointCloud<pcl::PointWithScale>);
        pcl::SIFTKeypoint<pcl::PointXYZI, pcl::PointWithScale> sift;
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
        sift.setSearchMethod(tree);
        sift.setInputCloud(input);
        sift.setScales(0.5f, 3, 4);          // 最小尺度0.5，3个八度，每八度4个尺度
        sift.setMinimumContrast(0.01f);        // 最小对比度阈值
        sift.compute(*keypoints);
        RCLCPP_INFO(this->get_logger(), "SIFT 关键点: %zu", keypoints->points.size());
        return keypoints;
    }

    /**
     * @brief Harris3D 关键点提取。
     *        原理：利用点云法向量信息检测角点，是 2D Harris 的 3D 推广。
     *        适用：结构化场景中的角点检测，如工业零件质检。
     * @param input 输入点云（PointXYZ）
     * @return Harris 关键点（PointXYZI，强度值表示响应强度）
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr extractHarrisKeypoints(
        const pcl::PointCloud<PointT>::Ptr& input)
    {
        RCLCPP_INFO(this->get_logger(), "=== 提取 Harris3D 关键点 ===");
        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::HarrisKeypoint3D<PointT, pcl::PointXYZI> harris;
        harris.setInputCloud(input);
        harris.setRadius(0.5f);               // 搜索半径
        harris.setNonMaxSupression(true);      // 非极大值抑制
        harris.setThreshold(0.0001f);          // 响应值阈值
        harris.compute(*keypoints);
        RCLCPP_INFO(this->get_logger(), "Harris 关键点: %zu", keypoints->points.size());
        return keypoints;
    }

    /**
     * @brief ISS (Intrinsic Shape Signatures) 关键点提取。
     *        原理：分析邻域协方差矩阵的特征值，通过显著性度量筛选关键点。
     *        适用：对噪声有一定鲁棒性，适用于 3D 物体识别和形状分析。
     * @param input 输入点云（PointXYZ）
     * @return ISS 关键点（PointXYZ）
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractISSKeypoints(
        const pcl::PointCloud<PointT>::Ptr& input)
    {
        RCLCPP_INFO(this->get_logger(), "=== 提取 ISS 关键点 ===");
        pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ISSKeypoint3D<PointT, pcl::PointXYZ> iss;
        iss.setInputCloud(input);
        iss.setSalientRadius(0.5f);           // 显著区域半径
        iss.setNonMaxRadius(0.2f);            // 非极大值抑制半径
        iss.setThreshold21(0.7f);              // 特征值比值阈值 (λ2/λ1)
        iss.setThreshold32(0.7f);              // 特征值比值阈值 (λ3/λ2)
        iss.setMinNeighbors(5);                // 最小邻居数
        iss.compute(*keypoints);
        RCLCPP_INFO(this->get_logger(), "ISS 关键点: %zu", keypoints->points.size());
        return keypoints;
    }

    /**
     * @brief AGAST 2D 关键点提取。
     *        原理：快速的 2D 角点检测算法，是 FAST 的改进版。
     *        适用：实时性要求高的 2D 特征提取。
     *        注意：输入为 PointXYZ，输出为 PointUV（图像坐标）。
     * @param input 输入点云（PointXYZ）
     * @return AGAST 关键点（PointUV，图像坐标系下的 2D 坐标）
     */
    pcl::PointCloud<pcl::PointUV>::Ptr extractAGASTKeypoints(
        const pcl::PointCloud<PointT>::Ptr& input)
    {
        RCLCPP_INFO(this->get_logger(), "=== 提取 AGAST 2D 关键点 ===");
        pcl::PointCloud<pcl::PointUV>::Ptr keypoints(new pcl::PointCloud<pcl::PointUV>);
        pcl::AgastKeypoint2D<PointT, pcl::PointUV> agast;
        agast.setInputCloud(input);
        agast.setThreshold(30);                // 检测阈值
        agast.compute(*keypoints);
        RCLCPP_INFO(this->get_logger(), "AGAST 关键点: %zu", keypoints->points.size());
        return keypoints;
    }

    // ========================================================================
    // Kd‑tree 应用 —— 与原始版本完全相同
    // ========================================================================

    /**
     * @brief 对输入点云构建 Kd‑tree，并执行 K 近邻和半径搜索。
     */
    void performKdTreeSearch(const pcl::PointCloud<PointT>::Ptr& cloud)
    {
        RCLCPP_INFO(this->get_logger(), "=== Kd‑tree 搜索 ===");

        pcl::KdTreeFLANN<PointT> kdtree;
        kdtree.setInputCloud(cloud);

        PointT searchPoint;
        searchPoint.x = (rand() % 2000 - 1000) / 100.0f;
        searchPoint.y = (rand() % 2000 - 1000) / 100.0f;
        searchPoint.z = (rand() % 2000 - 1000) / 100.0f;
        RCLCPP_INFO(this->get_logger(), "查询点: (%.2f, %.2f, %.2f)",
                    searchPoint.x, searchPoint.y, searchPoint.z);

        int K = 10;
        std::vector<int> pointIdxKNNSearch(K);
        std::vector<float> pointKNNSquaredDistance(K);
        if (kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
        {
            RCLCPP_INFO(this->get_logger(), "K近邻搜索 (K=%d) 找到 %zu 个邻居:", K, pointIdxKNNSearch.size());
            for (size_t i = 0; i < pointIdxKNNSearch.size(); ++i)
            {
                RCLCPP_INFO(this->get_logger(), "  邻居 %zu: 索引=%d, 距离²=%.2f",
                            i, pointIdxKNNSearch[i], pointKNNSquaredDistance[i]);
            }
        }

        float radius = 2.0f;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
            RCLCPP_INFO(this->get_logger(), "半径搜索 (radius=%.2f) 找到 %zu 个邻居:", radius, pointIdxRadiusSearch.size());
            for (size_t i = 0; i < std::min(pointIdxRadiusSearch.size(), (size_t)5); ++i)
            {
                RCLCPP_INFO(this->get_logger(), "  邻居 %zu: 索引=%d, 距离²=%.2f",
                            i, pointIdxRadiusSearch[i], pointRadiusSquaredDistance[i]);
            }
            if (pointIdxRadiusSearch.size() > 5)
                RCLCPP_INFO(this->get_logger(), "  ... 还有 %zu 个邻居", pointIdxRadiusSearch.size() - 5);
        }
    }

    // ========================================================================
    // Octree 应用 —— 与原始版本完全相同
    // ========================================================================

    /**
     * @brief 对输入点云构建八叉树，并执行体素内搜索、K近邻和半径搜索。
     */
    void performOctreeSearch(const pcl::PointCloud<PointT>::Ptr& cloud)
    {
        RCLCPP_INFO(this->get_logger(), "=== Octree 搜索 ===");

        float resolution = 0.5f;
        pcl::octree::OctreePointCloudSearch<PointT> octree(resolution);
        octree.setInputCloud(cloud);
        octree.addPointsFromInputCloud();

        PointT searchPoint;
        searchPoint.x = (rand() % 2000 - 1000) / 100.0f;
        searchPoint.y = (rand() % 2000 - 1000) / 100.0f;
        searchPoint.z = (rand() % 2000 - 1000) / 100.0f;
        RCLCPP_INFO(this->get_logger(), "查询点: (%.2f, %.2f, %.2f)",
                    searchPoint.x, searchPoint.y, searchPoint.z);

        std::vector<int> pointIdxVec;
        if (octree.voxelSearch(searchPoint, pointIdxVec))
        {
            RCLCPP_INFO(this->get_logger(), "体素内搜索: 找到 %zu 个同体素点", pointIdxVec.size());
            for (size_t i = 0; i < std::min(pointIdxVec.size(), (size_t)5); ++i)
                RCLCPP_INFO(this->get_logger(), "  索引=%d", pointIdxVec[i]);
            if (pointIdxVec.size() > 5)
                RCLCPP_INFO(this->get_logger(), "  ... 还有 %zu 个点", pointIdxVec.size() - 5);
        }

        int K = 10;
        std::vector<int> pointIdxKNNSearch;
        std::vector<float> pointKNNSquaredDistance;
        if (octree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
        {
            RCLCPP_INFO(this->get_logger(), "Octree K近邻搜索 (K=%d) 找到 %zu 个邻居:", K, pointIdxKNNSearch.size());
            for (size_t i = 0; i < pointIdxKNNSearch.size(); ++i)
            {
                RCLCPP_INFO(this->get_logger(), "  邻居 %zu: 索引=%d, 距离²=%.2f",
                            i, pointIdxKNNSearch[i], pointKNNSquaredDistance[i]);
            }
        }

        float radius = 2.0f;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
            RCLCPP_INFO(this->get_logger(), "Octree 半径搜索 (radius=%.2f) 找到 %zu 个邻居:", radius, pointIdxRadiusSearch.size());
            for (size_t i = 0; i < std::min(pointIdxRadiusSearch.size(), (size_t)5); ++i)
            {
                RCLCPP_INFO(this->get_logger(), "  邻居 %zu: 索引=%d, 距离²=%.2f",
                            i, pointIdxRadiusSearch[i], pointRadiusSquaredDistance[i]);
            }
            if (pointIdxRadiusSearch.size() > 5)
                RCLCPP_INFO(this->get_logger(), "  ... 还有 %zu 个邻居", pointIdxRadiusSearch.size() - 5);
        }
    }

    /**
     * @brief 使用 Octree 进行空间变化检测，比较两个点云，找出新增的点。
     */
    void performOctreeChangeDetection(const pcl::PointCloud<PointT>::Ptr& cloudA,
                                      const pcl::PointCloud<PointT>::Ptr& cloudB)
    {
        RCLCPP_INFO(this->get_logger(), "=== Octree 空间变化检测 ===");

        float resolution = 0.5f;
        pcl::octree::OctreePointCloudChangeDetector<PointT> octree(resolution);
        octree.setInputCloud(cloudA);
        octree.addPointsFromInputCloud();
        octree.switchBuffers();
        octree.setInputCloud(cloudB);
        octree.addPointsFromInputCloud();

        std::vector<int> newPointIdxVector;
        octree.getPointIndicesFromNewVoxels(newPointIdxVector);

        RCLCPP_INFO(this->get_logger(), "空间变化检测: 发现 %zu 个新点", newPointIdxVector.size());
        for (size_t i = 0; i < std::min(newPointIdxVector.size(), (size_t)10); ++i)
        {
            const auto& p = cloudB->points[newPointIdxVector[i]];
            RCLCPP_INFO(this->get_logger(), "  新点 %zu: (%.2f, %.2f, %.2f)", i, p.x, p.y, p.z);
        }
        if (newPointIdxVector.size() > 10)
            RCLCPP_INFO(this->get_logger(), "  ... 还有 %zu 个新点", newPointIdxVector.size() - 10);
    }

    /**
     * @brief 使用 Octree 对点云进行压缩和解压，打印压缩比并保存解压结果。
     */
    void performOctreeCompression(const pcl::PointCloud<PointT>::Ptr& cloud)
    {
        RCLCPP_INFO(this->get_logger(), "=== Octree 点云压缩 ===");

        pcl::io::compression_Profiles_e compressionProfile =
            pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;

        pcl::io::OctreePointCloudCompression<PointT> compression(compressionProfile, false);

        std::stringstream compressedData;
        compression.encodePointCloud(cloud, compressedData);

        size_t originalSize = cloud->points.size() * sizeof(PointT);
        size_t compressedSize = compressedData.str().size();
        RCLCPP_INFO(this->get_logger(), "压缩前大小: %zu 字节, 压缩后大小: %zu 字节, 压缩比: %.2f%%",
                    originalSize, compressedSize,
                    (compressedSize > 0) ? (float)compressedSize / originalSize * 100 : 0.0);

        pcl::PointCloud<PointT>::Ptr decompressedCloud(new pcl::PointCloud<PointT>);
        compression.decodePointCloud(compressedData, decompressedCloud);

        RCLCPP_INFO(this->get_logger(), "解压后点云点数: %zu (原始: %zu)",
                    decompressedCloud->points.size(), cloud->points.size());

        savePointCloud(*decompressedCloud, "decompressed_cloud.pcd");
    }

    // ========================================================================
    // 发布函数
    // ========================================================================

    /**
     * @brief 将所有存储的点云（原始、滤波、降采样、关键点）发布到对应的话题。
     *        使用泛型 lambda 支持不同点类型。
     */
    void publishAllClouds()
    {
        // 泛型 lambda：接受任何点云类型
        auto pub = [this](const auto& cloud,
                          rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
                          const std::string& frame = "map")
        {
            if (!cloud || cloud->empty()) return;
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*cloud, msg);
            msg.header.frame_id = frame;
            msg.header.stamp = this->now();
            publisher->publish(msg);
        };

        // 原始点云
        pub(original_cloud_, pub_original_);

        // 10 种滤波结果
        pub(filtered_pass_through_, pub_pass_through_);
        pub(filtered_conditional_, pub_conditional_);
        pub(filtered_statistical_, pub_statistical_);
        pub(filtered_radius_, pub_radius_);
        pub(filtered_voxel_, pub_voxel_);
        pub(filtered_approx_voxel_, pub_approx_voxel_);
        pub(filtered_projection_, pub_projection_);
        pub(filtered_crop_box_, pub_crop_box_);
        pub(filtered_crop_hull_, pub_crop_hull_);
        pub(filtered_frustum_, pub_frustum_);

        // 4 种降采样结果
        pub(downsampled_voxel_, pub_down_voxel_);
        pub(downsampled_approx_voxel_, pub_down_approx_voxel_);
        pub(downsampled_uniform_, pub_down_uniform_);
        pub(downsampled_random_, pub_down_random_);

        // 4 种关键点结果
        pub(keypoints_sift_, pub_keypoints_sift_);
        pub(keypoints_harris_, pub_keypoints_harris_);
        pub(keypoints_iss_, pub_keypoints_iss_);
        pub(keypoints_agast_, pub_keypoints_agast_);
    }
};

// ============================================================================
// 主函数
// ============================================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCLDemo1Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}