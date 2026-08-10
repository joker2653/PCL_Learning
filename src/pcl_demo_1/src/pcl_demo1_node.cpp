// ============================================================================
// 文件名: pcl_demo1_node.cpp
// 描述: ROS2 节点，演示 PCL 点云的创建、保存、读取、发布以及多种滤波和降采样方法。
//       包含 10 种滤波 + 4 种降采样，共发布 15 个话题，便于在 Rviz2 中对比观察。
// 作者: Auto Generated
// 日期: 2026-08-10
// ============================================================================

// -------------------- ROS2 核心与消息头文件 --------------------
#include <rclcpp/rclcpp.hpp>                  // ROS2 节点类、日志、定时器等
#include <sensor_msgs/msg/point_cloud2.hpp>   // ROS2 标准点云消息

// -------------------- PCL 基础头文件 --------------------
#include <pcl/point_cloud.h>                  // pcl::PointCloud 类
#include <pcl/point_types.h>                  // pcl::PointXYZ 等点类型
#include <pcl_conversions/pcl_conversions.h>  // PCL ↔ ROS2 消息转换函数
#include <pcl/io/pcd_io.h>                    // PCD 文件读写（二进制/ASCII）

// -------------------- 滤波相关 (10 种) --------------------
#include <pcl/filters/passthrough.h>               // 直通滤波
#include <pcl/filters/conditional_removal.h>       // 条件滤波（多条件组合）
#include <pcl/filters/statistical_outlier_removal.h> // 统计滤波（离群点剔除）
#include <pcl/filters/radius_outlier_removal.h>    // 半径滤波（孤立点剔除）
#include <pcl/filters/voxel_grid.h>                // 体素滤波（同时可用作降采样）
#include <pcl/filters/approximate_voxel_grid.h>    // 近似体素滤波（快速版）
#include <pcl/filters/project_inliers.h>           // 投影滤波（将点投影到模型）
#include <pcl/filters/crop_box.h>                  // 裁剪盒滤波（长方体区域）
#include <pcl/filters/crop_hull.h>                 // 裁剪凸包滤波（凸包内部）
#include <pcl/filters/frustum_culling.h>           // 视锥裁剪（模拟相机视锥）

// -------------------- 降采样相关 (4 种，其中体素和近似体素已包含，额外两种) --------------------
#include <pcl/filters/uniform_sampling.h>          // 均匀采样（体素内取最接近中心的点）
#include <pcl/filters/random_sample.h>             // 随机采样（随机抽取固定点数）

// -------------------- 辅助头文件 --------------------
#include <pcl/ModelCoefficients.h>                 // 模型系数（投影滤波需要）
#include <pcl/surface/convex_hull.h>               // 凸包计算（裁剪凸包需要）

#include <chrono>   // std::chrono 用于定时器
#include <memory>   // std::shared_ptr
#include <cstdlib>  // rand() 随机数
#include <string>   // std::string

// 为方便，将点类型定义为 pcl::PointXYZ
typedef pcl::PointXYZ PointT;

// ============================================================================
// 节点类：PCLDemo1Node
// 继承自 rclcpp::Node，实现所有功能
// ============================================================================
class PCLDemo1Node : public rclcpp::Node
{
public:
    // 构造函数
    PCLDemo1Node() : Node("pcl_demo1_node")
    {
        RCLCPP_INFO(this->get_logger(), "=== PCL 滤波与降采样 Demo (兼容版，已移除双边滤波) ===");

        // ---------- 步骤1：创建原始点云 ----------
        // 包含 500 个球面规则点 + 9500 个随机噪声点，共 10000 点
        original_cloud_ = createPointCloud();
        // 保存原始点云为 PCD 文件（二进制格式）
        savePointCloud(original_cloud_, "original_cloud.pcd");

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
        // 2.6 近似体素滤波：体素大小 0.1，快速近似版本（不计算质心，用体素中心代替）
        filtered_approx_voxel_   = filterApproxVoxel(original_cloud_, 0.1f);
        // 2.7 投影滤波：将所有点投影到 z=0 平面（XOY 平面）
        filtered_projection_     = filterProjection(original_cloud_);
        // 2.8 裁剪盒滤波：裁剪出 [-3,3]×[-3,3]×[-3,3] 范围内的点
        filtered_crop_box_       = filterCropBox(original_cloud_);
        // 2.9 裁剪凸包滤波：使用原始点云构建凸包，保留凸包内部的点
        filtered_crop_hull_      = filterCropHull(original_cloud_);
        // 2.10 视锥裁剪：模拟一个在原点看向 Z 正方向的相机视锥，保留视锥内的点
        filtered_frustum_        = filterFrustumCulling(original_cloud_);

        // 保存所有滤波结果到 PCD 文件
        savePointCloud(filtered_pass_through_,     "filtered_pass_through.pcd");
        savePointCloud(filtered_conditional_,      "filtered_conditional.pcd");
        savePointCloud(filtered_statistical_,      "filtered_statistical.pcd");
        savePointCloud(filtered_radius_,           "filtered_radius.pcd");
        savePointCloud(filtered_voxel_,            "filtered_voxel.pcd");
        savePointCloud(filtered_approx_voxel_,     "filtered_approx_voxel.pcd");
        savePointCloud(filtered_projection_,       "filtered_projection.pcd");
        savePointCloud(filtered_crop_box_,         "filtered_crop_box.pcd");
        savePointCloud(filtered_crop_hull_,        "filtered_crop_hull.pcd");
        savePointCloud(filtered_frustum_,          "filtered_frustum.pcd");

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
        savePointCloud(downsampled_voxel_,        "downsampled_voxel.pcd");
        savePointCloud(downsampled_approx_voxel_, "downsampled_approx_voxel.pcd");
        savePointCloud(downsampled_uniform_,      "downsampled_uniform.pcd");
        savePointCloud(downsampled_random_,       "downsampled_random.pcd");

        // ---------- 步骤4：创建 ROS2 发布者（共 15 个话题） ----------
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

        // ---------- 步骤5：立即发布一次所有点云 ----------
        // 使得已经在监听的订阅者（如 rviz2）能立即显示数据
        publishAllClouds();

        // ---------- 步骤6：创建定时器，每秒发布一次 ----------
        // 保证后启动的订阅者（如稍后打开的 rviz2）也能收到数据
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() { publishAllClouds(); }
        );

        // 打印提示信息，告知用户可订阅的话题
        RCLCPP_INFO(this->get_logger(), "Node ready. 共发布 15 个话题（1原始 + 10滤波 + 4降采样）");
        RCLCPP_INFO(this->get_logger(), "滤波话题: /pcl_demo1/pass_through, conditional, statistical, radius, voxel, approx_voxel, projection, crop_box, crop_hull, frustum");
        RCLCPP_INFO(this->get_logger(), "降采样话题: /pcl_demo1/down_voxel, down_approx_voxel, down_uniform, down_random");
    }

private:
    // ========================================================================
    // 成员变量：存储点云和发布者
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

    // ---------- ROS2 发布者 (15 个) ----------
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

    // ---------- 定时器 ----------
    rclcpp::TimerBase::SharedPtr timer_;

    // ========================================================================
    // 辅助函数
    // ========================================================================

    /**
     * @brief 创建原始点云，包含 10000 个点。
     *        前 500 个点位于半径为 5 的球面上，其余为 [-10,10] 范围内的随机点。
     * @return pcl::PointCloud<PointT>::Ptr 点云智能指针
     */
    pcl::PointCloud<PointT>::Ptr createPointCloud()
    {
        RCLCPP_INFO(this->get_logger(), "创建原始点云...");
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        cloud->width = 10000;          // 总点数
        cloud->height = 1;             // 无序点云
        cloud->is_dense = false;       // 可能存在 NaN 或无效点
        cloud->points.resize(cloud->width * cloud->height);

        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            // 生成随机坐标（范围 -10 ~ 10）
            float x = (rand() % 2000 - 1000) / 100.0f;
            float y = (rand() % 2000 - 1000) / 100.0f;
            float z = (rand() % 2000 - 1000) / 100.0f;

            // 前 500 个点替换为球面上的规则点（半径 5）
            if (i < 500)
            {
                float theta = 2.0f * M_PI * i / 500.0f;   // 方位角
                float phi = M_PI * i / 500.0f;            // 极角
                cloud->points[i].x = 5.0f * sin(phi) * cos(theta);
                cloud->points[i].y = 5.0f * sin(phi) * sin(theta);
                cloud->points[i].z = 5.0f * cos(phi);
            }
            else
            {
                // 其余点使用随机值
                cloud->points[i].x = x;
                cloud->points[i].y = y;
                cloud->points[i].z = z;
            }
        }

        RCLCPP_INFO(this->get_logger(), "原始点云共 %zu 个点", cloud->points.size());
        return cloud;
    }

    /**
     * @brief 将点云保存为二进制 PCD 文件。
     * @param cloud 点云智能指针
     * @param filename 文件名（相对路径）
     */
    void savePointCloud(const pcl::PointCloud<PointT>::Ptr& cloud, const std::string& filename)
    {
        // 若点云为空则跳过保存
        if (!cloud || cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "跳过空点云 %s", filename.c_str());
            return;
        }
        // 保存为二进制格式（节省空间，速度更快）
        if (pcl::io::savePCDFileBinary(filename, *cloud) == 0)
            RCLCPP_INFO(this->get_logger(), "保存 %s (%zu 点)", filename.c_str(), cloud->points.size());
        else
            RCLCPP_ERROR(this->get_logger(), "保存 %s 失败", filename.c_str());
    }

    // ========================================================================
    // 滤波方法 (10 种)
    // ========================================================================

    /**
     * @brief 直通滤波：沿指定轴保留在 [min, max] 范围内的点。
     * @param input 输入点云
     * @param field 滤波轴名称 ("x", "y", "z" 等)
     * @param min 下限
     * @param max 上限
     * @return 滤波后的点云
     */
    pcl::PointCloud<PointT>::Ptr filterPassThrough(const pcl::PointCloud<PointT>::Ptr& input,
                                                   const std::string& field,
                                                   float min, float max)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::PassThrough<PointT> f;                 // 创建滤波器对象
        f.setInputCloud(input);                     // 设置输入
        f.setFilterFieldName(field);                // 设置滤波字段
        f.setFilterLimits(min, max);                // 设置范围
        f.filter(*out);                             // 执行滤波，结果存入 out

        RCLCPP_INFO(this->get_logger(), "直通滤波 (%s in [%.1f,%.1f]) -> %zu 点",
                    field.c_str(), min, max, out->size());
        return out;
    }

    /**
     * @brief 条件滤波：保留满足所有条件（X>0 && Y>0 && Z>0）的点。
     * @param input 输入点云
     * @return 滤波后的点云
     */
    pcl::PointCloud<PointT>::Ptr filterConditional(const pcl::PointCloud<PointT>::Ptr& input)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);

        // 创建“与”条件：X>0, Y>0, Z>0
        pcl::ConditionAnd<PointT>::Ptr cond(new pcl::ConditionAnd<PointT>);
        cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(
            new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::GT, 0.0)));
        cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(
            new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::GT, 0.0)));
        cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(
            new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::GT, 0.0)));

        pcl::ConditionalRemoval<PointT> f;
        f.setCondition(cond);                       // 设置条件
        f.setInputCloud(input);
        f.filter(*out);

        RCLCPP_INFO(this->get_logger(), "条件滤波 (X>0 && Y>0 && Z>0) -> %zu 点", out->size());
        return out;
    }

    /**
     * @brief 统计滤波：对于每个点，计算其到 k 个近邻的平均距离。
     *        若该距离大于全局平均距离的 stddev 倍标准差，则视为离群点剔除。
     * @param input 输入点云
     * @param meanK 用于统计的近邻点数
     * @param stddev 标准差倍数阈值
     * @return 滤波后的点云
     */
    pcl::PointCloud<PointT>::Ptr filterStatisticalOutlier(const pcl::PointCloud<PointT>::Ptr& input,
                                                          int meanK, float stddev)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::StatisticalOutlierRemoval<PointT> f;
        f.setInputCloud(input);
        f.setMeanK(meanK);                         // 设置近邻数
        f.setStddevMulThresh(stddev);              // 设置标准差阈值
        f.filter(*out);

        RCLCPP_INFO(this->get_logger(), "统计滤波 (meanK=%d, stddev=%.1f) -> %zu 点",
                    meanK, stddev, out->size());
        return out;
    }

    /**
     * @brief 半径滤波：对于每个点，若其指定半径内的邻居数少于 minNeighbors，则剔除。
     * @param input 输入点云
     * @param radius 搜索半径
     * @param minNeighbors 最少邻居数
     * @return 滤波后的点云
     */
    pcl::PointCloud<PointT>::Ptr filterRadiusOutlier(const pcl::PointCloud<PointT>::Ptr& input,
                                                     float radius, int minNeighbors)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::RadiusOutlierRemoval<PointT> f;
        f.setInputCloud(input);
        f.setRadiusSearch(radius);                 // 设置搜索半径
        f.setMinNeighborsInRadius(minNeighbors);   // 设置最少邻居数
        f.filter(*out);

        RCLCPP_INFO(this->get_logger(), "半径滤波 (radius=%.2f, minNeighbors=%d) -> %zu 点",
                    radius, minNeighbors, out->size());
        return out;
    }

    /**
     * @brief 体素滤波：将点云划分为体素网格，每个体素内用质心代替所有点。
     *        同时可以达到降采样的效果。
     * @param input 输入点云
     * @param leaf 体素边长
     * @return 滤波后的点云
     */
    pcl::PointCloud<PointT>::Ptr filterVoxel(const pcl::PointCloud<PointT>::Ptr& input, float leaf)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::VoxelGrid<PointT> f;
        f.setInputCloud(input);
        f.setLeafSize(leaf, leaf, leaf);          // 设置三维体素大小
        f.filter(*out);

        RCLCPP_INFO(this->get_logger(), "体素滤波 (leaf=%.2f) -> %zu 点", leaf, out->size());
        return out;
    }

    /**
     * @brief 近似体素滤波：体素网格下采样的快速近似版本。
     *        不计算质心，直接用体素中心点坐标替代，速度更快。
     * @param input 输入点云
     * @param leaf 体素边长
     * @return 滤波后的点云
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
     * @brief 投影滤波：将点云投影到指定的几何模型上。
     *        本例投影到 z=0 平面（XOY 平面）。
     * @param input 输入点云
     * @return 投影后的点云（所有点 z 坐标变为 0）
     */
    pcl::PointCloud<PointT>::Ptr filterProjection(const pcl::PointCloud<PointT>::Ptr& input)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);

        // 定义平面模型系数：Ax+By+Cz+D=0，此处为 z=0 → A=0, B=0, C=1, D=0
        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
        coeff->values.resize(4);
        coeff->values[0] = 0.0;
        coeff->values[1] = 0.0;
        coeff->values[2] = 1.0;
        coeff->values[3] = 0.0;

        pcl::ProjectInliers<PointT> f;
        f.setModelType(pcl::SACMODEL_PLANE);      // 设置模型类型为平面
        f.setInputCloud(input);
        f.setModelCoefficients(coeff);            // 设置系数
        f.filter(*out);

        RCLCPP_INFO(this->get_logger(), "投影滤波 (投影到z=0) -> %zu 点", out->size());
        return out;
    }

    /**
     * @brief 裁剪盒滤波：保留指定长方体区域内的点。
     *        本例保留 [-3,3]×[-3,3]×[-3,3] 范围内的点。
     * @param input 输入点云
     * @return 裁剪后的点云
     */
    pcl::PointCloud<PointT>::Ptr filterCropBox(const pcl::PointCloud<PointT>::Ptr& input)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::CropBox<PointT> f;
        f.setInputCloud(input);
        // 设置包围盒最小角和最大角（第四个分量为 1，表示齐次坐标）
        f.setMin(Eigen::Vector4f(-3, -3, -3, 1));
        f.setMax(Eigen::Vector4f( 3,  3,  3, 1));
        // setNegative(false) 表示保留范围内的点（默认）
        f.filter(*out);

        RCLCPP_INFO(this->get_logger(), "裁剪盒滤波 ([-3,3]^3) -> %zu 点", out->size());
        return out;
    }

    /**
     * @brief 裁剪凸包滤波：利用点云构建凸包，保留凸包内部的点。
     * @param input 输入点云
     * @return 裁剪后的点云
     */
    pcl::PointCloud<PointT>::Ptr filterCropHull(const pcl::PointCloud<PointT>::Ptr& input)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);

        // 1. 构建凸包
        pcl::ConvexHull<PointT> hull;
        hull.setInputCloud(input);
        std::vector<pcl::Vertices> polygons;          // 存储凸包多边形的顶点索引
        pcl::PointCloud<PointT>::Ptr surface_hull(new pcl::PointCloud<PointT>);
        hull.reconstruct(*surface_hull, polygons);    // 重建凸包表面

        if (surface_hull->empty())
        {
            RCLCPP_WARN(this->get_logger(), "凸包构建失败，跳过裁剪凸包");
            return input;   // 若失败，返回原始点云
        }

        // 2. 使用凸包裁剪点云
        pcl::CropHull<PointT> f;
        f.setInputCloud(input);
        f.setHullIndices(polygons);                   // 设置凸包多边形索引
        f.setHullCloud(surface_hull);                 // 设置凸包点云
        f.setDim(3);                                  // 三维
        f.filter(*out);

        RCLCPP_INFO(this->get_logger(), "裁剪凸包滤波 -> %zu 点", out->size());
        return out;
    }

    /**
     * @brief 视锥裁剪：模拟相机视锥，保留视锥内部的点。
     *        本例设置相机位于原点，朝向 Z 正方向，水平视场角 60°，垂直 45°，近平面 0.1，远平面 10.0。
     * @param input 输入点云
     * @return 裁剪后的点云
     */
    pcl::PointCloud<PointT>::Ptr filterFrustumCulling(const pcl::PointCloud<PointT>::Ptr& input)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::FrustumCulling<PointT> f;
        f.setInputCloud(input);
        // 设置相机位姿（单位矩阵表示位于原点，无旋转）
        f.setCameraPose(Eigen::Matrix4f::Identity());
        // 设置视锥参数（角度需转为弧度）
        f.setHorizontalFOV(60.0f * M_PI / 180.0f);
        f.setVerticalFOV(45.0f * M_PI / 180.0f);
        f.setNearPlaneDistance(0.1f);
        f.setFarPlaneDistance(10.0f);
        f.filter(*out);

        RCLCPP_INFO(this->get_logger(), "视锥裁剪 -> %zu 点", out->size());
        return out;
    }

    // ========================================================================
    // 降采样方法 (4 种)
    // ========================================================================

    /**
     * @brief 体素降采样：与体素滤波相同，用体素质心代表点。
     *        这里单独列出以强调降采样用途。
     * @param input 输入点云
     * @param leaf 体素边长
     * @return 降采样后的点云
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
     * @brief 近似体素降采样：快速近似版本。
     * @param input 输入点云
     * @param leaf 体素边长
     * @return 降采样后的点云
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
     * @brief 均匀采样：将点云划分为体素，每个体素内保留距离体素中心最近的原始点。
     *        与体素降采样不同，它保留的是原始点坐标而非质心。
     * @param input 输入点云
     * @param radius 体素半径（实际为搜索半径）
     * @return 降采样后的点云
     */
    pcl::PointCloud<PointT>::Ptr downsampleUniform(const pcl::PointCloud<PointT>::Ptr& input, float radius)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::UniformSampling<PointT> f;
        f.setInputCloud(input);
        f.setRadiusSearch(radius);               // 设置体素大小（半径）
        f.filter(*out);

        RCLCPP_INFO(this->get_logger(), "均匀采样 (radius=%.2f) %zu -> %zu",
                    radius, input->size(), out->size());
        return out;
    }

    /**
     * @brief 随机采样：从输入点云中随机选择指定数量的点。
     * @param input 输入点云
     * @param sample 采样点数
     * @return 降采样后的点云
     */
    pcl::PointCloud<PointT>::Ptr downsampleRandom(const pcl::PointCloud<PointT>::Ptr& input, unsigned int sample)
    {
        pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
        pcl::RandomSample<PointT> f;
        f.setInputCloud(input);
        f.setSample(sample);                     // 设置采样点数
        f.filter(*out);

        RCLCPP_INFO(this->get_logger(), "随机采样 (sample=%u) %zu -> %zu",
                    sample, input->size(), out->size());
        return out;
    }

    // ========================================================================
    // 发布函数
    // ========================================================================

    /**
     * @brief 将存储的所有点云（原始、滤波、降采样）发布到对应话题。
     *        使用 lambda 函数简化重复代码。
     */
    void publishAllClouds()
    {
        // 定义发布 lambda：转换点云为 ROS2 消息并发布
        auto pub = [this](const pcl::PointCloud<PointT>::Ptr& cloud,
                          rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
                          const std::string& frame = "map")
        {
            if (!cloud || cloud->empty()) return;      // 空点云不发布
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*cloud, msg);                // 转换
            msg.header.frame_id = frame;               // 设置坐标系
            msg.header.stamp = this->now();            // 设置时间戳
            publisher->publish(msg);                   // 发布
        };

        // 发布原始点云
        pub(original_cloud_, pub_original_);

        // 发布 10 种滤波结果
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

        // 发布 4 种降采样结果
        pub(downsampled_voxel_, pub_down_voxel_);
        pub(downsampled_approx_voxel_, pub_down_approx_voxel_);
        pub(downsampled_uniform_, pub_down_uniform_);
        pub(downsampled_random_, pub_down_random_);
    }
};

// ============================================================================
// 主函数
// ============================================================================
int main(int argc, char** argv)
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);
    // 创建节点对象
    auto node = std::make_shared<PCLDemo1Node>();
    // 进入事件循环（阻塞），等待回调（定时器发布）
    rclcpp::spin(node);
    // 正常退出
    rclcpp::shutdown();
    return 0;
}