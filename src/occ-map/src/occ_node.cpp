#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

class VelodyneToGrid {
public:
    VelodyneToGrid() {
        sub_ = nh_.subscribe("/velodyne_points", 1, &VelodyneToGrid::pointCloudCallback, this);
        pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/local_map", 1);

        // OccupancyGrid 기본 설정
        grid_.info.resolution = 0.2; // 10cm
        grid_.info.width = 50;      // 20m
        grid_.info.height = 50;     // 20m
        grid_.info.origin.position.x = -5.0;
        grid_.info.origin.position.y = -5.0;
        grid_.info.origin.position.z = -0.7;
        grid_.data.resize(grid_.info.width * grid_.info.height, 0);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // OccupancyGrid 초기화
        std::fill(grid_.data.begin(), grid_.data.end(), 0);

        double z_threshold = -0.2;  // z 값이 이보다 낮으면 바닥으로 간주하여 제거

        for (const auto& pt : cloud.points) {
            if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;

            // 바닥 제거
            if (pt.z < z_threshold) continue;

            // 3차원 좌표를 2차원의 grid로 변환
            int x_idx = static_cast<int>((pt.x - grid_.info.origin.position.x) / grid_.info.resolution);
            int y_idx = static_cast<int>((pt.y - grid_.info.origin.position.y) / grid_.info.resolution);

            if (x_idx >= 0 && x_idx < grid_.info.width && y_idx >= 0 && y_idx < grid_.info.height) {
                int index = y_idx * grid_.info.width + x_idx;
                grid_.data[index] = 100; // 점이 있으면 점유 셀
            }
        }

        // 라이다가 차량 전방에 있어 차량 자체를 장애물로 인식할 수 있음으로 차량의 occupancy grid를 0으로 만들기
        for (int x = 21; x <= 25; x++){
            for(int y = 22; y <= 27; y++){
                int index = y * grid_.info.width + x;
                grid_.data[index] = 0;
            }
        }

        grid_.header.stamp = ros::Time::now();
        grid_.header.frame_id = "velodyne"; // 필요시 수정 velodyne이였음
        pub_.publish(grid_);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    nav_msgs::OccupancyGrid grid_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velodyne_to_grid");
    VelodyneToGrid vtg;
    ros::spin();
    return 0;
}
