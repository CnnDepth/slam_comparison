#include <iostream>
#include <fstream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;


int main(int argc, char* argv[]) {
    char* pcd_topic = argv[1];
    std::cout << "Publish to topic " << pcd_topic << std::endl;
    ros::init (argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud> (pcd_topic, 1);
    std::ifstream input_points("/home/kirill/habitat-api/points.txt");
    std::ifstream input_rgbs("/home/kirill/habitat-api/rgbs.txt");

    PointCloud::Ptr msg (new PointCloud);
    msg->header.frame_id = "points";
    msg->height = 1;
    double x, y, z;
    int r_, g_, b_;
    std::uint8_t r, g, b;
    while (input_points >> x >> y >> z)
    {
        input_rgbs >> r_ >> g_ >> b_;
        r = (uint8_t)(r_);
        g = (uint8_t)(g_);
        b = (uint8_t)(b_);
        pcl::PointXYZRGB point;
        point.x = x; point.y = y; point.z = z;
        point.r = r; point.g = g; point.b = b;
        msg->points.push_back(point);
    }
    msg->width = msg->points.size();
    std::cout << "Cloud contains " << msg->points.size() << " points" << std::endl;
    std::cout << "First point coords: " << msg->points[0].x << ' ' << msg->points[0].y << ' ' << msg->points[0].z << std::endl;
    std::cout << "First point color: " << msg->points[0].r << ' ' << msg->points[0].g << ' ' << msg->points[0].b << std::endl;

    ros::Rate loop_rate(4);
    while (nh.ok())
    {
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
        pub.publish (msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
