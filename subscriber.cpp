#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <fstream>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

std::vector<pcl::PointXYZRGB> points;
std::vector<geometry_msgs::PoseStamped> path;
std::vector<ros::Time> timestamps;

void pcd_callback(const PointCloud::ConstPtr& msg)
{
    printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    points.clear();
    BOOST_FOREACH (pcl::PointXYZRGB pt, msg->points)
    {
        points.push_back(pt);
    }
}

void path_callback(const nav_msgs::Path& msg)
{
    path.clear();
    timestamps.push_back(msg.header.stamp);
    BOOST_FOREACH (geometry_msgs::PoseStamped pose, msg.poses)
    {
        path.push_back(pose);
    }
}

int main(int argc, char** argv)
{
    char* pcd_topic = argv[1];
    char* path_topic = "/mapPath";
    std::cout << "subscribing to " << pcd_topic << std::endl;
    ros::init(argc, argv, "sub_pcl");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<PointCloud>(pcd_topic, 1, pcd_callback);
    ros::Subscriber path_subscriber = nh.subscribe(path_topic, 1, path_callback);
    ros::spin();
    // save pointcloud
    std::cout << "Saving pointcloud..." << std::endl;
    std::ofstream points_output(argv[2]);
    std::ofstream rgbs_output(argv[3]);
    // write pointcloud file cap
    points_output << \
"VERSION .5\n\
FIELDS x y z\n\
SIZE 4 4 4\n\
TYPE F F F\n\
COUNT 1 1 1\n\
WIDTH " << points.size() << std::endl << \
"HEIGHT 1\n\
POINTS " << points.size() << std::endl << \
"DATA ascii" << std::endl;
    // write pointcloud file data
    for (int i = 0; i < points.size(); i++)
    {
        points_output << points[i].x << ' ' << points[i].y << ' ' << points[i].z << std::endl;
        rgbs_output << (int)points[i].r << ' ' << (int)points[i].g << ' ' << (int)points[i].b << std::endl;
    }
    points_output.close();
    rgbs_output.close();
}