#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <fstream>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

std::vector<pcl::PointXYZRGB> points;
std::vector<geometry_msgs::PoseStamped> predicted_path;
std::vector<geometry_msgs::PoseStamped> true_path;
std::vector<ros::Time> predicted_timestamps;
std::vector<ros::Time> true_timestamps;

void pcd_callback(const PointCloud::ConstPtr& msg)
{
    printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    points.clear();
    BOOST_FOREACH (pcl::PointXYZRGB pt, msg->points)
    {
        points.push_back(pt);
    }
}

void predicted_path_callback(const nav_msgs::Path& msg)
{
    predicted_path.clear();
    predicted_timestamps.push_back(msg.header.stamp);
    BOOST_FOREACH (geometry_msgs::PoseStamped pose, msg.poses)
    {
        predicted_path.push_back(pose);
    }
}

void true_path_callback(const nav_msgs::Path& msg)
{
    true_path.clear();
    true_timestamps.push_back(msg.header.stamp);
    BOOST_FOREACH (geometry_msgs::PoseStamped pose, msg.poses)
    {
        true_path.push_back(pose);
    }
}

int main(int argc, char** argv)
{
    char* pcd_topic = argv[1];
    char* predicted_path_topic = "/mapPath";
    char* true_path_topic = "/true_path";
    std::cout << "subscribing to " << pcd_topic << std::endl;
    ros::init(argc, argv, "sub_pcl");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<PointCloud>(pcd_topic, 1, pcd_callback);
    ros::Subscriber predicted_path_subscriber = nh.subscribe(predicted_path_topic,
                                                             1,
                                                             predicted_path_callback);
    ros::Subscriber true_path_subscriber = nh.subscribe(true_path_topic,
                                                        1,
                                                        true_path_callback);
    ros::spin();
    // save pointcloud
    std::cout << "Saving pointcloud..." << std::endl;
    std::string save_dir(argv[2]);
    std::ofstream points_output(save_dir + "/points.txt");
    std::ofstream rgbs_output(save_dir + "/rgbs.txt");
    std::ofstream true_positions_output(save_dir + "/gt_positions.txt");
    std::ofstream true_rotations_output(save_dir + "/gt_rotations.txt");
    std::ofstream true_timestamps_output(save_dir + "/gt_times.txt");
    std::ofstream predicted_positions_output(save_dir + "/slam_positions.txt");
    std::ofstream predicted_rotations_output(save_dir + "/slam_rotations.txt");
    std::ofstream predicted_timestamps_output(save_dir + "/slam_times.txt");
    std::cout << save_dir << std::endl;
    /*points_output << \
"VERSION .5\n\
FIELDS x y z\n\
SIZE 4 4 4\n\
TYPE F F F\n\
COUNT 1 1 1\n\
WIDTH " << points.size() << std::endl << \
"HEIGHT 1\n\
POINTS " << points.size() << std::endl << \
"DATA ascii" << std::endl;*/
    for (int i = 0; i < points.size(); i++)
    {
        points_output << points[i].x << ' ' << points[i].y << ' ' << points[i].z << std::endl;
        rgbs_output << points[i].r << ' ' << points[i].g << ' ' << points[i].b << std::endl;
    }
    std::cout << "Path sizes: " << true_path.size() << ' ' << predicted_path.size() << std::endl;
    for (int i = 0; i < true_path.size(); i++)
    {
        true_positions_output << true_path[i].pose.position.x << ' ' \
                              << true_path[i].pose.position.y << ' ' \
                              << true_path[i].pose.position.z << std::endl;
        true_rotations_output << true_path[i].pose.orientation.x << ' ' \
                              << true_path[i].pose.orientation.y << ' ' \
                              << true_path[i].pose.orientation.z << ' ' \
                              << true_path[i].pose.orientation.w << std::endl;
        true_timestamps_output << true_timestamps[i] << std::endl;
    }
    for (int i = 0; i < predicted_path.size(); i++)
    {
        predicted_positions_output << predicted_path[i].pose.position.x << ' ' \
                              << predicted_path[i].pose.position.y << ' ' \
                              << predicted_path[i].pose.position.z << std::endl;
        predicted_rotations_output << predicted_path[i].pose.orientation.x << ' ' \
                              << predicted_path[i].pose.orientation.y << ' ' \
                              << predicted_path[i].pose.orientation.z << ' ' \
                              << predicted_path[i].pose.orientation.w << std::endl;
        predicted_timestamps_output << predicted_timestamps[i] << std::endl;
    }
    points_output.close();
    rgbs_output.close();
    true_positions_output.close();
    true_rotations_output.close();
    predicted_positions_output.close();
    predicted_rotations_output.close();
    true_timestamps_output.close();
    predicted_timestamps_output.close();
}