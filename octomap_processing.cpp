#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <octomap/OcTree.h>
#include <math.h>
#include <time.h>
#include <chrono>

using namespace octomap;

std::string metric;

point3d* get_nearest_point(OcTree& octree, point3d ray_start, point3d ray_end)
{
    //auto start_ray = std::chrono::high_resolution_clock::now();
    KeyRay ray;
    octree.computeRayKeys(ray_start, ray_end, ray);
    //auto end_ray = std::chrono::high_resolution_clock::now();
    //std::chrono::duration<double> diff = end_ray - start_ray;
    //std::cout << "Ray computing time: " << diff.count() << std::endl;
    //std::cout << "points in ray: " << std::endl;
    for (auto it = ray.begin(); it != ray.end(); it++)
    {
        OcTreeNode* node_found = octree.search(*it);
        /*if (node_found != nullptr)
        {
            point3d coords = octree.keyToCoord(*it);
            std::cout << coords.x() << ' ' << coords.y() << ' ' << coords.z() << std::endl;
        }*/
        if ((node_found != nullptr) && (octree.isNodeOccupied(*node_found)))
        {
            point3d* coords = new point3d;
            *coords = octree.keyToCoord(*it);
            return coords;
        }
    }
    //std::cout << std::endl;
    return nullptr;
}

double distance(point3d a, point3d b)
{
    point3d diff = a - b;
    return sqrt(diff.x() * diff.x() + diff.y() * diff.y() + diff.z() * diff.z());
}

double compare_pointclouds(const Pointcloud& gt_points,
                           const std::vector<pose6d>& gt_poses, 
                           const Pointcloud& slam_points, 
                           const std::vector<pose6d>& slam_poses,
                           double resolution,
                           std::ofstream& output)
{
    OcTree gt_octomap(resolution);
    OcTree slam_octomap(resolution);
    gt_octomap.insertPointCloud(gt_points, point3d(0., 0., 0.));
    slam_octomap.insertPointCloud(slam_points, point3d(0, 0, 0));
    double sum_distance = 0;
    double n_points = 0;
    std::cout << "Slam poses size is " << slam_poses.size() << std::endl;
    std::cout << "Groundtruth poses size is " << gt_poses.size() << std::endl;
    for (int i = 0; i < slam_poses.size(); i++)
    {
        std::cout << "i: " << i << std::endl;
        point3d slam_position = slam_poses[i].trans();
        point3d gt_position = gt_poses[i].trans();
        for (int h = 0; h < 240; h++)
        {
            for (int w = 0; w < 320; w++)
            {
                point3d direction_to_pixel(1, -double(w - 160) / 160., -double(h - 120) / 160.);
                point3d slam_ray_end = slam_poses[i].transform(direction_to_pixel * 100);
                point3d gt_ray_end = gt_poses[i].transform(direction_to_pixel * 100);
                point3d* nearest_point_slam = get_nearest_point(slam_octomap, slam_position, slam_ray_end);
                point3d* nearest_point_gt = get_nearest_point(gt_octomap, gt_position, gt_ray_end);
                if (nearest_point_slam != nullptr)
                {
                    n_points += 1;
                    if (nearest_point_gt == nullptr)
                    {
                        std::cout << "WARNING: no point in groundtruth map on pixel " << h << ' ' << w << std:: endl;
                        sum_distance += 0;
                    }
                    else
                    {
                        if (metric.substr(0, 3) == "abs")
                        {
                            sum_distance += distance(*nearest_point_slam, *nearest_point_gt);
                        }
                        if (metric.substr(0, 3) == "rel")
                        {
                            sum_distance += distance(*nearest_point_slam - slam_position, *nearest_point_gt - gt_position);
                        }
                    }
                }
                else
                {
                	std::cout << "WARNING: no point in predicted map on pixel " << h << ' ' << w << std:: endl;
                }
            }
        }
        std::cout << "Metric on i-th step: " << sum_distance / n_points << std::endl;
        output << sum_distance / n_points << std::endl;
    }
    return sum_distance / n_points;
}

int main(int argc, char* argv[]) {
    std::ifstream gt_points_file(argv[1]);
    std::ifstream gt_poses_file(argv[2]);
    std::ifstream slam_points_file(argv[3]);
    std::ifstream slam_poses_file(argv[4]);
    metric = std::string(argv[5]);
    std::ofstream metric_output(argv[6]);
    float x, y, z, roll, pitch, yaw;
    Pointcloud gt_points, slam_points;
    std::vector<pose6d> gt_poses;
    std::vector<pose6d> slam_poses;
    while (gt_points_file >> x >> y >> z)
        gt_points.push_back(point3d(x, y, z));
    while (slam_points_file >> x >> y >> z)
        slam_points.push_back(point3d(x, y, z));
    std::cout << "GT and slam pointcloud sizes: " << gt_points.size() << ' ' << slam_points.size() << std::endl;
    while (gt_poses_file >> x >> y >> z >> roll >> pitch >> yaw)
        gt_poses.push_back(pose6d(x, y, z, roll, pitch, yaw));
    while (slam_poses_file >> x >> y >> z >> roll >> pitch >> yaw)
        slam_poses.push_back(pose6d(x, y, z, roll, pitch, yaw));
    double metric = compare_pointclouds(gt_points, gt_poses, slam_points, slam_poses, 0.1, metric_output);
    std::cout << "Mean distance: " << metric << std::endl;
    gt_points_file.close();
    gt_poses_file.close();
    slam_points_file.close();
    slam_poses_file.close();
    metric_output.close();
    return 0;
}