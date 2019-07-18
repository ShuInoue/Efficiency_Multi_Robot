#ifndef _PLAN_HPP_
#define _PLAN_HPP_
#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseArray.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/Path.h>
#include<math.h>
#include<vector>

class plan
{
    // struct robotData
    // {
    //     std::vector<geometry_msgs::PoseStamped> candidateGoals;
    //     std::vector<nav_msgs::Path> candidatePaths;
    //     std::vector<nav_msgs::Odometry> currentlocation;
    // };
    double calcPathFromLocationToTarget(geometry_msgs::PoseStamped Goal, nav_msgs::Odometry Location, nav_msgs::Path path);
    // void targetSort();
    // void combinationPath();
};

#endif