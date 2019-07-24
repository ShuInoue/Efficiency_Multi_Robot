#ifndef _PLAN_HPP_
#define _PLAN_HPP_
#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseArray.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/Path.h>
#include<math.h>
#include<vector>
#include<string>
#include<algorithm>
#include<iomanip>
#include<random>

struct robotData
{
    geometry_msgs::PoseStamped goal;
    nav_msgs::Odometry location;
    nav_msgs::Path path;
    double pathLength;
    int memberID;

    bool operator<(const robotData &another) const
    {
        return  pathLength < another.pathLength;
    }
};
struct combinatedPaths
{
    double combinatedPathLength;
    std::vector<int> chosenID;
        bool operator<(const combinatedPaths &another) const
    {
        return  combinatedPathLength < another.combinatedPathLength;
    }
};
typedef struct combinatedPaths combinatedPaths_t;
class plan
{
    int numberOfRobots;
    int indexes;
    public:
    plan();
    void robotDataSetter(std::vector<robotData>& testRobotData);
    double calcPathFromLocationToTarget(geometry_msgs::PoseStamped Goal, nav_msgs::Odometry Location, nav_msgs::Path path);
    double getDistance(double x, double y, double x2, double y2);
    std::vector<combinatedPaths_t> combinatedPaths(std::vector<robotData> robot1data, std::vector<robotData> robot2data);
};

#endif