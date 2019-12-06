#ifndef _PLAN_HPP_
#define _PLAN_HPP_
#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseArray.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/Path.h>
#include<math.h>
#include<vector>
#include<string>
#include<algorithm>
#include<iomanip>
#include<random>
#include<iostream>
#include<functional>
#include<exploration_msgs/FrontierArray.h>
#include<exploration_libraly/struct.h>
#include<exploration_libraly/convert.h>
#include<multi_planning_server/voronoi_map.hpp>
#include<actionlib_msgs/GoalStatus.h>
#include<move_base_msgs/MoveBaseAction.h>
#include<actionlib_msgs/GoalStatusArray.h>
#include<actionlib/client/simple_action_client.h>
#include<voronoi_planner/planner_core.h>

struct robotData
{
    geometry_msgs::PoseStamped goal;
    nav_msgs::Odometry location;
    nav_msgs::Path path;
    double pathLength;
    int memberID;

    bool operator<(const robotData& another) const
    {
        return  pathLength < another.pathLength;
    }
    bool operator==(const robotData& r) const
    {
        return this->memberID==r.memberID;
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
    std::string name="global_costmap";
    tf::TransformListener tf;
    costmap_2d::Costmap2DROS globalCostmap;
    int numberOfRobots;
    int numberOfFrontiers;
    int *indexes;
    int avoidedTargetCounter;
    ros::Time extractionTimeStamp;
    std::vector<geometry_msgs::Point> recievedFrontierCoordinates;
    nav_msgs::Odometry recievedOdometry;
    std::vector<std::vector<int>> combinatedPatern;

    double getDistance(double x, double y, double x2, double y2);
    robotData transrateFromCombinatedPathsToRobotData(combinatedPaths_t comb, int robotNum);
    void recursive_comb(int *indexes, int s, int rest, std::function<void(int *)> f);
    void foreach_comb(int n, int k, std::function<void(int *)> f);
    double pathlengthFoundwithID(std::vector<std::vector<robotData>> &robotDataYouWantToKnowPathlength, std::vector<int> chosenID);
    bool avoidTargetInRobot(nav_msgs::Odometry& nowLocation,geometry_msgs::PoseStamped& candidateTarget);
    //void navStatusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr &status);


    public:
    plan();
    voronoi_planner::VoronoiPlanner vp;
    double waitTimeByDistance;
    //bool isRobotReachedGoal;
    int numberOfRobotGetter(void);
    void recievedFrontierCoordinatesSetter(const exploration_msgs::FrontierArray& recievedData);
    void robotDataSetter(const exploration_msgs::FrontierArray& frontiers, const geometry_msgs::PoseStamped& recievedOdometry,std::vector<robotData>& testRobotData);
    double calcPathFromLocationToTarget(geometry_msgs::PoseStamped Goal, nav_msgs::Odometry Location, nav_msgs::Path path);
    ros::Time timeStampGetter(void);
    std::vector<combinatedPaths_t> combinatedPaths(std::vector<std::vector<robotData>>robotDatas);
    std::vector<geometry_msgs::PoseStamped> robotToTarget(std::vector<combinatedPaths_t> combinatedPathsStruct, std::vector<std::vector<robotData>> tmpRobotDatas);
};

#endif