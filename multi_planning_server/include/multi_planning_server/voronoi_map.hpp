#ifndef _VORONOI_MAP_
#define _VORONOI_MAP_

#include<ros/ros.h>
#include<exploration_libraly/struct.h>
#include<voronoi_planner/planner_core.h>
#include<costmap_2d/costmap_2d_ros.h>
#include<nav_msgs/OccupancyGrid.h>

class voronoiMap
{
    private:
    tf::TransformListener tf;
    costmap_2d::Costmap2DROS globalCostmap;
    std::string name;
    nav_msgs::OccupancyGrid map;
    std::string topicNameSpace_;
    std::string globalFrameId_;
    

    public:
    voronoiMap(std::string initTopicNameSpace_,std::string initGlobalFrameId_);
    ~voronoiMap();
    nav_msgs::OccupancyGrid mapGetter(void);
    void poseSetter(std::string poseName, geometry_msgs::PoseStamped& pose);
    void mergeMapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& mergeMap);
    bool createVoronoiMap(geometry_msgs::PoseStamped& start,geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan);
    bool createVoronoiPath(geometry_msgs::PoseStamped& start,geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan);

};

#endif
