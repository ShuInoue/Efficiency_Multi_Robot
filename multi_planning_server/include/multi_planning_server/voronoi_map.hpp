#ifndef _VORONOI_MAP_
#define _VORONOI_MAP_

#include<exploration_libraly/struct.hpp>
#include<voronoi_planner/planner_core.h>
#include<ros/ros.h>
#include<nav_msgs/OccupancyGrid.h>

class voronoiMap
{
    nav_msgs::OccupancyGrid map;

    public:
    voronoiMap();
    ~voronoiMap();
    nav_msgs::OccupancyGrid mapGetter(void);
    void mergeMapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& mergeMap);
};

#endif
