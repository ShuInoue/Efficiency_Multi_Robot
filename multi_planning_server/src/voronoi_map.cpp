#include<multi_planning_server/voronoi_map.hpp>

std::string name="voronoi_map";
costmap_2d::Costmap2D* costmap;
std::string frame_id="test_frame";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "voronoi_map");
    ros::NodeHandle nh("~");


    DynamicVoronoi* voronoimap;
    voronoi_planner::VoronoiPlanner VP(name,costmap,frame_id);

    voronoiMap vm;
    while (ros::ok())
    {
        ros::spinOnce();
        VP.initialize(name,costmap,frame_id);
        nav_msgs::OccupancyGrid map=vm.mapGetter();
        VP.publishVoronoiGrid(voronoimap,map);
    }
    
}

void voronoiMap::mergeMapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& mergeMap)
{
    map = *mergeMap;
}

nav_msgs::OccupancyGrid voronoiMap::mapGetter(void)
{
    return map;
}