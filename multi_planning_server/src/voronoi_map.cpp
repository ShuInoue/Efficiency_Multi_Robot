#include<multi_planning_server/voronoi_map.hpp>
#include <exploration_libraly/path_planning.hpp>
#include <voronoi_planner/planner_core.h>
#include <navfn/navfn_ros.h>

using std::cout;
using std::endl;

voronoiMap::voronoiMap(std::string initTopicNameSpace_,std::string initGlobalFrameId_):tf(ros::Duration(10)),globalCostmap("global_costmap",tf)
{
    name = "global_costmap";
    topicNameSpace_=initTopicNameSpace_;
    globalFrameId_=initGlobalFrameId_;
}
voronoiMap::~voronoiMap()
{
}

void voronoiMap::mergeMapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& mergeMap)
{
    cout << "cb" << endl;
    map = *mergeMap;
    geometry_msgs::PoseStamped startData,goalData;
    poseSetter(startData);
    poseSetter(goalData);
    std::vector<geometry_msgs::PoseStamped> planData;
    createVoronoiMap(startData,goalData,planData);
}

nav_msgs::OccupancyGrid voronoiMap::mapGetter(void)
{
    return map;
}

void voronoiMap::poseSetter(geometry_msgs::PoseStamped& pose)
{
    std::cout << "start_pose_set" << std::endl;
    pose.header.frame_id = globalFrameId_;
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x=0.0;
    pose.pose.orientation.y=0.0;
    pose.pose.orientation.z=0.0;
    pose.pose.orientation.w=1.0;
    std::cout << "end start_pose_set" << std::endl;

}
bool voronoiMap::createVoronoiMap(geometry_msgs::PoseStamped& start,geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan)
{
    cout << "createVoronoiMap." << endl;
    ros::spinOnce();
    voronoi_planner::VoronoiPlanner VP;
    VP.initialize(name,&globalCostmap);
    return VP.makePlan(start,goal,plan);
}

int main(int argc, char **argv)
{
    cout << "voronoi_map started." << endl;
    ros::init(argc, argv, "voronoi_map");
    ros::NodeHandle nh("~");
    // geometry_msgs::PoseStamped start;
    // geometry_msgs::PoseStamped goal;
    // std::vector<geometry_msgs::PoseStamped> plan;
    std::string topicNameSpace;
    nh.getParam("namespace",topicNameSpace);
    cout << topicNameSpace << endl;
    std::string globalFrameId;
    nh.getParam("global_frame",globalFrameId);
    cout << globalFrameId << endl;
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::CallbackQueue queue;
    n.setCallbackQueue(&queue);
    // geometry_msgs::PoseStamped start, goal;
    // start.header.frame_id = globalFrameId;
    // goal.header.frame_id=globalFrameId;
    // std::vector<geometry_msgs::PoseStamped> path;
    // ExpLib::PathPlanning<navfn::NavfnROS> pp("global_cosmap", "voronoi_planner");
    // while(ros::ok())
    // {
    //     pp.createPath(start, goal, path);
    // }
    voronoiMap vm(topicNameSpace,globalFrameId);
    sub=n.subscribe("/robot1/move_base/global_costmap/costmap",1,&voronoiMap::mergeMapCallBack,&vm);
    cout << " etst "<< endl;
    cout << "setting completed." << endl;
    while(ros::ok())
    {
        cout << "main loop." << endl;
        queue.callOne(ros::WallDuration(1.0));
    }
    return 0;
}