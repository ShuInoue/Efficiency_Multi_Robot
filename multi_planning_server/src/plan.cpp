#include<multi_planning_server/plan.hpp>

double calcPathFromLocationToTarget(geometry_msgs::PoseStamped Goal, nav_msgs::Odometry Location, nav_msgs::Path path)
{
    double pathlength=0.0;
    double diffx, diffy;
    double locationToFirstPath, lastPathToGoal;
    
    diffx=path.poses[0].pose.position.x-Location.pose.pose.position.x;
    diffy=path.poses[0].pose.position.y-Location.pose.pose.position.y;
    locationToFirstPath=hypot(diffx,diffy);
    
    diffx=path.poses[path.poses.size()].pose.position.x-Goal.pose.position.x;
    diffy=path.poses[path.poses.size()].pose.position.y-Goal.pose.position.y;
    lastPathToGoal=hypot(diffx,diffy);

    for(int i=0; i<path.poses.size();i++)
    {
        diffx=path.poses[i+1].pose.position.x - path.poses[i].pose.position.x;
        diffy=path.poses[i+1].pose.position.y - path.poses[i].pose.position.y;
        pathlength += hypot(diffx, diffy);
    }
    pathlength = pathlength+locationToFirstPath+lastPathToGoal;
    return pathlength;
}
// void targetSort();
// void combinationPath();

int main(void)
{
    return 0;
}