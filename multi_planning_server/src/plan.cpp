#include<multi_planning_server/plan.hpp>

using std::cout;
using std::endl;

double calcPathFromLocationToTarget(geometry_msgs::PoseStamped Goal, nav_msgs::Odometry Location, nav_msgs::Path path)
{
    cout << "calc start." << endl;
    double pathlength=0.0;
    double diffx, diffy;
    double locationToFirstPath, lastPathToGoal;
    
    diffx=path.poses[0].pose.position.x-Location.pose.pose.position.x;
    diffy=path.poses[0].pose.position.y-Location.pose.pose.position.y;
    cout << "diffx : " << diffx << endl;
    cout << "diffy : " << diffy << endl;
    locationToFirstPath=hypot((double)diffx,(double)diffy);
    cout << "locationToFirstPath : " << locationToFirstPath << endl;
    
    diffx=path.poses[path.poses.size()-1].pose.position.x-Goal.pose.position.x;
    diffy=path.poses[path.poses.size()-1].pose.position.y-Goal.pose.position.y;
    cout << "diffx : " << diffx << endl;
    cout << "diffy : " << diffy << endl;
    lastPathToGoal=hypot(diffx,diffy);
    cout << "lastPathToGoal : " << lastPathToGoal << endl;
    cout << "pathposessie : " << path.poses.size() << endl;
    for(int i=0; i<path.poses.size()-1;i++)
    {
        diffx=path.poses[i+1].pose.position.x - path.poses[i].pose.position.x;
        diffy=path.poses[i+1].pose.position.y - path.poses[i].pose.position.y;
        cout << "diffx : " << diffx << endl;
        cout << "diffy : " << diffy << endl;
        pathlength += hypot(diffx, diffy);
        cout << "pathlength" << i << " : " << lastPathToGoal << endl;
    }
    pathlength = pathlength+locationToFirstPath+lastPathToGoal;
    cout << "calc end" << endl;
    return pathlength;
}
// void targetSort();
// void combinationPath();

int main(void)
{
    int count=0;
    nav_msgs::Path testPath;
    nav_msgs::Odometry testOdom;
    geometry_msgs::PoseStamped testGoal;

    testOdom.pose.pose.position.x=0;
    testOdom.pose.pose.position.y=0;

    testGoal.pose.position.x=11;
    testGoal.pose.position.y=11;
    testPath.poses.resize(10);
    for(int i=0;i<10;i++)
    {
        testPath.poses[i].pose.position.x=i+1;
        testPath.poses[i].pose.position.y=i+1;
    }
    cout << "pathlength : " << calcPathFromLocationToTarget(testGoal,testOdom, testPath) << endl;

    return 0;
}