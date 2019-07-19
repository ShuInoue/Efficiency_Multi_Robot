#include<multi_planning_server/plan.hpp>

using std::cout;
using std::endl;

double plan::getDistance(double x, double y, double x2, double y2)
{
    cout << "data : " << x <<" "<<  y <<" "<<  x2 <<" "<< y2 << endl;
    double distance=sqrt(pow(x2-x,2)+pow(y2-y,2));
    return distance;
}
double plan::calcPathFromLocationToTarget(geometry_msgs::PoseStamped Goal, nav_msgs::Odometry Location, nav_msgs::Path path)
{
    cout << "calc start." << endl;
    double pathlength=0.0;
    double locationToFirstPath, lastPathToGoal;
    
    locationToFirstPath=getDistance(Location.pose.pose.position.x,Location.pose.pose.position.y,path.poses[0].pose.position.x,path.poses[0].pose.position.y);
    cout << "locationToFirstPath : " << locationToFirstPath << endl;
    
    lastPathToGoal=getDistance(path.poses[path.poses.size()-1].pose.position.x,path.poses[path.poses.size()-1].pose.position.y,Goal.pose.position.x,Goal.pose.position.y);
    cout << "lastPathToGoal : " << lastPathToGoal << endl;

    cout << "pathposessie : " << path.poses.size() << endl;
    for(int i=0; i<path.poses.size()-1;i++)
    {
        pathlength += getDistance(path.poses[i+1].pose.position.x, path.poses[i+1].pose.position.y, path.poses[i].pose.position.x,path.poses[i].pose.position.y);
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
    plan p;
    int count=0;
    nav_msgs::Path testPath;
    nav_msgs::Odometry testOdom;
    geometry_msgs::PoseStamped testGoal;

    testOdom.pose.pose.position.x=0;
    testOdom.pose.pose.position.y=0;

    testGoal.pose.position.x=-11;
    testGoal.pose.position.y=-11;
    testPath.poses.resize(10);
    for(int i=0;i<10;i++)
    {
        testPath.poses[i].pose.position.x=-i-1;
        testPath.poses[i].pose.position.y=-i-1;
    }
    cout << "pathlength : " << p.calcPathFromLocationToTarget(testGoal,testOdom, testPath) << endl;

    return 0;
}