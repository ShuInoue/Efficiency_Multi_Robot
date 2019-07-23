#include<multi_planning_server/plan.hpp>

using std::cout;
using std::endl;
using std::sort;
using std::get;
using std::setw;

plan::plan()
{
    ros::NodeHandle nh("~");
    nh.getParam("number_of_robots",numberOfRobots);
}

double plan::getDistance(double x, double y, double x2, double y2)
{
    // cout << "data : " << x <<" "<<  y <<" "<<  x2 <<" "<< y2 << endl;
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
void plan::recursive_comb(int *indexes, int s, int rest, std::function<void(int *)> f)
{
    if(rest == 0)
    {
        f(indexes);
    }
    else
    {
        if(s<0) return ;
        recursive_comb(indexes, s-1, rest, f);
        indexes[rest - 1]=s;
        recursive_comb(indexes,s-1,rest-1,f);
    }
}

void plan::foreach_comb(int n, int k, std::function<void(int *)> f)
{
    int indexes[k];
    recursive_comb(indexes, n-1, k, f);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "plan");
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

    std::vector<robotData> testRobotData;
    int id=0;
    std::random_device rnd;
    double locationx=0.0;
    double locationy=0.0;
    cout << setw(15) << "goalx" << setw(15) << "goaly" << setw(15) << "locationx" <<  setw(15) << "locationy" << setw(15) << "Path" << setw(15) << "id" << endl;
    for(int i=0; i<10; i++)
    {
        double rndtergetx,rndtargety;
        rndtergetx=rnd();
        rndtargety=rnd();
        geometry_msgs::PoseStamped tmpgoal;
        tmpgoal.pose.position.x = rndtergetx;
        tmpgoal.pose.position.y = rndtargety;

        nav_msgs::Odometry tmplocation;
        tmplocation.pose.pose.position.x = locationx;
        tmplocation.pose.pose.position.y = locationy;

        nav_msgs::Path dummyPath;
        
        robotData tmprobotdata={tmpgoal,tmplocation,dummyPath,p.getDistance(rndtergetx,rndtargety,locationx,locationy),id++};
        testRobotData.push_back(tmprobotdata);
        cout << setw(15) << testRobotData[i].goal.pose.position.x << setw(15) << testRobotData[i].goal.pose.position.y << setw(15) << testRobotData[i].location.pose.pose.position.x <<  setw(15) << testRobotData[i].location.pose.pose.position.y << setw(15) << testRobotData[i].pathLength << setw(15) << testRobotData[i].memberID << endl;
    }
    std::sort(testRobotData.begin(),testRobotData.end());
    cout << "------------------------------------------------------------------------------------------------------" << endl;
    for(int i=0; i<10; i++)
        {
        cout << setw(15) << testRobotData[i].goal.pose.position.x << setw(15) << testRobotData[i].goal.pose.position.y << setw(15) << testRobotData[i].location.pose.pose.position.x <<  setw(15) << testRobotData[i].location.pose.pose.position.y << setw(15) << testRobotData[i].pathLength << setw(15) << testRobotData[i].memberID << endl;
    }

    p.foreach_comb(5,4,[](int *indexes)
    {
        std::cout << indexes[0] << "," << indexes[1] << "," << indexes[2] << "," << indexes[3] << std::endl;
    });

    return 0;
}