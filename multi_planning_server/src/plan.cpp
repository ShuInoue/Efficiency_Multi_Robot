#include<multi_planning_server/plan.hpp>

using std::cout;
using std::endl;
using std::sort;
using std::get;
using std::setw;

plan::plan()
{
    ros::NodeHandle nh("~");
    //nh.getParam("number_of_robots",numberOfRobots);
    numberOfRobots=3;
}
int plan::numberOfRobotGetter(void)
{
    return numberOfRobots;
}
// ここで処理する用に用意したrobotData型のインスタンスにデータをセットする関数
void plan::robotDataSetter(std::vector<robotData>& testRobotData)
{
    int count=0;
    nav_msgs::Path testPath;
    nav_msgs::Odometry testOdom;
    geometry_msgs::PoseStamped testGoal;

    testOdom.pose.pose.position.x=0;
    testOdom.pose.pose.position.y=0;
    testGoal.pose.position.x=11;
    testGoal.pose.position.y=sin(11);
    testPath.poses.resize(10);
    for(int i=0;i<10;i++)
    {
        testPath.poses[i].pose.position.x=i+1;
        testPath.poses[i].pose.position.y=sin(i+1);
    }
    cout << "pathlength : " << calcPathFromLocationToTarget(testGoal,testOdom, testPath) << endl;

    int id=1;
    std::random_device rnd;
    double locationx=0.0;
    double locationy=0.0;
    cout << setw(15) << "goalx" << setw(15) << "goaly" << setw(15) << "locationx" <<  setw(15) << "locationy" << setw(15) << "Path" << setw(15) << "id" << endl;
    for(int i=0; i<3; i++)
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

        robotData tmprobotdata={tmpgoal,tmplocation,dummyPath,getDistance(rndtergetx,rndtargety,locationx,locationy),id++};
        testRobotData.push_back(tmprobotdata);
        cout << setw(15) << testRobotData[i].goal.pose.position.x << setw(15) << testRobotData[i].goal.pose.position.y << setw(15) << testRobotData[i].location.pose.pose.position.x <<  setw(15) << testRobotData[i].location.pose.pose.position.y << setw(15) << testRobotData[i].pathLength << setw(15) << testRobotData[i].memberID << endl;
    }
    numberOfFrontiers=id;
}
// ロボットの現在位置から目的地までの距離を計算する関数
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
        pathlength += getDistance(path.poses[i].pose.position.x, path.poses[i].pose.position.y, path.poses[i+1].pose.position.x,path.poses[i+1].pose.position.y);
        cout << "pathlength" << i << " : " << lastPathToGoal << endl;
    }
    pathlength = pathlength+locationToFirstPath+lastPathToGoal;
    cout << "calc end" << endl;
    return pathlength;
}
//パスの情報を距離の長さに変換する関数
double plan::getDistance(double x, double y, double x2, double y2)
{
    // cout << "data : " << x <<" "<<  y <<" "<<  x2 <<" "<< y2 << endl;
    double distance=sqrt(pow(x2-x,2)+pow(y2-y,2));
    return distance;
}
//CombinatedPaths型からrobtoData型に変化する関数
robotData plan::transrateFromCombinatedPathsToRobotData(combinatedPaths_t comb, int robotNum)
{
    robotData tmpRobotData;
    tmpRobotData.memberID = comb.chosenID[robotNum-1];
    return tmpRobotData;
}
// 各ロボットの各目的地までの距離を組み合わせて合計していく関数（未完成）
std::vector<combinatedPaths_t> plan::combinatedPaths(std::vector<std::vector<robotData>> robotDatas)
{
    for(int i=0;i<robotDatas.size();i++)
    {
        int count=0;
        for(int j=0;j<robotDatas[i].size();j++)
        {
            ++count;
            robotDatas[i][j].memberID=count;
        }
    }
    std::vector<combinatedPaths_t> combinatedPath;
    // for(int i=0;i<robotDatas.size();i++)
    // {
    //     for(int j=0;j<robotDatas[i].size();j++)
    //     {
    //         combinatedPaths_t tmp;
    //         for(int k=0;k<robotDatas.size();k++)
    //         {
    //             tmp.combinatedPathLength = robotDatas;
    //         }
    //         tmp.combinatedPathLength = robot1data[i].pathLength + robot2data[j].pathLength;
    //         tmp.chosenID.push_back(robot1data[i].memberID);
    //         tmp.chosenID.push_back(robot2data[j].memberID);
    //         combinatedPath.push_back(tmp);
    //     }
    // }
    foreach_comb(numberOfFrontiers*numberOfRobots,numberOfRobots,[this](int *indexes)
    {
        cout << "foreach comb" << endl;
        static int foreach_combCounter=0;
        std::vector<int> combinatedElement;
        for(int i=0;i<numberOfRobots;i++)
        {
            cout << indexes[i] << ",";
            combinatedElement.push_back(indexes[i]);
        }
        combinatedPatern.push_back(combinatedElement);
        cout << endl;
    });
    cout << "before cout combinatedpattern" << endl;
    for(int i=0; i<combinatedPatern.size();i++)
    {
        for(int j=0;j<combinatedPatern[i].size();j++)
        {
            cout << combinatedPatern[i][j] << ",";
        }
        cout << endl;
    }
    for(int i=0;i<combinatedPath.size();i++)
    {
        cout << setw(15) << combinatedPath[i].combinatedPathLength << setw(5) << combinatedPath[i].chosenID[0] << setw(5) << combinatedPath[i].chosenID[1] << endl;
    }
    sort(combinatedPath.begin(),combinatedPath.end());
    cout << "---------------------------------------------------------------------------------" << endl;
    for(int i=0;i<combinatedPath.size();i++)
    {
        cout << setw(15) << combinatedPath[i].combinatedPathLength << setw(5) << combinatedPath[i].chosenID[0] << setw(5) << combinatedPath[i].chosenID[1] << endl;
    }
    return combinatedPath;
}
// robotData型のインスタンスに組み合わせの合計距離が最小となった時の目的地の情報を格納する関数
std::vector<geometry_msgs::PoseStamped> plan::robotToTarget(std::vector<combinatedPaths_t> combinatedPathsStruct, std::vector<robotData> robot1, std::vector<robotData> robot2)
{
    std::vector<geometry_msgs::PoseStamped> robotToTarget;
    robotData tmprobot1=transrateFromCombinatedPathsToRobotData(combinatedPathsStruct[0],1);
    std::vector<robotData>::iterator itr1=std::find(robot1.begin(),robot1.end(),robotData{tmprobot1});
    if(itr1 != robot1.end())
    {
        cout << robot1[itr1-robot1.begin()].goal << endl;
        robotToTarget.push_back(robot1[itr1-robot1.begin()].goal);
    }
    robotData tmprobot2=transrateFromCombinatedPathsToRobotData(combinatedPathsStruct[0],2);
    std::vector<robotData>::iterator itr2=std::find(robot2.begin(),robot2.end(),robotData{tmprobot2});
    if(itr2 != robot2.end())
    {
        cout << robot2[itr2-robot2.begin()].goal << endl;
        robotToTarget.push_back(robot2[itr2-robot2.begin()].goal);
    }
    return robotToTarget;
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
// 検査用メイン関数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "plan");
    plan p;
    std::vector<std::vector<robotData>> robotDatas;
    std::vector<combinatedPaths_t> combinatedPathesResult;
    for(int i=0;i<p.numberOfRobotGetter();i++)
    {
        std::vector<robotData> tmpRobotData;
        p.robotDataSetter(tmpRobotData);
        robotDatas.push_back(tmpRobotData);
    }
    cout << "robotDatas size : " <<robotDatas.size() << endl;
    combinatedPathesResult=p.combinatedPaths(robotDatas);
    std::vector<geometry_msgs::PoseStamped> test=p.robotToTarget(combinatedPathesResult,robotDatas[0],robotDatas[1]);


    return 0;
}