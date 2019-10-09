#include<multi_planning_server/plan.hpp>

using std::cout;
using std::endl;
using std::sort;
using std::get;
using std::setw;

plan::plan():tf(ros::Duration(10)),globalCostmap("global_costmap",tf)
{
    ros::NodeHandle nh("~");
    //nh.getParam("number_of_robots",numberOfRobots);
    numberOfRobots=1;
}
int plan::numberOfRobotGetter(void)
{
    return numberOfRobots;
}
// ここで処理する用に用意したrobotData型のインスタンスにデータをセットする関数
void plan::robotDataSetter(exploration_msgs::FrontierArray& frontiers,nav_msgs::Odometry& recievedOdometry,std::vector<robotData>& testRobotData)
{
    cout << "frontiers size : " << frontiers.frontiers.size() << endl;
    cout << "odom : " << recievedOdometry.pose.pose << endl;
    cout << "testRobotData size :" << testRobotData.size() << endl;
    int count=0;
    int id=0;
    /*
    //test data
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

    std::random_device rnd;
    double locationx=0.0;
    double locationy=0.0;
    cout << setw(15) << "goalx" << setw(15) << "goaly" << setw(15) << "locationx" <<  setw(15) << "locationy" << setw(15) << "Path" << setw(15) << "id" << endl;
    
    for(int i=0; i<numberOfFrontiers; i++)
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

        robotData tmprobotdata={tmpgoal,tmplocation,dummyPath,getDistance(rndtergetx,rndtargety,locationx,locationy),++id};
        testRobotData.push_back(tmprobotdata);
        cout << setw(15) << testRobotData[i].goal.pose.position.x << setw(15) << testRobotData[i].goal.pose.position.y << setw(15) << testRobotData[i].location.pose.pose.position.x <<  setw(15) << testRobotData[i].location.pose.pose.position.y << setw(15) << testRobotData[i].pathLength << setw(15) << testRobotData[i].memberID << endl;
    }
    */
    nav_msgs::Odometry tmplocation;
    //tmplocation = *recievedOdometry;
    tmplocation.pose.pose.position.x = recievedOdometry.pose.pose.position.x;
    tmplocation.pose.pose.position.y = recievedOdometry.pose.pose.position.y;
    geometry_msgs::PoseStamped stampedLocation;
    stampedLocation.header.frame_id="robot1/map";
    stampedLocation.pose.position.x=recievedOdometry.pose.pose.position.x;
    stampedLocation.pose.position.y=recievedOdometry.pose.pose.position.y;
    for (int i = 0; i < numberOfFrontiers; i++)
    {
        geometry_msgs::PoseStamped tmpgoal;
        tmpgoal.pose.position.x = frontiers.frontiers[i].point.x;
        tmpgoal.pose.position.y = frontiers.frontiers[i].point.y;


        if(avoidTargetInRobot(tmplocation,tmpgoal))
        {
            //nav_msgs::Path Path;
            std::vector<geometry_msgs::PoseStamped> foundPath;
            nav_msgs::Path foundNavPath;
            voronoi_planner::VoronoiPlanner vp;
            vp.initialize(name,&globalCostmap);
            vp.makePlan(stampedLocation,tmpgoal,foundPath);
            for(int i=0;i<foundPath.size();i++)
            {
                foundNavPath.poses[i]=foundPath[i];
            }
            tmpgoal.header.frame_id="robot1/map";
            tmpgoal.pose.orientation.w=1.0;
            robotData tmprobotdata = {tmpgoal, tmplocation, foundNavPath, getDistance(tmpgoal.pose.position.x, tmpgoal.pose.position.y, tmplocation.pose.pose.position.x, tmplocation.pose.pose.position.y), ++id};
            testRobotData.push_back(tmprobotdata);
            cout << setw(15) << testRobotData[i].goal.pose.position.x << setw(15) << testRobotData[i].goal.pose.position.y << setw(15) << testRobotData[i].location.pose.pose.position.x << setw(15) << testRobotData[i].location.pose.pose.position.y << setw(15) << testRobotData[i].pathLength << setw(15) << testRobotData[i].memberID << endl;
        }
        else
        {
        }
        
    }
}
// ロボットの現在位置から目的地までの距離を計算する関数
double plan::calcPathFromLocationToTarget(geometry_msgs::PoseStamped Goal, nav_msgs::Odometry Location, nav_msgs::Path path)
{
    double pathlength=0.0;
    double locationToFirstPath, lastPathToGoal;

    locationToFirstPath=getDistance(Location.pose.pose.position.x,Location.pose.pose.position.y,path.poses[0].pose.position.x,path.poses[0].pose.position.y);
    lastPathToGoal=getDistance(path.poses[path.poses.size()-1].pose.position.x,path.poses[path.poses.size()-1].pose.position.y,Goal.pose.position.x,Goal.pose.position.y);
    for(int i=0; i<path.poses.size()-1;i++)
    {
        pathlength += getDistance(path.poses[i].pose.position.x, path.poses[i].pose.position.y, path.poses[i+1].pose.position.x,path.poses[i+1].pose.position.y);
    }
    pathlength = pathlength+locationToFirstPath+lastPathToGoal;
    return pathlength;
}
//パスの情報を距離の長さに変換する関数
double plan::getDistance(double x, double y, double x2, double y2)
{
    // cout << "data : " << x <<" "<<  y <<" "<<  x2 <<" "<< y2 << endl;
    double distance=sqrt(pow(x2-x,2)+pow(y2-y,2));
    return distance;
}   
//robotdataの二重配列からIDを使って距離を返す関数
double plan::pathlengthFoundwithID(std::vector<std::vector<robotData>> &robotDataYouWantToKnowPathlength, std::vector<int> chosenID)
{
    double totalPathlength=0;
    for(int i=0;i<chosenID.size();i++)
    {
        double Pathlength=0;
        robotData tmprobot;
        tmprobot.memberID=chosenID[i];
        std::vector<robotData>::iterator itr1=std::find(robotDataYouWantToKnowPathlength[i].begin(),robotDataYouWantToKnowPathlength[i].end(),robotData{tmprobot});
        if(itr1 != robotDataYouWantToKnowPathlength[i].end())
        {
            Pathlength = robotDataYouWantToKnowPathlength[i][itr1-robotDataYouWantToKnowPathlength[i].begin()].pathLength;
        }
        cout << "pathlength : " << Pathlength << endl;
        totalPathlength += Pathlength;
    }
    return totalPathlength;
}
//CombinatedPaths型からrobtoData型に変化する関数
robotData plan::transrateFromCombinatedPathsToRobotData(combinatedPaths_t comb, int robotNum)
{
    robotData tmpRobotData;
    tmpRobotData.memberID = comb.chosenID[robotNum-1];
    return tmpRobotData;
}
// 各ロボットの各目的地までの距離を組み合わせて合計していく関数
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
    foreach_comb(numberOfFrontiers,numberOfRobots,[this](int *indexes)
    {
        static int foreach_combCounter=0;
        std::vector<int> combinatedElement;
        for(int i=0;i<numberOfRobots;i++)
        {
            combinatedElement.push_back(indexes[i]+1);
        }
        combinatedPatern.push_back(combinatedElement);
    });

    std::vector<combinatedPaths_t> combinatedPath;
    combinatedPath.resize(combinatedPatern.size());
    for(int i=0; i<combinatedPatern.size();i++)
    {
        for(int j=0;j<combinatedPatern[i].size();j++)
        {
            cout << combinatedPatern[i][j] << ",";
            combinatedPath[i].chosenID.push_back(combinatedPatern[i][j]);
        }
        cout << endl;
        combinatedPath[i].combinatedPathLength = pathlengthFoundwithID(robotDatas,combinatedPatern[i]);
        cout << "totalPathlength : " << combinatedPath[i].combinatedPathLength << endl;
    }
    sort(combinatedPath.begin(),combinatedPath.end());
    return combinatedPath;
}
// robotData型のインスタンスに組み合わせの合計距離が最小となった時の目的地の情報を格納する関数
// robotToTargetのサイズはロボットの個数に等しく、ロボットの順番が配列順になっている（要検証）
std::vector<geometry_msgs::PoseStamped> plan::robotToTarget(std::vector<combinatedPaths_t> combinatedPathsStruct, std::vector<std::vector<robotData>> tmpRobotDatas)
{
    std::vector<geometry_msgs::PoseStamped> robotToTarget;
    for(int i=0; i<numberOfRobots; i++)
    {
        robotData tmprobot=transrateFromCombinatedPathsToRobotData(combinatedPathsStruct[0],i+1);
        std::vector<robotData>::iterator itr1=std::find(tmpRobotDatas[i].begin(),tmpRobotDatas[i].end(),robotData{tmprobot});
        if(itr1 != tmpRobotDatas[i].end())
        {
            cout << tmpRobotDatas[i][itr1-tmpRobotDatas[i].begin()].goal << endl;
            robotToTarget.push_back(tmpRobotDatas[i][itr1-tmpRobotDatas[i].begin()].goal);
        }
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

void plan::recievedFrontierCoordinatesSetter(const exploration_msgs::FrontierArray &recievedData)
{
    for(int i=0;i<recievedData.frontiers.size();i++)
    {
        recievedFrontierCoordinates.push_back(recievedData.frontiers[i].point);
    }
    numberOfFrontiers = recievedData.frontiers.size();
}

bool plan::avoidTargetInRobot(nav_msgs::Odometry& nowLocation,geometry_msgs::PoseStamped& candidateTarget)
{
    double robotRadius=0.4;
    double lengthFromRobotcenterToTarget=sqrt(pow((candidateTarget.pose.position.x-nowLocation.pose.pose.position.x),2)+pow(candidateTarget.pose.position.y-nowLocation.pose.pose.position.y,2));
    cout << "lengthFromRobotcenterToTarget : " <<lengthFromRobotcenterToTarget << endl;
    cout << "robotRadius : " << robotRadius << endl;
    if(lengthFromRobotcenterToTarget<robotRadius)
    {
        cout << "length < radius" << endl;
        return false;
    }
    else
    {
        cout << "length > radius" << endl;
        return true;
    }
}
// 検査用メイン関数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "plan");
    plan p;
    ExpLib::Struct::subStruct<exploration_msgs::FrontierArray> frontierCoordinatesSub("/extraction_target",1);
    ExpLib::Struct::subStruct<nav_msgs::Odometry> odometrySub("/robot1/odom",1);
    ExpLib::Struct::pubStruct<geometry_msgs::PoseStamped> goalPosePub("/robot1/move_base_simple/goal",1);
    while (ros::ok())
    {
        cout << "plan start" << endl;
        frontierCoordinatesSub.q.callOne(ros::WallDuration(1.5));
        odometrySub.q.callOne(ros::WallDuration(1.0));
        p.recievedFrontierCoordinatesSetter(frontierCoordinatesSub.data);
        cout << "test1" << endl;
        std::vector<std::vector<robotData>> robotDatas;
        std::vector<combinatedPaths_t> combinatedPathesResult;
        for(int i=0;i<p.numberOfRobotGetter();i++)
        {
            cout << "testloop" << endl;
            std::vector<robotData> tmpRobotData;
            cout << "frontierCoordinatesSub data size : " << frontierCoordinatesSub.data.frontiers.size()  << endl;
            p.robotDataSetter(frontierCoordinatesSub.data,odometrySub.data,tmpRobotData);
            robotDatas.push_back(tmpRobotData);
        }
        cout << "robotDatas size : " <<robotDatas.size() << endl;
        combinatedPathesResult=p.combinatedPaths(robotDatas);
        cout << "combinatedPathesResult size : " << combinatedPathesResult.size() << endl;
        std::vector<geometry_msgs::PoseStamped> test=p.robotToTarget(combinatedPathesResult,robotDatas);
        cout << "test size : " << test.size() << endl;
        goalPosePub.pub.publish(test.front());//robotの個数分のサイズの配列になっている
        cout << "plan end" << endl;
    }
    
    return 0;
}