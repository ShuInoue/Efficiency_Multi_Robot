#include<iostream>
#include<vector>
#include<geometry_msgs/PoseArray.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/OccupancyGrid.h>


struct MapInformation
{
    int mapWidth;
    int mapHeight;
    int mapResolution;
    geometry_msgs::Pose mapOrigin;
};
struct SerchVoronoiWindow
{
    float serachLength;
    int searchLengthCell;
    int halfSquare;
    int halfLeftX;//四角形の左半分の長さ
    int halfRightX;//四角形の右半分の長さ
    int halfTopY;//四角形の上半分の長さ
    int halfBottomY;//四角形の下半分の長さ
};

class Extraction
{
    Extraction();
    ~Extraction();
    std::vector<geometry_msgs::Pose> frontiersCoordinate;
    nav_msgs::OccupancyGrid recievedMapData;

    public:
    void initiarize(void);
    void frontiersCoordinateSetter(void);
    void extractionTarget(vv mapData);

};