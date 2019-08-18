#include<iostream>
#include<vector>
#include<geometry_msgs/PoseArray.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/OccupancyGrid.h>
#include<common_lib.hpp>

struct MapInformation
{
    int mapWidth;
    int mapHeight;
    int mapResolution;
    geometry_msgs::Pose mapOrigin;
    std::vector<std::vector<int>> mapData;
};
struct SearchVoronoiWindow
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
    struct MapInformation MI;
    struct SearchVoronoiWindow SVW;

    public:
    void initiarize(void);
    void frontiersCoordinateSetter(void);
    void extractionTarget(void);

};