#include<iostream>
#include<vector>
#include<geometry_msgs/PoseArray.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/OccupancyGrid.h>


class Extraction
{
    Extraction();
    ~Extraction();
    std::vector<geometry_msgs::Pose> frontiersCoordinate;
    nav_msgs::OccupancyGrid recievedMapData;

    public:
    void frontiersCoordinateSetter(void);
    void extractionTarget(vv mapData);

};