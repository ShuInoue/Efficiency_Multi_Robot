#ifndef _EXTRACTION_HPP_
#define _EXTRACTION_HPP_

#include<iostream>
#include<vector>
#include<geometry_msgs/PoseArray.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/OccupancyGrid.h>
#include<exploration_libraly/struct.hpp>
#include<exploration_msgs/FrontierArray.h>

struct MapInformation
{
    uint32_t mapWidth;
    uint32_t mapHeight;
    float mapResolution;
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
    std::vector<geometry_msgs::Point> frontiersCoordinate;
    struct MapInformation MI;
    struct SearchVoronoiWindow SVW;

    public:
        exploration_msgs::FrontierArray extractedCoordinates;
        Extraction(nav_msgs::OccupancyGrid recievedMapData, exploration_msgs::FrontierArray recievedFrontierArray);
        ~Extraction();
        void frontiersCoordinateSetter(const exploration_msgs::FrontierArray &poseArray);
        void mapInformationSetter(nav_msgs::OccupancyGrid originalMapData,std::vector<std::vector<int>> originalTransMapData);
        void extractionTarget(void);
        void searchVoronoiWindowSetter(float windowLength);
};

#endif