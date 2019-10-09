#include<multi_planning_server/Extraction.hpp>

using std::cout;
using std::endl;

typedef std::vector<std::vector<int8_t>> vv;

Extraction::Extraction(nav_msgs::OccupancyGrid recievedMapData, exploration_msgs::FrontierArray recievedFrontierArray)
{
    cout << "recievedMapData resolution : " << recievedMapData.info.resolution << endl;
    cout << "recievedFrontierArray size : " << recievedFrontierArray.frontiers.size() << endl;
    std::vector<int8_t> rowTransMapData;
    std::vector<std::vector<int8_t>> transMapData;
    int mapDataCounter=0;
    for(int i=0;i<recievedMapData.info.height;i++)
    {
        for(int j=0;j<recievedMapData.info.width;j++)
        {
            rowTransMapData.push_back(recievedMapData.data[mapDataCounter]);
            mapDataCounter++;
        }
        transMapData.push_back(rowTransMapData);
        rowTransMapData.clear();
    }

    mapInformationSetter(recievedMapData,transMapData);
    searchVoronoiWindowSetter(0.2);
    frontiersCoordinateSetter(recievedFrontierArray);
}
Extraction::~Extraction()
{

}

void Extraction::searchVoronoiWindowSetter(float windowLength)
{
    SVW.serachLength = windowLength;
    SVW.searchLengthCell = SVW.serachLength / MI.mapResolution;
    SVW.halfSquare = SVW.searchLengthCell / 2;
}

void Extraction::mapInformationSetter(nav_msgs::OccupancyGrid originalMapData, std::vector<std::vector<int8_t>> originalTransMapData)
{
    MI.mapWidth = originalMapData.info.width;
    MI.mapHeight = originalMapData.info.height;
    MI.mapResolution = originalMapData.info.resolution;
    MI.mapOrigin = originalMapData.info.origin;
    MI.mapData = originalTransMapData;
}


void Extraction::extractionTarget(void)
{
    cout << "[Extraction_Target]----------------------------------------" << endl;
    int Extracted_sum = 0;
    std::vector<double> previousFrontierX,previousFrontierY;

    //発見した目的地の座標を離散化する（メートル表記の座標からマス目表記の座標に直す）
    for(int i=0; i<frontiersCoordinate.size(); i++)
    {
        previousFrontierX.push_back((frontiersCoordinate[i].x-MI.mapOrigin.position.x)/MI.mapResolution);
        previousFrontierY.push_back((frontiersCoordinate[i].y - MI.mapOrigin.position.y) / MI.mapResolution);
    }

    //マップの端っこに座標があったら探索窓がマップの端を超えないように探査窓の一辺の長さを変更する
    int k;
    for(k=0;k<frontiersCoordinate.size();k++)
    {
		if(previousFrontierX[k]-SVW.halfSquare < 0)
        {
			SVW.halfLeftX = previousFrontierX[k];
		}
		else
        {
			SVW.halfLeftX = SVW.halfSquare;
		}
		if(previousFrontierX[k]+SVW.halfSquare > (MI.mapWidth-1))
        {
			SVW.halfRightX = (MI.mapWidth-1)-previousFrontierX[k];
		}
		else
        {
			SVW.halfRightX = SVW.halfSquare;
		}
		if(previousFrontierY[k]-SVW.halfSquare < 0)
        {
			SVW.halfTopY = previousFrontierY[k];
		}
		else
        {
			SVW.halfTopY = SVW.halfSquare;
		}
		if(previousFrontierY[k]+SVW.halfSquare > (MI.mapHeight-1))
        {
			SVW.halfBottomY = (MI.mapHeight-1)-previousFrontierY[k];
		}
		else
        {
			SVW.halfBottomY = SVW.halfSquare;
		}

        //検出されたフロンティア座標を中心に窓を作り、その窓内に拡張ボロノイ図に登録されているボロノイ線があればそれを追加
		int frontierSum = 0;
		for(int i=(previousFrontierY[k]-SVW.halfTopY);i<(previousFrontierY[k]+SVW.halfBottomY+1);i++)
        {
            for(int j=(previousFrontierX[k]-SVW.halfLeftX);j<(previousFrontierX[k]+SVW.halfRightX+1);j++)
            {
                frontierSum+=(-1)*MI.mapData[j][i];
			}
		}
		if(frontierSum>128)
        {
			MI.mapData[previousFrontierX[k]][previousFrontierY[k]] = 1;
		}
    }

    for(int j=0;j<MI.mapHeight;j++)
    {
        for(int i=0;i<MI.mapWidth;i++)
        {
            if (MI.mapData[i][j] == 1)
            {
                exploration_msgs::Frontier extractedPoint;
                extractedPoint.point.x = MI.mapResolution * i + MI.mapOrigin.position.x;
                extractedPoint.point.y = MI.mapResolution * j + MI.mapOrigin.position.y;
                extractedCoordinates.frontiers.push_back(extractedPoint);
            }
        }
    }
    cout << "extractionjCoordinates size = " << extractedCoordinates.frontiers.size() << endl;
    cout << "[Extraction_Target]----------------------------------------\n" << endl;
}

void Extraction::frontiersCoordinateSetter(const exploration_msgs::FrontierArray& poseArray)
{
    for(int i=0;i<poseArray.frontiers.size();i++)
    {
        frontiersCoordinate.push_back(poseArray.frontiers[i].point);
    }
}
visualization_msgs::Marker Extraction::publishExtractionTargetMarker(exploration_msgs::FrontierArray& extractionTargetForVisualize)
{
    uint32_t shape=visualization_msgs::Marker::CUBE_LIST;
    extractionTargetMarker.header.frame_id="robot1/map";
    extractionTargetMarker.header.stamp=ros::Time::now();

    extractionTargetMarker.ns="extraction_target";
    extractionTargetMarker.id=0;
    extractionTargetMarker.type=shape;

    extractionTargetMarker.action=visualization_msgs::Marker::ADD;


    extractionTargetMarker.scale.x = 0.05;
    extractionTargetMarker.scale.y = 0.05;
    extractionTargetMarker.scale.z = 0.05;

    extractionTargetMarker.color.r = 0.0f;
    extractionTargetMarker.color.g = 0.0f;
    extractionTargetMarker.color.b = 1.0f;
    extractionTargetMarker.color.a = 1.0f;
    extractionTargetMarker.lifetime=ros::Duration();

    geometry_msgs::Point tmpPoint;
    for (int i = 0; i < extractedCoordinates.frontiers.size();i++)
    {
        tmpPoint.x = extractedCoordinates.frontiers[i].point.x;
        tmpPoint.y = extractedCoordinates.frontiers[i].point.y;
        extractionTargetMarker.points.push_back(tmpPoint);
    }
    return extractionTargetMarker;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"Extraction");
    ExpLib::Struct::subStruct<nav_msgs::OccupancyGrid> voronoiGridTopicSub("/robot1/voronoi_map/global_costmap/voronoi_grid",1);
    ExpLib::Struct::subStruct<exploration_msgs::FrontierArray> frontierCoordinateSub("/Frontier_Target",1);
    ExpLib::Struct::pubStruct<exploration_msgs::FrontierArray> publishData("/extraction_target", 10);
    ExpLib::Struct::pubStruct<visualization_msgs::Marker>markerPub("extraction_target_marker",1);
    while(ros::ok())
    {
        voronoiGridTopicSub.q.callOne(ros::WallDuration(5.0));
        frontierCoordinateSub.q.callOne(ros::WallDuration(1.0));
        cout << "resolution : " <<  voronoiGridTopicSub.data.info.resolution << endl;
        cout << "height : " << voronoiGridTopicSub.data.info.height << endl;
        cout << "width : " << voronoiGridTopicSub.data.info.width << endl;
        Extraction E(voronoiGridTopicSub.data, frontierCoordinateSub.data);
        E.extractionTarget();
        publishData.pub.publish(E.extractedCoordinates);
        E.extractionTargetMarker = E.publishExtractionTargetMarker(E.extractedCoordinates);
        markerPub.pub.publish(E.extractionTargetMarker);
        sleep(1.0);
    }
    
    return 0;
}