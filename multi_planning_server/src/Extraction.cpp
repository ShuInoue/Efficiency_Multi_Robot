#include<multi_planning_server/Extraction.hpp>

using std::cout;
using std::endl;

typedef std::vector<std::vector<int>> vv;

Extraction::Extraction()
{
    
}
Extraction::~Extraction()
{

}

void Extraction::initialize(void)
{
    struct MapInformation MI={};
    struct SearchVoronoiWindow SVW={0.3,SVW.serachLength/MI.mapResolution,SVW.searchLengthCell/2,};
}

void Extraction::extractionTarget(void)
{
    cout << "[Extraction_Target]----------------------------------------" << endl;
    int Extracted_sum = 0;
    std::vector<double> previousFrontierX,previousFrontierY;


    //発見した目的地の座標を離散化する（メートル表記の座標からマス目表記の座標に直す）
    for(int i=0; i<frontiersCoordinate.size(); i++)
    {
        previousFrontierX.push_back((frontiersCoordinate[i].x-recievedMapData.info.origin.position.x)/MI.mapResolution);
        previousFrontierY.push_back((frontiersCoordinate[i].y-recievedMapData.info.origin.position.y)/MI.mapResolution);
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
    for (int i = 0; i < MI.mapData.size(); i++)
    {
        for (int j = 0; j < MI.mapData[i].size(); i++)
        {
            cout << MI.mapData[i][j] << endl;
        }
    }
    cout << "[Extraction_Target]----------------------------------------\n" << endl;
}

void Extraction::frontiersCoordinateSetter(const exploration_msgs::FrontierArray& poseArray)
{
    for(int i=0;i<poseArray.frontiers.size();i++)
    {
        frontiersCoordinate.push_back(poseArray.frontiers[i].point);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"Extraction");
    Extraction E;
    ExpLib::Struct::subStruct<exploration_msgs::FrontierArray> voronoiGridTopicSub("/robot1/voronoi_map/global_costmap/voronoi_grid",1);
    while(ros::ok())
    {
        voronoiGridTopicSub.q.callOne();
        E.initialize();
        E.frontiersCoordinateSetter(voronoiGridTopicSub.data); 
        E.extractionTarget();
        sleep(1.0);
    }
    
    return 0;
}