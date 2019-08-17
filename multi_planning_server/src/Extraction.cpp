#include<Extraction.hpp>

using std::cout;
using std::endl;

typedef std::vector<std::vector<int>> vv;

Extraction::Extraction()
{

}

void Extraction::extractionTarget(vv mapData)
{
    cout << "[Extraction_Target]----------------------------------------" << endl;
    int Extracted_sum = 0;
    std::vector<double> pre_frox;
    std::vector<double> pre_froy;
    //発見した目的地の座標を離散化する（メートル表記の座標からマス目表記の座標に直す）
    for(int i=0; i<frontiersCoordinate.size(); i++)
    {
        pre_frox.push_back((frontiersCoordinate[i].position.x-recievedMapData.info.origin.position.x)/recievedMapData.info.resolution);
        pre_froy.push_back((frontiersCoordinate[i].position.y-recievedMapData.info.origin.position.y)/recievedMapData.info.resolution);
    }

    //マップの端っこに座標があったら探索窓がマップの端を超えないように探査窓の一辺の長さを変更する
    int k;
    for(k=0;k<frontiersCoordinate.size();k++)
    {
		if(pre_frox[k]-half_sq < 0)
        {
			half_leftx = pre_frox[k];
		}
		else
        {
			half_leftx = half_sq;
		}
		if(pre_frox[k]+half_sq > (map_width-1))
        {
			half_rightx = (map_width-1)-pre_frox[k];
		}
		else
        {
			half_rightx = half_sq;
		}
		if(pre_froy[k]-half_sq < 0)
        {
			half_topy = pre_froy[k];
		}
		else
        {
			half_topy = half_sq;
		}
		if(pre_froy[k]+half_sq > (map_height-1))
        {
			half_bottomy = (map_height-1)-pre_froy[k];
		}
		else
        {
			half_bottomy = half_sq;
		}

        //検出されたフロンティア座標を中心に窓を作り、その窓内に拡張ボロノイ図に登録されているボロノイ線があればそれを追加
		r1_frontier_sum = 0;
        r2_frontier_sum = 0;
		for(int i=(pre_froy[k]-half_topy);i<(pre_froy[k]+half_bottomy+1);i++)
        {
			for(int j=(pre_frox[k]-half_leftx);j<(pre_frox[k]+half_rightx+1);j++)
            {
				r1_frontier_sum+=(-1)*r1_enhanced_Voronoi_grid_array[j][i];
			}
		}
		if(r1_frontier_sum>128)
        {
			r1_enhanced_Voronoi_grid_array[pre_frox[k]][pre_froy[k]] = 1;
		}
        for(int i=(pre_froy[k]-half_topy);i<(pre_froy[k]+half_bottomy+1);i++)
        {
			for(int j=(pre_frox[k]-half_leftx);j<(pre_frox[k]+half_rightx+1);j++)
            {
				r2_frontier_sum+=(-1)*r2_enhanced_Voronoi_grid_array[j][i];
			}
		}
		if(r2_frontier_sum>128)
        {
			r2_enhanced_Voronoi_grid_array[pre_frox[k]][pre_froy[k]] = 1;
		}
	}

    //拡張
    for(int j=0; j < map_height; j++)
    {
        for(int i=0; i < map_width; i++)
        {
            if(mapData[i][j] == 1)
            {
                t_target.pose.position.x = map_resolution * i + map_origin.position.x;
                t_target.pose.position.y = map_resolution * j + map_origin.position.y;
                t_target.pose.orientation.x = 0.0;
                t_target.pose.orientation.y = 0.0;
                t_target.pose.orientation.z = 0.0;
                t_target.pose.orientation.w = 1.0;
                Extraction_Target_r1.push_back(t_target);
            }
        }
    }
    cout << "Extraction_Target_r1  :" << Extraction_Target_r1.size() << endl;

    cout << "[Extraction_Target]----------------------------------------\n" << endl;
}

void Extraction::frontiersCoordinateSetter(geometry_msgs::posearray poseArray)
{
    for(int i=0;i<poseArray.poses.size();i++)
    {
        frontiersCoordinate.push_back(poseArray.poses[i]);
    }
}