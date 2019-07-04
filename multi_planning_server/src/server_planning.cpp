#include<multi_planning_server/server_planning.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "server_planning");
    ros::Publisher pub2FS;
    ros::Subscriber subFS;
    
    //firstturn用
    ros::NodeHandle turn_nh;
    ros::Publisher turn_req_pub;
    ros::Subscriber turn_sub;

    //パラメータサーバー用
    ros::NodeHandle robot_num_nh;
    ros::ServiceClient firstturnClient;
    
    //timing用
    ros::NodeHandle timing_nh;
    ros::Publisher timing_pub;
    timing_pub = timing_nh.advertise<std_msgs::Int8>("/timing_check",1);


    server_planning SP;


    /*
   //ロボットの個数をパラメータサーバーから取得
    robot_num_nh.getParam("/multi_planning_server/robot_num",robot_num);
    robot_num_nh.getParam("/multi_planning_server/given_robot_num",given_robot_num);

    // robot_movingからの初期回転の完了を検知する。
    ros::NodeHandle srv_nh;
    std_srvs::Empty srv;
    std::cout << "test1" << std::endl;
    SP.timing_int.data = 1;
    timing_pub.publish(SP.timing_int);
    while(ros::ok() && !(robot_num == given_robot_num))
    {
        robot_num_nh.getParam("/multi_planning_server/robot_num",robot_num);
        std::cout << "robot_num: " << robot_num << " given_robot_num: "<< given_robot_num<< std::endl;
        sleep(1);
    }
    
    
    int count=0;
    bool flag =false;
    std::string tmp_name;
    std::string srv_name;
    std::cout << "test2" << std::endl;
    
    while(ros::ok() && !(count == robot_num))
    {
        std::cout << "test3" << std::endl;
        if(!flag)
        {
            std::ostringstream oss;
            oss << count+1;
            tmp_name = oss.str();
            srv_name = "/robot"+tmp_name+"/TURN";
            std::cout << "service name : " << srv_name << std::endl;
            firstturnClient = srv_nh.serviceClient<std_srvs::Empty>(srv_name);
            flag = true;
        }
        bool result = firstturnClient.call(srv);
        if(result)
        {
          ROS_INFO_STREAM("complete" << tmp_name);
          count++;
          flag = false;
          tmp_name = "";
          ROS_INFO_STREAM("initialized : [" << tmp_name << "]");
        }
        else
        {
            ROS_INFO_STREAM("false");
            sleep(1);
        }

    }
    */

    /*
    turn_req_pub = turn_nh.advertise<std_msgs::String>("/firstturn",1);
    SP.pub_msg.data = "turn now";
    while(ros::ok() && !SP.turn_fin)
    {
        turn_sub = turn_nh.subscribe("/turnfin", 1, &server_planning::turn_fin_CB, &SP);
        turn_req_pub.publish(SP.pub_msg);
        std::cout << SP.sub_msg << std::endl;
        ros::spinOnce();
    }
    */
   cout << "---------------------【SERVER_PLANINNG START】-------------------" << endl;
    while((!SP.odom_queue_flag || !SP.securedR1VoronoiGridArray) && ros::ok())
    {
        SP.robot1_odom_queue.callOne(ros::WallDuration(1));
        SP.r1_voronoi_map_queue.callOne(ros::WallDuration(0.1));
        cout << "odom_queue_flag:" << SP.odom_queue_flag << endl;
        cout << "securedR1VoronoiGridArray:" << SP.securedR1VoronoiGridArray << endl;
    }
    SP.odom_queue_flag=false;
    sleep(1);
    int count2 = 0;
    while((!SP.odom_queue_flag || !SP.securedR2VoronoiGridArray) && ros::ok())
    {
        SP.robot2_odom_queue.callOne(ros::WallDuration(1));
        SP.r2_voronoi_map_queue.callOne(ros::WallDuration(0.1));
        cout << "odom_queue_flag:" << SP.odom_queue_flag << endl;
        cout << "securedR2VoronoiGridArray:" << SP.securedR2VoronoiGridArray << endl;
    }
    //メインループ
    while(ros::ok())
    {
        SP.queueM.callOne(ros::WallDuration(1));
        if(SP.map_isinput())
        {
            //Frontier_Searchからの座標を取得
            RERUN_FRONTIER:
            while(!SP.queueF_judge && ros::ok())
            {
                SP.queueF.callOne(ros::WallDuration(1));//FSノードから出てきた座標の情報を格納しているキューを購読しマップ配列の座標と対応する箇所を１，その他を０で埋める。
            }
            SP.queueF_judge = false;
            std::cout << "queueF.callOne was done." << std::endl;
            //ここにボロノイ図を作成するためにfinaltargetをおくる必要がある。これがないと一度メインループを抜けるとループに戻らなくなってしまう。
            //マップとボロノイ図を比較してボロノイ経路上の目的地を絞り込む
            cout << "securedR1VoronoiGridArray:" << SP.securedR1VoronoiGridArray << endl;
            cout << "securedR2VoronoiGridArray:" << SP.securedR2VoronoiGridArray << endl;
            while((SP.securedR1VoronoiGridArray == false || SP.securedR2VoronoiGridArray == false) && ros::ok())
            {
                if(SP.isStartedUpdate == true)
                {
                    SP.robot1_odom_queue.callOne(ros::WallDuration(1));
                    SP.r1_voronoi_map_queue.callOne(ros::WallDuration(0.1));//ロボット１から出てきたボロノイ図を蓄えているキューを購読。ボロノイ図用の配列を確保し、そこに情報を反映する。r1_voronoi_map_updateフラグはここで立つ
                    SP.robot1_odom_queue.callOne(ros::WallDuration(1));
                    SP.r2_voronoi_map_queue.callOne(ros::WallDuration(0.1));//ロボット２から出てきたボロノイ図を蓄えているキューを購読。ボロノイ図用の配列を確保し、そこに情報を反映する。r2_voronoi_map_updateフラグはここで立つ
                    SP.isStartedUpdate = false;
                }
                else
                {
                    SP.robot1_odom_queue.callOne(ros::WallDuration(1));
                    SP.r1_voronoi_map_queue.callOne(ros::WallDuration(0.1));//ロボット１から出てきたボロノイ図を蓄えているキューを購読。ボロノイ図用の配列を確保し、そこに情報を反映する。r1_voronoi_map_updateフラグはここで立つ
                    SP.robot1_odom_queue.callOne(ros::WallDuration(1));
                    SP.r2_voronoi_map_queue.callOne(ros::WallDuration(0.1));//ロボット２から出てきたボロノイ図を蓄えているキューを購読。ボロノイ図用の配列を確保し、そこに情報を反映する。r2_voronoi_map_updateフラグはここで立つ
                    
                }
            }
            if(SP.securedR1VoronoiGridArray && SP.securedR2VoronoiGridArray)
            {
                cout << "r1 and r2 voronoi_map_update" << endl;
                SP.Extraction_Target();
                cout << "SP.Extraction_Target_r1.size() : " << SP.Extraction_Target_r1.size() << endl;
                cout << "SP.Extraction_Target_r2.size() : " << SP.Extraction_Target_r2.size() << endl;
                if(SP.Extraction_Target_r1.size() == 0 || SP.Extraction_Target_r2.size() == 0)
                {
                    static int counter;
                    if(counter <= 10)
                    {
                        std::cout << "RERUN FRONTIER" << std::endl;
                        SP.Clear_Vector_by_Extraction_Target();
                        goto RERUN_FRONTIER;
                    }
                    else
                    {
                        std::cout << "Extracted error." << std::endl;
                        exit(0);
                    }
                }
                SP.Publish_marker();
                SP.FT2robots();//取得したフロンティア領域を各ロボットの目的地として配布。
                if(SP.non_extracted_r1 == true && SP.non_extracted_r2 == true)
                {
                    static int non_extracted_rs_count = 0;
                    if(non_extracted_rs_count == 10)
                    {
                        std::cout << "Extracted error." << std::endl;
                        exit(0);
                    }
                    else
                    {
                        SP.Clear_Vector_by_FT2robots();
                        goto RERUN_FRONTIER;
                    }
                }
                SP.OptimalTarget();
                if(SP.zero_for_sort == true)
                {
                    static int zero_for_sort_count = 0;
                    if(zero_for_sort_count == 10)
                    {
                        std::cout << "Extracted error." << std::endl;
                        exit(0);
                    }
                    else
                    {
                        goto RERUN_FRONTIER;
                    }
                }
                SP.arrive1 = 0;
                SP.arrive2 = 0;
                cout << "cant_find_final_target_flag" << SP.cant_find_final_target_flag << endl;
                while(SP.cant_find_final_target_flag == 0 && ros::ok())
                {
                    cout << "arrive1:" << SP.arrive1 << endl;
                    cout << "arrive2:" << SP.arrive2 << endl;
                    while(SP.arrive1 == 0 && SP.arrive2 == 0 && ros::ok())
                    {
                        SP.arrive1_queue.callOne();
                        SP.arrive2_queue.callOne();
                        sleep(0.1);
                    }
                    cout << "arrive1 : " << SP.arrive1 << endl;
                    cout << "arrive2 : " << SP.arrive2 << endl;
                    if(SP.arrive1 == 1 || SP.arrive2 == 1)
                    {
                        cout << "1 or 1" << endl;
                        goto OUT_MOVE_TO_TARGET;
                    }
                    else if(SP.arrive1 == 2 || SP.arrive2 == 2)//目標へのパス生成不可
                    {
                        cout << "2 or 2" << endl;
                        if(SP.update_target(false) == 1)
                        {
                            goto OUT_MOVE_TO_TARGET;
                        }
                    }
                    else if(SP.arrive1 == 3 || SP.arrive2 == 3)//目標への移動不可
                    {
                        cout << "3 or 3" << endl;
                        if(SP.update_target(false) == 1)
                        {
                            goto OUT_MOVE_TO_TARGET;
                        }
                    }
                    else
                    {
                        cout << "what state?" << endl;
                        cout << "SP.arrive1:" << SP.arrive1 << endl;
                        cout << "SP.arrive2:" << SP.arrive2 << endl;
                    }
                    SP.arrive1 = 0;
                    SP.arrive2 = 0;
                    cout << "cant_find_final_target_flag loop end" << endl;
                }
                cout << "voronoi update flags initialize" << endl;
                OUT_MOVE_TO_TARGET:
                SP.securedR1VoronoiGridArray = false;
                SP.securedR2VoronoiGridArray = false;
                SP.cant_find_final_target_flag = false;
                SP.update_target(true);
                cout << "if end" << endl;
            }
            else
            {
                continue;
            }
            //それぞれのロボットの自己位置とゴールを結ぶパスを取得する。
            //
            /*
            if(robot_num >= fro_num)
            {
                //ロボットの数の方がフロンティアセルより多いのでロボットの振り分けをする。
                //SP.robot_sort1();
                std::cout << "check if 2" << std::endl;
            }
            else if(robot_num < fro_num)
            {
                //ロボットの数の方がフロンティアセルより少ない時でのロボットの振り分け方。
                //SP.robot_sort2();
                std::cout << "check if 3" << std::endl;
            }
            else
            {
                //ROS_ERROR_STREAM("フロンティアセルとロボット数の条件分岐でのエラー。");
                ROS_ERROR_STREAM("if error");
            }
            */
        }
        else if(!SP.map_isinput())
        {
            //ROS_FATAL_STREAM("Planning_Serveでマップの更新なし。");
            ROS_FATAL_STREAM("Planning_Serve not map-refresh");
        }
        else
        {
            //ROS_ERROR_STREAM("Planning_Serverでマップの更新に失敗。");
            ROS_ERROR_STREAM("Planning_Server miss map-refresh");
            return 0;
        }
        cout << "securedR1VoronoiGridArray:" << SP.securedR1VoronoiGridArray << endl;
        cout << "securedR2VoronoiGridArray:" << SP.securedR2VoronoiGridArray << endl;
        SP.SP_Memory_release();
        SP.Clear_Vector();
        SP.Clear_Num();

        cout << "main loop ended" << endl;
        cout << "\n" << endl;
    }
    SP.timing_int.data = 2;
    timing_pub.publish(SP.timing_int);
    robot_num_nh.setParam("/multi_planning_server/robot_num",0);
    robot_num_nh.setParam("/multi_planning_server/given_robot_num",0);
    cout << "---------------------【SERVER_PLANINNG END】-------------------" << endl;
    return 0;
}