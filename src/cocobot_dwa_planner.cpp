#include "cocobot_dwa_planner/cocobot_dwa_planner.h"

CocobotDWA::CocobotDWA():private_nh_("~")
{
  // param
  private_nh_.param("hz", hz_, {10});
  private_nh_.param("weight_heading", weight_heading_, {0.4});
  private_nh_.param("weight_glocal_heading", weight_glocal_heading_, {0.4});
  private_nh_.param("weight_dist", weight_dist_, {0.8});
  private_nh_.param("weight_vel", weight_vel_, {0.2});
  private_nh_.param("weight_cost_map", weight_cost_map_, {0.9});
  private_nh_.param("search_range", search_range_, {7.0});
  private_nh_.param("margin", margin_, {0.5});
  private_nh_.param("min_cost", min_cost_, {30.0});
  private_nh_.param("min_vel", min_vel_, {0.0});
  private_nh_.param("max_vel", max_vel_, {1.2});
  private_nh_.param("max_yawrate", max_yawrate_, {4.5});
  private_nh_.param("max_accel", max_accel_, {10.0});
  private_nh_.param("max_yawaccel", max_yawaccel_, {10.0});
  private_nh_.param("goal_tolerance", goal_tolerance_, {0.3});
  private_nh_.param("dist_to_update_glocal_goal", dist_to_update_glocal_goal_, {1.5});
  private_nh_.param("world_frame", world_frame_, {"odom"});
  private_nh_.param("robot_frame", robot_frame_, {"base_footprint"});
  private_nh_.param("vel_reso", vel_reso_, {0.1});
  private_nh_.param("yawrate_reso", yawrate_reso_, {0.1});
  private_nh_.param("dt", dt_, {0.1});
  private_nh_.param("predict_time", predict_time_, {3.0});

  // subscriber
  sub_local_goal_ = nh_.subscribe("/local_goal", 1, &CocobotDWA::local_goal_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_glocal_path_ = nh_.subscribe("/glocal_path", 1, &CocobotDWA::glocal_path_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_people_states_ = nh_.subscribe("/transformed_people_states", 1, &CocobotDWA::people_states_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_cost_map_ = nh_.subscribe("/cost_map", 1, &CocobotDWA::cost_map_callback, this, ros::TransportHints().reliable().tcpNoDelay());

  // publisher
  pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/local/cmd_vel", 1);
  pub_cmd_pos_ = nh_.advertise<ccv_dynamixel_msgs::CmdPoseByRadian>("/local/cmd_pos", 1);

  // debug
  pub_predict_path_ = nh_.advertise<nav_msgs::Path>("/predict_local_paths", 1);
  pub_optimal_path_ = nh_.advertise<nav_msgs::Path>("/optimal_local_path", 1);
  pub_glocal_goal_ = nh_.advertise<geometry_msgs::PointStamped>("/glocal_goal", 1);
}

// local_goalコールバック関数
void CocobotDWA::local_goal_callback(const geometry_msgs::PointStampedConstPtr& msg)
{
  geometry_msgs::TransformStamped transform;

  try
  {
    transform = tf_buffer_.lookupTransform(robot_frame_, world_frame_, ros::Time(0));
    flag_local_goal_ = true;
  }
  catch(tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    flag_local_goal_ = false;
    return;
  }

    tf2::doTransform(*msg, local_goal_, transform);
}

// glocal_pathのコールバック関数
void CocobotDWA::glocal_path_callback(const nav_msgs::PathConstPtr& msg)
{
  glocal_path_ = *msg;
  
  if(glocal_path_.poses.size() > 0)
  {
    flag_glocal_path_ = true;
    flag_sub_glocal_path_ = true;
    sub_glocal_path_count_ = 0;  // glocal_pathの更新待ち回数を初期化
  }
}

// 歩行者情報のコールバック関数
void CocobotDWA::people_states_callback(const pedestrian_msgs::PeopleStatesConstPtr& msg)
{
  while(people_states_.size() > 0)
  {
    // people_states_の配列のうち取得済みのデータ（配列の先頭の要素）を削除
    // これをしないと，front() でデータを取得する際，同じデータしか取得できない
    people_states_.pop();
  }

  people_states_.emplace(msg);
  flag_people_states_ = true;
}

// cost_mapのコールバック関数
void CocobotDWA::cost_map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  cost_map_ = *msg;
  flag_cost_map_ = true;
  flag_sub_cost_map_ = true;
  sub_cost_map_count_ = 0;  // cost_mapの更新待ち回数を初期化
}

// 距離を計算
double CocobotDWA::calc_dist(const double x1, const double y1, const double x2, const double y2)
{
  const double dx = x1 - x2;
  const double dy = y1 - y2;

  return hypot(dx, dy);
}

// goalに着くまでtrueを返す
bool CocobotDWA::is_goal()
{
  // glocal_pathが更新されていない回数をカウント
  if(flag_glocal_path_ == false)
  {
    sub_glocal_path_count_++;
    
    if(sub_glocal_path_count_ > 3)  // 3回以上glocal_pathが更新されていない場合はglocal_pathのノードが正常に動作していないと判断
      flag_sub_glocal_path_ = false;
  }

  // cost_mapが更新されていない回数をカウント
  if(flag_cost_map_ == false)
  {
    sub_cost_map_count_++;
    
    if(sub_cost_map_count_ > 3)  // 3回以上cost_mapが更新されていない場合はcost_mapのノードが正常に動作していないと判断
      flag_sub_cost_map_ = false;
  }

  // msg受信済みか確認
  if((flag_local_goal_) && (flag_sub_glocal_path_) && (flag_people_states_) && (flag_sub_cost_map_))
  {
    const double dist = calc_dist(local_goal_.point.x, local_goal_.point.y, 0.0, 0.0);

    if(dist > goal_tolerance_)
        return true;
    else
        return false;
  }
  else
  {
    return false;
  }
}

// glocal_goalの更新
void CocobotDWA::update_glocal_goal()
{
  int glocal_path_index = 0;  // glocal_pathのインデックス

  double dist_to_glocal_goal = calc_dist(glocal_path_.poses[glocal_path_index].pose.position.x, glocal_path_.poses[glocal_path_index].pose.position.y, 0.0, 0.0);

  while(dist_to_glocal_goal < dist_to_update_glocal_goal_)
  {
    glocal_path_index++;
    dist_to_glocal_goal = calc_dist(glocal_path_.poses[glocal_path_index].pose.position.x, glocal_path_.poses[glocal_path_index].pose.position.y, 0.0, 0.0);

    // glocal_path_indexがglocal_path_index.posesの配列の要素数を超えたら、glocal_goalとしてglocal_pathのゴールを設定
    if(glocal_path_index >= glocal_path_.poses.size())
    {
      glocal_path_index = glocal_path_.poses.size() - 1;
      break;
    }
  }

  // glocal_goalの更新
  glocal_goal_.header.frame_id = robot_frame_;
  glocal_goal_.point.x = glocal_path_.poses[glocal_path_index].pose.position.x;
  glocal_goal_.point.y = glocal_path_.poses[glocal_path_index].pose.position.y;
  pub_glocal_goal_.publish(glocal_goal_);  // デバック用
}

// ダイナミックウィンドウを計算
void CocobotDWA::calc_dynamic_window()
{
  // 車両モデルによるWindow
  double Vs[] = {min_vel_, max_vel_, -max_yawrate_, max_yawrate_};

  // 運動モデルによるWindow
  double Vd[] = { ccv_.velocity - max_accel_*dt_,
                  ccv_.velocity + max_accel_*dt_,
                  ccv_.yawrate  - max_yawaccel_*dt_,
                  ccv_.yawrate  + max_yawaccel_*dt_ };

  // 最終的なDynamic Window
  dw_.min_vel     = std::max(Vs[0], Vd[0]);
  dw_.max_vel     = std::min(Vs[1], Vd[1]);
  dw_.min_yawrate = std::max(Vs[2], Vd[2]);
  dw_.max_yawrate = std::min(Vs[3], Vd[3]);
}

// 適切な角度（-M_PI ~ M_PI）を返す
double CocobotDWA::normalize_angle(double theta)
{
  while(M_PI  < theta) theta -= 2.0*M_PI;
  while(theta < -M_PI) theta += 2.0*M_PI;

  return theta;
}

// 座標からグリッドのインデックスを返す
int CocobotDWA::xy_to_grid_index(const double x, const double y)
{
  const int index_x = int(floor((x - cost_map_.info.origin.position.x) / cost_map_.info.resolution));
  const int index_y = int(floor((y - cost_map_.info.origin.position.y) / cost_map_.info.resolution));

  return index_x + (index_y * cost_map_.info.width);
}

// 予測軌跡作成時における仮想ロボットを移動
void CocobotDWA::move(State& state, const double velocity, const double yawrate)
{
  state.yaw      += yawrate * dt_;
  state.yaw       = normalize_angle(state.yaw);
  state.x        += velocity * cos(state.yaw) * dt_;
  state.y        += velocity * sin(state.yaw) * dt_;
  state.velocity  = velocity;
  state.yawrate   = yawrate;
}

// 予測軌跡を生成
std::vector<State> CocobotDWA::calc_traj(const double velocity, const double yawrate)
{
  std::vector<State> trajectory;            // 軌跡格納用の動的配列
  State state = {0.0, 0.0, 0.0, 0.0, 0.0};  // 軌跡作成用の仮想ロボット

  // 軌跡を格納
  for(double t=0.0; t<=predict_time_; t+=dt_)
  {
    move(state, velocity, yawrate);
    trajectory.push_back(state);
  }

  return trajectory;
}

// 評価関数を計算
double CocobotDWA::calc_evaluation(const std::vector<State>& traj)
{
  const double heading_score  = weight_heading_  * calc_heading_eval(traj);
  const double glocal_heading_score = weight_glocal_heading_ * calc_glocal_heading_eval(traj);
  const double distance_score = weight_dist_     * calc_dist_eval(traj);
  const double velocity_score = weight_vel_      * calc_vel_eval(traj);
  const double cost_map_score = weight_cost_map_ * calc_cost_map_eval(traj);

  const double total_score = heading_score + distance_score + velocity_score;

  return total_score;
}

// heading（1項目）の評価関数を計算
double CocobotDWA::calc_heading_eval(const std::vector<State>& traj)
{
  // 最終時刻におけるロボットの方位
  const double theta = traj.back().yaw;

  // 最終時刻の位置に対するゴールの方位
  const double goal_theta = atan2(local_goal_.point.y - traj.back().y, local_goal_.point.x - traj.back().x);

  // ゴールまでの方位差分
  double target_theta = 0.0;
  if(goal_theta > theta)
    target_theta = goal_theta - theta;
  else
    target_theta = theta - goal_theta;

  // headingの評価値
  const double heading_eval = (M_PI - abs(normalize_angle(target_theta)))/M_PI; // 正規化

  return heading_eval;
}

// glocal_heading（2項目）の評価関数を計算
double CocobotDWA::calc_glocal_heading_eval(const std::vector<State>& traj)
{
  // 最終時刻におけるロボットの方位
  const double theta = traj.back().yaw;

  // 最終時刻の位置に対するゴールの方位
  const double goal_theta = atan2(glocal_goal_.point.y - traj.back().y, glocal_goal_.point.x - traj.back().x);

  // ゴールまでの方位差分
  double target_theta = 0.0;
  if(goal_theta > theta)
    target_theta = goal_theta - theta;
  else
    target_theta = theta - goal_theta;

  // headingの評価値
  const double heading_eval = (M_PI - abs(normalize_angle(target_theta)))/M_PI; // 正規化

  return heading_eval;
}

// distance（3項目）の評価関数を計算
double CocobotDWA::calc_dist_eval(const std::vector<State>& traj)
{  
  double min_dist = search_range_;  // 探索範囲で初期化

  const auto people = people_states_.front();

  // pathの点と障害物のすべての組み合わせを探索
  for(const auto& state : traj)
  {
    for(const auto& person : people->people_states)
    {
      // pathのうちの１点と障害物の距離を計算
      const double dist = calc_dist(person.pose.position.x, person.pose.position.y, state.x, state.y);
      
      // 障害物に衝突したパスを評価
      if(dist <= margin_)
        return -1e6;

      // 最小距離を更新
      if(dist < min_dist)
        min_dist = dist;
    }
  }

  return min_dist / search_range_; // 正規化
}

// velocity（4項目）の評価関数を計算
double CocobotDWA::calc_vel_eval(const std::vector<State>& traj)
{
  if(0.0 < traj.back().velocity) // 前進
    return traj.back().velocity/max_vel_; // 正規化
  else // 後退
    return 0.0;
}

// cost_map（5項目）の評価関数を計算する
// 大きいほど良くない
double CocobotDWA::calc_cost_map_eval(const std::vector<State>& traj)
{
  double total_cost = 0;  // 通過した場所の走行コストの合計値

  for(auto& state : traj)
  {
    const int grid_index = xy_to_grid_index(state.x, state.y);
    double cost = cost_map_.data[grid_index];

    total_cost += cost;
  }

  int traj_size = traj.size();
  if(traj_size == 0)
    traj_size = 1;  // ゼロ割り防止
  
  double cost_map_eval = total_cost / (min_cost_ * traj_size);

  return cost_map_eval;
}

// 最適な制御入力を計算
std::vector<double> CocobotDWA::calc_input()
{
  //input[0] = velocity, input[1] = yawrate;
  std::vector<double> input{0.0, 0.0};

  std::vector<std::vector<State>> trajectories;  // すべての軌跡格納用
  double max_score = -1e6;                       // 評価値の最大値格納用
  int index_of_max_score = 0;                    // 評価値の最大値に対する軌跡のインデックス格納用

  // ダイナミックウィンドウを計算
  calc_dynamic_window();

  // 並進速度と旋回速度のすべての組み合わせを評価
  int i = 0; // 現在の軌跡のインデックス保持用
  for(double velocity=dw_.min_vel; velocity<=dw_.max_vel; velocity+=vel_reso_)
  {
    for(double yawrate=dw_.min_yawrate; yawrate<=dw_.max_yawrate; yawrate+=yawrate_reso_)
    {
      const std::vector<State> trajectory = calc_traj(velocity, yawrate); // 予測軌跡の生成
      const double score = calc_evaluation(trajectory);                   // 予測軌跡に対する評価値の計算
      trajectories.push_back(trajectory);

      // 最大値の更新
      if(max_score < score)
      {
        max_score = score;
        input[0]  = velocity;
        input[1]  = yawrate;
        index_of_max_score = i;
      }

      i++;
    }
  }

  // 現在速度の記録
  ccv_.velocity = input[0];
  ccv_.yawrate  = input[1];

  // pathの可視化
  ros::Time now = ros::Time::now();
  for(i=0; i<trajectories.size(); i++)
  {
    if(i == index_of_max_score)
      visualize_traj(trajectories[i], pub_optimal_path_, now);
    else
      visualize_traj(trajectories[i], pub_predict_path_, now);
  }

   return input;
}

// 軌跡を可視化
void CocobotDWA::visualize_traj(const std::vector<State>& traj, const ros::Publisher& pub_local_path, ros::Time now)
{
  nav_msgs::Path local_path;
  local_path.header.stamp = now;
  local_path.header.frame_id = robot_frame_;

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = now;
  pose.header.frame_id = robot_frame_;

  for(const auto& state : traj)
  {
    pose.pose.position.x = state.x;
    pose.pose.position.y = state.y;
    local_path.poses.push_back(pose);
  }

  pub_local_path.publish(local_path);
}

// CCVの制御入力
void CocobotDWA::ccv_control(const double  velocity, const double yawrate)
{
  // 並進速度と角速度
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = velocity;
  cmd_vel.angular.z = yawrate;
  pub_cmd_vel_.publish(cmd_vel);
  
  // ステア角
  ccv_dynamixel_msgs::CmdPoseByRadian cmd_pos;
  cmd_pos.steer_r = 0.0;
  cmd_pos.steer_l = 0.0;
  double pitch_offset = 3.0*M_PI/180.0;
  cmd_pos.fore=pitch_offset;
  cmd_pos.rear=pitch_offset;
  cmd_pos.roll=0.0;
  pub_cmd_pos_.publish(cmd_pos);
}

//メイン文で実行する関数
void CocobotDWA::process()
{
 ros::Rate loop_rate(hz_);
 tf2_ros::TransformListener tf_listener(tf_buffer_);

  while(ros::ok())
  {
    if(is_goal())
    {
      update_glocal_goal();                      // glocal_goalを更新
      std::vector<double> input = calc_input();  // 最適な制御入力を計算
      ccv_control(input[0], input[1]);
    }
    else
    {
      ccv_control(0.0, 0.0);
    }

    // msgの受け取り判定用flagをfalseに戻す
    flag_local_goal_ = false;
    flag_glocal_path_ = false;
    flag_people_states_ = false;
    flag_cost_map_ = false;

    ros::spinOnce();
    loop_rate.sleep();
  }
}