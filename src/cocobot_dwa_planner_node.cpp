#include "cocobot_dwa_planner/cocobot_dwa_planner.h"

CocobotDWA::CocobotDWA():private_nh_("~")
{
  // param
  private_nh_.param("hz", hz_, {10});
  private_nh_.param("weight_heading", weight_heading_, {0.4});
  private_nh_.param("weight_dist", weight_dist_, {0.8});
  private_nh_.param("weight_vel", weight_vel_, {0.2});
  private_nh_.param("search_range", search_range_, {7.0});
  private_nh_.param("margin", margin_, {0.5});
  private_nh_.param("min_vel", min_vel_, {0.0});
  private_nh_.param("max_vel", max_vel_, {1.2});
  private_nh_.param("max_yawrate", max_yawrate_, {4.5});
  private_nh_.param("max_accel", max_accel_, {10.0});
  private_nh_.param("max_yawaccel", max_yawaccel_, {10.0});
  private_nh_.param("goal_tolerance", goal_tolerance_, {0.3});
  private_nh_.param("world_frame", world_frame_, {"odom"});
  private_nh_.param("robot_frame", robot_frame_, {"base_footprint"});
  private_nh_.param("vel_reso", vel_reso_, {0.1});
  private_nh_.param("yawrate_reso", yawrate_reso_, {0.1});
  private_nh_.param("dt", dt_, {0.1});
  private_nh_.param("predict_time", predict_time_, {3.0});

  // subscriber
  sub_people_states_ = nh_.subscribe("/transformed_people_states", 1, &CocobotDWA::people_states_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_local_goal_ = nh_.subscribe("/local_goal", 1, &CocobotDWA::local_goal_callback, this, ros::TransportHints().reliable().tcpNoDelay());

  // publisher
  pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/local/cmd_vel", 1);
  pub_cmd_pos_ = nh_.advertise<ccv_dynamixel_msgs::CmdPoseByRadian>("/local/cmd_pos", 1);

  // debug
  pub_predict_path_ = nh_.advertise<nav_msgs::Path>("/predict_local_paths", 1);
  pub_optimal_path_ = nh_.advertise<nav_msgs::Path>("/optimal_local_path", 1);
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
  // msg受信済みか確認
  if((flag_local_goal_) && (flag_people_states_))
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

// distance（2項目）の評価関数を計算
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

// velocity（3項目）の評価関数を計算
double CocobotDWA::calc_vel_eval(const std::vector<State>& traj)
{
  if(0.0 < traj.back().velocity) // 前進
    return traj.back().velocity/max_vel_; // 正規化
  else // 後退
    return 0.0;
}

// 評価関数を計算
double CocobotDWA::calc_evaluation(const std::vector<State>& traj)
{
  const double heading_score  = weight_heading_ * calc_heading_eval(traj);
  const double distance_score = weight_dist_    * calc_dist_eval(traj);
  const double velocity_score = weight_vel_     * calc_vel_eval(traj);

  const double total_score = heading_score + distance_score + velocity_score;

  return total_score;
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
      std::vector<double> input = calc_input();
      ccv_control(input[0], input[1]);
    }
    else
    {
      ccv_control(0.0, 0.0);
    }

    // msgの受け取り判定用flagをfalseに戻す
    flag_people_states_ = false;
    flag_local_goal_ = false;

    ros::spinOnce();
    loop_rate.sleep();
  }
}