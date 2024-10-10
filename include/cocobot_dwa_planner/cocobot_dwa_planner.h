#ifndef COCOBOT_DWA_PLANNER_H
#define COCOBOT_DWA_PLANNER_H

#include <ros/ros.h>
#include <queue>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include<tf2_ros/transform_listener.h>
#include <tf2/utils.h>

// original_msgs
#include <pedestrian_msgs/PersonState.h>
#include <pedestrian_msgs/PeopleStates.h>
#include<ccv_dynamixel_msgs/CmdPoseByRadian.h>
#include<dynamixel_workbench_msgs/DynamixelStateList.h>

// ===== 構造体 =====
struct State
{
  double x;        // [m]
  double y;        // [m]
  double yaw;      // [rad]
  double velocity; // [m/s]
  double yawrate;  // [rad/s]
};

struct DynamicWindow
{
  double min_vel;     // [m/s]
  double max_vel;     // [m/s]
  double min_yawrate; // [rad/s]
  double max_yawrate; // [rad/s]
};

// クラス
class CocobotDWA
{
public:
  CocobotDWA();
  void process();

private:
  // コールバック関数
  void local_goal_callback(const geometry_msgs::PointStampedConstPtr& msg);
  void glocal_path_callback(const nav_msgs::PathConstPtr& msg);
  void people_states_callback(const pedestrian_msgs::PeopleStatesConstPtr& msg);
  void cost_map_callback(const nav_msgs::OccupancyGridConstPtr& msg);

  // 引数あり関数
  double calc_dist(const double x1, const double y1, const double x2, const double y2);                      // 距離を計算
  double normalize_angle(double theta);                                                                      // 適切な角度（-M_PI ~ M_PI）を返す
  void move(State& state, const double velocity, const double yawrate);                                      // 予測軌跡作成時における仮想ロボットを移動
  std::vector<State> calc_traj(const double velocity, const double yawrate);                                 // 予測軌跡を生成
  double calc_heading_eval(const std::vector<State>& traj);                                                  // heading（1項目）の評価関数を計算
  double calc_dist_eval(const std::vector<State>& traj);                                                     // distance（2項目）の評価関数を計算
  double calc_vel_eval(const std::vector<State>& traj);                                                      // velocity（3項目）の評価関数を計算
  double calc_evaluation(const std::vector<State>& traj);                                                    // 評価関数を計算
  void visualize_traj(const std::vector<State>& traj, const ros::Publisher& pub_local_path, ros::Time now);  // 軌跡を可視化
  void ccv_control(const double  velocity, const double yawrate);                                            // CCVの制御入力

  // 引数なし関数
  bool is_goal();                    // goalに着くまでtrueを返す
  void calc_dynamic_window();        // ダイナミックウィンドウを計算
  std::vector<double> calc_input();  // 最適な制御入力を計算

  // yamlファイルで設定可能な変数
  int hz_;                   // ループ周波数 [Hz]
  double weight_heading_;    // 評価関数1項目　重みづけ定数
  double weight_dist_;       // 評価関数2項目　重みづけ定数
  double weight_vel_;        // 評価関数3項目　重みづけ定数
  double weight_cost_map_;   // 評価関数4項目　重みづけ定数
  double search_range_;      // 評価関数２項目(distance)探索範囲 [m]
  double margin_;            // 障害物とロボットの衝突半径（マージン込み） [m]
  double min_vel_;           // 最低並進速度 [m/s]
  double max_vel_;           // 最高並進速度 [m/s]
  double max_yawrate_;       // 最高旋回速度 [rad/s]
  double max_accel_;         // 最高並進加速度 [m/s^2]
  double max_yawaccel_;      // 最高旋回加速度 [rad/s^2]
  double goal_tolerance_;    // local_goal_に対する許容誤差 [m]
  std::string world_frame_;  // local_goal_のframe_id
  std::string robot_frame_;  // ロボットのframe_id
  double vel_reso_;          // 並進速度を計算するときの刻み幅 [m/s]
  double yawrate_reso_;      // 旋回速度を計算するときの刻み幅 [rad/s]
  double dt_;                // 微小時間 [s]
  double predict_time_;      // 予測時間 [s]

  // msgの受け取り判定用
  bool flag_local_goal_ = false;
  bool flag_glocal_path_ = false;
  bool flag_people_states_ = false;
  bool flag_cost_map_ = false;

  // CCV
  State ccv_;
  DynamicWindow dw_;

  // NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Subscriber
  ros::Subscriber sub_local_goal_;
  ros::Subscriber sub_glocal_path_;
  ros::Subscriber sub_people_states_;
  ros::Subscriber sub_cost_map_;

  // Publisher
  ros::Publisher pub_cmd_vel_;
  ros::Publisher pub_cmd_pos_;
  ros::Publisher pub_optimal_path_;
  ros::Publisher pub_predict_path_;

  // tf
  tf2_ros::Buffer tf_buffer_;

  // 各種オブジェクト
  geometry_msgs::PointStamped local_goal_;                           // local_goal
  nav_msgs::Path glocal_path_;                                       // glocal_path
  geometry_msgs::PointStamped glocal_goal_;                          // glocal_goal
  std::queue<pedestrian_msgs::PeopleStatesConstPtr> people_states_;  // 歩行者情報
  nav_msgs::OccupancyGrid cost_map_;                                 // コストマップ

};

#endif  // COCOBOT_DWA_PLANNER_H