// Copyright 2023 REMAKE.AI, KAIA.AI, MAKERSPET.COM
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "kaiaai_msgs/msg/kaiaai_telemetry.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "lds_ydlidar_x4.h"
#include "lds_ydlidar_x3_pro.h"
#include "lds_ydlidar_x2.h"
#include "lds_lds02rr.h"

using std::placeholders::_1;

class KaiaaiTelemetry : public rclcpp::Node
{
public:
  inline static const std::string NODE_NAME = "kaiaai_telemetry_node";
  static constexpr double DEG_TO_RAD = 0.017453292519943295769236907684886;

public:
  KaiaaiTelemetry()
  : Node(NODE_NAME)
  {
    this->declare_parameter("laser_sensor.model", std::vector<std::string>({"LDS02RR", "LDS02RR"}));
    this->declare_parameter("laser_sensor.angle_offset_deg", std::vector<double>({0.0, -180.0}));
    this->declare_parameter("laser_sensor.pub_scan_size", std::vector<int>({561, 360}));
    this->declare_parameter("laser_sensor.range_min_meters", std::vector<double>({0.15, 0.15}));
    this->declare_parameter("laser_sensor.range_max_meters", std::vector<double>({12.0, 6.0}));

    this->declare_parameter("laser_scan.topic_name_pub", "scan");
    this->declare_parameter("laser_scan.frame_id", "base_scan");
    this->declare_parameter("laser_scan.lds_model", "YDLIDAR-X4");
    this->declare_parameter("laser_scan.mask_radius_meters", 0.0);

    this->declare_parameter("telemetry.topic_name_sub", "telemetry");

    this->declare_parameter("tf.frame_id", "odom");
    this->declare_parameter("tf.child_frame_id", "base_footprint");

    this->declare_parameter("joints.topic_name_pub", "joint_states");
    this->declare_parameter("joints.wheel.right", "wheel_right_joint");
    this->declare_parameter("joints.wheel.left", "wheel_left_joint");

    this->declare_parameter("odometry.frame_id", "odom");
    this->declare_parameter("odometry.child_frame_id", "base_footprint");
    this->declare_parameter("odometry.topic_name_pub", "odom");

    telem_sub_ = this->create_subscription<kaiaai_msgs::msg::KaiaaiTelemetry>(
      this->get_parameter("telemetry.topic_name_sub").as_string(),
      rclcpp::SensorDataQoS(), std::bind(&KaiaaiTelemetry::topic_callback, this, _1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      this->get_parameter("odometry.topic_name_pub").as_string(), 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      this->get_parameter("joints.topic_name_pub").as_string(), 10);
    laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      this->get_parameter("laser_scan.topic_name_pub").as_string(), 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    plds = NULL;
    angle_offset_deg_ = 0.0;
    pub_scan_size_ = 360;
    range_min_meters_ = 0.15;
    range_max_meters_ = 12.0;
    mask_radius_meters_ = 0.0;

    clear_ranges_buffer();
    seq_last_ = 0;
    lds_data_idx_ = 0;
    pmsg = NULL;
  }

  ~KaiaaiTelemetry()
  {
    if (plds != NULL) {
      delete plds;
      plds = NULL;
    }
  }

private:
  void topic_callback(const kaiaai_msgs::msg::KaiaaiTelemetry & telem_msg) // const
  {
    long int seq_diff = (long int)telem_msg.seq - (long int)seq_last_;
    seq_last_ = telem_msg.seq;
    //RCLCPP_INFO(this->get_logger(), "Seq %u (%ld) len %lu", telem_msg.seq, seq_diff, telem_msg.lds.size());

    if (seq_diff > 1)
      RCLCPP_INFO(this->get_logger(), "%ld message(s) lost", seq_diff-1);

    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.frame_id = this->get_parameter("odometry.frame_id").as_string();
    odom_msg.child_frame_id = this->get_parameter("odometry.child_frame_id").as_string();
    odom_msg.header.stamp = telem_msg.stamp;
    odom_msg.pose.pose.position.x = telem_msg.odom_pos_x;
    odom_msg.pose.pose.position.y = telem_msg.odom_pos_y;
    odom_msg.pose.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, telem_msg.odom_pos_yaw); // roll, pitch, yaw
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = telem_msg.odom_vel_x;
    odom_msg.twist.twist.angular.z = telem_msg.odom_vel_yaw;

    odom_pub_->publish(odom_msg);


    auto joint_state_msg = sensor_msgs::msg::JointState();
    std::vector<std::string> name;
    name.resize(2);
    name[0] = this->get_parameter("joints.wheel.left").as_string();
    name[1] = this->get_parameter("joints.wheel.right").as_string();

    joint_state_msg.name = name;  // deep copy

    std::vector<double> position;
    position.resize(2);
    position[0] = telem_msg.joint_pos[0];
    position[1] = telem_msg.joint_pos[1];
    joint_state_msg.position = position;

    std::vector<double> velocity;
    velocity.resize(2);
    velocity[0] = telem_msg.joint_vel[0];
    velocity[1] = telem_msg.joint_vel[1];
    joint_state_msg.velocity = velocity;
    //float64[] effort
    joint_state_msg.header.stamp = telem_msg.stamp;

    joint_state_pub_->publish(joint_state_msg);


    // https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Cpp.html
    geometry_msgs::msg::TransformStamped tf_stamped_msg;
    tf_stamped_msg.header.stamp = telem_msg.stamp;
    tf_stamped_msg.header.frame_id = this->get_parameter("tf.frame_id").as_string();
    tf_stamped_msg.child_frame_id = this->get_parameter("tf.child_frame_id").as_string();

    tf_stamped_msg.transform.translation.x = odom_msg.pose.pose.position.x;
    tf_stamped_msg.transform.translation.y = odom_msg.pose.pose.position.y;
    tf_stamped_msg.transform.translation.z = odom_msg.pose.pose.position.z;

    tf_stamped_msg.transform.rotation.x = odom_msg.pose.pose.orientation.x;
    tf_stamped_msg.transform.rotation.y = odom_msg.pose.pose.orientation.y;
    tf_stamped_msg.transform.rotation.z = odom_msg.pose.pose.orientation.z;
    tf_stamped_msg.transform.rotation.w = odom_msg.pose.pose.orientation.w;

    tf_broadcaster_->sendTransform(tf_stamped_msg);

    if (telem_msg.lds.size() > 0) {
      if (plds == NULL)
        lds_setup();
      process_lds_data(telem_msg);
    }
  }

  void lds_setup()
  {
    //RCLCPP_INFO(this->get_logger(), "string[] %s, double[] %s",
    //            this->get_parameter("laser_sensor.model").value_to_string().c_str(),
    //            this->get_parameter("laser_sensor.angle_offset_deg").value_to_string().c_str());
    if (plds != NULL)
      return;

    const std::vector<std::string> model = this->get_parameter("laser_sensor.model").as_string_array();
    const std::vector<long int> pub_scan_size = this->get_parameter("laser_sensor.pub_scan_size").as_integer_array();
    const std::vector<double> angle_offset_deg = this->get_parameter("laser_sensor.angle_offset_deg").as_double_array();
    const std::vector<double> range_min_meters = this->get_parameter("laser_sensor.range_min_meters").as_double_array();
    const std::vector<double> range_max_meters = this->get_parameter("laser_sensor.range_max_meters").as_double_array();

    long unsigned int model_count = model.size();
    if (pub_scan_size.size() != model_count || angle_offset_deg.size() != model_count
        || range_min_meters.size() != model_count || range_max_meters.size() != model_count) {
      RCLCPP_FATAL(this->get_logger(), "laser_sensor parameter array sizes must be equal");
      rclcpp::shutdown();
    }

    const std::string lds_model = this->get_parameter("laser_scan.lds_model").as_string();

    int model_idx = 0;
    for (auto &s: model) {

      if (lds_model.compare(s) == 0) {
        if (s.compare(LDS_LDSRR02::get_model_name()) == 0) {
          plds = new LDS_LDSRR02();
          break;
        } else {
          if (s.compare(LDS_YDLidarX2::get_model_name()) == 0) {
            plds = new LDS_YDLidarX2();
            break;
          } else {
            if (s.compare(LDS_YDLidarX3PRO::get_model_name()) == 0) {
              plds = new LDS_YDLidarX3PRO();
              break;
            } else {
              if (s.compare(LDS_YDLidarX4::get_model_name()) == 0) {
                plds = new LDS_YDLidarX4();
                break;
              }
            }
          }
        }
      }
      model_idx++;
    }

    if (plds == NULL) {
      RCLCPP_FATAL(this->get_logger(), "LDS model %s not found", lds_model.c_str());
      rclcpp::shutdown();
      return;
    }

    plds->setReadByteCallback(read_byte_callback);
    plds->setScanPointCallback(scan_point_callback);

    angle_offset_deg_ = angle_offset_deg[model_idx];
    pub_scan_size_ = pub_scan_size[model_idx];
    range_min_meters_ = range_min_meters[model_idx];
    range_max_meters_ = range_max_meters[model_idx];
    ranges_.resize(pub_scan_size_);

    // RCLCPP_INFO(this->get_logger(), "Laser sensor model %s, pub_scan_size_ %d, angle_offset_deg_ %f",
    //   lds_model.c_str(), pub_scan_size_, angle_offset_deg_);
    RCLCPP_INFO(this->get_logger(), "LDS model %s", lds_model.c_str());
  }

  void process_lds_data(const kaiaai_msgs::msg::KaiaaiTelemetry & telem_msg)
  {
    if (plds == NULL)
      return;

    //RCLCPP_INFO(this->get_logger(), "process_lds_data() %lu", telem_msg.lds.size());
    lds_data_idx_ = 0;
    lds_msg_count_++;

    pmsg = const_cast<kaiaai_msgs::msg::KaiaaiTelemetry *>(& telem_msg);
    while (lds_data_idx_ < telem_msg.lds.size()) {

      int err = plds->decode_data(this);

      switch(err) {
        case LDS::RESULT_OK:
          break;
        case LDS::RESULT_CRC_ERROR:
          RCLCPP_INFO(this->get_logger(), "RESULT_CRC_ERROR");
          lds_crc_error_count_++;
          break;
        case LDS::RESULT_NOT_READY:
          break;
        case LDS::RESULT_INVALID_PACKET:
          RCLCPP_INFO(this->get_logger(), "RESULT_INVALID_PACKET");
          lds_invalid_packet_count_++;
          break;
        default:
          RCLCPP_WARN(this->get_logger(), "Unexpected lds.decode_data() result code %d", err);
      }
    }
    pmsg = NULL;
  }

  int read_lds_byte()
  {
    if ((pmsg == NULL) || (lds_data_idx_ >= pmsg->lds.size()))
      return -1;

    lds_data_length_++;
    return pmsg->lds[lds_data_idx_++];
  }

  static int read_byte_callback(const void * context)
  {
    void * ctx = const_cast<void *>(context);
    return reinterpret_cast<KaiaaiTelemetry*>(ctx)->read_lds_byte();
  }

  static void scan_point_callback(const void * context,
    float angle_deg, float distance_mm, float quality, bool scan_completed)
  {
    void * ctx = const_cast<void *>(context);
    return reinterpret_cast<KaiaaiTelemetry*>(ctx)->process_scan_point(
      angle_deg, distance_mm, quality, scan_completed);
  }

  void process_scan_point(float angle_deg, float distance_mm, float quality, bool scan_completed)
  {
    (void)quality; // Suppress unused parameter warning

//    RCLCPP_INFO(this->get_logger(), "process_scan_point() %f angle_deg %f distance_mm %f quality %d scan_completed",
//      angle_deg, distance_mm, quality, scan_completed);

    if (scan_completed)
    {
      publish_scan();
      clear_ranges_buffer();
      return;
    }

    scan_point_count_total_++;
    if (distance_mm == 0.0)  // Invalid measurement
      return;


    double distance_meters = distance_mm*0.001;
    if (mask_radius_meters_ >= distance_meters) // Ignore
      return;

    angle_deg = fmod(angle_deg, 360.0);
    angle_deg = angle_deg < 0 ? angle_deg + 360 : angle_deg;

    double laser_scan_angle_increment = 360.0 / pub_scan_size_;
    int idx = round(angle_deg / laser_scan_angle_increment);

    if (idx >= 0 && idx < ((long int)ranges_.size()))
    {
      ranges_[idx] = distance_meters;
      scan_point_count_valid_++;
    }
  }

  void clear_ranges_buffer()
  {
    std::fill(ranges_.begin(), ranges_.end(), 0);
    scan_point_count_valid_ = 0;
    scan_point_count_total_ = 0;
    lds_invalid_packet_count_ = 0;
    lds_crc_error_count_ = 0;
    lds_msg_count_ = 0;
    lds_data_length_ = 0;
    mask_radius_meters_ = this->get_parameter("laser_scan.mask_radius_meters").as_double();
  }

  void publish_scan()
  {
    //RCLCPP_INFO(this->get_logger(), "publish_scan() total %u valid %u fail %u crc %u msg %u len %u",
    //  scan_point_count_total_, scan_point_count_valid_, lds_invalid_packet_count_, lds_crc_error_count_,
    //  lds_msg_count_, lds_data_length_);

    auto laser_scan_msg = sensor_msgs::msg::LaserScan();
    double laser_scan_angle_increment = 360.0 / pub_scan_size_;

    laser_scan_msg.ranges = ranges_;
    laser_scan_msg.header.stamp = pmsg->stamp;
    laser_scan_msg.header.frame_id = this->get_parameter("laser_scan.frame_id").as_string();
    laser_scan_msg.angle_min = angle_offset_deg_ * DEG_TO_RAD;
    laser_scan_msg.angle_max = (angle_offset_deg_ + 360.0 * (pub_scan_size_ - 1) / pub_scan_size_) * DEG_TO_RAD;
    laser_scan_msg.angle_increment = laser_scan_angle_increment * DEG_TO_RAD;
    laser_scan_msg.range_min = range_min_meters_;
    laser_scan_msg.range_max = range_max_meters_;
    laser_scan_msg.time_increment = 0; // TODO
    //float32 scan_time;
    //float32[] intensities;

    laser_scan_pub_->publish(laser_scan_msg);
  }

  rclcpp::Subscription<kaiaai_msgs::msg::KaiaaiTelemetry>::SharedPtr telem_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<float> ranges_;
  unsigned int seq_last_;
  unsigned int scan_point_count_valid_;
  unsigned int scan_point_count_total_;
  unsigned int lds_invalid_packet_count_;
  unsigned int lds_crc_error_count_;
  unsigned int lds_msg_count_;
  unsigned int lds_data_idx_;
  unsigned int lds_data_length_;

  LDS * plds;
  double angle_offset_deg_;
  int pub_scan_size_;
  double range_min_meters_;
  double range_max_meters_;
  double mask_radius_meters_;

  kaiaai_msgs::msg::KaiaaiTelemetry * pmsg;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KaiaaiTelemetry>());
  rclcpp::shutdown();
  return 0;
}
