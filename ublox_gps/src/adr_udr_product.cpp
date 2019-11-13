#include <functional>
#include <stdexcept>
#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

#include <ublox_msgs/msg/esf_ins.hpp>
#include <ublox_msgs/msg/esf_meas.hpp>
#include <ublox_msgs/msg/esf_raw.hpp>
#include <ublox_msgs/msg/esf_status.hpp>
#include <ublox_msgs/msg/nav_att.hpp>

#include <ublox_gps/gnss.hpp>

namespace ublox_gps {

AdrUdrProduct::AdrUdrProduct(diagnostic_updater::Updater * updater) : updater_(updater)
{
}

//
// u-blox ADR devices, partially implemented
//
void AdrUdrProduct::getRosParams(const rclcpp::Node * node, const GNSS & gnss) {
  (void)gnss;

  use_adr_ = node->declare_parameter("use_adr", true);
  esf_enabled_ = node->declare_parameter("publish.esf.all", true);
  bool nav_enabled_ = node->get_parameter("publish.nav.all");
  nav_att_enabled_ = node->declare_parameter("publish.nav.att", nav_enabled_);
  esf_ins_enabled_ = node->declare_parameter("publish.esf.ins", esf_enabled_);
  esf_meas_enabled_ = node->declare_parameter("publish.esf.meas", esf_enabled_);
  esf_raw_enabled_ = node->declare_parameter("publish.esf.raw", esf_enabled_);
  esf_status_enabled_ = node->declare_parameter("publish.esf.status", esf_enabled_);
  hnr_pvt_enabled_ = node->declare_parameter("publish.hnr.pvt", true);

  // Check the nav rate
  float nav_rate_hz = 1000.0 / (meas_rate * nav_rate);
  if (nav_rate_hz != 1) {
    // ROS_WARN("Nav Rate recommended to be 1 Hz");
  }
}

bool AdrUdrProduct::configureUblox(const GNSS & gnss) {
  (void)gnss;
  if (!gps.setUseAdr(use_adr_)) {
    throw std::runtime_error(std::string("Failed to ")
                             + (use_adr_ ? "enable" : "disable") + "use_adr");
  }
  return true;
}

void AdrUdrProduct::subscribe(const rclcpp::Node * node) {
  // Subscribe to NAV ATT messages
  if (nav_att_enabled_) {
    gps.subscribe<ublox_msgs::msg::NavATT>(std::bind(
      publish<ublox_msgs::msg::NavATT>, std::placeholders::_1, "navatt"), 1);
  }

  // Subscribe to ESF INS messages
  if (esf_ins_enabled_) {
    gps.subscribe<ublox_msgs::msg::EsfINS>(std::bind(
        publish<ublox_msgs::msg::EsfINS>, std::placeholders::_1, "esfins"), 1);
  }

  // Subscribe to ESF Meas messages
  if (esf_meas_enabled_) {
    gps.subscribe<ublox_msgs::msg::EsfMEAS>(std::bind(
        publish<ublox_msgs::msg::EsfMEAS>, std::placeholders::_1, "esfmeas"), 1);
    // also publish sensor_msgs::Imu
    gps.subscribe<ublox_msgs::msg::EsfMEAS>(std::bind(
      &AdrUdrProduct::msg::callbackEsfMEAS, this, std::placeholders::_1), 1);

    imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>("imu_meas", 1);
    time_ref_pub_ = node->create_publisher<sensor_msgs::msg::TimeReference>("interrupt_time", 1);
  }

  // Subscribe to ESF Raw messages
  if (esf_raw_enabled_) {
    gps.subscribe<ublox_msgs::msg::EsfRAW>(std::bind(
        publish<ublox_msgs::msg::EsfRAW>, std::placeholders::_1, "esfraw"), 1);
  }

  // Subscribe to ESF Status messages
  if (esf_status_enabled_) {
    gps.subscribe<ublox_msgs::msg::EsfSTATUS>(std::bind(
        publish<ublox_msgs::msg::EsfSTATUS>, std::placeholders::_1, "esfstatus"), 1);
  }

  // Subscribe to HNR PVT messages
  if (hnr_pvt_enabled_) {
    gps.subscribe<ublox_msgs::msg::HnrPVT>(std::bind(
        publish<ublox_msgs::msg::HnrPVT>, std::placeholders::_1, "hnrpvt"), 1);
  }
}

void AdrUdrProduct::callbackEsfMEAS(const ublox_msgs::msg::EsfMEAS &m) {
  if (esf_meas_enabled_) {
    auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
    auto t_ref_msg = std::make_unique<sensor_msgs::msg::TimeReference>();

    imu_msg->header.stamp = rclcpp::Time::now();
    imu_msg->header.frame_id = frame_id;

    float deg_per_sec = pow(2, -12);
    float m_per_sec_sq = pow(2, -10);
    float deg_c = 1e-2;

    std::vector<unsigned int> imu_data = m.data;
    for (size_t i = 0; i < imu_data.size(); i++){
      unsigned int data_type = imu_data[i] >> 24; //grab the last six bits of data
      double data_sign = (imu_data[i] & (1 << 23)); //grab the sign (+/-) of the rest of the data
      unsigned int data_value = imu_data[i] & 0x7FFFFF; //grab the rest of the data...should be 23 bits

      if (data_sign == 0) {
        data_sign = -1;
      } else {
        data_sign = 1;
      }

      // ROS_INFO("data sign (+/-): %f", data_sign); //either 1 or -1....set by bit 23 in the data bitarray

      imu_msg->orientation_covariance[0] = -1;
      imu_msg->linear_acceleration_covariance[0] = -1;
      imu_msg->angular_velocity_covariance[0] = -1;

      if (data_type == 14) {
        if (data_sign == 1) {
	  imu_msg->angular_velocity.x = 2048 - data_value * deg_per_sec;
        } else {
          imu_msg->angular_velocity.x = data_sign * data_value * deg_per_sec;
        }
      } else if (data_type == 16) {
        // ROS_INFO("data_sign: %f", data_sign);
        // ROS_INFO("data_value: %u", data_value * m);
        if (data_sign == 1) {
	  imu_msg->linear_acceleration.x = 8191 - data_value * m_per_sec_sq;
        } else {
          imu_msg->linear_acceleration.x = data_sign * data_value * m_per_sec_sq;
        }
      } else if (data_type == 13) {
        if (data_sign == 1) {
	  imu_msg->angular_velocity.y = 2048 - data_value * deg_per_sec;
        } else {
          imu_msg->angular_velocity.y = data_sign * data_value * deg_per_sec;
        }
      } else if (data_type == 17) {
        if (data_sign == 1) {
	  imu_msg->linear_acceleration.y = 8191 - data_value * m_per_sec_sq;
        } else {
          imu_msg->linear_acceleration.y = data_sign * data_value * m_per_sec_sq;
        }
      } else if (data_type == 5) {
        if (data_sign == 1) {
	  imu_msg->angular_velocity.z = 2048 - data_value * deg_per_sec;
        } else {
          imu_msg->angular_velocity.z = data_sign * data_value * deg_per_sec;
        }
      } else if (data_type == 18) {
        if (data_sign == 1) {
	  imu_msg->linear_acceleration.z = 8191 - data_value * m_per_sec_sq;
        } else {
          imu_msg->linear_acceleration.z = data_sign * data_value * m_per_sec_sq;
        }
      } else if (data_type == 12) {
        // ROS_INFO("Temperature in celsius: %f", data_value * deg_c);
      } else {
        // ROS_INFO("data_type: %u", data_type);
        // ROS_INFO("data_value: %u", data_value);
      }

      // create time ref message and put in the data
      // t_ref_msg.header.seq = m.risingEdgeCount;
      // t_ref_msg.header.stamp = ros::Time::now();
      // t_ref_msg.header.frame_id = frame_id;

      // t_ref_msg.time_ref = ros::Time((m.wnR * 604800 + m.towMsR / 1000), (m.towMsR % 1000) * 1000000 + m.towSubMsR);

      // std::ostringstream src;
      // src << "TIM" << int(m.ch);
      // t_ref_msg.source = src.str();

      t_ref_msg->header.stamp = rclcpp::Time::now(); // create a new timestamp
      t_ref_msg->header.frame_id = frame_id;

      time_ref_pub_->publish(std::move(t_ref));
      imu_pub_->publish(std::move(imu));
    }
  }

  updater_->force_update();
}

}
