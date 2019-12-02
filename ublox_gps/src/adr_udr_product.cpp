#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/TimeReference.h>

#include <ublox_msgs/EsfINS.h>
#include <ublox_msgs/EsfMEAS.h>
#include <ublox_msgs/EsfRAW.h>
#include <ublox_msgs/EsfSTATUS.h>
#include <ublox_msgs/HnrPVT.h>
#include <ublox_msgs/NavATT.h>

#include <ublox_gps/adr_udr_product.hpp>
#include <ublox_gps/utils.hpp>

namespace ublox_node {

//
// u-blox ADR devices, partially implemented
//
AdrUdrProduct::AdrUdrProduct(uint16_t nav_rate, uint16_t meas_rate, const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, ros::NodeHandle* node)
  : nav_rate_(nav_rate), meas_rate_(meas_rate), frame_id_(frame_id), updater_(updater), node_(node)
{
  imu_pub_ =
    node_->advertise<sensor_msgs::Imu>("imu_meas", 1);
  time_ref_pub_ =
    node_->advertise<sensor_msgs::TimeReference>("interrupt_time", 1);
  nav_att_pub_ = node_->advertise<ublox_msgs::NavATT>("navatt", 1);
  esf_ins_pub_ = node_->advertise<ublox_msgs::EsfINS>("esfins", 1);
  esf_meas_pub_ = node_->advertise<ublox_msgs::EsfMEAS>("esfmeas", 1);
  esf_raw_pub_ = node_->advertise<ublox_msgs::EsfRAW>("esfraw", 1);
  esf_status_pub_ = node_->advertise<ublox_msgs::EsfSTATUS>("esfstatus", 1);
  hnr_pvt_pub_ = node_->advertise<ublox_msgs::HnrPVT>("hnrpvt", 1);
}

void AdrUdrProduct::getRosParams() {
  use_adr_ = getRosBoolean(node_, "use_adr");
  // Check the nav rate
  float nav_rate_hz = 1000 / (meas_rate_ * nav_rate_);
  if (nav_rate_hz != 1) {
    ROS_WARN("Nav Rate recommended to be 1 Hz");
  }
}

bool AdrUdrProduct::configureUblox(std::shared_ptr<ublox_gps::Gps> gps) {
  if (!gps->setUseAdr(use_adr_)) {
    throw std::runtime_error(std::string("Failed to ")
                             + (use_adr_ ? "enable" : "disable") + "use_adr");
  }
  return true;
}

void AdrUdrProduct::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  // Subscribe to NAV ATT messages
  if (getRosBoolean(node_, "publish/nav/att")) {
    gps->subscribe<ublox_msgs::NavATT>([this](const ublox_msgs::NavATT &m) { nav_att_pub_.publish(m); },
                                       1);
  }

  // Subscribe to ESF INS messages
  if (getRosBoolean(node_, "publish/esf/ins")) {
    gps->subscribe<ublox_msgs::EsfINS>([this](const ublox_msgs::EsfINS &m) { esf_ins_pub_.publish(m); },
                                       1);
  }

  // Subscribe to ESF Meas messages
  if (getRosBoolean(node_, "publish/esf/meas")) {
    gps->subscribe<ublox_msgs::EsfMEAS>([this](const ublox_msgs::EsfMEAS &m) { esf_meas_pub_.publish(m); },
                                        1);
    // also publish sensor_msgs::Imu
    gps->subscribe<ublox_msgs::EsfMEAS>(std::bind(
      &AdrUdrProduct::callbackEsfMEAS, this, std::placeholders::_1), 1);
  }

  // Subscribe to ESF Raw messages
  if (getRosBoolean(node_, "publish/esf/raw")) {
    gps->subscribe<ublox_msgs::EsfRAW>([this](const ublox_msgs::EsfRAW &m) { esf_raw_pub_.publish(m); },
                                       1);
  }

  // Subscribe to ESF Status messages
  if (getRosBoolean(node_, "publish/esf/status")) {
    gps->subscribe<ublox_msgs::EsfSTATUS>([this](const ublox_msgs::EsfSTATUS &m) { esf_status_pub_.publish(m); },
                                          1);
  }

  // Subscribe to HNR PVT messages
  if (getRosBoolean(node_, "publish/hnr/pvt")) {
    gps->subscribe<ublox_msgs::HnrPVT>([this](const ublox_msgs::HnrPVT &m) { hnr_pvt_pub_.publish(m); },
                                       1);
  }
}

void AdrUdrProduct::callbackEsfMEAS(const ublox_msgs::EsfMEAS &m) {
  if (getRosBoolean(node_, "publish/esf/meas")) {
    imu_.header.stamp = ros::Time::now();
    imu_.header.frame_id = frame_id_;

    float deg_per_sec = pow(2, -12);
    float m_per_sec_sq = pow(2, -10);
    float deg_c = 1e-2;

    std::vector<unsigned int> imu_data = m.data;
    for (int i = 0; i < imu_data.size(); i++){
      unsigned int data_type = imu_data[i] >> 24; //grab the last six bits of data
      double data_sign = (imu_data[i] & (1 << 23)); //grab the sign (+/-) of the rest of the data
      unsigned int data_value = imu_data[i] & 0x7FFFFF; //grab the rest of the data...should be 23 bits

      if (data_sign == 0) {
        data_sign = -1;
      } else {
        data_sign = 1;
      }

      //ROS_INFO("data sign (+/-): %f", data_sign); //either 1 or -1....set by bit 23 in the data bitarray

      imu_.orientation_covariance[0] = -1;
      imu_.linear_acceleration_covariance[0] = -1;
      imu_.angular_velocity_covariance[0] = -1;

      if (data_type == 14) {
        if (data_sign == 1) {
	  imu_.angular_velocity.x = 2048 - data_value * deg_per_sec;
        } else {
          imu_.angular_velocity.x = data_sign * data_value * deg_per_sec;
        }
      } else if (data_type == 16) {
        //ROS_INFO("data_sign: %f", data_sign);
        //ROS_INFO("data_value: %u", data_value * m);
        if (data_sign == 1) {
	  imu_.linear_acceleration.x = 8191 - data_value * m_per_sec_sq;
        } else {
          imu_.linear_acceleration.x = data_sign * data_value * m_per_sec_sq;
        }
      } else if (data_type == 13) {
        if (data_sign == 1) {
	  imu_.angular_velocity.y = 2048 - data_value * deg_per_sec;
        } else {
          imu_.angular_velocity.y = data_sign * data_value * deg_per_sec;
        }
      } else if (data_type == 17) {
        if (data_sign == 1) {
	  imu_.linear_acceleration.y = 8191 - data_value * m_per_sec_sq;
        } else {
          imu_.linear_acceleration.y = data_sign * data_value * m_per_sec_sq;
        }
      } else if (data_type == 5) {
        if (data_sign == 1) {
	  imu_.angular_velocity.z = 2048 - data_value * deg_per_sec;
        } else {
          imu_.angular_velocity.z = data_sign * data_value * deg_per_sec;
        }
      } else if (data_type == 18) {
        if (data_sign == 1) {
	  imu_.linear_acceleration.z = 8191 - data_value * m_per_sec_sq;
        } else {
          imu_.linear_acceleration.z = data_sign * data_value * m_per_sec_sq;
        }
      } else if (data_type == 12) {
        //ROS_INFO("Temperature in celsius: %f", data_value * deg_c);
      } else {
        ROS_INFO("data_type: %u", data_type);
        ROS_INFO("data_value: %u", data_value);
      }

      // create time ref message and put in the data
      //t_ref_.header.seq = m.risingEdgeCount;
      //t_ref_.header.stamp = ros::Time::now();
      //t_ref_.header.frame_id = frame_id_;

      //t_ref_.time_ref = ros::Time((m.wnR * 604800 + m.towMsR / 1000), (m.towMsR % 1000) * 1000000 + m.towSubMsR);

      //std::ostringstream src;
      //src << "TIM" << int(m.ch);
      //t_ref_.source = src.str();

      t_ref_.header.stamp = ros::Time::now(); // create a new timestamp
      t_ref_.header.frame_id = frame_id_;

      time_ref_pub_.publish(t_ref_);
      imu_pub_.publish(imu_);
    }
  }

  updater_->force_update();
}

}  // namespace ublox_node