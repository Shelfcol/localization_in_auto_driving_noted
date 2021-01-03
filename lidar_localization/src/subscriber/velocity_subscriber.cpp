/*
 * @Description: 订阅imu数据
 * @Author: Ren Qian
 * @Date: 2019-06-14 16:44:18
 */
#include "lidar_localization/subscriber/velocity_subscriber.hpp"

#include "glog/logging.h"

namespace lidar_localization{
VelocitySubscriber::VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &VelocitySubscriber::msg_callback, this);
}

void VelocitySubscriber::msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr) {
    buff_mutex_.lock();
    VelocityData velocity_data;
    velocity_data.time = twist_msg_ptr->header.stamp.toSec();

    velocity_data.linear_velocity.x = twist_msg_ptr->twist.linear.x;
    velocity_data.linear_velocity.y = twist_msg_ptr->twist.linear.y;
    velocity_data.linear_velocity.z = twist_msg_ptr->twist.linear.z;

    velocity_data.angular_velocity.x = twist_msg_ptr->twist.angular.x;
    velocity_data.angular_velocity.y = twist_msg_ptr->twist.angular.y;
    velocity_data.angular_velocity.z = twist_msg_ptr->twist.angular.z;

    new_velocity_data_.push_back(velocity_data);
    buff_mutex_.unlock();
}

void VelocitySubscriber::ParseData(std::deque<VelocityData>& velocity_data_buff) {
    buff_mutex_.lock();//运行之后如果没有unlock，则下次再到此位置时就会堵在这里。lock和unlock必须成对出现。
    //也以用std::lock_guard<std::mutex> my_lock_guard(buff_mutex_)，这个只在当前作用域有效，出了这个作用域相当于自动执行unlock
    if (new_velocity_data_.size() > 0) {
        velocity_data_buff.insert(velocity_data_buff.end(), new_velocity_data_.begin(), new_velocity_data_.end());
        new_velocity_data_.clear();
    }
    buff_mutex_.unlock();
}
}