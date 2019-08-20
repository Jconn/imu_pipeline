#include "imu_transformer/imu_transformer_node.hpp"
#include <tf2/transform_datatypes.h>
#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <chrono>
#include "imu_transformer/tf2_sensor_msgs.h"
#include <math.h>       /* atan2 */
#include <numeric> //std::accumulator
#include <deque>
//#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
//Remove this header when https://github.com/ros/geometry_experimental/pull/78 is released
//#include "imu_transformer/msgs/tf2_sensor_msgs.h"

namespace imu_transformer
{

    using ImuMsg = sensor_msgs::msg::Imu;
    using MagMsg = sensor_msgs::msg::MagneticField;
    using ImuSubscriber = message_filters::Subscriber<ImuMsg>;
    //  typedef message_filters::Subscriber<MagMsg> MagSubscriber;
    using MagSubscriber = message_filters::Subscriber<MagMsg>;
    using ImuFilter = tf2_ros::MessageFilter<ImuMsg>;
    using MagFilter = tf2_ros::MessageFilter<MagMsg>;

    ImuTransformer::ImuTransformer() :
        target_frame_(""),
        imu_frame_("")
    {
        node_ = std::make_shared<rclcpp::Node>("ImuTransformer",
                rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    }

    void ImuTransformer::get_parameters()
    {
        node_->get_parameter("target_frame", target_frame_);
        node_->get_parameter("imu_frame", imu_frame_);
    }

    void ImuTransformer::init_message_filters()
    {

        RCLCPP_INFO(node_->get_logger(), "setting up message filters, to topics (%s,%s), and (%s,%s)", imu_topic_in_.c_str(),imu_frame_.c_str(), mag_topic_in_.c_str(), imu_frame_.c_str());

        imu_sub_ = std::make_unique<ImuSubscriber>(
                node_.get(), imu_topic_in_, rmw_qos_profile_sensor_data);

        imu_filter_ = std::make_unique<ImuFilter>(
                *imu_sub_, *tf2_buffer_, imu_frame_, 10, node_);


        imu_filter_->registerCallback(std::bind(&ImuTransformer::imuCallback, this, std::placeholders::_1));

        mag_sub_ = std::make_unique<MagSubscriber>(
                node_.get(), mag_topic_in_, rmw_qos_profile_sensor_data);

        mag_filter_ = std::make_unique<MagFilter>(
                *mag_sub_, *tf2_buffer_, imu_frame_, 10, node_);

        mag_filter_->registerCallback(std::bind(&ImuTransformer::magCallback, this, std::placeholders::_1));


    }

    void ImuTransformer::init_publishers() {

        RCLCPP_INFO(node_->get_logger(), "setting up message publishers");

        imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(imu_topic_out_, rclcpp::SensorDataQoS() );

        mag_pub_ = node_->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic_out_, rclcpp::SensorDataQoS() );
    }

    void ImuTransformer::onInit(){

        get_parameters();

        tf2_buffer_ = 
            std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf2_listener_ = 
            std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

        init_message_filters();
        init_publishers();

    }
    static inline float aMean(std::deque<float> in) {
        return std::accumulate(in.begin(), in.end(), 0.0)/in.size(); 
    }

    void ImuTransformer::pose_estimator(ImuMsg::ConstSharedPtr msg)
    {
        const size_t filter_size = 180; 
        static std::deque<float> qY, qX, qZ;

        qX.push_back(msg->linear_acceleration.x);
        qY.push_back(msg->linear_acceleration.y);
        qZ.push_back(msg->linear_acceleration.z);
        
        if(qX.size() > filter_size)
        {
            qX.pop_front();
            qY.pop_front();
            qZ.pop_front();
        }
        else
        {
            return;
        }
        
        float roll, pitch, ax, ay, az;

        ax = aMean(qX); 
        ay = aMean(qY); 
        az = aMean(qZ); 

        roll = atan2(ay, az);
        pitch = atan2(-ax, sqrt(ay*ay + az*az) );
        RCLCPP_INFO(node_->get_logger(), "roll: %f,\npitch: %f", roll, pitch);
        if(!rpy_init_)
        {
            rpy_init_ = true;
            roll_init_ = roll;
            pitch_init_ = pitch;
        }
    }
    void ImuTransformer::imuCallback(ImuMsg::ConstSharedPtr msg)
    {
        pose_estimator(msg);
        try
        {
            ImuMsg imu_out;
            //tf2_buffer_->transform(*imu_in, imu_out, target_frame_);

            tf2_buffer_->canTransform(msg->header.frame_id, target_frame_, tf2::TimePoint(), tf2::durationFromSec(1.0));
            auto transform_tmp = tf2_buffer_->lookupTransform(msg->header.frame_id, target_frame_, tf2::TimePoint());
            tf2::Quaternion q_found_pose, q_orig, q_new;
            
            q_orig.setValue(transform_tmp.transform.rotation.x,
                    transform_tmp.transform.rotation.y,
                    transform_tmp.transform.rotation.z,
                    transform_tmp.transform.rotation.w);

            q_found_pose.setRPY(roll_init_, pitch_init_, 0);
            q_new = q_orig*q_found_pose;
            q_new.normalize();

            transform_tmp.transform.rotation.x = q_new.x();
            transform_tmp.transform.rotation.y = q_new.y();
            transform_tmp.transform.rotation.z = q_new.z();
            transform_tmp.transform.rotation.w = q_new.w();
            RCLCPP_INFO(node_->get_logger(), "transforming imu");
            tf2::doTransform(*msg,
                    imu_out, 
                    transform_tmp
                    );
            imu_out.header = msg->header;
            imu_out.header.frame_id = target_frame_;
            imu_pub_->publish(imu_out);
        }
        catch (tf2::TransformException ex)
        {
            RCLCPP_ERROR(node_->get_logger(), "failure performing IMU transform");
            return;
        }
    }

    // Need to support two types of magnemoter message for now, replace with MagMsg subscriber when IMU REP goes into
    // effect
    void ImuTransformer::magCallback(MagMsg::ConstSharedPtr msg)
    {
        std::string error;

        try
        {
            MagMsg mag_out;

            tf2_buffer_->canTransform(msg->header.frame_id, target_frame_, tf2::TimePoint(), tf2::durationFromSec(1.0));
            auto transform_tmp = tf2_buffer_->lookupTransform(msg->header.frame_id, target_frame_, tf2::TimePoint());

            RCLCPP_INFO(node_->get_logger(), "transforming mag");
            tf2::doTransform(*msg,
                mag_out, 
                transform_tmp
                );
            //tf2_buffer_->transform(*msg, out, target_frame_);
            mag_pub_->publish(mag_out);
        }
        catch (tf2::TransformException ex)
        {
            RCLCPP_ERROR(node_->get_logger(), "failure performing mag transform");
            return;
        }
        return;

    }

}



int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto imu_transform = std::make_shared<imu_transformer::ImuTransformer>();
    imu_transform->onInit(); 
    rclcpp::spin(imu_transform->node_);
    rclcpp::shutdown();
}
