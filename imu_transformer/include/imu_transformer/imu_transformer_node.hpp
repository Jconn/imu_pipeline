#ifndef IMU_TRANSFORMER_IMU_TRANSFORMER_NODELET
#define IMU_TRANSFORMER_IMU_TRANSFORMER_NODELET

#include <eigen3/Eigen/Geometry>
#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "message_filters/subscriber.h"

#include "tf2_ros/message_filter.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/transform_listener.h"

// cpplint: c++ system headers
#include <algorithm>
#include <csignal>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace imu_transformer
{
    using ImuMsg = sensor_msgs::msg::Imu;
    using MagMsg = sensor_msgs::msg::MagneticField;
    using ImuSubscriber = message_filters::Subscriber<ImuMsg>;
    using MagSubscriber = message_filters::Subscriber<MagMsg>;
    using ImuFilter = tf2_ros::MessageFilter<ImuMsg>;
    using MagFilter = tf2_ros::MessageFilter<MagMsg>;

    class ImuTransformer
    {

        public:
            ImuTransformer();
            virtual void onInit();
            std::shared_ptr<rclcpp::Node> node_;

        private:
            const std::string imu_topic_in_ = "ImuImu";
            const std::string imu_topic_out_ = "imu_data_out";

            const std::string mag_topic_in_ = "ImuMag";
            const std::string mag_topic_out_ = "mag_data_out";

            std::string imu_frame_;
            std::string target_frame_;

            std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

            std::unique_ptr<ImuSubscriber> imu_sub_;
            std::unique_ptr<MagSubscriber> mag_sub_;


            std::unique_ptr<ImuFilter> imu_filter_;
            std::unique_ptr<MagFilter> mag_filter_;


            rclcpp::Publisher<ImuMsg>::SharedPtr imu_pub_;
            rclcpp::Publisher<MagMsg>::SharedPtr mag_pub_;


            void imuCallback(ImuMsg::ConstSharedPtr imu_in);
            void magCallback(MagMsg::ConstSharedPtr msg);
            void get_parameters();
            void init_message_filters();
            void init_publishers();
            void pose_estimator(ImuMsg::ConstSharedPtr msg);
            float roll_init_ = 0;
            float pitch_init_ = 0;
            bool rpy_init_ = false;
    };

}  // namespace imu_transformer

#endif  // IMU_TRANSFORMER_IMU_TRANSFORMER_NODELET
