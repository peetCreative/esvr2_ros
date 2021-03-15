#pragma once
#include "Esvr2.h"
#include "Esvr2PoseState.h"

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

namespace esvr2_ros
{
    class PoseStateROSNode:
            virtual public esvr2::PoseState
    {
    private:
        ros::NodeHandle mNh;
        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener;
        void update();
    public:
        explicit PoseStateROSNode(ros::NodeHandle &nh);
        esvr2::RealArray16 getPose() override;
        esvr2::RealArray3 getPosition() override;
        esvr2::RealArray4 getOrientation() override;
    };
}