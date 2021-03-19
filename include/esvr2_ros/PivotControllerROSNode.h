#pragma once

#include "Esvr2.h"
#include "Esvr2LaparoscopeController.h"
#include "Esvr2Component.h"

#include "pivot_control_messages_ros/SetPose.h"
#include "pivot_control_messages_ros/LaparoscopeDOFPose.h"
#include "pivot_control_messages_ros/LaparoscopeDOFBoundaries.h"
#include "PivotControlMessages.h"

#include <ros/ros.h>

namespace esvr2_ros {
    class PivotControllerROSNode :
            public esvr2::LaparoscopeController
    {
    private:
        ros::NodeHandle mNh;

        ros::Publisher mLaparoscopeTargetDOFPosePub;
        ros::Subscriber mLaparoscopeBoundariesSub;
        ros::Subscriber mLaparoscopeCurDOFPoseSub;
        int mLaparoscopePoseSeq {0};
        ros::ServiceServer mForceSetDofPoseService;
        ros::Timer mForceSetDofPoseTimer;
        bool mIsForceTargetDOFPose {false};
        pivot_control_messages::DOFPose mForceTargetDOFPose;

        std::unique_ptr<pivot_control_messages::DOFPose>
            mLaparoscopeDOFPoseCur {nullptr};
        std::unique_ptr<pivot_control_messages::DOFBoundaries>
            mLaparoscopeDOFBoundaries {nullptr};
    public:
        explicit PivotControllerROSNode(ros::NodeHandle &nodeHandle);
        ~PivotControllerROSNode();

        bool initialize() override;
        void deinitialize() override;

        bool getQuit() override;

        bool setTargetDOFPose(
                pivot_control_messages::DOFPose) override;
        void laparoscopeDOFPoseCallback(
                const pivot_control_messages_ros::LaparoscopeDOFPose &laparoscopePose);
        bool getCurrentDOFPose(
                pivot_control_messages::DOFPose &laparoscopeDofPose) override;
        void laparoscopeDOFBoundariesCallback(
                const pivot_control_messages_ros::LaparoscopeDOFBoundaries &laparoscopeDOFBoundaries);
        bool getDOFBoundaries(
                pivot_control_messages::DOFBoundaries &laparoscopeDofBoundaries) override;

        bool forceSetDofPose(
                pivot_control_messages_ros::SetPose::Request&,
                pivot_control_messages_ros::SetPose::Response&);
        void forceSetDofPosePublish(const ros::TimerEvent&);
    };
}
