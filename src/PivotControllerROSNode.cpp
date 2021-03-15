// created by peetcreative 11.3.2021
#include "PivotControllerROSNode.h"

#include "Esvr2.h"

#include "PivotControlMessagesRos.h"
#include "pivot_control_messages_ros/SetPose.h"
#include "pivot_control_messages_ros/LaparoscopeDOFPose.h"
#include "pivot_control_messages_ros/LaparoscopeDOFBoundaries.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <message_filters/synchronizer.h>

#include <tf2_ros/transform_listener.h>

#include <mutex>

using namespace pivot_control_messages_ros;

std::vector<std::string> getEsvr2ConfigFilePath(std::string urlstr);

namespace esvr2_ros
{
    PivotControllerROSNode::PivotControllerROSNode(ros::NodeHandle &nh):
            LaparoscopeController(),
            mNh( nh , "pivot_controller" ),
            mLaparoscopeTargetDOFPosePub(),
            mLaparoscopeBoundariesSub(),
            mLaparoscopeCurDOFPoseSub()
    {}

    PivotControllerROSNode::~PivotControllerROSNode()
    = default;

    bool PivotControllerROSNode::initialize()
    {
        //Service force
        if(!ros::service::exists("force_set_dof_pose", true))
            mForceSetDofPoseService = mNh.advertiseService(
                    "force_set_dof_pose",
                    &PivotControllerROSNode::forceSetDofPose, this);
        LOG << "Register Force New Pose Service" << LOGEND;

        mLaparoscopeTargetDOFPosePub = mNh.advertise<
                pivot_control_messages_ros::LaparoscopeDOFPose>(
                "target/laparoscope_dof_pose", 1);
        mLaparoscopeBoundariesSub = mNh.subscribe(
                "laparoscope_dof_boundaries", 1,
                &PivotControllerROSNode::laparoscopeDOFBoundariesCallback, this);
        mLaparoscopeCurDOFPoseSub = mNh.subscribe(
                "current/laparoscope_dof_pose", 1,
                &PivotControllerROSNode::laparoscopeDOFPoseCallback, this);
        mDofBoundariesReady = true;
        mDofPoseReady = true;
        return true;
    }

    void PivotControllerROSNode::deinitialize()
    {
        ros::shutdown();
    }

    bool PivotControllerROSNode::getQuit()
    {
        return !mNh.ok() || mQuit;
    }

    bool PivotControllerROSNode::setTargetDOFPose(
            pivot_control_messages::DOFPose pose)
    {
        //if we are about to force a new position don't interupt
        if (mIsForceTargetDOFPose)
            return false;
        LaparoscopeDOFPose poseMsg =
                pivot_control_messages_ros::toROSDOFPose(
                        pose,
                        "LaparoscopeTargetDOFPose",
                        mLaparoscopePoseSeq++);
        mLaparoscopeTargetDOFPosePub.publish(poseMsg);
        return true;
    }

    void PivotControllerROSNode::laparoscopeDOFPoseCallback(
            const pivot_control_messages_ros::LaparoscopeDOFPose &laparoscopePose)
    {
        if (!mLaparoscopeDOFPoseCur)
        {
            mLaparoscopeDOFPoseCur =
                    std::make_unique<pivot_control_messages::DOFPose>();
        }
        if (mIsForceTargetDOFPose &&
            mLaparoscopeDOFPoseCur->closeTo(mForceTargetDOFPose, 0.01, 0.005))
        {
            //We reached a good point
            mIsForceTargetDOFPose = false;
        }
        *mLaparoscopeDOFPoseCur = pivot_control_messages_ros::toDOFPose(laparoscopePose);
        mDofPoseReady = true;
    }

    bool PivotControllerROSNode::getCurrentDOFPose(
            pivot_control_messages::DOFPose &laparoscopePose)
    {
        if (!mLaparoscopeDOFPoseCur)
            return false;
        laparoscopePose = *mLaparoscopeDOFPoseCur;
        return true;
    }

    void PivotControllerROSNode::laparoscopeDOFBoundariesCallback(
            const pivot_control_messages_ros::LaparoscopeDOFBoundaries
            &laparoscopeDOFBoundaries)
    {
        if (!mLaparoscopeDOFBoundaries)
        {
            mLaparoscopeDOFBoundaries =
                    std::make_unique<pivot_control_messages::DOFBoundaries>();
        }
        *mLaparoscopeDOFBoundaries =
                pivot_control_messages_ros::toDOFBoundaries(laparoscopeDOFBoundaries);
        mDofBoundariesReady = true;
    }
    bool PivotControllerROSNode::getDOFBoundaries(
            pivot_control_messages::DOFBoundaries &laparoscopeDofBoundaries)
    {
        if (!mLaparoscopeDOFBoundaries)
            return false;
        laparoscopeDofBoundaries = *mLaparoscopeDOFBoundaries;
        return true;
    }

    bool PivotControllerROSNode::forceSetDofPose(
            SetPose::Request& req, SetPose::Response& resp)
    {
        //TODO: at least close
        LaparoscopeDOFPose forceTargetDOFPoseMsg = toROSDOFPose(
                req, "LaparoscopeTargetDOFPose", mLaparoscopePoseSeq++);
        mLaparoscopeTargetDOFPosePub.publish(forceTargetDOFPoseMsg);
        mForceTargetDOFPose = toDOFPose(req);
        mIsForceTargetDOFPose = true;
        resp.success = true;
        return true;
    }
}
