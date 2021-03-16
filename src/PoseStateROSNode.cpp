#include "PoseStateROSNode.h"

namespace esvr2_ros
{

    PoseStateROSNode::PoseStateROSNode(ros::NodeHandle &nh):
        mNh(nh, "pose_state"),
        mTfListener(mTfBuffer, mNh)
    {
        mReady = true;
    }

    esvr2::RealArray16 PoseStateROSNode::getPose(void)
    {
        update();
        return mPose;
    }

    esvr2::RealArray3 PoseStateROSNode::getPosition(void)
    {
        update();
        return mPosition;
    }
    esvr2::RealArray4 PoseStateROSNode::getOrientation(void)
    {
        update();
        return mOrientation;
    }


    void PoseStateROSNode::update()
    {
        geometry_msgs::TransformStamped pose;
        bool isTransform = true;
        try{
            if(!mTfBuffer._frameExists("checkerboard"))
            {
                return;
            }
            pose = mTfBuffer.lookupTransform( "checkerboard", "camera", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            isTransform = false;
        }
        if(isTransform)
        {
            esvr2::RealArray3 trans {
                    static_cast<esvr2::Real>(pose.transform.translation.x),
                    static_cast<esvr2::Real>(pose.transform.translation.y),
                    static_cast<esvr2::Real>(pose.transform.translation.z)};
            esvr2::RealArray4 orientation{
                    static_cast<esvr2::Real>(pose.transform.rotation.w),
                    static_cast<esvr2::Real>(pose.transform.rotation.x),
                    static_cast<esvr2::Real>(pose.transform.rotation.y),
                    static_cast<esvr2::Real>(pose.transform.rotation.z)};
            setPose( trans, orientation );
        }
    }
}