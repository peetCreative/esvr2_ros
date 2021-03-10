#ifndef _Esvr2_ROSNode_H_
#define _Esvr2_ROSNode_H_

#include "Esvr2.h"
#include "Esvr2VideoLoader.h"
//#include "Esvr2PoseState.h"
#include "Esvr2LaparoscopeController.h"
#include "Esvr2Component.h"

#include "pivot_control_messages_ros/SetPose.h"
#include "pivot_control_messages_ros/LaparoscopeDOFPose.h"
#include "pivot_control_messages_ros/LaparoscopeDOFBoundaries.h"
#include "PivotControlMessages.h"

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

#include <tf2_ros/transform_listener.h>
#include <mutex>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproximatePolicy;
typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

using namespace esvr2;

namespace esvr2_ros
{
    typedef enum {
        // if no Input type is configured ROS will not
        // provide Videoloader
        RIT_NONE,
        // subscribes only to the topic image_raw and camera_config
        RIT_MONO,
        // subscribes to the topic image_raw and camera_config
        // and splits the image by it self
        RIT_STEREO_SLICED,
        // subscribes to the topics
        // left/image_raw, left/image_undist and left/image_undist_rect
        // right/image_raw, right/image_undist and right/image_undist_rect
        // and left/camera_config and right/camera_config
        // and uses them for different "distortions"
        RIT_STEREO_SPLIT,
        // left/image_raw and right/image_raw
        // and left/camera_config and right/camera_config
        // and uses them to calculate the undistortet and undist and rect
        RIT_STEREO_SPLIT_RAW
    } RosInputType;

    static RosInputType getRosInputType(std::string input_str);

    class VideoROSNode :
            public esvr2::VideoLoader,
//            public esvr2::PoseState,
            public esvr2::LaparoscopeController
    {
    private:
        ros::NodeHandle *mNh {nullptr};
        ros::Subscriber mSubImage;
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageLeftRaw {nullptr};
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageRightRaw {nullptr};
        std::shared_ptr<ApproximateSync> mApproximateSyncRaw {nullptr};
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageLeftUndist {nullptr};
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageRightUndist {nullptr};
        std::shared_ptr<ApproximateSync> mApproximateSyncUndist {nullptr};
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageLeftUndistRect {nullptr};
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageRightUndistRect {nullptr};
        std::shared_ptr<ApproximateSync> mApproximateSyncUndistRect {nullptr};
        ros::Subscriber mSubCamInfoLeft;
        ros::Subscriber mSubCamInfoRight;
        RosInputType mRosInputType;
        std::string mRosNamespace, mRosTopicNameRaw,
                mRosTopicNameUndist, mRosTopicNameUndistRect;
        std::mutex *mCameraConfigLock;
        bool mIsCameraInfoInit[2];
        bool mSubscribePose;

        tf2_ros::Buffer *mTfBuffer {nullptr};
        tf2_ros::TransformListener *mTfListener {nullptr};

        bool mEnableLaparoscopeController;
        ros::Publisher mLaparoscopeTargetDOFPosePub;
        ros::Subscriber mLaparoscopeBoundariesSub;
        ros::Subscriber mLaparoscopeCurDOFPoseSub;
        int mLaparoscopePoseSeq {0};
        ros::ServiceServer mForceSetDofPoseService;
        bool mIsForceTargetDOFPose {false};
        pivot_control_messages::DOFPose mForceTargetDOFPose;

        pivot_control_messages::DOFPose *mLaparoscopeDOFPoseCur {nullptr};
        pivot_control_messages::DOFBoundaries *mLaparoscopeDOFBoundaries {nullptr};

        void newROSCameraInfoCallback();
    public:
        VideoROSNode(ros::NodeHandle* nodeHandle,
                     Distortion distortion,
                     bool stereo,
                     RosInputType rosInputType,
                     bool enableLaparoscopeController);
        ~VideoROSNode();

        void setDistortion( Distortion distortion );

        void newROSPose(const geometry_msgs::TransformStamped pose);
        void newROSImageStereoSliced( const sensor_msgs::Image::ConstPtr& img );
        void newROSImageMono( const sensor_msgs::Image::ConstPtr& img );
        void newROSImageCallback (
                const sensor_msgs::Image::ConstPtr& imgLeft,
                const sensor_msgs::Image::ConstPtr& imgRight );
        template <int eye>
        void newROSCameraInfoCallback (
                const sensor_msgs::CameraInfo::ConstPtr& camInfo );
        void setStereoCameraConfig (
                StereoCameraConfig stereoCameraConfig);

        bool initialize(void);
        void deinitialize(void);
        void update( );

        bool getQuit();

        bool isReady() override;

        bool setTargetDOFPose(
                pivot_control_messages::DOFPose);
        void laparoscopeDOFPoseCallback(
                const pivot_control_messages_ros::LaparoscopeDOFPose &laparoscopePose);
        bool getCurrentDOFPose(
                pivot_control_messages::DOFPose &laparoscopeDofPose);
        void laparoscopeDOFBoundariesCallback(
                const pivot_control_messages_ros::LaparoscopeDOFBoundaries &laparoscopeDOFBoundaries);
        bool getDOFBoundaries(
                pivot_control_messages::DOFBoundaries &laparoscopeDofBoundaries);

        bool forceSetDofPose(
                pivot_control_messages_ros::SetPose::Request&,
                pivot_control_messages_ros::SetPose::Response&);
    };
}

#endif
