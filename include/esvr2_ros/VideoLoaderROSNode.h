#pragma once

#include "Esvr2.h"
#include "Esvr2VideoLoader.h"
#include "Esvr2Component.h"

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproximatePolicy;
typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

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

    RosInputType getRosInputType(const std::string &input_str);

    class VideoLoaderROSNode :
            public esvr2::VideoLoader
    {
    private:
        ros::NodeHandle mNh;
        ros::Subscriber mSubImage;
        const uint32_t mQueueSize {20};
        message_filters::Subscriber<sensor_msgs::Image> mSubImageLeftRaw;
        message_filters::Subscriber<sensor_msgs::Image> mSubImageRightRaw;
        ApproximateSync mApproximateSyncRaw {ApproximatePolicy(mQueueSize)};
        message_filters::Subscriber<sensor_msgs::Image> mSubImageLeftUndist;
        message_filters::Subscriber<sensor_msgs::Image> mSubImageRightUndist;
        ApproximateSync mApproximateSyncUndist {ApproximatePolicy(mQueueSize)};
        message_filters::Subscriber<sensor_msgs::Image> mSubImageLeftUndistRect;
        message_filters::Subscriber<sensor_msgs::Image> mSubImageRightUndistRect;
        ApproximateSync mApproximateSyncUndistRect {ApproximatePolicy(mQueueSize)};
        ros::Subscriber mSubCamInfoLeft;
        ros::Subscriber mSubCamInfoRight;
        RosInputType mRosInputType;
        std::string mRosNamespace, mRosTopicNameRaw,
                mRosTopicNameUndist, mRosTopicNameUndistRect;
        bool mIsCameraInfoInit[2] {false, false};
    public:
        VideoLoaderROSNode(ros::NodeHandle &nodeHandle,
                     esvr2::Distortion distortion,
                     bool stereo,
                     RosInputType rosInputType);
        ~VideoLoaderROSNode();

        void setDistortion( esvr2::Distortion distortion ) override;

        void newROSImageStereoSliced( const sensor_msgs::Image::ConstPtr& img );
        void newROSImageMono( const sensor_msgs::Image::ConstPtr& img );
        void newROSImageCallback (
                const sensor_msgs::Image::ConstPtr& imgLeft,
                const sensor_msgs::Image::ConstPtr& imgRight );
        template <int eye>
        void newROSCameraInfoCallback (
                const sensor_msgs::CameraInfo::ConstPtr& camInfo );
        void setStereoCameraConfig (
                esvr2::StereoCameraConfig stereoCameraConfig);

        bool initialize() override;
        void deinitialize() override;

        bool getQuit() override;
    };
}
