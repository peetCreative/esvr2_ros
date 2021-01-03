#ifndef _Esvr2_ROSNode_H_
#define _Esvr2_ROSNode_H_

#include "Esvr2.h"
#include "Esvr2VideoLoader.h"
//#include "Esvr2PoseState.h"
#include "Esvr2LaparoscopeController.h"

#include "mediassist3_panda_pivoting/LaparoscopeDOFPose.h"
#include "mediassist3_panda_pivoting/LaparoscopeDOFBoundaries.h"

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
        ros::NodeHandle *mNh;
        ros::Subscriber mSubImage;
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageLeftRaw;
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageRightRaw;
        std::shared_ptr<ApproximateSync> mApproximateSyncRaw;
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageLeftUndist;
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageRightUndist;
        std::shared_ptr<ApproximateSync> mApproximateSyncUndist;
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageLeftUndistRect;
        message_filters::Subscriber<sensor_msgs::Image>* mSubImageRightUndistRect;
        std::shared_ptr<ApproximateSync> mApproximateSyncUndistRect;
        ros::Subscriber mSubCamInfoLeft;
        ros::Subscriber mSubCamInfoRight;
        RosInputType mRosInputType;
        std::string mRosNamespace, mRosTopicNameRaw,
                mRosTopicNameUndist, mRosTopicNameUndistRect;
        std::mutex *mCameraConfigLock;
        bool mIsCameraInfoInit[2];
        bool mSubscribePose;

        tf2_ros::Buffer *mTfBuffer;
        tf2_ros::TransformListener *mTfListener;

        bool mEnableLaparoscopeController;
        ros::Publisher mLaparoscopeCurDOFPosePub;
        ros::Subscriber mLaparoscopeBoundariesSub;
        ros::Subscriber mLaparoscopePoseSub;
        int mLaparoscopePoseSeq;
        LaparoscopeDOFPose *mLaparoscopeDOFPoseCur;
        LaparoscopeDOFBoundaries *mLaparoscopeDOFBoundaries;

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

        std::string getEsvr2ConfigFilePath();

        bool initialize(void);
        void deinitialize(void);
        void update( );

        bool getQuit();

        bool moveLaparoscopeTo(
                LaparoscopeDOFPose);
        void laparoscopeDOFPoseCallback(
                const mediassist3_panda_pivoting::LaparoscopeDOFPose &laparoscopePose);
        bool getLaparoscopePose(
                LaparoscopeDOFPose &laparoscopeDofPose);
        void laparoscopeDOFBoundariesCallback(
                const mediassist3_panda_pivoting::LaparoscopeDOFBoundaries &laparoscopeDOFBoundaries);
        bool getLaparoscopeBoundaries(
                LaparoscopeDOFBoundaries &laparoscopeDofBoundaries);

    };
}

#endif
