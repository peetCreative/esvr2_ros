#include "Esvr2ROSNode.h"

#include "Esvr2.h"
#include "Esvr2VideoLoader.h"
#include "Esvr2ParseYml.h"

#include "PivotControlMessagesRos.h"
#include "pivot_control_messages_ros/SetPose.h"
#include "pivot_control_messages_ros/LaparoscopeDOFPose.h"
#include "pivot_control_messages_ros/LaparoscopeDOFBoundaries.h"

#include "opencv2/opencv.hpp"

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "ros/advertise_service_options.h"
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
#include <memory>

using namespace esvr2;
using namespace pivot_control_messages_ros;

std::vector<std::string> getEsvr2ConfigFilePath(std::string urlstr);

namespace esvr2_ros
{
    RosInputType getRosInputType(std::string input_str)
    {
        RosInputType input = RIT_NONE;
        if (input_str == "MONO")
            input = RIT_MONO;
        if (input_str == "STEREO_SLICED")
            input = RIT_STEREO_SLICED;
        if (input_str == "STEREO_SPLIT")
            input = RIT_STEREO_SPLIT;
        if (input_str == "STEREO_SPLIT_RAW")
            input = RIT_STEREO_SPLIT_RAW;
        return input;
    }

    VideoROSNode::VideoROSNode( ros::NodeHandle *nh,
                                Distortion distortion,
                                bool stereo,
                                RosInputType rosInputType,
                                bool enableLaparoscopeController):
            VideoLoader( distortion, stereo ),
//            PoseState(),
            LaparoscopeController(),
            mNh( nh ),
            mRosInputType(rosInputType),
            mRosTopicNameRaw( "image_raw" ),
            mRosTopicNameUndist( "image_undist" ),
            mRosTopicNameUndistRect( "image_undist_rect" ),
            mIsCameraInfoInit{ false, false },
            mSubscribePose(true),
            mEnableLaparoscopeController(enableLaparoscopeController),
            mLaparoscopeTargetDOFPosePub(),
            mLaparoscopeBoundariesSub(),
            mLaparoscopeCurDOFPoseSub()
    {
        mRosNamespace = mNh->getNamespace();
//        if (cameraConfig)
//        {
//            mCameraConfig = *cameraConfig;
//            mIsCameraInfoInit[LEFT] = true;
//            mIsCameraInfoInit[RIGHT] = true;
//        }
    }

    VideoROSNode::~VideoROSNode() {}

    bool VideoROSNode::initialize(void)
    {
        std::string topic;
        switch (mRosInputType)
        {
            case RIT_NONE:
                mQuit = true;
                return false;
            case RIT_MONO:
                LOG << "RIT_MONO" << LOGEND;
                topic = mRosNamespace + "/image_raw";
                LOG << "Subscribe to " << topic << LOGEND;
                mSubImage = mNh->subscribe(
                        topic, 1,
                        &VideoROSNode::newROSImageMono, this);
                break;
            case RIT_STEREO_SLICED:
                LOG << "RIT_STEREO_SLICED" << LOGEND;
                topic = mRosNamespace + "/image";
                LOG << "Subscribe to " << topic << LOGEND;
                mSubImage = mNh->subscribe(
                        topic, 1,
                        &VideoROSNode::newROSImageStereoSliced, this);
                break;
            case RIT_STEREO_SPLIT_RAW:
                LOG << "RIT_STEREO_SPLIT_RAW" << LOGEND;
                topic = mRosNamespace + "/left/" + mRosTopicNameRaw;
                LOG << "Subscribe to " << topic << LOGEND;
                mSubImageLeftRaw= new
                        message_filters::Subscriber<sensor_msgs::Image> (
                        *mNh, topic, 20);
                topic = mRosNamespace + "/right/" + mRosTopicNameRaw;
                LOG << "Subscribe to " << topic << LOGEND;
                mSubImageRightRaw = new
                        message_filters::Subscriber<sensor_msgs::Image> (
                        *mNh, topic, 20);
                mApproximateSyncRaw.reset(
                        new ApproximateSync(
                                ApproximatePolicy(20),
                                *mSubImageLeftRaw, *mSubImageRightRaw));
                mApproximateSyncRaw->registerCallback(
                        boost::bind( &VideoROSNode::newROSImageCallback, this,_1, _2));
                break;
            case RIT_STEREO_SPLIT:
                LOG << "RIT_STEREO_SPLIT" << LOGEND;
                if(mRosTopicNameRaw.compare("") != 0)
                {
                    topic = mRosNamespace + "/left/" + mRosTopicNameRaw;
                    LOG << "Subscribe to " << topic << LOGEND;
                    mSubImageLeftRaw = new
                            message_filters::Subscriber<sensor_msgs::Image> (
                            *mNh, topic, 20);
                    topic = mRosNamespace + "/right/" + mRosTopicNameRaw;
                    LOG << "Subscribe to " << topic << LOGEND;
                    mSubImageRightRaw = new
                            message_filters::Subscriber<sensor_msgs::Image> (
                            *mNh, topic, 20);
                    mApproximateSyncRaw.reset(
                            new ApproximateSync(
                                    ApproximatePolicy(20),
                                    *mSubImageLeftRaw, *mSubImageRightRaw));
                    mApproximateSyncRaw->registerCallback(
                            boost::bind( &VideoROSNode::newROSImageCallback, this,_1, _2));
                }
                if(mRosTopicNameUndist.compare("") != 0)
                {
                    topic = mRosNamespace + "/left/" + mRosTopicNameUndist;
                    LOG << "Subscribe to " << topic << LOGEND;
                    mSubImageLeftUndist = new
                            message_filters::Subscriber<sensor_msgs::Image> (
                            *mNh, topic, 20);
                    topic = mRosNamespace + "/right/" + mRosTopicNameUndist;
                    LOG << "Subscribe to " << topic << LOGEND;
                    mSubImageRightUndist = new
                            message_filters::Subscriber<sensor_msgs::Image> (
                            *mNh, topic, 20);
                    mApproximateSyncUndist.reset(
                            new ApproximateSync(
                                    ApproximatePolicy(20),
                                    *mSubImageLeftUndist, *mSubImageRightUndist));
                    mApproximateSyncUndist->registerCallback(
                            boost::bind( &VideoROSNode::newROSImageCallback, this,_1, _2));
                }
                if(mRosTopicNameUndistRect.compare("") != 0)
                {
                    topic = mRosNamespace + "/left/" + mRosTopicNameUndistRect;
                    LOG << "Subscribe to " << topic << LOGEND;
                    mSubImageLeftUndistRect = new
                            message_filters::Subscriber<sensor_msgs::Image> (
                            *mNh, topic, 20);
                    topic = mRosNamespace + "/right/" + mRosTopicNameUndistRect;
                    LOG << "Subscribe to " << topic << LOGEND;
                    mSubImageRightUndistRect = new
                            message_filters::Subscriber<sensor_msgs::Image> (
                            *mNh, topic, 20);
                    mApproximateSyncUndistRect.reset(
                            new ApproximateSync(
                                    ApproximatePolicy(20),
                                    *mSubImageLeftUndistRect, *mSubImageRightUndistRect));
                    mApproximateSyncUndistRect->registerCallback(
                            boost::bind( &VideoROSNode::newROSImageCallback, this,_1, _2));
                }
        }

        //Service force
        if(!ros::service::exists("force_set_dof_pose", true))
            mForceSetDofPoseService = mNh->advertiseService(
                    "force_set_dof_pose",
                    &VideoROSNode::forceSetDofPose, this);
        LOG << "Register Force New Pose Service" << LOGEND;

        setDistortion( mDistortion );

        if (!mIsCameraInfoInit[LEFT] && !mIsCameraInfoInit[RIGHT])
        {
            topic = mRosNamespace + "/left/camera_info";
            LOG << "Subscribe to " << topic << LOGEND;
            mSubCamInfoLeft = mNh->subscribe(
                    topic, 1,
                    &VideoROSNode::newROSCameraInfoCallback<LEFT>, this);
            topic = mRosNamespace + "/right/camera_info";
            LOG << "Subscribe to " << topic << LOGEND;
            mSubCamInfoRight = mNh->subscribe(
                    topic, 1,
                    &VideoROSNode::newROSCameraInfoCallback<RIGHT>, this);
//             if( mSubCamInfoLeft.getNumPublishers() == 0 ||
//                 mSubCamInfoRight.getNumPublishers() == 0 )
//             {
//                 LOG << "no Publisher for camera_info" << LOGEND;
//                 return false;
//             }
        }
        else
        {
            updateDestinationSize(
                    mCameraConfig.leftCameraConfig.width,
                    mCameraConfig.leftCameraConfig.height, 4u,
                    mCameraConfig.leftCameraConfig.width* mCameraConfig.leftCameraConfig.height* 4u );
            updateMaps();
            mReady = true;
        }

        if(mSubscribePose)
        {
            mTfBuffer = new tf2_ros::Buffer();
            mTfListener = new tf2_ros::TransformListener(*mTfBuffer, mNh);
//             topic = "/tf";
//             LOG << "Subscribe to " << topic << LOGEND;
        }

        if(mEnableLaparoscopeController)
        {
            mLaparoscopeTargetDOFPosePub = mNh->advertise<
                    pivot_control_messages_ros::LaparoscopeDOFPose>(
                            "target/laparoscope_dof_pose", 1);
            mLaparoscopeBoundariesSub = mNh->subscribe(
                            "laparoscope_dof_boundaries", 1,
                            &VideoROSNode::laparoscopeDOFBoundariesCallback, this);
            mLaparoscopeCurDOFPoseSub = mNh->subscribe(
                            "current/laparoscope_dof_pose", 1,
                            &VideoROSNode::laparoscopeDOFPoseCallback, this);
        }
        //TODO:This should not make da diddeerence
        else
        {
            mDofBoundariesReady = true;
            mDofPoseReady = true;
        }
        return true;
    }

    void VideoROSNode::setDistortion( Distortion distortion )
    {
        mDistortion = distortion;
        if (mRosInputType == RIT_STEREO_SPLIT)
        {
            if (mSubImageLeftRaw && mSubImageRightRaw)
            {
                if(mDistortion == DIST_RAW)
                {
                    LOG << "Image set to DIST_RAW" << LOGEND;
                    mSubImageLeftRaw->subscribe();
                    mSubImageRightRaw->subscribe();
                }
                else
                {
                    mSubImageLeftRaw->unsubscribe();
                    mSubImageRightRaw->unsubscribe();
                }
            }
            if ( mSubImageLeftUndist && mSubImageRightUndist )
            {
                if(mDistortion == DIST_UNDISTORT)
                {
                    LOG << "Image set to DIST_UNDISTORT" << LOGEND;
                    mSubImageLeftUndist->subscribe();
                    mSubImageRightUndist->subscribe();
                }
                else
                {
                    mSubImageLeftUndist->unsubscribe();
                    mSubImageRightUndist->unsubscribe();
                }
            }
            if ( mSubImageLeftUndistRect && mSubImageRightUndistRect )
            {
                if(mDistortion == DIST_UNDISTORT_RECTIFY)
                {
                    LOG << "Image set to DIST_UNDISTORT_RECTIFY" << LOGEND;
                    mSubImageLeftUndistRect->subscribe();
                    mSubImageRightUndistRect->subscribe();
                }
                else
                {
                    mSubImageLeftUndistRect->unsubscribe();
                    mSubImageRightUndistRect->unsubscribe();
                }
            }
        }
    }

//    void VideoROSNode::newROSPose(
//            const geometry_msgs::TransformStamped pose)
//    {
//        LOG << "getNewPose" << LOGEND;
//        Ogre::Vector3 trans(
//                pose.transform.translation.x, pose.transform.translation.y, pose.transform.translation.z);
//        Ogre::Quaternion rotation(
//                pose.transform.rotation.x, pose.transform.rotation.y,
//                pose.transform.rotation.z, pose.transform.rotation.w);
//        setPose( trans, rotation );
//    }

    void VideoROSNode::newROSImageStereoSliced(
            const sensor_msgs::Image::ConstPtr& imgRaw)
    {
        mSeq = imgRaw->header.seq;
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(
                    imgRaw, sensor_msgs::image_encodings::BGR8 );
        }
        catch (cv_bridge::Exception& e)
        {
            LOG << "cv_bridge exception:" << e.what() << LOGEND;
            return;
        }
        setImageDataFromSplitSliced(&(cv_ptr->image));

    }

    void VideoROSNode::newROSImageMono(
            const sensor_msgs::Image::ConstPtr& imgRaw)
    {
        mSeq = imgRaw->header.seq;
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(
                    imgRaw, sensor_msgs::image_encodings::BGR8 );
            cv::Mat image = cv_ptr->image.clone();
            setImageDataFromRaw( &image, nullptr );
        }
        catch (cv_bridge::Exception& e)
        {
            quit();
            std::cout <<"cv_bridge exception: " << e.what() << std::endl;
            return;
        }
//        catch( Ogre::Exception &e )
//        {
//            quit();
//            std::cout << "ROS oh sth went wront with OGRE!!" << std::endl;
//            //TODO: let's unregister this as well
//            throw e;
//        }
        catch( ... )
        {
            quit();
//             destroySystems( graphicsGameState, graphicsSystem );
        }
    }

    void VideoROSNode::newROSImageCallback(
            const sensor_msgs::Image::ConstPtr& imgLeft,
            const sensor_msgs::Image::ConstPtr& imgRight )
    {
        if (!mIsCameraInfoInit[LEFT] || !mIsCameraInfoInit[RIGHT])
            return;
        mSeq = imgLeft->header.seq;
        cv_bridge::CvImageConstPtr cv_ptr_left;
        cv_bridge::CvImageConstPtr cv_ptr_right;
        try
        {
            //TRY BGRA8
            cv_ptr_left = cv_bridge::toCvShare(
                    imgLeft, sensor_msgs::image_encodings::BGR8 );
            cv_ptr_right = cv_bridge::toCvShare(
                    imgRight, sensor_msgs::image_encodings::BGR8 );
            //TODO:copy to new mat probably inefficient
            cv::Mat left = cv_ptr_left->image.clone();
            cv::Mat right = cv_ptr_right->image.clone();
            if ( mRosInputType == RIT_STEREO_SPLIT_RAW )
            {
                setImageDataFromRaw(
                        &(left), &(right));
            }
            else if ( mRosInputType == RIT_STEREO_SPLIT )
            {
                setImageData(&(left), &(right));
            }
        }
        catch (cv_bridge::Exception& e)
        {
            std::cout <<"cv_bridge exception: " << e.what() << std::endl;
            return;
        }
        catch( ... )
        {
//             destroySystems( graphicsGameState, graphicsSystem );
        }
    }

    template<int eye>
    void VideoROSNode::newROSCameraInfoCallback(
            const sensor_msgs::CameraInfo::ConstPtr& camInfo )
    {
        LOG << "camera_info " << eye << LOGEND;
        if ( !mIsCameraInfoInit[eye] )
        {
            mCameraConfig.cfg[eye]->eye_str = eye == LEFT ? "left": "right";
            mCameraConfig.cfg[eye]->width = camInfo->width;
            mCameraConfig.cfg[eye]->height = camInfo->height;
            for( size_t i = 0; i < 9; i++ )
                mCameraConfig.cfg[eye]->K[i] = camInfo->K[i];
            for( size_t i = 0; i < 12; i++ )
                mCameraConfig.cfg[eye]->P[i] = camInfo->P[i];
            for( size_t i = 0; i < 5; i++ )
                mCameraConfig.cfg[eye]->D[i] = camInfo->D[i];
            for( size_t i = 0; i < 9; i++ )
                mCameraConfig.cfg[eye]->R[i] = camInfo->R[i];
            mIsCameraInfoInit[eye] = true;
        }
        if (eye == LEFT)
        {
            mSubCamInfoLeft.shutdown();
        }
        else if (eye == RIGHT)
        {
            mSubCamInfoRight.shutdown();
        }
        if ( mIsCameraInfoInit[LEFT] && mIsCameraInfoInit[RIGHT] )
        {
            updateDestinationSize(
                    mCameraConfig.leftCameraConfig.width,
                    mCameraConfig.leftCameraConfig.height, 4u,
                    mCameraConfig.leftCameraConfig.width* mCameraConfig.leftCameraConfig.height* 4u );
            updateMaps();
            mReady = true;
        }
    }

    void VideoROSNode::deinitialize(void)
    {
        //TODO: delete all the other things
        ros::shutdown();
    }

    void VideoROSNode::update( )
    {
        ros::spinOnce();
        geometry_msgs::TransformStamped pose;
        bool isTransform = true;
        try{
            if(!mTfBuffer->_frameExists("checkerboard"))
            {
//                 LOG << "no frame named checkerboard" << LOGEND;
                return;
            }
            pose = mTfBuffer->lookupTransform( "checkerboard", "camera", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            isTransform = false;
        }
//        TODO: capture trans
//        if(isTransform)
//        {
//            Ogre::Vector3 trans(
//                    pose.transform.translation.x, pose.transform.translation.y, pose.transform.translation.z);
//            Ogre::Quaternion orientation(
//                    pose.transform.rotation.w,
//                    pose.transform.rotation.x,
//                    pose.transform.rotation.y,
//                    pose.transform.rotation.z);
//            if ( !mOrientation.orientationEquals(orientation)
//                 || mPosition != trans )
//            {
////                 LOG << "ROS update orientation and position" << pose.header.stamp.sec << " " << pose.header.stamp.nsec;
////                 LOG << " Pos: " << trans.x << " " << trans.y << " " <<trans.z;
////                 LOG << " Orientation: " << orientation.w << " "<< orientation.x << " " << orientation.y << " " <<orientation.z<<LOGEND;
//                setPose( trans, orientation );
//            }
//        }
    }

    bool VideoROSNode::getQuit()
    {
        return !mNh->ok() || mQuit;
    }

    bool VideoROSNode::setTargetDOFPose(
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

    void VideoROSNode::laparoscopeDOFPoseCallback(
            const pivot_control_messages_ros::LaparoscopeDOFPose &laparoscopePose)
    {
        if (!mLaparoscopeDOFPoseCur)
        {
            mLaparoscopeDOFPoseCur =
                    new pivot_control_messages::DOFPose();
        }
        if (mIsForceTargetDOFPose &&
            mLaparoscopeDOFPoseCur->closeTo(mForceTargetDOFPose, 0.01, 0.005))
        {
            //We reached a good point
            mIsForceTargetDOFPose = false;
        }
        mLaparoscopeDOFPoseCur->yaw = laparoscopePose.yaw;
        mLaparoscopeDOFPoseCur->pitch = laparoscopePose.pitch;
        mLaparoscopeDOFPoseCur->roll = laparoscopePose.roll;
        mLaparoscopeDOFPoseCur->transZ = laparoscopePose.trans_z;
        mDofPoseReady = true;
    }

    bool VideoROSNode::getCurrentDOFPose(
            pivot_control_messages::DOFPose &laparoscopePose)
    {
        if (!mLaparoscopeDOFPoseCur)
            return false;
        laparoscopePose = *mLaparoscopeDOFPoseCur;
        return true;
    }

    void VideoROSNode::laparoscopeDOFBoundariesCallback(
            const pivot_control_messages_ros::LaparoscopeDOFBoundaries
                &laparoscopeDOFBoundaries)
    {
        if (!mLaparoscopeDOFBoundaries)
        {
            mLaparoscopeDOFBoundaries =
                    new pivot_control_messages::DOFBoundaries();
        }
        mLaparoscopeDOFBoundaries->yawMax = laparoscopeDOFBoundaries.yaw_max;
        mLaparoscopeDOFBoundaries->yawMin = laparoscopeDOFBoundaries.yaw_min;
        mLaparoscopeDOFBoundaries->pitchMax = laparoscopeDOFBoundaries.pitch_max;
        mLaparoscopeDOFBoundaries->pitchMin = laparoscopeDOFBoundaries.pitch_min;
        mLaparoscopeDOFBoundaries->transZMax = laparoscopeDOFBoundaries.trans_z_max;
        mLaparoscopeDOFBoundaries->transZMin = laparoscopeDOFBoundaries.trans_z_min;
        mLaparoscopeDOFBoundaries->rollMax = laparoscopeDOFBoundaries.roll_max;
        mLaparoscopeDOFBoundaries->rollMin = laparoscopeDOFBoundaries.roll_min;
        mDofBoundariesReady = true;
    }
    bool VideoROSNode::getDOFBoundaries(
            pivot_control_messages::DOFBoundaries &laparoscopeDofBoundaries)
    {
        if (!mLaparoscopeDOFBoundaries)
            return false;
        laparoscopeDofBoundaries = *mLaparoscopeDOFBoundaries;
        return true;
    }

    bool VideoROSNode::forceSetDofPose(
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

    bool VideoROSNode::isReady() {
        return LaparoscopeController::isReady() && VideoLoader::isReady();
    }
}


//ROS
//                if ( cfg.exists("ros"))
//                {
//                    if ( input == IT_NONE )
//                        input = IT_ROS;
//                    Setting& vs = cfg.lookup("ros");
//                    if (vs.exists("input_type"))
//                    {
//                        std::string ros_input_str;
//                        ros_input_str = vs["input_type"].c_str();
//                        rosInputType =
//                                getRosInputType(ros_input_str);
//                        isStereo = rosInputType > RIT_MONO;
//                    }
//                    if (vs.exists("namespace"))
//                    {
//                        rosNamespace = vs["namespace"].c_str();
//                    }
//                    if (vs.exists("topic_name_raw"))
//                    {
//                        rosTopicNameRaw = vs["topic_name_raw"].c_str();
//                    }
//                    if (vs.exists("topic_name_undist"))
//                    {
//                        rosTopicNameUndist = vs["topic_name_undist"].c_str();
//                    }
//                    if (vs.exists("topic_name_undist_rect"))
//                    {
//                        rosTopicNameUndistRect = vs["topic_name_undist_rect"].c_str();
//                    }
//                }

std::vector<std::string> getEsvr2ConfigFilePath(std::string urlstr)
{
    size_t start;
    size_t end = 0;
    char delim = ';';
    std::vector<std::string> urls;
    while ((start = urlstr.find_first_not_of(delim, end)) != std::string::npos) {
        end = urlstr.find(delim, start);
        std::string url = urlstr.substr(start, end - start);
        ROS_INFO("Process!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!: %s", url.c_str()    );

        if (url.substr(0, 8) == "file:///") {
            //absolute file path
            ROS_INFO("FILE: %s", url.c_str()    );
            urls.push_back(url.substr(7));
        }
        std::string prefix = "package://";
        size_t prefix_len = prefix.length();
        if (url.substr(0, prefix_len) == prefix) {
            ROS_INFO("PACKAGE: %s", url.c_str()    );
            size_t rest = url.find('/', prefix_len);
            std::string packageName = url.substr(prefix_len,
                                                 rest - prefix_len);
            std::string pkgPath = ros::package::getPath(packageName);
            std::string restStr = url.substr(rest);
            if (pkgPath.empty()) {
                continue;
            } else {
                urls.push_back(pkgPath + url.substr(rest));
            }
        }
        LOG << "Did not recognize package" << LOGEND;
        continue;
    }
    return urls;
}

//main
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "esvr2");
    ros::NodeHandle *nh = new ros::NodeHandle();
    ros::NodeHandle *pnh = new ros::NodeHandle("~");
    LOG << "global" << LOGEND;
    std::vector<std::string> paramNames;
    nh->getParamNames(paramNames);

    std::string paramName;
    std::string urlstr = "";
    if(pnh->searchParam("config_files", paramName))
    {
        if (!pnh->getParam(paramName, urlstr))
            return 1;
    }

    std::string ritStr = "STEREO_SPLIT";
    if(pnh->searchParam("ros_input_type", paramName))
    {
        if (!pnh->getParam(paramName, ritStr))
            return 1;
    }
    esvr2_ros::RosInputType rosInputType = esvr2_ros::getRosInputType(ritStr);
    bool stereo = rosInputType != esvr2_ros::RIT_MONO;

    std::string distortionStr = "DIST_RAW";
    if(pnh->searchParam("distortion", paramName))
    {
        if (!pnh->getParam(paramName, distortionStr))
            return 1;
    }
    Distortion distortion = getDistortionType(distortionStr);

    bool enableLaparoscopeController = false;
    if(pnh->searchParam("enable_laparoscope_controller", paramName))
    {
        if (!pnh->getParam(paramName, enableLaparoscopeController))
            return 1;
    }
    if(enableLaparoscopeController)
        ROS_INFO("Enabled Laparoscope COntroller");
    else
        ROS_INFO("DISABLED Laparoscope COntroller");

    std::shared_ptr<esvr2_ros::VideoROSNode> sharedRosNode =
            std::make_shared<esvr2_ros::VideoROSNode>(
                    nh, distortion, stereo, rosInputType,
                    enableLaparoscopeController);
    std::shared_ptr<Esvr2Config> config = std::make_shared<Esvr2Config>();
    std::vector<std::string> configFilePaths =
            getEsvr2ConfigFilePath(urlstr);
    for (auto it = configFilePaths.begin();
         it != configFilePaths.end(); it++)
    {
        if (!readConfigYml(
                *it,
                config))
        {
            ROS_ERROR("Cannot read Configfile.");
            return 1;
        }
    }
    config->resourcePath = RESOURCES_FILE;
    std::shared_ptr<LaparoscopeController> laparoscopeController =
            enableLaparoscopeController ? sharedRosNode : nullptr;
    Esvr2 esvr2(config, sharedRosNode, laparoscopeController,nullptr);
//            Esvr2( config, rosNode, rosNode, rosNode);
    return esvr2.run();
}
