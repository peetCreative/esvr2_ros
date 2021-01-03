#include "Esvr2ROSNode.h"

#include "Esvr2.h"
#include "Esvr2VideoLoader.h"
#include "Esvr2ParseYml.h"

#include "mediassist3_panda_pivoting/LaparoscopeDOFPose.h"
#include "mediassist3_panda_pivoting/LaparoscopeDOFBoundaries.h"

#include "opencv2/opencv.hpp"

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <ros/package.h>
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

using namespace esvr2;

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
            mSubImageLeftRaw( nullptr ),
            mSubImageRightRaw( nullptr ),
            mSubImageLeftUndist( nullptr ),
            mSubImageRightUndist( nullptr ),
            mSubImageLeftUndistRect( nullptr ),
            mSubImageRightUndistRect( nullptr ),
            mApproximateSyncRaw( nullptr ),
            mApproximateSyncUndist( nullptr ),
            mApproximateSyncUndistRect( nullptr ),
            mRosInputType(rosInputType),
            mRosTopicNameRaw( "image_raw" ),
            mRosTopicNameUndist( "image_undist" ),
            mRosTopicNameUndistRect( "image_undist_rect" ),
            mIsCameraInfoInit{ false, false },
            mSubscribePose(true),
            mTfBuffer(nullptr),
            mTfListener(nullptr),
            mEnableLaparoscopeController(enableLaparoscopeController),
            mLaparoscopeCurDOFPosePub(),
            mLaparoscopeBoundariesSub(),
            mLaparoscopePoseSub(),
            mLaparoscopePoseSeq(0),
            mLaparoscopeDOFPoseCur(nullptr),
            mLaparoscopeDOFBoundaries(nullptr)
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

    std::string VideoROSNode::getEsvr2ConfigFilePath()
    {
        std::string url;
        mNh->param<std::string>(
                "config_file",
                url,
                "package://esvr2_ros/config/general.yml");
        if (url.substr(0,8) == "file:///")
        {
            //absolute file path
            return url.substr(7);
        }
        std::string prefix = "package://";
        size_t prefix_len = prefix.length();
        if (url.substr(0, prefix_len) == prefix)
        {
            size_t rest = url.find('/', prefix_len);
            std::string packageName = url.substr(prefix_len, rest - prefix_len);
            std::string pkgPath = ros::package::getPath(packageName);
            std::string restStr = url.substr(rest);
            if (pkgPath.empty())
            {
                return "";
            }
            else
            {
                return pkgPath + url.substr(rest);
            }
        }
        LOG << "Did not recognize package" << LOGEND;
        return "";
    }

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
            mLaparoscopeCurDOFPosePub = mNh->advertise<
                    mediassist3_panda_pivoting::LaparoscopeDOFPose>(
                            "target/laparoscope_dof_pose", 1);
            mLaparoscopeBoundariesSub = mNh->subscribe(
                            "laparoscope_dof_boundaries", 1,
                            &VideoROSNode::laparoscopeDOFBoundariesCallback, this);
            mLaparoscopePoseSub = mNh->subscribe(
                            "current/laparoscope_dof_pose", 1,
                            &VideoROSNode::laparoscopeDOFPoseCallback, this);
        }
        //TODO:This should not make da diddeerence
        else
        {
            mLaparoscopeDofBoundariesReady = true;
            mLaparoscopeDofPoseReady = true;
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

    bool VideoROSNode::moveLaparoscopeTo(
            LaparoscopeDOFPose pose)
    {
        mediassist3_panda_pivoting::LaparoscopeDOFPose poseMsg;
        poseMsg.header = std_msgs::Header();
        poseMsg.header.frame_id = "LaparoscopeTargetDOFPose";
        poseMsg.header.seq = mLaparoscopePoseSeq++;
        poseMsg.header.stamp = ros::Time::now();
        poseMsg.swing_x = pose.swingX;
        poseMsg.swing_y = pose.swingY;
        poseMsg.trans_z = pose.transZ;
        poseMsg.rot_z = pose.rotZ;
        mLaparoscopeCurDOFPosePub.publish(poseMsg);
        return true;
    }

    void VideoROSNode::laparoscopeDOFPoseCallback(
            const mediassist3_panda_pivoting::LaparoscopeDOFPose &laparoscopePose)
    {
        if (!mLaparoscopeDOFPoseCur)
        {
            mLaparoscopeDOFPoseCur = new LaparoscopeDOFPose();
        }
        mLaparoscopeDOFPoseCur->swingX = laparoscopePose.swing_x;
        mLaparoscopeDOFPoseCur->swingY = laparoscopePose.swing_y;
        mLaparoscopeDOFPoseCur->rotZ = laparoscopePose.rot_z;
        mLaparoscopeDOFPoseCur->transZ = laparoscopePose.trans_z;
        mLaparoscopeDofPoseReady = true;
    }

    //good question if we should implement this as service or as message
    //for now we do messages
    bool VideoROSNode::getLaparoscopePose(LaparoscopeDOFPose &laparoscopePose)
    {
        if (!mLaparoscopeDOFPoseCur)
            return false;
        laparoscopePose = *mLaparoscopeDOFPoseCur;
        return true;
    }

    void VideoROSNode::laparoscopeDOFBoundariesCallback(
            const mediassist3_panda_pivoting::LaparoscopeDOFBoundaries
            &laparoscopeDOFBoundaries)
    {
        if (!mLaparoscopeDOFBoundaries)
        {
            mLaparoscopeDOFBoundaries = new LaparoscopeDOFBoundaries();
        }
        mLaparoscopeDOFBoundaries->swingXMax = laparoscopeDOFBoundaries.swingXMax;
        mLaparoscopeDOFBoundaries->swingXMin = laparoscopeDOFBoundaries.swingXMin;
        mLaparoscopeDOFBoundaries->swingYMax = laparoscopeDOFBoundaries.swingYMax;
        mLaparoscopeDOFBoundaries->swingYMin = laparoscopeDOFBoundaries.swingYMin;
        mLaparoscopeDOFBoundaries->transZMax = laparoscopeDOFBoundaries.transZMax;
        mLaparoscopeDOFBoundaries->transZMin = laparoscopeDOFBoundaries.transZMin;
        mLaparoscopeDOFBoundaries->rotZMax = laparoscopeDOFBoundaries.rotZMax;
        mLaparoscopeDOFBoundaries->rotZMin = laparoscopeDOFBoundaries.rotZMin;
        mLaparoscopeDofBoundariesReady = true;
    }
    bool VideoROSNode::getLaparoscopeBoundaries(
            LaparoscopeDOFBoundaries &laparoscopeDofBoundaries)
    {
        if (!mLaparoscopeDOFBoundaries)
            return false;
        laparoscopeDofBoundaries = *mLaparoscopeDOFBoundaries;
        return true;
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



//main
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "esvr2");
    ros::NodeHandle *nh = new ros::NodeHandle();
    ros::NodeHandle *pnh = new ros::NodeHandle("~");
    std::string ritStr;
    pnh->param<std::string>("ros_input_type", ritStr, "STEREO_SPLIT");
    esvr2_ros::RosInputType rosInputType = esvr2_ros::getRosInputType(ritStr);
    bool stereo = rosInputType != esvr2_ros::RIT_MONO;
    std::string distortionStr;
    pnh->param<std::string>("distortion", distortionStr, "DIST_RAW");
    Distortion distortion = getDistortionType(distortionStr);
    bool enableLaparoscopeController = true;
    pnh->param<bool>(
            "enable_laparoscope_controller",
            enableLaparoscopeController,
            false);
    if(enableLaparoscopeController)
        ROS_INFO("Enabled Laparoscope COntroller");
    else
        ROS_INFO("DISABLED Laparoscope COntroller");


    esvr2_ros::VideoROSNode *rosNode =
            new esvr2_ros::VideoROSNode(
                    nh, distortion, stereo, rosInputType,
                    enableLaparoscopeController);
    std::shared_ptr<esvr2_ros::VideoROSNode> sharedRosNode(rosNode);
    std::shared_ptr<Esvr2Config> config = std::make_shared<Esvr2Config>();
    if (!readConfigYml(
            rosNode->getEsvr2ConfigFilePath(),
            config))
    {
        ROS_ERROR("Cannot read Configfile.");
        return 1;
    }
    rosNode->getEsvr2ConfigFilePath();
    std::shared_ptr<LaparoscopeController> laparoscopeController =
            enableLaparoscopeController ? sharedRosNode : nullptr;
    Esvr2 esvr2 = Esvr2(config, sharedRosNode,
                        laparoscopeController,nullptr);
//            Esvr2( config, rosNode, rosNode, rosNode);
    return esvr2.run();
}
