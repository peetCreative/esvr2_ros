#include "VideoLoaderROSNode.h"

namespace esvr2_ros {
    RosInputType getRosInputType(const std::string &input_str)
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

    VideoLoaderROSNode::VideoLoaderROSNode(ros::NodeHandle &nh,
                                           esvr2::Distortion distortion,
                                           bool stereo,
                                           RosInputType rosInputType):
            VideoLoader( distortion, stereo ),
            mNh( nh, "video_loader" ),
            mRosInputType(rosInputType),
            mRosTopicNameRaw( "image_raw" ),
            mRosTopicNameUndist( "image_undist" ),
            mRosTopicNameUndistRect( "image_undist_rect" ),
            mIsCameraInfoInit{ false, false }
    {
        mRosNamespace = mNh.getNamespace();
    }

    VideoLoaderROSNode::~VideoLoaderROSNode() = default;

    bool VideoLoaderROSNode::initialize(void)
    {
        std::string topic;
        switch (mRosInputType)
        {
            case RIT_NONE:
                mQuit = true;
                return false;
            case RIT_MONO:
                ROS_INFO_STREAM_NAMED("esvr2_ros", "RIT_MONO");
                topic = mRosNamespace + "image_raw";
                ROS_INFO_STREAM_NAMED("esvr2_ros", "Subscribe to " << topic);
                mSubImage = mNh.subscribe(
                        topic, 1,
                        &VideoLoaderROSNode::newROSImageMono, this);
                break;
            case RIT_STEREO_SLICED:
                ROS_INFO_STREAM_NAMED("esvr2_ros", "RIT_STEREO_SLICED");
                topic = mRosNamespace + "/image";
                ROS_INFO_STREAM_NAMED("esvr2_ros", "Subscribe to " << topic);
                mSubImage = mNh.subscribe(
                        topic, 1,
                        &VideoLoaderROSNode::newROSImageStereoSliced, this);
                break;
            case RIT_STEREO_SPLIT_RAW:
                ROS_INFO_STREAM_NAMED("esvr2_ros", "RIT_STEREO_SPLIT_RAW");
                topic = mRosNamespace + "/left/" + mRosTopicNameRaw;
                ROS_INFO_STREAM_NAMED("esvr2_ros", "Subscribe to " << topic);
                mSubImageLeftRaw.subscribe(mNh, topic, 20);
                topic = mRosNamespace + "/right/" + mRosTopicNameRaw;
                ROS_INFO_STREAM_NAMED("esvr2_ros", "Subscribe to " << topic);
                mSubImageRightRaw.subscribe(mNh, topic, 20);
                mApproximateSyncRaw.connectInput(
                                mSubImageLeftRaw, mSubImageRightRaw);
                mApproximateSyncRaw.registerCallback(
                        boost::bind(&VideoLoaderROSNode::newROSImageCallback, this, _1, _2));
                break;
            case RIT_STEREO_SPLIT:
                ROS_INFO_STREAM_NAMED("esvr2_ros", "RIT_STEREO_SPLIT");
                if(!mRosTopicNameRaw.empty())
                {
                    topic = mRosNamespace + "/left/" + mRosTopicNameRaw;
                    ROS_INFO_STREAM_NAMED("esvr2_ros", "Subscribe to " << topic);
                    mSubImageLeftRaw.subscribe(mNh, topic, 20);
                    topic = mRosNamespace + "/right/" + mRosTopicNameRaw;
                    ROS_INFO_STREAM_NAMED("esvr2_ros", "Subscribe to " << topic);
                    mSubImageRightRaw.subscribe(mNh, topic, 20);
                    mApproximateSyncRaw.connectInput(
                                    mSubImageLeftRaw, mSubImageRightRaw);
                    mApproximateSyncRaw.registerCallback(
                            boost::bind(&VideoLoaderROSNode::newROSImageCallback, this, _1, _2));
                }
                if(!mRosTopicNameUndist.empty())
                {
                    topic = mRosNamespace + "/left/" + mRosTopicNameUndist;
                    ROS_INFO_STREAM_NAMED("esvr2_ros", "Subscribe to " << topic);
                    mSubImageLeftUndist.subscribe(mNh, topic, 20);
                    topic = mRosNamespace + "/right/" + mRosTopicNameUndist;
                    ROS_INFO_STREAM_NAMED("esvr2_ros", "Subscribe to " << topic);
                    mSubImageRightUndist.subscribe(mNh, topic, 20);
                    mApproximateSyncUndist.connectInput(
                            mSubImageLeftUndist, mSubImageRightUndist);
                    mApproximateSyncUndist.registerCallback(
                            boost::bind(&VideoLoaderROSNode::newROSImageCallback, this, _1, _2));
                }
                if(!mRosTopicNameUndistRect.empty())
                {
                    topic = mRosNamespace + "/left/" + mRosTopicNameUndistRect;
                    ROS_INFO_STREAM_NAMED("esvr2_ros", "Subscribe to " << topic);
                    mSubImageLeftUndistRect.subscribe(mNh, topic, 20);
                    topic = mRosNamespace + "/right/" + mRosTopicNameUndistRect;
                    ROS_INFO_STREAM_NAMED("esvr2_ros", "Subscribe to " << topic);
                    mSubImageRightUndistRect.subscribe(mNh, topic, 20);
                    mApproximateSyncUndistRect.connectInput(
                            mSubImageLeftUndistRect, mSubImageRightUndistRect);
                    mApproximateSyncUndistRect.registerCallback(
                            boost::bind(&VideoLoaderROSNode::newROSImageCallback, this, _1, _2));
                }
        }

        setDistortion( mDistortion );

        if (!mIsCameraInfoInit[LEFT] && !mIsCameraInfoInit[RIGHT])
        {
            topic = mRosNamespace + "/left/camera_info";
            ROS_INFO_STREAM_NAMED("esvr2_ros", "Subscribe to " << topic);
            mSubCamInfoLeft = mNh.subscribe(
                    topic, 1,
                    &VideoLoaderROSNode::newROSCameraInfoCallback<LEFT>, this);
            topic = mRosNamespace + "/right/camera_info";
            ROS_INFO_STREAM_NAMED("esvr2_ros", "Subscribe to " << topic);
            mSubCamInfoRight = mNh.subscribe(
                    topic, 1,
                    &VideoLoaderROSNode::newROSCameraInfoCallback<RIGHT>, this);
//             if( mSubCamInfoLeft.getNumPublishers() == 0 ||
//                 mSubCamInfoRight.getNumPublishers() == 0 )
//             {
//                 return false;
//             }
        }
        else
        {
            //TODO: rethink what we are doing here
            updateDestinationSize(
                    mCameraConfig.leftCameraConfig.width,
                    mCameraConfig.leftCameraConfig.height, 4u,
                    mCameraConfig.leftCameraConfig.width* mCameraConfig.leftCameraConfig.height* 4u );
            updateMaps();
            mReady = true;
        }

        return true;
    }

    void VideoLoaderROSNode::setDistortion(esvr2::Distortion distortion )
    {
        mDistortion = distortion;
        if (mRosInputType == RIT_STEREO_SPLIT)
        {
            if(mDistortion == esvr2::DIST_RAW)
            {
                ROS_INFO_STREAM_NAMED("esvr2_ros", "Image set to DIST_RAW");
                mSubImageLeftRaw.subscribe();
                mSubImageRightRaw.subscribe();
            }
            else
            {
                mSubImageLeftRaw.unsubscribe();
                mSubImageRightRaw.unsubscribe();
            }
            if(mDistortion == esvr2::DIST_UNDISTORT)
            {
                ROS_INFO_STREAM_NAMED("esvr2_ros", "Image set to DIST_UNDISTORT");
                mSubImageLeftUndist.subscribe();
                mSubImageRightUndist.subscribe();
            }
            else
            {
                mSubImageLeftUndist.unsubscribe();
                mSubImageRightUndist.unsubscribe();
            }
            if(mDistortion == esvr2::DIST_UNDISTORT_RECTIFY)
            {
                ROS_INFO_STREAM_NAMED("esvr2_ros", "Image set to DIST_UNDISTORT_RECTIFY");
                mSubImageLeftUndistRect.subscribe();
                mSubImageRightUndistRect.subscribe();
            }
            else
            {
                mSubImageLeftUndistRect.unsubscribe();
                mSubImageRightUndistRect.unsubscribe();
            }
        }
    }

    void VideoLoaderROSNode::newROSImageStereoSliced(
            const sensor_msgs::Image::ConstPtr& imgRaw)
    {
        mSeq = imgRaw->header.seq;
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(
                    imgRaw,  imgRaw->encoding);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_WARN_STREAM_NAMED("esvr2_ros", "cv_bridge exception:" << e.what());
            return;
        }
        setImageDataFromSplitSliced(&(cv_ptr->image));

    }

    void VideoLoaderROSNode::newROSImageMono(
            const sensor_msgs::Image::ConstPtr& imgRaw)
    {
        mSeq = imgRaw->header.seq;
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(
                    imgRaw, imgRaw->encoding );
            cv::Mat image = cv_ptr->image.clone();
            setImageDataFromRaw( &image, nullptr );
        }
        catch (cv_bridge::Exception& e)
        {
            quit();
            ROS_WARN_STREAM_NAMED("esvr2_ros", "cv_bridge exception:" << e.what());
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

    void VideoLoaderROSNode::newROSImageCallback(
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
                    imgLeft, imgLeft->encoding );
            cv_ptr_right = cv_bridge::toCvShare(
                    imgRight, imgRight->encoding );
            if(imgLeft->encoding == "rgb8")
                mColorConversion = cv::COLOR_RGB2BGRA;
            if(imgLeft->encoding == "bgr8")
                mColorConversion = cv::COLOR_BGR2BGRA;
            if(imgLeft->encoding == "rgba8")
                mColorConversion = cv::COLOR_RGBA2BGRA;
            if(imgLeft->encoding == "bgra8")
                mColorConversion = cv::COLOR_COLORCVT_MAX;
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
    void VideoLoaderROSNode::newROSCameraInfoCallback(
            const sensor_msgs::CameraInfo::ConstPtr& camInfo )
    {
        ROS_INFO_STREAM_NAMED("esvr2_ros", "camera_info " << eye);
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

    void VideoLoaderROSNode::setStereoCameraConfig (
            esvr2::StereoCameraConfig stereoCameraConfig)
    {
        mCameraConfig = stereoCameraConfig;
        updateDestinationSize(
                mCameraConfig.leftCameraConfig.width,
                mCameraConfig.leftCameraConfig.height, 4u,
                mCameraConfig.leftCameraConfig.width* mCameraConfig.leftCameraConfig.height* 4u );
        updateMaps();
        mSubCamInfoLeft.shutdown();
        mSubCamInfoRight.shutdown();
        mIsCameraInfoInit[LEFT] = true;
        mIsCameraInfoInit[RIGHT] = true;
        mReady = true;
    }

    void VideoLoaderROSNode::deinitialize()
    {
        ros::shutdown();
    }

    bool VideoLoaderROSNode::getQuit()
    {
        return !mNh.ok() || mQuit;
    }
}
