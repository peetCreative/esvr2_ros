#include "PivotControllerROSNode.h"
#include "VideoLoaderROSNode.h"

#include "Esvr2.h"
#include "Esvr2ParseYml.h"

#include <ros/ros.h>
#include <ros/package.h>
#include "ros/timer.h"
#include <ros/time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Trigger.h>
#include <mutex>
#include <memory>

std::vector<std::string> getEsvr2ConfigFilePath(std::string urlstr)
{
    size_t start;
    size_t end = 0;
    char delim = ';';
    std::vector<std::string> urls;
    while ((start = urlstr.find_first_not_of(delim, end)) != std::string::npos) {
        end = urlstr.find(delim, start);
        std::string url = urlstr.substr(start, end - start);
        ROS_INFO("Process: %s", url.c_str()    );
        if (url.size() > 0 && url.at(0) == '/')
        {
            urls.push_back(url);
        }

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
    }
    return urls;
}

class Esvr2ROSNode
{
private:
    ros::NodeHandle mNh;
    ros::NodeHandle mPNh;
    bool mIsUserStudy {false};
    bool mQuit {false};
    ros::ServiceServer mQuitService;
    //TODO: move this to seperate file
    std::shared_ptr<esvr2::Esvr2> mEsvr2 {nullptr};
    ros::Timer mHeadPoseTimer;
    int mSeq {0};
public:
    std::shared_ptr<esvr2_ros::PivotControllerROSNode> mPivotControllerNode {nullptr};
    std::shared_ptr<esvr2_ros::VideoLoaderROSNode> mVideoLoaderNode {nullptr};
//    std::shared_ptr<esvr2_ros::PoseStateROSNode> mPoseStateNode {nullptr};
    std::shared_ptr<esvr2::Esvr2Config> mConfig {nullptr};
    std::string mParticipantId {};
    std::string mSetupId {};

    Esvr2ROSNode():
            mNh(),
            mPNh("~")
    {};

    bool quit(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& resp)
    {
        mQuit = true;
        if (mEsvr2)
            mEsvr2->quit();
        resp.success = true;
        resp.message = "quit";
        return true;
    }

    bool initialize()
    {
        LOG << "global" << LOGEND;
        std::vector<std::string> paramNames;
        mNh.getParamNames(paramNames);

        mQuitService = mNh.advertiseService("display/quit", &Esvr2ROSNode::quit, this);

        std::string paramName;
        std::string urlstr;
        if(mPNh.searchParam("config_files", paramName))
        {
            if (!mPNh.getParam(paramName, urlstr))
                return false;
        }

        std::string logFolder {};
        if(mPNh.searchParam("log_folder", paramName))
        {
            if (!mPNh.getParam(paramName, logFolder))
                return false;
        }

        if(mPNh.searchParam("is_userstudy", paramName))
        {
            if (!mPNh.getParam(paramName, mIsUserStudy))
                return false;
        }

        if(mIsUserStudy)
        {
            while((mParticipantId.empty() || mSetupId.empty()) && ros::ok() && !mQuit)
            {
                if(mPNh.searchParam("participant_id", paramName))
                {
                    if (!mPNh.getParam(paramName, mParticipantId))
                        return false;
                }

                if(mPNh.searchParam("setup_id",  paramName))
                {
                    if (!mPNh.getParam(paramName, mSetupId))
                    {
                        return false;
                    }
                }
                sleep(1);
            }
        }

        std::string ritStr = "NONE";
        if(mPNh.searchParam("ros_input_type", paramName))
        {
            if (!mPNh.getParam(paramName, ritStr))
                return false;
        }
        esvr2_ros::RosInputType rosInputType = esvr2_ros::getRosInputType(ritStr);
        bool stereo = rosInputType != esvr2_ros::RIT_MONO;

        std::string distortionStr = "DIST_RAW";
        if(mPNh.searchParam("distortion", paramName))
        {
            if (!mPNh.getParam(paramName, distortionStr))
                return false;
        }
        esvr2::Distortion distortion = esvr2::getDistortionType(distortionStr);

        if (rosInputType != esvr2_ros::RIT_NONE)
        {
            mVideoLoaderNode =
                    std::make_shared<esvr2_ros::VideoLoaderROSNode>(
                            mNh, distortion, stereo, rosInputType);
        }

        bool enablePivotController = false;
        if(mPNh.searchParam("enable_pivot_controller", paramName))
        {
            if (!mPNh.getParam(paramName, enablePivotController))
                return false;
        }
        if(enablePivotController)
        {
            ROS_INFO("Enabled Pivot Controller");
            mPivotControllerNode = std::make_shared<esvr2_ros::PivotControllerROSNode>(mNh, mVideoLoaderNode);
        }
        else
            ROS_INFO("DISABLED Pivot Controller");

        mConfig = std::make_shared<esvr2::Esvr2Config>();
        esvr2::VideoInputConfigPtr videoInputConfig = std::make_shared<esvr2::VideoInputConfig>();
        std::vector<std::string> configFilePaths =
                getEsvr2ConfigFilePath(urlstr);
        for (const auto& it: configFilePaths)
        {
            if (!esvr2::readConfigYml(
                    it, mConfig, videoInputConfig))
            {
                ROS_ERROR("Cannot read Configfile.");
                return false;
            }
        }
        if(videoInputConfig->stereoCameraConfig.leftCameraConfig.valid() &&
           videoInputConfig->stereoCameraConfig.rightCameraConfig.valid())
        {
            ROS_INFO_STREAM_NAMED(
                    "esvr2_ros",
                    "set camera config");
            mVideoLoaderNode->setStereoCameraConfig(
                    videoInputConfig->stereoCameraConfig);
        }
        mConfig->resourcePath = RESOURCES_FILE;
        if(!logFolder.empty())
            mConfig->logFolder = logFolder;
        return true;
    }

    void publishHeadPose()
    {
        std::array<esvr2::Real, 3> translation;
        std::array<esvr2::Real, 4> rotation;
        mEsvr2->getHeadPose(translation, rotation);
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.seq = mSeq++;
        transformStamped.header.frame_id = "vr_world";
        transformStamped.child_frame_id = "vr_head";
        transformStamped.transform.translation.x = translation.at(0);
        transformStamped.transform.translation.y = translation.at(1);
        transformStamped.transform.translation.z = translation.at(2);
        transformStamped.transform.rotation.w = rotation.at(0);
        transformStamped.transform.rotation.x = rotation.at(1);
        transformStamped.transform.rotation.y = rotation.at(2);
        transformStamped.transform.rotation.z = rotation.at(3);
        br.sendTransform(transformStamped);
    }

    void startHeadPosePublisher(std::shared_ptr<esvr2::Esvr2> esvr2)
    {
        mEsvr2 = esvr2;
        mHeadPoseTimer = mNh.createTimer(
                ros::Duration(ros::Rate(30)),
                boost::bind(&Esvr2ROSNode::publishHeadPose, this),
                false,
                true);
    }
};

void update(esvr2::uint64 t)
{
    ros::spinOnce();
}

//main
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "esvr2");
    Esvr2ROSNode esvr2RosNode {};
    if(!esvr2RosNode.initialize())
    {
        ROS_ERROR_NAMED("esvr2_ros", "could not initialize Main-Node");
        return 0;
    }

    if(!esvr2RosNode.mParticipantId.empty() && !esvr2RosNode.mSetupId.empty())
    {
        esvr2RosNode.mConfig->logPrefix = esvr2RosNode.mParticipantId + "_" + esvr2RosNode.mSetupId;
        esvr2RosNode.mConfig->cachePrefixParticipantId = esvr2RosNode.mParticipantId;
    }

    std::shared_ptr<esvr2::Esvr2> esvr2 = std::make_shared<esvr2::Esvr2>(esvr2RosNode.mConfig);
    if (esvr2RosNode.mVideoLoaderNode)
        esvr2->setVideoLoader(esvr2RosNode.mVideoLoaderNode);
    if (esvr2RosNode.mPivotControllerNode)
        esvr2->setLaparoscopeController(esvr2RosNode.mPivotControllerNode);
    esvr2->registerUpdateCallback(boost::bind(&update, _1));
    esvr2RosNode.startHeadPosePublisher(esvr2);
    bool succ = esvr2->run();
    return succ;
}
