/////////////////////////////////////////////////////////////////
//															   //
//               StateMachine Leica Bridge UDP                 //
//															   //
//				Author: Manuel P. J. (aka. manuoso)			   //
//															   //
/////////////////////////////////////////////////////////////////

#include <chrono>
#include <mutex>
#include <thread>
#include <math.h>

#include <Eigen/Eigen>
#include <rapidjson/document.h>

#include <ros/ros.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"

#include <boost/array.hpp>
#include <boost/asio.hpp>

#include "LogManager.h"

using boost::asio::ip::udp;

class StateMachine
{
    public:
        struct data{
            float x;
            float y;
            float z;
            uint64_t timestamp;
        };

        bool init(int _argc, char** _argv);
        bool run();

    private:
        bool initSocket(std::string _ip, int _port);
        void leicaListenCallback();
        
        /// Utils
        float deg2Rad(float _deg);

    private:
        udp::socket *socket_;
        std::string ip_;
        int port_;

        data data_, latestData_;
        int typeTopic_;
        geometry_msgs::PointStamped sendPoint_;
        geometry_msgs::PoseStamped sendPose_;
        ros::Publisher pub_;
        std::chrono::high_resolution_clock::time_point leicaTime_;

        bool finishThreadListen_ = true;
        std::thread leicaListeningThread_;
        std::mutex lockLeica_;

        float xOffset_ = 0.0, yOffset_ = 0.0, zOffset_ = 0.0;
        float xRot_ = 0.0, yRot_ = 0.0, zRot_ = 0.0;

        Eigen::Matrix4f matTc_;
};
