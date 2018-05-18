/////////////////////////////////////////////////////////////////
//															   //
//               StateMachine Leica Bridge UDP                 //
//															   //
//				Author: Manuel P. J. (aka. manuoso)			   //
//															   //
/////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include "LogManager.h"
#include <chrono>
#include <mutex>
#include <thread>
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <std_msgs/String.h>

using boost::asio::ip::udp;

class StateMachine
{
    public:
        struct datos{
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

    private:
        udp::socket *mSocket;
        std::string mIP;
        int mPort;

        datos mData;
        geometry_msgs::PoseStamped mSend;
        ros::Publisher mPub;
        std::chrono::high_resolution_clock::time_point mLeicaTime;

        bool mFinishThreadListen = true;
        std::thread mLeicaListeningThread;
        std::mutex mSecureLeica;
};
