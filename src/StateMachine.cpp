/////////////////////////////////////////////////////////////////
//															   //
//               StateMachine Leica Bridge UDP                 //
//															   //
//				Author: Manuel P. J. (aka. manuoso)			   //
//															   //
/////////////////////////////////////////////////////////////////

#include "StateMachine.h"

using boost::asio::ip::udp;

//---------------------------------------------------------------------------------------------------------------------
bool StateMachine::init(int _argc, char**_argv) {
    
    LogManager::init("LeicaBridge_" + std::to_string(time(NULL)));

    std::cout << "Type of coordinates you want to receive: " << std::endl;
    std::cout << "- 1 Cartesian coordinates" << std::endl;
    std::cout << "- 2 Spherical coordinates" << std::endl;
    std::cin >> mCoord;

    if(mCoord == 2){
        mXOffset = 0.0;
        mYOffset = 0.0;
        mZOffset = 1.3;
        std::cout << "Offsets by defects: X -> " << mXOffset << " Y-> " << mYOffset << " Z-> " << mZOffset << std::endl;
    }

    std::cout << "Enter IP of server to connect: ";	
	std::cin >> mIP;
    LogManager::get()->status("Server IP: " + mIP, false);

	std::cout << "Enter PORT of server to connect: ";
	std::cin >> mPort;
    LogManager::get()->status("Server Port: " + std::to_string(mPort), false);

    initSocket(mIP, mPort);
 
    ros::NodeHandle n;
    mPub = n.advertise<geometry_msgs::PoseStamped>("/uav_1/mavros/vision_pose/pose", 0);

    mLeicaListeningThread = std::thread(&StateMachine::leicaListenCallback, this);
    
    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool StateMachine::run() {
 
    LogManager::get()->status("Starting main loop", true);
    while(ros::ok()){
					
		auto end = std::chrono::high_resolution_clock::now();
		auto delay = std::chrono::duration_cast<std::chrono::milliseconds>(end - mLeicaTime);
        if(delay.count() < 50){
            LogManager::get()->status("Reasonable delay. Delay: " + std::to_string(delay.count()), true);
        }else{
            LogManager::get()->warning("WARNING! exceeded timeout. Delay: " + std::to_string(delay.count()), true);
        }

        if((mData.x == 0) && (mData.y == 0) && (mData.z == 0) && (mData.timestamp == 0)){
            LogManager::get()->warning("WARNING! Prism lost be careful", true);
        }
		
        mSecureLeica.lock();
        mSend.header.stamp = ros::Time::now();
        mSend.header.frame_id = "fcu";
        mSend.pose.position.x = mData.x; 
        mSend.pose.position.y = mData.y; 
        mSend.pose.position.z = mData.z;   
        mSend.pose.orientation.w = 1; 
        mSend.pose.orientation.x = 0; 
        mSend.pose.orientation.y = 0; 
        mSend.pose.orientation.z = 0;  
        mSecureLeica.unlock();

        mPub.publish(mSend); 
        LogManager::get()->message(std::to_string(mData.x) + "," + std::to_string(mData.y) + "," + std::to_string(mData.z) + "," + std::to_string(mData.timestamp), "PRISM_SENT", false);

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

//---------------------------------------------------------------------------------------------------------------------
bool StateMachine::initSocket(std::string _ip, int _port){

    try {
		boost::asio::ip::udp::endpoint endpoint(boost::asio::ip::address::from_string(_ip), _port);

        boost::asio::io_service io_service;
		mSocket = new udp::socket(io_service);
		mSocket->open(udp::v4());

		boost::array<char, 1> send_buf = { { 0 } };
		mSocket->send_to(boost::asio::buffer(send_buf), endpoint);

        LogManager::get()->status("Socket created!", true);

        return true;
	}
	catch (std::exception& e)
	{
		std::cerr << e.what() << std::endl;
        return false;
	}
}

//---------------------------------------------------------------------------------------------------------------------
void StateMachine::leicaListenCallback(){

    LogManager::get()->status("Starting Listen Thread", true);
    while(mFinishThreadListen){
        try {
            boost::array<char, sizeof(data)> recv_buf;
		    udp::endpoint sender_endpoint;
		    size_t len = mSocket->receive_from(boost::asio::buffer(recv_buf), sender_endpoint);
            mLeicaTime = std::chrono::high_resolution_clock::now();

		    data data_recv;
		    memcpy(&data_recv, &recv_buf[0], sizeof(data));
            LogManager::get()->message(std::to_string(data_recv.x) + "," + std::to_string(data_recv.y) + "," + std::to_string(data_recv.z) + "," + std::to_string(data_recv.timestamp), "PRISM_RECEIVED", false);

            mSecureLeica.lock();
            if(mCoord == 1){
                mData.x = data_recv.x;
                mData.y = data_recv.y;
                mData.z = data_recv.z;
                mData.timestamp = data_recv.timestamp;
            }else if(mCoord == 2){
                mData.x = mXOffset + data_recv.z*sin(data_recv.x)*sin(data_recv.y);
                mData.y = mYOffset + data_recv.z*cos(data_recv.x)*sin(data_recv.y);
                mData.z = mZOffset + data_recv.z*cos(data_recv.y);
                mData.timestamp = data_recv.timestamp;
            }else{
                std::cout << "Unrecognized type of coordinates!" << std::endl;
            }
            mSecureLeica.unlock();
	    }
        catch (std::exception& e)
	    {
		    std::cerr << e.what() << std::endl;
	    }

        if(mCoord == 1){
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }else if(mCoord == 2){
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }else{
            std::cout << "Unrecognized type of coordinates!" << std::endl;
        }
    }   
}
