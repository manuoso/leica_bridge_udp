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
            boost::array<char, sizeof(datos)> recv_buf;
		    udp::endpoint sender_endpoint;
		    size_t len = mSocket->receive_from(boost::asio::buffer(recv_buf), sender_endpoint);
            mLeicaTime = std::chrono::high_resolution_clock::now();

		    datos data_recv;
		    memcpy(&data_recv, &recv_buf[0], sizeof(datos));
            LogManager::get()->status("Position Prism: " + std::to_string(data_recv.x) + " , " + std::to_string(data_recv.y) + " , " + std::to_string(data_recv.z) + " , " + std::to_string(data_recv.timestamp), false);

            mSecureLeica.lock();
            mData = data_recv;
            mSecureLeica.unlock();
	    }
        catch (std::exception& e)
	    {
		    std::cerr << e.what() << std::endl;
	    }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }   
}
