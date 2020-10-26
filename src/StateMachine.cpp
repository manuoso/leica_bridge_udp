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
    std::cin >> coord_;

    if(coord_ == 2){
        xOffset_ = 0.0;
        yOffset_ = 0.0;
        zOffset_ = 1.3;
        std::cout << "Offsets by default: X -> " << xOffset_ << " Y-> " << yOffset_ << " Z-> " << zOffset_ << std::endl;
    }

    std::cout << "Enter IP of server to connect: ";	
	std::cin >> ip_;
    LogManager::get()->status("Server IP: " + ip_, false);

	std::cout << "Enter PORT of server to connect: ";
	std::cin >> port_;
    LogManager::get()->status("Server Port: " + std::to_string(port_), false);

    initSocket(ip_, port_);
    
    std::string topic;
    std::cout << "Enter TOPIC to publish: ";	
	std::cin >> topic;
    LogManager::get()->status("Topic name: " + topic, false);

    std::cout << "Enter Type of topic (1-> PoseStamped, Other number PointStamped): ";	
	std::cin >> typeTopic_;

    ros::NodeHandle n;
    if(typeTopic_ == 1){
        pub_ = n.advertise<geometry_msgs::PoseStamped>(topic.c_str(), 1);
    }else{
        pub_ = n.advertise<geometry_msgs::PointStamped>(topic.c_str(), 1);
    }

    leicaListeningThread_ = std::thread(&StateMachine::leicaListenCallback, this);
    
    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool StateMachine::run() {
    LogManager::get()->status("Starting main loop", true);
    
    ros::Rate rate(20);

    while(ros::ok()){
					
		auto end = std::chrono::high_resolution_clock::now();
		auto delay = std::chrono::duration_cast<std::chrono::milliseconds>(end - leicaTime_);
        if(delay.count() < 50){
            LogManager::get()->status("Reasonable delay. Delay: " + std::to_string(delay.count()), true);
        }else{
            LogManager::get()->warning("WARNING! exceeded timeout. Delay: " + std::to_string(delay.count()), true);
        }

        if((data_.x == 0) && (data_.y == 0) && (data_.z == 0) && (data_.timestamp == 0)){
            LogManager::get()->warning("WARNING! Prism lost be careful", true);
        }
		
        if(typeTopic_ == 1){
            lockLeica_.lock();
            sendPose_.header.stamp = ros::Time::now();
            sendPose_.header.frame_id = "fcu";
            sendPose_.pose.position.x = data_.x; 
            sendPose_.pose.position.y = data_.y; 
            sendPose_.pose.position.z = data_.z;  
            sendPose_.pose.orientation.w = 1.0; 
            sendPose_.pose.orientation.x = 0.0; 
            sendPose_.pose.orientation.y = 0.0; 
            sendPose_.pose.orientation.z = 0.0;
            lockLeica_.unlock();

            pub_.publish(sendPose_); 
        }else{
            lockLeica_.lock();
            sendPoint_.header.stamp = ros::Time::now();
            sendPoint_.header.frame_id = "fcu";
            sendPoint_.point.x = data_.x; 
            sendPoint_.point.y = data_.y; 
            sendPoint_.point.z = data_.z; 
            lockLeica_.unlock();

            pub_.publish(sendPoint_); 
        }

        LogManager::get()->message(std::to_string(data_.x) + "," + std::to_string(data_.y) + "," + std::to_string(data_.z) + "," + std::to_string(data_.timestamp), "PRISM_SENT", false);

        rate.sleep();
    }
}

//---------------------------------------------------------------------------------------------------------------------
bool StateMachine::initSocket(std::string _ip, int _port){
    try {
		boost::asio::ip::udp::endpoint endpoint(boost::asio::ip::address::from_string(_ip), _port);

        boost::asio::io_service io_service;
		socket_ = new udp::socket(io_service);
		socket_->open(udp::v4());

		boost::array<char, 1> send_buf = { { 0 } };
		socket_->send_to(boost::asio::buffer(send_buf), endpoint);

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

    ros::Rate rate(20);

    while(finishThreadListen_){
        try {
            boost::array<char, sizeof(data)> recv_buf;
		    udp::endpoint sender_endpoint;
		    size_t len = socket_->receive_from(boost::asio::buffer(recv_buf), sender_endpoint);
            leicaTime_ = std::chrono::high_resolution_clock::now();

		    data data_recv;
		    memcpy(&data_recv, &recv_buf[0], sizeof(data));
            LogManager::get()->message(std::to_string(data_recv.x) + "," + std::to_string(data_recv.y) + "," + std::to_string(data_recv.z) + "," + std::to_string(data_recv.timestamp), "PRISM_RECEIVED", false);

            if(coord_ == 1){
                lockLeica_.lock();
                data_.x = data_recv.x;
                data_.y = data_recv.y;
                data_.z = data_recv.z;
                data_.timestamp = data_recv.timestamp;
                lockLeica_.unlock();
            }else if(coord_ == 2){
                lockLeica_.lock();
                data_.x = xOffset_ + data_recv.z*sin(data_recv.x)*sin(data_recv.y);
                data_.y = yOffset_ + data_recv.z*cos(data_recv.x)*sin(data_recv.y);
                data_.z = zOffset_ + data_recv.z*cos(data_recv.y);
                data_.timestamp = data_recv.timestamp;
                lockLeica_.unlock();
            }else{
                std::cout << "Unrecognized type of coordinates!" << std::endl;
            }
	    }
        catch (std::exception& e)
	    {
		    std::cerr << e.what() << std::endl;
	    }

        rate.sleep();
    }   
}
