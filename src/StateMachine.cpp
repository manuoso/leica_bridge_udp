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

    std::ifstream rawFile("src/leica_bridge_udp/config/config.json");
    if (!rawFile.is_open()) {
        std::cout << "Error opening config file" << std::endl;
        return false;
    }

    std::stringstream strStream;
    strStream << rawFile.rdbuf();
    std::string json = strStream.str(); 

    rapidjson::Document configFile;
    if(configFile.Parse(json.c_str()).HasParseError()){
        std::cout << "Error parsing json" << std::endl;
        return false;
    }

    ip_ = configFile["ip"].GetString();
    LogManager::get()->status("Server IP: " + ip_, false);

    port_ = configFile["port"].GetInt();
    LogManager::get()->status("Server Port: " + std::to_string(port_), false);
    
    std::string topic = configFile["topic"].GetString();
    LogManager::get()->status("Topic name: " + topic, false);
    
    typeTopic_ = configFile["typeTopic"].GetInt();

    xOffset_ = configFile["offsetX"].GetDouble();
    yOffset_ = configFile["offsetY"].GetDouble();
    zOffset_ = configFile["offsetZ"].GetDouble();

    xRot_ = configFile["rotX"].GetDouble();
    yRot_ = configFile["rotY"].GetDouble();
    zRot_ = configFile["rotZ"].GetDouble();
    
    Eigen::Vector3f cTmat(xOffset_, yOffset_, zOffset_);

    Eigen::Matrix3f cRmat = Eigen::Matrix3f::Identity();   
    matTc_.setIdentity();  
    matTc_.block<3,3>(0,0) = cRmat;
    matTc_.block<3,1>(0,3) = cTmat;

    initSocket(ip_, port_);
    
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
		
        lockLeica_.lock();
        Eigen::Vector4f normPosition(data_.x, data_.y, data_.z, 1);
        lockLeica_.unlock();

        // Transform position from camera frame to UAV frame
        Eigen::Vector4f rotPosition;
        rotPosition = matTc_ * normPosition;

        Eigen::Vector3f currPosition = Eigen::Vector3f(rotPosition[0] , rotPosition[1] , rotPosition[2]);

        Eigen::Matrix3f rot;
        if(xRot_ != 0){
            rot = Eigen::AngleAxisf(deg2Rad(xRot_), Eigen::Vector3f::UnitX()).matrix();
            currPosition = rot * currPosition;
        }

        if(yRot_ != 0){
            rot = Eigen::AngleAxisf(deg2Rad(yRot_), Eigen::Vector3f::UnitY()).matrix();
            currPosition = rot * currPosition;
        }

        if(zRot_ != 0){
            rot = Eigen::AngleAxisf(deg2Rad(zRot_), Eigen::Vector3f::UnitZ()).matrix();
            currPosition = rot * currPosition;          
        }

        if(typeTopic_ == 1){
            sendPose_.header.stamp = ros::Time::now();
            sendPose_.header.frame_id = "fcu";
            sendPose_.pose.position.x = currPosition[0]; 
            sendPose_.pose.position.y = currPosition[1]; 
            sendPose_.pose.position.z = currPosition[2];  
            sendPose_.pose.orientation.w = 1.0; 
            sendPose_.pose.orientation.x = 0.0; 
            sendPose_.pose.orientation.y = 0.0; 
            sendPose_.pose.orientation.z = 0.0;

            pub_.publish(sendPose_); 
        }else{
            sendPoint_.header.stamp = ros::Time::now();
            sendPoint_.point.x = currPosition[0];
            sendPoint_.point.y = currPosition[1];
            sendPoint_.point.z = currPosition[2];

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

            if((data_recv.x == 0) && (data_recv.y == 0) && (data_recv.z == 0) && (data_recv.timestamp == 0)){
                LogManager::get()->warning("WARNING! Prism lost be careful", true);
                lockLeica_.lock();
                data_.x = latestData_.x;
                data_.y = latestData_.y;
                data_.z = latestData_.z;
                data_.timestamp = latestData_.timestamp;
                lockLeica_.unlock();
            }else{
                lockLeica_.lock();
                data_.x = data_recv.x;
                data_.y = data_recv.y;
                data_.z = data_recv.z;
                data_.timestamp = data_recv.timestamp;
                lockLeica_.unlock();
            }

            latestData_.x = data_recv.x;
            latestData_.y = data_recv.y;
            latestData_.z = data_recv.z;
            latestData_.timestamp = data_recv.timestamp;
	    }
        catch (std::exception& e)
	    {
		    std::cerr << e.what() << std::endl;
	    }

        rate.sleep();
    }   
}

//---------------------------------------------------------------------------------------------------------------------
float StateMachine::deg2Rad(float _deg){
    return _deg * (3.141592653589793238463 / 180.0);
}