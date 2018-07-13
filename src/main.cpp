/////////////////////////////////////////////////////////////////
//															   //
//                   Main Leica Bridge UDP                     //
//															   //
//				Author: Manuel P. J. (aka. manuoso)			   //
//															   //
/////////////////////////////////////////////////////////////////

#include <iostream>
#include "StateMachine.h"

int main(int _argc, char* _argv[]) {

	ros::init(_argc, _argv, "leica_bridge");
    //std::thread spinThread = std::thread([&](){
    //    ros::spin();
    //});
    
	ros::AsyncSpinner spinner(4); // Use 4 threads
	spinner.start();

    StateMachine stateMachine;
    if(!stateMachine.init(_argc, _argv)){
        std::cout << "Error initializing the application" << std::endl;
        return -1;
    }

    stateMachine.run();

	return 0;
}
