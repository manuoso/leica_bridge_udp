///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "LogManager.h"
#include <iostream>

using namespace std;

LogManager *LogManager::mSingleton = nullptr;

//---------------------------------------------------------------------------------------------------------------------
void LogManager::init(const string _appName) {
	if (!mSingleton)
        mSingleton = new LogManager(_appName);
	else
		mSingleton->warning("Someone tried to reinitialize the Log Manager");
}

//---------------------------------------------------------------------------------------------------------------------
void LogManager::close(){
	delete mSingleton;
}

//---------------------------------------------------------------------------------------------------------------------
LogManager * LogManager::get(){
	return mSingleton;
}

//---------------------------------------------------------------------------------------------------------------------
void LogManager::message(const std::string & _msg, const std::string & _tag, bool _useCout) {
	double timeSpan = std::chrono::duration<double>(chrono::high_resolution_clock::now() - mInitTime).count();
	std::string logLine = to_string(timeSpan) + "\t [" + _tag + "] " + _msg + "\n";
	mSecureGuard.lock();
	mLogFile << logLine;
	mLogFile.flush();
	mSecureGuard.unlock();
    if(_useCout){
		cout << logLine;
        cout.flush();
	}
}

//---------------------------------------------------------------------------------------------------------------------
void LogManager::status(const std::string & _msg, bool _useCout){
    message(_msg, "STATUS", _useCout);
}

//---------------------------------------------------------------------------------------------------------------------
void LogManager::warning(const std::string & _msg, bool _useCout){
    message(_msg, "WARNING", _useCout);
}

//---------------------------------------------------------------------------------------------------------------------
void LogManager::error(const std::string & _msg, bool _useCout){
    message(_msg, "ERROR", _useCout);
}

//---------------------------------------------------------------------------------------------------------------------
LogManager::LogManager(const std::string _appName) {
	mLogFile.open(_appName + to_string(time(NULL))+".txt");
	mInitTime = chrono::high_resolution_clock::now();
	status("Initialized log");
}

//---------------------------------------------------------------------------------------------------------------------
LogManager::~LogManager() {
	mLogFile.close();
}
