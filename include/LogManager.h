///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef LOGMANAGER_H_
#define LOGMANAGER_H_

#include <string>
#include <fstream>
#include <mutex>
#include <chrono>

/// Thread safe class used as logging system. 
class LogManager {
public:	//	Static interface.
	/// Initialize the logging system. 
	/// \param _appName: Base name used for the log file.
	/// \param _useCout: Write to cout too.
    static void init(const std::string _appName);

	/// Close the logging system. It makes sure that the log is closed properly.
	static void close();

	/// Get current instance of the logging system.
	static LogManager* get();

public:	// Public interface.
	/// Write message to the log system with a custom tag
    void message(const std::string &_msg, const std::string &_tag, bool _useCout = false);
	
	/// Write to the log system with status tag.
    void status(const std::string &_msg, bool _useCout = false);

	/// Write to the log system with warning tag.
    void warning(const std::string &_msg, bool _useCout = false);

	/// Write to the log system with error tag.
    void error(const std::string &_msg, bool _useCout = false);


private:	// Private interface.
    LogManager(const std::string _appName);
	~LogManager();

	static LogManager *mSingleton;

	bool mUseCout = false;

	std::chrono::high_resolution_clock::time_point  mInitTime;
	std::ofstream mLogFile;
	std::mutex mSecureGuard;
};

#endif
