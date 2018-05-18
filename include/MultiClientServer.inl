//---------------------------------------------------------------------------------------------------------------------
//  MULTI CLIENT SERVER
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 - Pablo Ramon Soria (a.k.a. Bardo91) - University of Seville
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
//  and associated documentation files (the "Software"), to deal in the Software without restriction, 
//  including without limitation the rights to use, copy, modify, merge, publish, distribute, 
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial 
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES 
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#include<iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>

namespace mcs {
	//-----------------------------------------------------------------------------------------------------------------
	// PRIVATE CLASS
	//-----------------------------------------------------------------------------------------------------------------
	// TCP interface
	//-----------------------------------------------------------------------------------------------------------------
	template<>
	template<>
	MultiClientServer<eSocketType::TCP>::SocketServer<eSocketType::TCP>::SocketServer(int _port) {
		mPort = _port;
		mRun = true;
		std::cout << "Creating TCP server...";
		mListenThread = std::thread([&]() {
			try {
				boost::asio::io_service io_service;
				boost::asio::ip::tcp::acceptor acceptor(io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), mPort));

				std::cout << "awaiting for connetions." << std::endl;
				while (mRun) {
					boost::asio::ip::tcp::socket *socket = new boost::asio::ip::tcp::socket(io_service);
					acceptor.accept(*socket);
					std::cout << "Received new connection: " << socket->native_handle() << std::endl;

					mSafeGuard.lock();
					mTcpConnections.push_back(socket);
					mSafeGuard.unlock();
				}
			}
			catch (std::exception &e) {
				std::cerr << e.what() << std::endl;
			}
		});
	}
	//-----------------------------------------------------------------------------------------------------------------
	template<>
	template<>
	template<typename DataTypeInner_>
	void MultiClientServer<eSocketType::TCP>::SocketServer<eSocketType::TCP>::writeOnClients(DataTypeInner_ &_buffer) {
		assert(false);
	}

	//-----------------------------------------------------------------------------------------------------------------
	template<>
	template<>
	void MultiClientServer<eSocketType::TCP>::SocketServer<eSocketType::TCP>::writeOnClients(std::string &_buffer) {
		try {
			mSafeGuard.lock();
			for (auto &con : mTcpConnections) {
				boost::system::error_code error;
				if (con->is_open())
					boost::asio::write(*con, boost::asio::buffer(_buffer), error);
			}
			mSafeGuard.unlock();
		}
		catch (std::exception &e) {
			std::cerr << e.what() << std::endl;
			mSafeGuard.unlock();
		}
	}

	//-----------------------------------------------------------------------------------------------------------------
	// UDP interface
	//-----------------------------------------------------------------------------------------------------------------
	template<>
	template<>
	MultiClientServer<eSocketType::UDP>::SocketServer<eSocketType::UDP>::SocketServer(int _port) {
		mPort = _port;
		mRun = true;
		std::cout << "Creating UDP server...";
		mListenThread = std::thread([&]() {
			try {
				boost::asio::io_service io_service;
				mServerSocket = new boost::asio::ip::udp::socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), mPort));

				std::cout << "awaiting for connetions." << std::endl;
				while (mRun) {
					boost::array<char, 1> recv_buf;
					boost::asio::ip::udp::endpoint *remote_endpoint = new boost::asio::ip::udp::endpoint();
					boost::system::error_code error;
					mServerSocket->receive_from(boost::asio::buffer(recv_buf), *remote_endpoint, 0, error);

					if (error && error != boost::asio::error::message_size) {
						for (auto it = mUdpConnections.begin(); it != mUdpConnections.end();) {
							if (*(*it) == *remote_endpoint) {
								std::cout << "Connection from " << remote_endpoint->address() << " has droped" << std::endl;
								mSafeGuard.lock();
								it = mUdpConnections.erase(it);
								mSafeGuard.unlock();
							}else{
								it++;
							}
						}
					}
					else {
						std::cout << "Received new connection from " << remote_endpoint->address().to_string() << std::endl;
						mSafeGuard.lock();
						mUdpConnections.push_back(remote_endpoint);
						mSafeGuard.unlock();
					}
				}
			}
			catch (std::exception &e) {
				std::cerr << e.what() << std::endl;
			}
		});
	}

	//-----------------------------------------------------------------------------------------------------------------
	template<>
	template<>
	template<typename DataTypeInner_>
	void MultiClientServer<eSocketType::UDP>::SocketServer<eSocketType::UDP>::writeOnClients(DataTypeInner_ &_buffer) {
		
		mSafeGuard.lock();
		for (auto &con : mUdpConnections) {
			boost::system::error_code error;
			boost::system::error_code ignored_error;

			boost::array<char, sizeof(DataTypeInner_)> send_buffer;
			memcpy(&send_buffer[0], &_buffer, sizeof(DataTypeInner_));
			try {
				mServerSocket->send_to(boost::asio::buffer(send_buffer), *con, 0, ignored_error);
			}
			catch (std::exception &e) {
				std::cerr << e.what() << std::endl;
			}
		}
		mSafeGuard.unlock();
		
		
	}

	//-----------------------------------------------------------------------------------------------------------------
	template<>
	template<>
	void MultiClientServer<eSocketType::UDP>::SocketServer<eSocketType::UDP>::writeOnClients(std::string &_buffer) {
		assert(false);
	}

	//-----------------------------------------------------------------------------------------------------------------
	template<>
	MultiClientServer<eSocketType::TCP>::MultiClientServer(int _port) {
		mSocketServer = new SocketServer<eSocketType::TCP>(_port);
	}

	//-----------------------------------------------------------------------------------------------------------------
	template<>
	MultiClientServer<eSocketType::UDP>::MultiClientServer(int _port) {
		mSocketServer = new SocketServer<eSocketType::UDP>(_port);
	}

	//-----------------------------------------------------------------------------------------------------------------
	template<>
	template<>
	void MultiClientServer<eSocketType::TCP>::writeOnClients<std::string>(std::string &_data) {
		mSocketServer->writeOnClients(_data);
	}

	//-----------------------------------------------------------------------------------------------------------------
	template<>
	template<typename DataType_>
	void MultiClientServer<eSocketType::UDP>::writeOnClients(DataType_ &_data) {
		mSocketServer->writeOnClients<DataType_>(_data);

	}
	
}