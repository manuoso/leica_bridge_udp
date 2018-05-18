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

#ifndef MCS_MULTICLIENTSERVER_H_
#define MCS_MULTICLIENTSERVER_H_

#include <functional>
#include <thread>
#include <mutex>
#include <vector>
#include <boost/asio.hpp>


namespace mcs {
	enum class eSocketType { UDP, TCP };

	template<eSocketType SocketType_>
	class MultiClientServer {
	public:
		/// Initialize server that allows multiple connection from different clients
		/// \param _port: port to listen the connections
		MultiClientServer(int _port);

		/// Write message
		template<typename DataType_>
		void writeOnClients(DataType_ &_data);

	private:
		template<eSocketType SocketTypeInner_>
		class SocketServer {
		public:
			SocketServer(int  _port);

			// 666 coolify this
			template<typename DataTypeInner_>
			void writeOnClients(DataTypeInner_ &_buffer);

			void writeOnClients(std::string &_buffer);

			void stop(){ mRun = false; }
		private:
			std::thread mListenThread;
			bool mRun = false;
			std::mutex mSafeGuard;

			int mPort;

			// 666 TODO: gather following variables with traits
		 	std::vector<boost::asio::ip::tcp::socket*> mTcpConnections;
			
			std::vector<boost::asio::ip::udp::endpoint*> mUdpConnections;
			boost::asio::ip::udp::socket *mServerSocket;
		};

	private:
		eSocketType mSocketType = eSocketType::TCP;
		SocketServer<SocketType_> *mSocketServer;
	};

}

#include <MultiClientServer.inl>

#endif