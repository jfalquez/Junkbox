/*****************************************************************************
 * TODO:
 * - Add # of retries in exception handling loops to avoid looping forever.
 * - Clear Node table whenever we receive a re-synch packet.
 *
 ******************************************************************************/

#ifndef _NODE_H_
#define _NODE_H_

#include <map>
#include <string>
#include <sstream>
#include <iostream>

#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <boost/thread.hpp>

#include <zmq.hpp>
#include <zmq.h>

#include <google/protobuf/message.h>


namespace rpg
{

    class Node : public boost::mutex
    {
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        typedef void(*FuncPtr)( google::protobuf::Message&, google::protobuf::Message&, void * );


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        struct RPC {
            FuncPtr                     RpcFunc;
            google::protobuf::Message*  ReqMsg;
            google::protobuf::Message*  RepMsg;
            void*                       UserData;
        };

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        unsigned int _RandomizePort()
        {
			// Dynamic/Private Ports: 49152 through 65535
            std::map < std::string, zmq::socket_t* >::iterator it;

            unsigned int nPort = rand() % 1000 + 50000;

            std::ostringstream address;
            address << "tcp://*:" << nPort;

            // check if port is already in use
            it = m_mSockets.find( address.str() );
            while( it != m_mSockets.end() ) {
                nPort = rand() % 1000 + 50000;
                address.str("");
                address << "tcp://*:" << nPort;
                it = m_mSockets.find( address.str() );
            }
            return nPort;
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void _StartRPC()
        {
            m_pRpcSocket = new zmq::socket_t( *m_pContext, ZMQ_REP );

            std::ostringstream address;
            unsigned int nPort = _RandomizePort();
            address << "tcp://*:" << nPort;

            while(1) {
                try {
                    m_pRpcSocket->bind( address.str().c_str() );
                    break;
                }
                catch( zmq::error_t error ) {
                    std::cout << "[Node] Problem starting RPC listener at '" << address.str() << "'. Retrying..." << std::endl;
                    address.str("");
                    nPort = _RandomizePort();
                    address << "tcp://*:" << nPort;
                }
            }

            std::cout << "[Node] Starting RPC listener at " << address.str() << std::endl;

            // add RPC port to auto-discovery table
            address.str("");
            address << m_sHost << ":" << nPort;

            lock();
            m_mNodeTable[ m_sNodeName ] = address.str();
            unlock();

            // propagate changes
            _AutoDiscPropagate();

            // run listener in a different thread...
            boost::thread RPCThread( _RPCThreadFunc, this );
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        static void _RPCThreadFunc( Node *pThis )
        {
            while( 1 ) {
                // wait for request
                zmq::message_t ZmqReq;
                if( pThis->m_pRpcSocket->recv( &ZmqReq ) == false ) {
                    // error receiving
                    std::cout << "[Node] WARNING! RPC listener was terminated." << std::endl;
                    exit(1);
                }
                // obtain "header" which contains function name
                unsigned char FuncNameSize;
                memcpy( &FuncNameSize, ZmqReq.data(), sizeof(FuncNameSize) );
                std::string FuncName( (char*)(ZmqReq.data()) + sizeof(FuncNameSize), FuncNameSize );

                // prepare reply message
                unsigned int PbOffset = sizeof(FuncNameSize) + FuncNameSize;
                unsigned int PbByteSize = ZmqReq.size() - PbOffset;

                // look-up function
                std::map < std::string, RPC* >::iterator it;

                it = pThis->m_mRpcTable.find( FuncName );
                if( it != pThis->m_mRpcTable.end() ) {

                    RPC* pRPC = it->second;
                    FuncPtr Func = pRPC->RpcFunc;
                    google::protobuf::Message* Req = pRPC->ReqMsg;
                    google::protobuf::Message* Rep = pRPC->RepMsg;

                    if( !Req->ParseFromArray( (char*)ZmqReq.data() + PbOffset, PbByteSize ) ) {
                        continue;
                    }

                    // call function
                    (*Func)( *(Req),*(Rep), pRPC->UserData );

                    // send reply
                    zmq::message_t ZmqRep( Rep->ByteSize() );
                    if( !Rep->SerializeToArray( ZmqRep.data(), Rep->ByteSize() ) ) {
                        // error serializing protobuf to ZMQ message
                    }
                    pThis->m_pRpcSocket->send( ZmqRep );
                } else {
                    // send empty reply
                    zmq::message_t ZmqRep(0);
                    pThis->m_pRpcSocket->send( ZmqRep );
                }
            }
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void _AutoDiscPoll()
        {
            // send token
            unsigned int NullVal = 0;

            // init message
            zmq::message_t ZmqMsg( sizeof(NullVal) );
            memcpy( ZmqMsg.data(), &NullVal, sizeof(NullVal) );

            // send message
            m_pAdPub->send( ZmqMsg );
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void _AutoDiscPropagate()
        {
            // calculate size of message
            unsigned int NumEntries = m_mNodeTable.size();

            if( NumEntries == 0 ) {
                return;
            }

            unsigned int MsgSize = sizeof(NumEntries);

            lock();
            std::map < std::string, std::string>::iterator it;
            for( it = m_mNodeTable.begin() ; it != m_mNodeTable.end(); it++ ) {
                MsgSize += 2 * sizeof(unsigned int);
                MsgSize += (it->first).size() + (it->second).size();
            }

            // init message
            zmq::message_t ZmqMsg( MsgSize );

            // insert header
            char* MsgPtr = (char*)ZmqMsg.data();
            memcpy( MsgPtr, &NumEntries, sizeof(NumEntries) );
            MsgPtr += sizeof(NumEntries);

            // pack update message
            for( it = m_mNodeTable.begin() ; it != m_mNodeTable.end(); it++ ) {
                unsigned int sSize;

                sSize = (it->first).size();
                memcpy( MsgPtr, &sSize, sizeof(sSize) );
                MsgPtr += sizeof(sSize);
                memcpy( MsgPtr, (it->first).c_str(), sSize );
                MsgPtr += sSize;

                sSize = (it->second).size();
                memcpy( MsgPtr, &sSize, sizeof(sSize) );
                MsgPtr += sizeof(sSize);
                memcpy( MsgPtr, (it->second).c_str(), sSize );
                MsgPtr += sSize;
            }
            unlock();

            // send message
            m_pAdPub->send( ZmqMsg );
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        static void _AutoDiscThreadFunc( Node *pThis )
        {
            while( 1 ) {
                // receive updates
                zmq::message_t ZmqMsg;
                if( pThis->m_pAdSub->recv( &ZmqMsg ) == false ) {
                    // error receiving
                    std::cout << "[Node] WARNING! Auto-discovery listener was terminated." << std::endl;
                    exit(1);
                }

                // obtain "header" which contains number of entries in map
                unsigned int NumEntries;
                memcpy( &NumEntries, ZmqMsg.data(), sizeof(NumEntries) );

                if( NumEntries == 0 ) {
                    // someone is asking for our services
                    pThis->_AutoDiscPropagate();
                } else {
                    // someone updated the table... update ours
                    char* MsgPtr = (char*)ZmqMsg.data() + sizeof(NumEntries);
                    std::map < std::string, std::string >::iterator it;
                    for( unsigned int ii = 0; ii < NumEntries; ii++ ) {
                        unsigned int sSize;

                        std::memcpy( &sSize, MsgPtr, sizeof(sSize) );
                        MsgPtr += sizeof(sSize);
                        std::string sService( MsgPtr, sSize );
                        MsgPtr += sSize;

                        std::memcpy( &sSize, MsgPtr, sizeof(sSize) );
                        MsgPtr += sizeof(sSize);
                        std::string sAddress( MsgPtr, sSize );
                        MsgPtr += sSize;

                        it = pThis->m_mNodeTable.find( sService );
                        if( it == pThis->m_mNodeTable.end() ) {
                            pThis->lock();
                            pThis->m_mNodeTable[ sService ] = sAddress;
                            pThis->unlock();
                        }
                    }
					// print table
					/*
                    std::cout << "--------------------------------------------------------" << std::endl;
                    for( it = pThis->m_mNodeTable.begin() ; it != pThis->m_mNodeTable.end(); it++ ) {
                        std::cout << "Service '" << it->first << "' at '" << it->second << "'" << std::endl;
                    }
					std::cout << "--------------------------------------------------------" << std::endl;
					/* */
                }
            }
        }


    public:

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        Node(
			std::string							sNodeName		//< Input: Node identifier
			)
        {
            // Find hosts's network information
			char			buf[1024];
			struct ifconf	ifc;
			struct ifreq*	ifr;
			int				sck;
			int				nInterfaces;

			// get a socket handle
			sck = socket(AF_INET, SOCK_DGRAM, 0);
			if(sck < 0)
			{
                std::cout << "[Node] Error obtaining network information." << std::endl;
				exit(0);
			}

			// query available interfaces
			ifc.ifc_len = sizeof(buf);
			ifc.ifc_buf = buf;
			if(ioctl(sck, SIOCGIFCONF, &ifc) < 0)
			{
                std::cout << "[Node] Error obtaining network information." << std::endl;
				exit(0);
			}

			// iterate through the list of interfaces
			ifr         = ifc.ifc_req;
			nInterfaces = ifc.ifc_len / sizeof(struct ifreq);

			int CurPriority = INT_MAX;

			for(int ii = 0; ii < nInterfaces; ii++) {
				struct ifreq *item = &ifr[ii];

				int Priority;
				std::string sIF = item->ifr_name;

				if( sIF == "eth0" ) {
					Priority = 0;
				}
				if( sIF == "eth1" ) {
					Priority = 1;
				}
				if( sIF == "wlan0" ) {
					Priority = 2;
				}
				if( sIF == "lo" ) {
					Priority = 3;
				}

				if( Priority < CurPriority ) {
					CurPriority = Priority;
					m_sHost = inet_ntoa(((struct sockaddr_in *)&item->ifr_addr)->sin_addr);
				}
			}

            std::cout << "[Node] Node started at '" << m_sHost << "'" << std::endl;

			// set node name
			m_sNodeName = sNodeName;

            // init context
            m_pContext = new zmq::context_t(1);

            // set RPC socket to NULL
            m_pRpcSocket = NULL;

            // initialize auto-discovery
            m_pAdSub = new zmq::socket_t( *m_pContext, ZMQ_SUB );
            m_pAdSub->setsockopt( ZMQ_SUBSCRIBE, NULL, 0 );
			std::string AdAddress = "epgm://" + m_sHost + ";225.0.0.1:49999";
            m_pAdSub->connect( AdAddress.c_str() );
            m_pAdPub = new zmq::socket_t( *m_pContext, ZMQ_PUB );
            m_pAdPub->bind( AdAddress.c_str() );

            // run auto-discovery in a different thread...
            boost::thread ADThread( _AutoDiscThreadFunc, this );

			// poll service table
            _AutoDiscPoll();

			// wait 100ms for table propagation
			usleep(100000);
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ~Node()
        {
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        template <class Req, class Rep>
        bool Register(
				const std::string&				sName,						//< Input: Function name
				void							(*pFunc)(Req&,Rep&,void*),	//< Input: Function pointer
				void							*pUserData					//< Input: User data passed to the function
				)
        {
            // set up RPC listener if not started already
            if( m_pRpcSocket == NULL ) {
                _StartRPC();
            }

            // check if function with that name is already registered
            std::map < std::string, RPC* >::iterator it;
            it = m_mRpcTable.find( sName );
            if( it != m_mRpcTable.end() ) {
                return false;
            } else {
                RPC* pRPC = new RPC;
                pRPC->RpcFunc = (FuncPtr)pFunc;
				pRPC->ReqMsg = new Req;
				pRPC->RepMsg = new Rep;
                pRPC->UserData = pUserData;
                m_mRpcTable[ sName ] = pRPC;
                return true;
            }
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool Call(
                const std::string&					sHost,			//< Input: Remote host name
                const std::string&					sFuncName,		//< Input: Remote function to call
                const google::protobuf::Message&	MsgReq,			//< Input: Protobuf message containing request
                google::protobuf::Message&			MsgRep,			//< Output: Protobuf message holding reply
				unsigned int						TimeOut = 10	//< Input: Number of miliseconds to wait for reply
                )
        {
            std::map < std::string, zmq::socket_t* >::iterator it;
            zmq::socket_t* pSock;

            // check if socket is already open for this host
            it = m_mSockets.find( sHost );
            if( it != m_mSockets.end() ) {
                // socket is already open, lets use it
                pSock = it->second;
            } else {
                // socket is not open, lets find this hostname
				std::map < std::string, std::string >::iterator its;
	            its = m_mNodeTable.find( sHost );
		        if( its == m_mNodeTable.end() ) {
					std::cout << "[Node] Node '" << sHost << "' not found on cache table." << std::endl;
					return false;
				} else {
					// lets open a new socket
					pSock = new zmq::socket_t( *m_pContext, ZMQ_REQ );

					// lets connect using the socket
					try {
						pSock->connect( ("tcp://" + its->second).c_str() );
					}
					catch( zmq::error_t error ) {
						// oops, an error occurred lets rollback
						delete pSock;
						return false;
					}
					m_mSockets[ sHost ] = pSock;
				}
            }
            // prepare to append function information
            std::string FuncName = sFuncName;
            if( sFuncName.size() > 254 ) {
                FuncName.resize(254);
            }
            unsigned char FuncNameSize = FuncName.size();

            // prepare message
            zmq::message_t ZmqReq( sizeof(FuncNameSize) + FuncNameSize + MsgReq.ByteSize() );
            std::memcpy( ZmqReq.data(), &FuncNameSize, sizeof(FuncNameSize) );
            std::memcpy( (char*)ZmqReq.data() + sizeof(FuncNameSize), FuncName.c_str(), FuncNameSize );
            if( !MsgReq.SerializeToArray( (char*)ZmqReq.data() + sizeof(FuncNameSize) + FuncNameSize, MsgReq.ByteSize() ) ) {
                // error serializing protobuf to ZMQ message
                return false;
            }

            // send request
			try {
				if( pSock->send( ZmqReq, ZMQ_NOBLOCK ) == false ) {
					// error sending request
					// delete socket -- see "Lazy pirate ZMQ"
					pSock->close();
					it = m_mSockets.find( sHost );
					m_mSockets.erase(it);
					return false;
				}
			}
			catch( zmq::error_t error ) {
				return false;
			}

            // wait reply
            unsigned int nCount = 0;
            zmq::message_t ZmqRep;
            while( pSock->recv( &ZmqRep, ZMQ_NOBLOCK ) == false && nCount < TimeOut ) {
                nCount++;
                usleep(1000);
            }
            if( nCount == TimeOut ) {
                // timeout... error receiving
				// delete socket -- see "Lazy Pirate ZMQ"
				std::cout << "[Node] Warning: Call timed out waiting for reply." << std::endl;
                pSock->close();
				it = m_mSockets.find( sHost );
				m_mSockets.erase(it);
                return false;
            }
            if( MsgRep.ParseFromArray( ZmqRep.data(), ZmqRep.size() ) == false ) {
                // bad protobuf format
                return false;
            }
            return true;
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool Publish(
                const std::string&				sTopic        	//< Input: Topic name
                )
        {
			std::string NURI = m_sNodeName + "/" + sTopic;

            // check if socket is already open for this topic
            std::map < std::string, zmq::socket_t* >::iterator it;
            it = m_mSockets.find( NURI );
            if( it != m_mSockets.end() ) {
                // socket is already open, return false
                return false;
            } else {
                // no socket open.. lets open a new one

                // check if port is already in use
                std::ostringstream address;
                unsigned int nPort = _RandomizePort();
                address << "tcp://*:" << nPort;
                it = m_mSockets.find( address.str() );
                if( it != m_mSockets.end() ) {
                    // port is in use, return false
                    return false;
                }

                // create socket
                zmq::socket_t* pSock = new zmq::socket_t( *m_pContext, ZMQ_PUB );
                while(1) {
                    try {
                        pSock->bind(address.str().c_str());
                        break;
                    }
                    catch( zmq::error_t error ) {
                        std::cout << "[Node] Problem binding Publisher address. Retrying..." << std::endl;
                        address.str("");
                        nPort = _RandomizePort();
                        address << "tcp://*:" << nPort;
                    }
                }
                std::cout << "[Node] Publishing topic '" << sTopic << "' on " << address.str() << std::endl;
                m_mSockets[ NURI ] = pSock;

                // add entry to auto-discovery table
                address.str("");
                address << m_sHost << ":" << nPort;

                lock();
                m_mNodeTable[ NURI ] = address.str();
                unlock();

                // propagate changes
                _AutoDiscPropagate();

                return true;
            }
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool Write(
                const std::string&					sTopic,     //< Input: Topic to write to
                const google::protobuf::Message&	Msg			//< Input: Message to send
                )
        {
			std::string NURI = m_sNodeName + "/" + sTopic;

            // check if socket is already open for this topic
            std::map < std::string, zmq::socket_t* >::iterator it;
            it = m_mSockets.find( NURI );
            if( it == m_mSockets.end() ) {
                // no socket found
                return false;
            } else {
                zmq::socket_t* pSock = it->second;
                zmq::message_t ZmqMsg( Msg.ByteSize() );
                if( !Msg.SerializeToArray( ZmqMsg.data(), Msg.ByteSize() ) ) {
                    // error serializing protobuf to ZMQ message
                    return false;
                }
				try {
					return pSock->send( ZmqMsg );
				}
				catch( zmq::error_t error ) {
					return false;
				}
			}
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool Write(
                const std::string&				sTopic,			//< Input: Topic to write to
                zmq::message_t&					Msg				//< Input: Message to send
                )
        {
			std::string NURI = m_sNodeName + "/" + sTopic;

            // check if socket is already open for this topic
            std::map < std::string, zmq::socket_t* >::iterator it;
            it = m_mSockets.find( NURI );
            if( it == m_mSockets.end() ) {
                // no socket found
                return false;
            } else {
                zmq::socket_t* pSock = it->second;
				try {
					return pSock->send( Msg );
				}
				catch( zmq::error_t error ) {
					return false;
				}
            }
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool Subscribe(
				const std::string&				NURI		//< Input: Node URI in the form of "NodeName/Topic"
                )
        {
            // check if socket is already open for this topic
            std::map < std::string, zmq::socket_t* >::iterator it;
            it = m_mSockets.find( NURI );
            if( it != m_mSockets.end() ) {
                // subscription for that topic already exists
                return false;
            } else {
                // lets find this node's IP
				std::map < std::string, std::string >::iterator its;
	            its = m_mNodeTable.find( NURI );
		        if( its == m_mNodeTable.end() ) {
					std::cout << "[Node] NURI '" << NURI << "' not found on cache table." << std::endl;
					return false;
				} else {
					// create socket
					zmq::socket_t* pSock = new zmq::socket_t( *m_pContext, ZMQ_SUB );

					// lets connect using the socket
					try {
						pSock->setsockopt( ZMQ_SUBSCRIBE, NULL, 0 );
						pSock->connect( ("tcp://" + its->second).c_str() );
					}
					catch( zmq::error_t error ) {
						// oops, an error occurred lets rollback
						delete pSock;
						return false;
					}
					m_mSockets[ NURI ] = pSock;
					return true;
				}
            }
        }


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool Read(
                const std::string&				NURI,			//< Input: Node URI in the form of "NodeName/Topic"
                google::protobuf::Message&		Msg				//< Output: Message read
                )
        {
            // check if socket is already open for this topic
            std::map < std::string, zmq::socket_t* >::iterator it;
            it = m_mSockets.find( NURI );
            if( it == m_mSockets.end() ) {
                // no socket found
                return false;
            } else {
                zmq::socket_t* pSock = it->second;
                zmq::message_t ZmqMsg;
				try {
					if( pSock->recv( &ZmqMsg, ZMQ_NOBLOCK ) == false ) {
						// nothing to read
						return false;
	                }
				}
				catch( zmq::error_t error ) {
					return false;
				}
                if( !Msg.ParseFromArray( ZmqMsg.data(), ZmqMsg.size() ) ) {
                    return false;
                }
                return true;
            }
        }

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool Read(
                const std::string&				NURI,			//< Input: Node URI in the form of "NodeName/Topic"
                zmq::message_t&					ZmqMsg			//< Output: ZMQ Output message
                )
        {
            // check if socket is already open for this topic
            std::map < std::string, zmq::socket_t* >::iterator it;
            it = m_mSockets.find( NURI );
            if( it == m_mSockets.end() ) {
                // no socket found
                return false;
            } else {
                zmq::socket_t* pSock = it->second;
				try {
					if( pSock->recv( &ZmqMsg, ZMQ_NOBLOCK ) == false ) {
						// nothing to read
						return false;
					}
				}
				catch( zmq::error_t error ) {
					return false;
				}
                return true;
            }
        }


    private:
        zmq::context_t*                         m_pContext;     // global context

        // Node topic/port auto-discovery
        std::string                             m_sHost;		// node's machine IP
		std::string								m_sNodeName;	// node unique name
        zmq::socket_t*                          m_pAdPub;       // auto-discovery publisher
        zmq::socket_t*                          m_pAdSub;       // auto-discovery subscriber
        std::map < std::string, std::string>    m_mNodeTable;   // lookup table

        // List of open sockets
        std::map< std::string, zmq::socket_t* > m_mSockets;     // map of hosts + sockets

        // Variables for RPC (ReqRep)
        zmq::socket_t*                          m_pRpcSocket;   // global RPC socket
        std::map< std::string, RPC* >           m_mRpcTable;    // map of functions + RPC structs

    };

}

#endif
