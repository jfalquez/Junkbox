#ifndef _NODE_H_
#define _NODE_H_

#include <map>
#include <string>
#include <sstream>
#include <iostream> // not needed

#include <zmq.hpp>

#include <google/protobuf/message.h>

#define MASTER 127.0.0.1:1337

typedef void(*FuncPtr)(std::string&,std::string&);

namespace rpg
{

    class Node
    {

    public:

        Node(
            int nPort       //< Input: RPC Port
            )
        {
            // init context
            m_pContext = new zmq::context_t(1);
            m_pRpcSocket = new zmq::socket_t( *m_pContext, ZMQ_REP );
            std::ostringstream address;
            address << "tcp://*:" << nPort;
            std::cout << "RPC Listener at " << address.str() << std::endl;
            m_pRpcSocket->bind( address.str().c_str() );
            m_mHosts[ address.str() ] = m_pRpcSocket;
            // run listener in a different thread...
            // how to handle data synch tho????????
        }


        ~Node()
        {
        }


        bool Publish(
                std::string sTopic,             //< Input:
                unsigned int nPort              //< Input: for now port, later it is random
                )
        {
            std::map < std::string, zmq::socket_t* >::iterator Sock;

            // check if socket is already open for this topic
            Sock = m_mTopics.find( sTopic );
            if( Sock != m_mTopics.end() ) {
                // socket is already open, return false
                return false;
            } else {
                // no socket open.. lets open a new one

                // check if port is already in use
                std::ostringstream address;
                address << "tcp://*:" << nPort;
                Sock = m_mHosts.find( address.str() );
                if( Sock != m_mHosts.end() ) {
                    // port is in use, return false
                    return false;
                }

                // create socket
                zmq::socket_t* pSock = new zmq::socket_t( *m_pContext, ZMQ_PUB );
                try {
                    std::cout << "Publishing topic " << sTopic << " on " << address.str() << std::endl;
                    pSock->bind(address.str().c_str());
                }
                catch( zmq::error_t error ) {
                    // oops, an error occurred lets rollback
                    delete pSock;
                    return false;
                }
                m_mHosts[ address.str() ] = pSock;
                m_mTopics[ sTopic ] = pSock;
                return true;
            }
        }


        bool Write(
                std::string sTopic,             //< Input:
                google::protobuf::Message& Msg  //< Input:
                )
        {
            std::map < std::string, zmq::socket_t* >::iterator Sock;

            // check if socket is already open for this topic
            Sock = m_mTopics.find( sTopic );
            if( Sock == m_mTopics.end() ) {
                // no socket found
                return false;
            } else {
                zmq::socket_t* pSock = (*Sock).second;
                zmq::message_t ZmqMsg( Msg.ByteSize() );
                if( !Msg.SerializeToArray( ZmqMsg.data(), Msg.ByteSize() ) ) {
                    // error serializing protobuf to ZMQ message
                    return false;
                }
                return pSock->send( ZmqMsg );
            }
        }


        bool Subscribe(
                const std::string& sTopic,      //< Input:
                const std::string& sHost        //< Input: for now, host info.. later, just the topic
                )
        {
            std::map < std::string, zmq::socket_t* >::iterator Sock;

            // check if socket is already open for this topic
            Sock = m_mTopics.find( sTopic );
            if( Sock != m_mTopics.end() ) {
                // subscription for that topic already exists
                return false;
            } else {
                // check if host+port is already in use
                Sock = m_mHosts.find( sHost );
                if( Sock != m_mHosts.end() ) {
                    // port is in use, return false
                    return false;
                }
                // create socket
                zmq::socket_t* pSock = new zmq::socket_t( *m_pContext, ZMQ_SUB );

                // lets connect using the socket
                try {
                    pSock->setsockopt( ZMQ_SUBSCRIBE, NULL, 0 );
                    pSock->connect( ("tcp://" + sHost).c_str() );
                }
                catch( zmq::error_t error ) {
                    // oops, an error occurred lets rollback
                    delete pSock;
                    return false;
                }
                m_mHosts[ sHost ] = pSock;
                m_mTopics[ sTopic ] = pSock;
                return true;
            }
        }


        bool Read(
                std::string sTopic,             //< Input:
                google::protobuf::Message& Msg  //< Input:
                )
        {
            std::map < std::string, zmq::socket_t* >::iterator Sock;

            // check if socket is already open for this topic
            Sock = m_mTopics.find( sTopic );
            if( Sock == m_mTopics.end() ) {
                // no socket found
                return false;
            } else {
                zmq::socket_t* pSock = (*Sock).second;
                zmq::message_t ZmqMsg;
                if( !pSock->recv( &ZmqMsg, ZMQ_NOBLOCK ) ) {
                    return false;
                }
                if( !Msg.ParseFromArray( ZmqMsg.data(), ZmqMsg.size() ) ) {
                    return false;
                }
                return true;
            }
        }


        bool RegisterService(
                const std::string& sFuncName,                   //< Input: name of function we will call
                void (*pFunc)(std::string&,std::string&)        //< Input: Pointer to function we will call
                )
        {
            std::map < std::string, FuncPtr >::iterator Func;

            // check if func is already registered
            Func = m_mFuncTable.find( sFuncName );
            if( Func != m_mFuncTable.end() ) {
                return false;
            } else {
                m_mFuncTable[ sFuncName ] = pFunc;
                return true;
            }
        }


        bool ProvideService( )
        {
            // wait for request
            zmq::message_t ZmqReq;
            if( !m_pRpcSocket->recv( &ZmqReq ) ) {
                // error receiving
                return false;
            }
            // obtain "header" which contains function name
            unsigned char FuncNameSize;
            memcpy( &FuncNameSize, ZmqReq.data(), sizeof(FuncNameSize) );
            std::string FuncName( (char*)(ZmqReq.data()) + sizeof(FuncNameSize), FuncNameSize );

            // prepare reply message
            unsigned int PbOffset = sizeof(FuncNameSize) + FuncNameSize;
            unsigned int PbByteSize = ZmqReq.size() - PbOffset;

            // prepare parameters for function call
            std::string MsgReq( (char*)(ZmqReq.data()) + PbOffset, PbByteSize );
            std::string MsgRep;

            // look-up function
            std::map < std::string, FuncPtr >::iterator Func;

            Func = m_mFuncTable.find( FuncName );
            if( Func != m_mFuncTable.end() ) {
                // function exists... call it
                (*(*Func).second)( MsgReq, MsgRep );
            }

            // prepare return message
            zmq::message_t ZmqRep( MsgRep.size() );

            memcpy( ZmqRep.data(), MsgRep.c_str(), MsgRep.size() );

            // send reply
            return m_pRpcSocket->send( ZmqRep );
        }


        bool Call(
                const std::string& sHost,                   //< Input:
                const std::string& sFuncName,               //< Input:
                const google::protobuf::Message& MsgReq,    //< Input:
                google::protobuf::Message& MsgRep           //< Output:
                )
        {
            std::map < std::string, zmq::socket_t* >::iterator Sock;
            zmq::socket_t* pSock;


            // check if socket is already open for this host
            Sock = m_mHosts.find( sHost );
            if( Sock != m_mHosts.end() ) {
                // socket is already open, lets use it
                pSock = (*Sock).second;
            } else {
                // socket is not open, lets open one
                pSock = new zmq::socket_t( *m_pContext, ZMQ_REQ );

                // lets connect using the socket
                try {
                    pSock->connect( ("tcp://" + sHost).c_str() );
                }
                catch( zmq::error_t error ) {
                    // oops, an error occurred lets rollback
                    delete pSock;
                    return false;
                }
                m_mHosts[ sHost ] = pSock;
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
            if( !pSock->send( ZmqReq ) ) {
                // error sending request
                return false;
            }

            // wait reply
            zmq::message_t ZmqRep;
            if( !pSock->recv( &ZmqRep ) ) {
                // error receiving
                return false;
            }
            if( !MsgRep.ParseFromArray( ZmqRep.data(), ZmqRep.size() ) ) {
                // bad protobuf format
                return false;
            }
            return true;
        }


    private:

        zmq::context_t*                         m_pContext;     // global context

        std::map< std::string, zmq::socket_t* > m_mHosts;       // map of hosts + endpoints

        // Variables for Topics (PubSubs)
        std::map< std::string, zmq::socket_t* > m_mTopics;      // map of topics + endpoints

        // Variables for Services
        zmq::socket_t*                          m_pRpcSocket;   // global RPC socket (port 1337)
        std::map< std::string, FuncPtr >        m_mFuncTable;   // map of topics + functions

    };

}

#endif