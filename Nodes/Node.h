#ifndef _NODE_H_
#define _NODE_H_

#include <map>
#include <utility>
#include <string>
#include <sstream>
#include <iostream> // not needed

#include <zmq.hpp>

#include <google/protobuf/message.h>

#define MASTER 127.0.0.1

//typedef void(*f)(google::protobuf::Message&,google::protobuf::Message&) FuncPtr;

namespace rpg
{

    class Node
    {        
       
        struct Endpoint {
            std::string                 m_sHost;
            zmq::socket_t*              m_pSocket;
        };

    public:
        
        Node()
        {
            // init context
            m_pContext = new zmq::context_t(1);
            
            // init RPC listener
//            m_pRpcSocket = new zmq::socket_t( *m_pContext, ZMQ_REP );
//            m_pRpcSocket->bind("tcp://*:1337");
//            m_mHosts["tcp://*:1337"] = m_pRpcSocket;
            // run listener in a different thread...
        }


        ~Node()
        {
        }


        bool Publish(
                std::string sTopic,             //< Input:
                unsigned int nPort              //< Input: for now port, later it is randomly
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
                    std::cout << "Publishing on: " << address.str() << std::endl;
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
                    pSock->connect( sHost.c_str() );
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

        
/*
        template <class A, class B>
        bool ProvideService( 
                const std::string& sFuncName,//< Input: name of function we will call
                void (*pFunc)(A&,B&) //< Input: Pointer to funciton we will call
                )
        {
            m_mFuncTable[sFuncName] = pFunc; 
        }
        
        bool ProvideService( unsigned int nPort,
                             void (*pFunc)(google::protobuf::Message&,google::protobuf::Message&),
                             google::protobuf::Message& MsgReq,
                             google::protobuf::Message& MsgRep
                            )
        {
            std::string sKey = _GenerateKey( EP_REP, nPort );
            Endpoint *pEP = _FindEndpoint( sKey );
            // create socket if necessary
            if( pEP == NULL ) {
                pEP = new Endpoint;
                pEP->m_eType = EP_REP;
                pEP->m_sHost = "*";
                pEP->m_nPort = nPort;
                pEP->m_pSocket = new zmq::socket_t( *m_pContext, ZMQ_REP );
                pEP->m_pFunc = pFunc;
                pEP->m_sPbTypeName = MsgReq.GetTypeName();
                try {
                    std::ostringstream address;
                    address << "tcp://*:" << nPort;
                    std::cout << address.str() << std::endl;
                    pEP->m_pSocket->bind(address.str().c_str());
                }
                catch( zmq::error_t error ) {
                    // oops, an error occurred lets rollback
                    delete pEP->m_pSocket;
                    delete pEP;
                    return false;
                }
                m_mEndpoints[sKey] = pEP;
            }
            // wait for request
            zmq::message_t ZmqReq;
            if( !pEP->m_pSocket->recv( &ZmqReq ) ) {
                // error receiving
                return false;
            }
            if( !MsgReq.ParseFromArray( ZmqReq.data(), ZmqReq.size() ) ) {
                // bad protobuf format
                return false;
            }
            // call function
			(*(pEP->m_pFunc))(MsgReq, MsgRep);
            // send reply
            zmq::message_t ZmqRep( MsgRep.ByteSize() );
            if( !MsgRep.SerializeToArray( ZmqRep.data(), MsgRep.ByteSize() ) ) {
                // error serializing protobuf to ZMQ message
                return false;
            }
            return pEP->m_pSocket->send( ZmqRep );
        }


        template <class A, class B>
        bool Call( 
                const std::string& sHost,
                unsigned int nPort,
                const std::string& sFuncName, //< Input:
                A& MsgReq,
                B& MsgRep
                )
        {
            std::string sKey = _GenerateKey( EP_REQ, nPort, sHost );
            Endpoint *pEP = _FindEndpoint( sKey );
            // create socket if necessary
            if( pEP == NULL ) {
                pEP = new Endpoint;
                pEP->m_eType = EP_REP;
                pEP->m_sHost = sHost;
                pEP->m_nPort = nPort;
                pEP->m_pSocket = new zmq::socket_t( *m_pContext, ZMQ_REQ );
                pEP->m_sPbTypeName = MsgReq.GetTypeName();
                try {
                    std::ostringstream address;
                    address << "tcp://" << sHost << ":" << nPort;
                    std::cout << address.str() << std::endl;
                    pEP->m_pSocket->connect(address.str().c_str());
                }
                catch( zmq::error_t error ) {
                    // oops, an error occurred lets rollback
                    delete pEP->m_pSocket;
                    delete pEP;
                    return false;
                }
                m_mEndpoints[sKey] = pEP;
                return true;
            }
            // send request
            zmq::message_t ZmqReq( MsgReq.ByteSize() );
            if( !MsgReq.SerializeToArray( ZmqReq.data(), MsgReq.ByteSize() ) ) {
                // error serializing protobuf to ZMQ message
                return false;
            }
            if( !pEP->m_pSocket->send( ZmqReq ) ) {
                // error sending request
                return false;
            }
            zmq::message_t ZmqRep;
            if( !pEP->m_pSocket->recv( &ZmqRep ) ) {
                // error receiving
                return false;
            }
            if( !MsgRep.ParseFromArray( ZmqRep.data(), ZmqRep.size() ) ) {
                // bad protobuf format
                return false;
            }
            return true;
        }
 







        bool RequestService( const std::string& sHost,
                             unsigned int nPort,
                             google::protobuf::Message& MsgReq,
                             google::protobuf::Message& MsgRep
                           )
        {
            std::string sKey = _GenerateKey( EP_REQ, nPort, sHost );
            Endpoint *pEP = _FindEndpoint( sKey );
            // create socket if necessary
            if( pEP == NULL ) {
                pEP = new Endpoint;
                pEP->m_eType = EP_REP;
                pEP->m_sHost = sHost;
                pEP->m_nPort = nPort;
                pEP->m_pSocket = new zmq::socket_t( *m_pContext, ZMQ_REQ );
                pEP->m_sPbTypeName = MsgReq.GetTypeName();
                try {
                    std::ostringstream address;
                    address << "tcp://" << sHost << ":" << nPort;
                    std::cout << address.str() << std::endl;
                    pEP->m_pSocket->connect(address.str().c_str());
                }
                catch( zmq::error_t error ) {
                    // oops, an error occurred lets rollback
                    delete pEP->m_pSocket;
                    delete pEP;
                    return false;
                }
                m_mEndpoints[sKey] = pEP;
                return true;
            }
            // send request
            zmq::message_t ZmqReq( MsgReq.ByteSize() );
            if( !MsgReq.SerializeToArray( ZmqReq.data(), MsgReq.ByteSize() ) ) {
                // error serializing protobuf to ZMQ message
                return false;
            }
            if( !pEP->m_pSocket->send( ZmqReq ) ) {
                // error sending request
                return false;
            }
            zmq::message_t ZmqRep;
            if( !pEP->m_pSocket->recv( &ZmqRep ) ) {
                // error receiving
                return false;
            }
            if( !MsgRep.ParseFromArray( ZmqRep.data(), ZmqRep.size() ) ) {
                // bad protobuf format
                return false;
            }
            return true;
        }
*/

        
    private:
        
        zmq::context_t*                         m_pContext;     // global context

        std::map< std::string, zmq::socket_t* > m_mHosts;       // map of hosts + endpoints

        // Variables for Topics (PubSubs)
        std::map< std::string, zmq::socket_t* > m_mTopics;      // map of topics + endpoints
        
        // Variables for Services
        zmq::socket_t*                          m_pRpcSocket;   // global RPC socket (port 1337)
//        std::map< std::string, FuncPtr >        m_mFuncTable;   // map of topics + functions

    };

}

#endif
