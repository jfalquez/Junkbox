#ifndef _NODE_H_
#define _NODE_H_

#include <map>
#include <utility>
#include <string>
#include <sstream>
#include <iostream> // not needed

#include <zmq.hpp>

#include <google/protobuf/message.h>

typedef void(*f)(google::protobuf::Message&,google::protobuf::Message&) FuncPtr;

namespace rpg
{

    class Node
    {        
        enum EndpointType
        {
            EP_PUB = 0,
            EP_SUB = 1,
            EP_REQ = 2,
            EP_REP = 3
        };
        
        struct Endpoint {
            std::string                 m_sHost;
            unsigned int                m_nPort;
            EndpointType                m_eType;
            zmq::socket_t*              m_pSocket;
            std::string                 m_sPbTypeName;
            void                        (*m_pFunc)(google::protobuf::Message&,google::protobuf::Message&);
        };

    public:
        
        Node()
        {
            m_pContext = new zmq::context_t(1);
            // start RPC thread too...
        }


        ~Node()
        {
            std::map < std::string, Endpoint* >::iterator it;

            for( it = m_mEndpoints.begin(); it != m_mEndpoints.end(); it++ ) {
                delete (*it).second->m_pSocket;
                delete (*it).second;
                m_mEndpoints.erase( it );
            }
        }

        bool Publish( unsigned int nPort,
                      google::protobuf::Message& Msg
                    )
        {
            std::string sKey = _GenerateKey( EP_PUB, nPort );
            Endpoint *pEP = _FindEndpoint( sKey );
            if( pEP == NULL ) {
                // ok, no publisher is running on that port
                
                // lets check if there is already a publisher for that protobuf
                std::map < std::string, Endpoint* >::iterator ep;
                ep = m_mProtobufs.find( "PUB" + Msg.GetTypeName() );
            
                if( ep != m_mProtobufs.end() ) {
                    // oops, someone is already publishing that protobuf.. no need to have another!
                    return false;
                } else {
                    pEP = new Endpoint;
                    pEP->m_eType = EP_PUB;
                    pEP->m_sHost = "*";
                    pEP->m_nPort = nPort;
                    pEP->m_pSocket = new zmq::socket_t( *m_pContext, ZMQ_PUB );
                    pEP->m_sPbTypeName = Msg.GetTypeName();
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
                    m_mProtobufs["PUB" + pEP->m_sPbTypeName] = pEP;
                    return true;
                }
            } else {
                // a publisher already exists on that port
                return false;
            }
        }
        
        bool Write( google::protobuf::Message& Msg )
        {
            std::map < std::string, Endpoint* >::iterator ep;

            ep = m_mProtobufs.find( "PUB" + Msg.GetTypeName() );
            
            if( ep == m_mProtobufs.end() ) {
                // no endpoint found by that protobuf definition
                return false;
            } else {
                Endpoint* pEP = (*ep).second;
                zmq::message_t ZmqMsg( Msg.ByteSize() );
                if( !Msg.SerializeToArray( ZmqMsg.data(), Msg.ByteSize() ) ) {
                    // error serializing protobuf to ZMQ message
                    return false;
                }
                return pEP->m_pSocket->send( ZmqMsg );
            }
        }


        bool Subscribe( const std::string& sHost,
                        unsigned int nPort,
                        google::protobuf::Message& Msg
                      )
        {
            std::string sKey = _GenerateKey( EP_SUB, nPort, sHost );
            Endpoint *pEP = _FindEndpoint( sKey );
            if( pEP == NULL ) {
                // endpoint for that port+host does not exists... good!
                pEP = new Endpoint;
                pEP->m_eType = EP_SUB;
                pEP->m_sHost = sHost;
                pEP->m_nPort = nPort;
                pEP->m_sPbTypeName = Msg.GetTypeName();

                // now lets check if there is a subscription with that same protobuf open
                // if so, we can just share socket instead of creating another one
                std::map < std::string, Endpoint* >::iterator ep;
                ep = m_mProtobufs.find( "SUB" + Msg.GetTypeName() );
            
                if( ep == m_mProtobufs.end() ) {
                    // no subscription found, so lets create a new socket for it
                    pEP->m_pSocket = new zmq::socket_t( *m_pContext, ZMQ_SUB );
                } else {
                    // a subscription was found, so lets use same socket!
                    pEP->m_pSocket = (*ep).second->m_pSocket;
                }
                // lets connect using the socket
                try {
                    std::ostringstream address;
                    address << "tcp://" << sHost << ":" << nPort;
                    std::cout << address.str().c_str() << std::endl;
                    pEP->m_pSocket->setsockopt(ZMQ_SUBSCRIBE, NULL, 0);
                    pEP->m_pSocket->connect(address.str().c_str());
                }
                catch( zmq::error_t error ) {
                    // oops, an error occurred lets rollback
                    if( ep == m_mProtobufs.end() ) {
                        delete pEP->m_pSocket;
                    }
                    delete pEP;
                    return false;
                }
                m_mEndpoints[sKey] = pEP;
                if( ep == m_mProtobufs.end() ) {
                    m_mProtobufs["SUB" + pEP->m_sPbTypeName] = pEP;
                }
                return true;
            } else {
                // a subscription already exists for that port+host
                return false;
            }
        }

        
        bool Read( google::protobuf::Message& Msg )
        {
            std::map < std::string, Endpoint* >::iterator ep;

            ep = m_mProtobufs.find( "SUB" + Msg.GetTypeName() );
            
            if( ep == m_mProtobufs.end() ) {
                return false;
            } else {
                Endpoint* pEP = (*ep).second;
                zmq::message_t ZmqMsg;
                if( !pEP->m_pSocket->recv( &ZmqMsg, ZMQ_NOBLOCK ) ) {
                    return false;
                }
                if( !Msg.ParseFromArray( ZmqMsg.data(), ZmqMsg.size() ) ) {
                    return false;
                }
                return true;
            }
        }


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
        
        unsigned int Count() { return m_mEndpoints.size(); }

        // this is to test things
        unsigned int Protobufs() { return m_mProtobufs.size(); }

        
    private:
        
        std::string _GenerateKey( EndpointType eType,
                                  unsigned int nPort,
                                  std::string sHost = ""
                                )
        {
            std::ostringstream Key;
            Key << eType << nPort;
            if( sHost.empty() == false ) {
                Key << sHost;
            }
            return Key.str();
        }
        
        
        Endpoint* _FindEndpoint( std::string Key )
        {
            std::map < std::string, Endpoint* >::iterator ep;

            ep = m_mEndpoints.find( Key );
            if( ep != m_mEndpoints.end() ) {
                return (*ep).second;
            } else {
                return NULL;
            }
        }

        Endpoint* _FindEndpoint( EndpointType eType,
                                 unsigned int nPort,
                                 std::string sHost = ""
                               )
        {
            return _FindEndpoint( _GenerateKey( eType, nPort, sHost ) );
        }

/*		
	node n;

	n.Subscribe( host, port, pb );

	n.CallService( host1, port, "Func1", req, rep );
	n.CallService( host1, port, "Func2", req, rep );
	n.CallService( host2, port, "Func", req, rep );

	while(1){
             n.Read( pb );
        }

        ///////////////// 

	void f( req, rep )
	{

	}

	node n( host );

	n.Publish( port, pb );
	n.ProvidesService( port, "Func1", f1, req, rep );
	n.ProvidesService( port, "Func2", f2, req, rep );
*/




    private:
        
        zmq::context_t*                         m_pContext;     // global context
        std::map< std::string, Endpoint* >     m_mEndpoints;   // map of endpoints by type+port+host
        std::map< std::string, Endpoint* >     m_mProtobufs;   // map of endpoints by protobuf
        std::map< std::string,FuncPtr >         m_mFuncTable;

    };

}

#endif
