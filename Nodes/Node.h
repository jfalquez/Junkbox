#ifndef _NODE_H_
#define _NODE_H_

#include <map>
#include <utility>
#include <string>
#include <sstream>

#include <zmq.hpp>

#include <google/protobuf/message.h>

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
            google::protobuf::Message*  m_pProtobuf;
            void						(*m_pFunc)(google::protobuf::Message*,google::protobuf::Message*);

        };

    public:
        
        Node();
        ~Node();

        bool Publish( unsigned int nPort,
                      google::protobuf::Message& Msg
                    )
        {
            std::string sKey = _GenerateKey( EP_PUB, "", nPort );
            Endpoint *pEP = _FindEndpoint( sKey );
            if( pEP == NULL ) {
                pEP = new Endpoint;
                pEP->m_eType = EP_PUB;
                pEP->m_nPort = nPort;
                pEP->m_pSocket = new zmq::socket_t( *m_pContext, ZMQ_PUB );
                try {
                    std::ostringstream address;
                    address << "tcp://*:" << nPort;
                    pEP->m_pSocket->bind(address.str().c_str());
                }
                catch( zmq::error_t error ) {
                    delete pEP->m_pSocket;
                    delete pEP;
                    return false;
                }
                m_mEndpoints[sKey] = pEP;
                return true;
            } else {
                return false;
            }
        }


        bool Subscribe( const std::string& sHost,
                        unsigned int nPort,
                        google::protobuf::Message& Msg
                      )
        {
            std::string sKey = _GenerateKey( EP_SUB, sHost, nPort );
            Endpoint *pEP = _FindEndpoint( sKey );
            if( pEP == NULL ) {
                pEP = new Endpoint;
                pEP->m_eType = EP_SUB;
                pEP->m_sHost = sHost;
                pEP->m_nPort = nPort;
                pEP->m_pSocket = new zmq::socket_t( *m_pContext, ZMQ_SUB );
                try {
                    std::ostringstream address;
                    address << "tcp://" << sHost << nPort;
                    pEP->m_pSocket->connect(address.str().c_str());
                }
                catch( zmq::error_t error ) {
                    delete pEP->m_pSocket;
                    delete pEP;
                    return false;
                }
                m_mEndpoints[sKey] = pEP;
                return true;
            } else {
                return false;
            }
        }

        void ProvideService( unsigned int nPort )
        {

        }

        void RequestService( const std::string& sHost,
                             unsigned int nPort,
                             google::protobuf::Message& Req,
                             google::protobuf::Message& Rep
                           )
        {

        }
        
        unsigned int Count() {
            return m_mEndpoints.size();
        }
        
    private:
        
        std::string _GenerateKey( EndpointType eType, std::string sHost, unsigned int nPort ) {
            std::ostringstream Key;
            Key << eType << nPort;
            if( sHost.empty() == false ) {
                Key << sHost;
            }
            return Key.str();
        }
        
        
        Endpoint* _FindEndpoint( std::string Key ) {
            std::map < std::string, Endpoint* >::iterator ep;

            ep = m_mEndpoints.find( Key );
            if( ep != m_mEndpoints.end() ) {
                return (*ep).second;
            } else {
                return NULL;
            }
        }

        Endpoint* _FindEndpoint( EndpointType eType, std::string sHost, unsigned int nPort ) {
            return _FindEndpoint( _GenerateKey( eType, sHost, nPort ) );
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
        
        zmq::context_t*                         m_pContext;
        std::map < std::string, Endpoint* >     m_mEndpoints;

    };

Node::Node()
{
    m_pContext = new zmq::context_t(1);;
}


Node::~Node()
{
    std::map < std::string, Endpoint* >::iterator it;

    for( it = m_mEndpoints.begin(); it != m_mEndpoints.end(); it++ ) {
        delete (*it).second->m_pSocket;
        delete (*it).second->m_pProtobuf;
        delete (*it).second;
        m_mEndpoints.erase( it );
    }
}








}

#endif