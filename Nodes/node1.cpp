#include "Testmsgs.pb.h"
#include "Node.h"


#include <stdio.h>

void TestFunc( Professor& X, Student& Y )
{
    printf("Professor %s owns!\n", X.name().c_str());
    Y.set_name("Pitufo");
    Y.set_id(2);
}


int main() {

    rpg::Node Test(5001);

    bool n;

    Staff SysAdmin;
    SysAdmin.set_id(1);
    SysAdmin.set_name("Pedro");
    SysAdmin.set_email("pedro@failboat.com");
    n = Test.Publish("LeftLeg", 5002);
    printf("1st publisher: %d\n",n);

    n = Test.Register( "Owns", &TestFunc );

    while(1) {
        n = Test.Write("LeftLeg", SysAdmin);
        printf("Sending[%d] %s - %s.\n", n, SysAdmin.name().c_str(),SysAdmin.email().c_str());
        sleep(1);
    }
    return 0;
}

#if 0
#include "Testmsgs.pb.h"
#include <iostream>
#include <zmq.hpp>

#include <google/protobuf/message.h>
using namespace std;
using namespace google::protobuf;

/*
template <class ReqType, class RepType>
struct FunctionHandler
{
    RepType* (*m_pFunc)( ReqType* );
//    static zmq::message_t& (*m_pEncodeFunc)( const Rep_type& );
//    static Req_type (*m_pDecodeFunc)( const zmq::message_t& );
};

template <class T >
static T DecodeFunc( const zmq::message_t& ZmqMsg )
{
    T t;
    t.ParseFromArray( ZmqMsg.data(), ZmqMsg.size() );
    return t;
}

template <class T >
static zmq::message_t DecodeFunc( const T& t )
{
    zmq::message_t msg( t.ByteSize() );
    t.SerialToArray( msg.data(), msg.size() );
    return msg;
}
*/


//typedef Message(*FuncPtr)(Message&);
typedef void(*FuncPtr)(Message&,Message&);
typedef Message*(*ParsePtr)(zmq::message_t* );

map < string, FuncPtr >      mFuncTable;
map < string, ParsePtr >     mParseTable;
map < string, Message* >    mReqPbTable;
map < string, Message* >    mRepPbTable;



template <class T>
struct ParseFuncWrapper
{
    static T* AllocAndParseFunc( zmq::message_t *msg )
    {
        T* pb = new T;
        if( msg ){
            pb->ParseFromArray( msg->data(), msg->size() );
        }
        return pb;
    }
};


template <class Req, class Rep>
struct AllocPBs
{
    static Req* AllocateReq()
    {
        Req* pb = new Req;
        return pb;
    }

    static Rep* AllocateRep()
    {
        Rep* pb = new Rep;
        return pb;
    }
};


//Student TestFunc( Professor&  )
//{
//    Professor y;
//    cout << y.GetTypeName();
//    Student s;
//    return s;
//}

void TestFunc( Professor& y, Student& s )
{
    cout << y.GetTypeName() << " and  " << s.GetTypeName() << endl;
    s.set_id(3);
}


template <class Req, class Rep>
//void Register( const std::string& sName, Rep(*pFunc)(Req&) )
void Register( const std::string& sName, void(*pFunc)(Req&,Rep&) )
{
//    mParseTable[sName] = (ParsePtr)ParseFuncWrapper<Req>::AllocAndParseFunc;
    mFuncTable[sName]  = (FuncPtr)pFunc;
    mReqPbTable[sName] = AllocPBs<Req,Rep>::AllocateReq();
    mRepPbTable[sName] = AllocPBs<Req,Rep>::AllocateRep();
}




int main()
{


//    A.m_pFunc = TestFunc;
//    mFuncTable["Test"] = (FuncPtr)TestFunc;
//    mParseTable["Test"] = (ParsePtr)MyParseFunc;

    Register( "Test", TestFunc );

    ParsePtr f = mParseTable["Test"];

    zmq::message_t* msg = 0;
//    Message* req = (*f)( msg );

//    mReqPbTable["Test"]->ParseFromArray(msg->data(), msg->size());

//    mReqPbTable["Test"] = new Professor();

    FuncPtr fun = mFuncTable["Test"];

//    Message* rep = new Message();

//    Message* rep = (*fun)( *req );
//    Message* rep = mEncodeTable["Test"];
//    (*fun)( *req,*(mRepPbTable["Test"]) );
    (*fun)( *(mReqPbTable["Test"]),*(mRepPbTable["Test"]) );

//    delete req;

//    map< string, Message* > m;
//    m["123"] = new Professor;
//    cout << m["123"]->GetTypeName() << endl;


//    Message& pb = X;


//    mFuncTables["Test"] = (FunctionHandler<Message,Message>*)&A;

//    Professor X;
//    (*mFuncTables["Test"]->m_pFunc)(X);

//    map < string, Message* >      mFuncTables;

//    mFuncTables["Test"] = &X;


    return 0;

}
#endif