#include "Testmsgs.pb.h"
#include "Node.h"


#include <stdio.h>


//void TestFunc(Professor& X, Professor& Y) {
void TestFunc(google::protobuf::Message& X, google::protobuf::Message& Y) {
    printf("Professor %s owns Professor %s.\n", X.name().c_str(), Y.email().c_str());
}

/// protoc would output this
void f( Professor& Juan, Professor& Gabe )
{
    printf("write me!\n");
}

int main() {

    rpg::Node Test;

    bool n;
    
    Staff SysAdmin;
    SysAdmin.set_name("Pedro");
    SysAdmin.set_email("pedro@failboat.com");
    SysAdmin.set_id(1);

//    n = Test.Publish( "admin", 1111 );
 
    Professor A,B;
//    n = Test.ProvideService( 2222, &TestFunc, A, B );
    n = Test.ProvideService( "func", f ); // rpc alwuas uses port 1337
 
    while(1) {
        n = Test.Write(SysAdmin);

        printf("Sending[%d] %s - %s.\n", n, SysAdmin.name().c_str(),SysAdmin.email().c_str());
        sleep(1);
    }
    return 0;
}
