#include "Testmsgs.pb.h"
#include "Node.h"


#include <stdio.h>


//void TestFunc(Professor& X, Professor& Y) {
void TestFunc(google::protobuf::Message& X, google::protobuf::Message& Y) {
    printf("Professor %s owns Professor %s.\n", X.name().c_str(), Y.email().c_str());
}


int main() {

    rpg::Node Test;

    bool n;
    
    Staff SysAdmin;
    SysAdmin.set_name("Pedro");
    SysAdmin.set_email("pedro@failboat.com");
    SysAdmin.set_id(1);
    n = Test.Publish(1111, SysAdmin);
    printf("1st publisher: %d\n",n);
    
    Professor A,B;
    n = Test.ProvideService( 2222, &TestFunc, A, B );
    
    while(1) {
        n = Test.Write(SysAdmin);
        printf("Sending[%d] %s - %s.\n", n, SysAdmin.name().c_str(),SysAdmin.email().c_str());
        sleep(1);
    }
    return 0;
}