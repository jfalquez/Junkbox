#include "Testmsgs.pb.h"
#include "Node.h"


#include <stdio.h>


//void TestFunc(Professor& X, Professor& Y) {
//void TestFunc(google::protobuf::Message& X, google::protobuf::Message& Y) {
//    Professor X;
//    X.ParseFromArray();
//    printf("Professor %s owns Professor %s.\n", X.name().c_str(), Y.email().c_str());
//}


int main() {

    rpg::Node Test;

    bool n;
    
    Staff SysAdmin;
    SysAdmin.set_id(1);
    SysAdmin.set_name("Pedro");
    SysAdmin.set_email("pedro@failboat.com");
    n = Test.Publish("LeftLeg", 1111);
    printf("1st publisher: %d\n",n);
    
    while(1) {
        n = Test.Write("LeftLeg", SysAdmin);
        printf("Sending[%d] %s - %s.\n", n, SysAdmin.name().c_str(),SysAdmin.email().c_str());
        sleep(1);
    }
    return 0;
}