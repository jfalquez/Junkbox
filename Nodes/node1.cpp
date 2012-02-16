#include "Testmsgs.pb.h"
#include "Node.h"


#include <stdio.h>


void TestFunc(std::string& X, std::string& Y) {
    // this is needed at the beginning of each func
    Professor MsgX, MsgY;
    MsgX.ParseFromString(X);

    printf("Professor %s owns!\n", MsgX.name().c_str());
    MsgY.set_name("Pitufo");
    MsgY.set_id(2);

    // this is needed at the end of each func
    MsgY.SerializeToString(&Y);
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

    n = Test.RegisterService( "Owns", &TestFunc );

    n = Test.ProvideService();

    while(1) {
        n = Test.Write("LeftLeg", SysAdmin);
        printf("Sending[%d] %s - %s.\n", n, SysAdmin.name().c_str(),SysAdmin.email().c_str());
        sleep(1);
    }
    return 0;
}