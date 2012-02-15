#include "Node.h"
#include "Testmsgs.pb.h"

#include <stdio.h>

int main() {

    rpg::Node Test(6001);
    
    bool n;
    
    Staff SysAdmin;
   
    n = Test.Subscribe("LeftLeg", "tcp://localhost:5002");
    printf("1st publisher: %d\n",n);

    n = Test.Subscribe("LeftLeg", "tcp://localhost:5002");
    printf("2nd publisher: %d\n",n);

    while(1) {
        n = Test.Read( "LeftLeg", SysAdmin );
        printf("Got[%d] %s - %s.\n", n, SysAdmin.name().c_str(),SysAdmin.email().c_str());
        sleep(1);
    }

    return 0;
}