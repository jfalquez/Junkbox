#include "Node.h"
#include "Testmsgs.pb.h"

#include <stdio.h>

int main() {

    rpg::Node Test(6001);

    bool n;

    Staff SysAdmin;

    n = Test.Subscribe("LeftLeg", "localhost:5002");
    printf("1st publisher: %d\n",n);

    Professor A,B;
    A.set_id(3);
    A.set_name("Pepe");
    Test.Call( "localhost:5001", "Owns", A, B );

    printf("I got back %s with id %d.\n",B.name().c_str(), B.id());

    while(1) {
        n = Test.Read( "LeftLeg", SysAdmin );
        printf("Got[%d] %s - %s.\n", n, SysAdmin.name().c_str(),SysAdmin.email().c_str());
        sleep(1);
    }

    return 0;
}
