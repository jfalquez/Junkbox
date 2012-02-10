#include "Node.h"
#include "Testmsgs.pb.h"

#include <stdio.h>

int main() {

    rpg::Node Test;
    
    bool n;
    
    Staff SysAdmin;
   
    n = Test.Subscribe("localhost", 1111, SysAdmin);
    printf("1st publisher: %d\n",n);

    printf("Count: %d-%d\n", Test.Count(), Test.Protobufs());

    n = Test.Subscribe("localhost", 1111, SysAdmin);
    printf("2nd publisher: %d\n",n);

    printf("Count: %d-%d\n", Test.Count(), Test.Protobufs());
    
    while(1) {
        bool n;
        n = Test.Read( SysAdmin );
        printf("Got[%d] %s - %s.\n", n, SysAdmin.name().c_str(),SysAdmin.email().c_str());
        sleep(1);
    }

    return 0;
}