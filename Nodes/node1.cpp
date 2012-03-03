#include "Testmsgs.pb.h"
#include "Node.h"


#include <stdio.h>

void TestFunc( Professor& X, Student& Y )
{
    printf("Professor %s owns!\n", X.name().c_str());
    Y.set_name("Pitufo");
    Y.set_id(2);
}


int main() 
{

    rpg::Node Test;

    bool n;

    Staff SysAdmin;
    SysAdmin.set_id(1);
    SysAdmin.set_name("Pedro");
    SysAdmin.set_email("pedro@failboat.com");

    n = Test.Publish( "LeftLeg", 5002 );

    printf("1st publisher: %d\n",n);

    n = Test.Register( "Owns", &TestFunc );

    while(1) {
        n = Test.Write( "LeftLeg", SysAdmin );
        printf("Sending[%d] %s - %s.\n", n, SysAdmin.name().c_str(),SysAdmin.email().c_str());
        sleep(1);
    }
 
    return 0;
}

