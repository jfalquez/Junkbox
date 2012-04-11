#include "Messages.pb.h"
#include "Node.h"

#include <stdio.h>

std::string gText;

void SetValue( Msg& Req, Msg& Rep, void* userData )
{
    // print value we got from network
    printf("\n--- Incoming message! The value within: %s ---\n", Req.value().c_str());

    // do something
    gText = Req.value();

    // prepare reply message
    Rep.set_value("Value set!");
}

int main()
{
    gText = "Hello!";

    // initialize node
    rpg::Node MyNode("Node1");

    // set up a publisher
    if( MyNode.Publish( "Node1Topic" ) == false ) {
        printf("Error setting publisher.\n");
    }

    // set up a publisher
    if( MyNode.Register( "Node1Func", &SetValue, NULL ) == false ) {
        printf("Error registering RPC function.\n");
    }

    while(1) {
        Msg PubMsg;
        PubMsg.set_value( gText );
        if ( MyNode.Write( "Node1Topic", PubMsg ) == false ) {
            printf("Error sending message.\n");
        }
        std::cout << "." << std::flush;
        sleep(1);
    }

    return 0;
}

