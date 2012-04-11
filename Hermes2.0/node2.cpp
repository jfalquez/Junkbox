#include "Node.h"
#include "Messages.pb.h"

#include <stdio.h>

int main() {

    rpg::Node MyNode("Node2");

    // subscribe to Node1's topic
    if( MyNode.Subscribe( "Node1/Node1Topic" ) == false ) {
        printf("Error subscribing to topic.\n");
    }

    if( MyNode.Publish( "Node2Topic" ) == false ) {
        printf("Error subscribing to topic.\n");
    }


    unsigned int nCount = 0;

    while(1) {

        Msg mMsg;

        MyNode.Read( "Node1/Node1Topic", mMsg );
        printf("Got %s.\n", mMsg.value().c_str());

        nCount++;
        if( nCount == 5 ) {
            printf("--- Sending RPC message! ---\n");
            mMsg.set_value( "Bye!" );
            MyNode.Call( "Node1", "Node1Func", mMsg, mMsg );
        }

        sleep(1);
    }

    return 0;
}