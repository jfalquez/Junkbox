#include <stdio.h>
#include <RPG/Devices.h>

int main() {

    Hermes Test;
    Test.AddEndpoint("Test", EP_PUBLISH );
    Test.AddEndpoint("Test", EP_PUBLISH );
    Test.AddEndpoint("Test2", EP_PUBLISH );
    printf("%d nodes alive.\n",Test.GetEndpointCount());
    Test.RemoveEndpoint("Test2");
    printf("%d nodes alive.\n",Test.GetEndpointCount());    

    return 0;
}
