// AcoNetwork.ned
package acorouting;

// Simple router node module
simple RouterNode
{
    parameters:
        int address;  // unique router ID
        @display("i=abstract/router");
    gates:
        inout port[];  // variable number of ports
        input directIn @directIn;  // NEW: dedicated gate for sendDirect()
}

// Device (host) module
simple Device
{
    parameters:
        int address;  // unique device ID
        int connectedRouter;  // router this device connects to
        @display("i=device/pc");
    gates:
        inout port;  // single connection to router
}

// Network module defining topology
network AcoSimpleNetwork
{
    parameters:
        @display("bgb=800,600");

    submodules:
        router[6]: RouterNode {
            address = index;
            @display("p=123,458");
        }

        device[3]: Device {
            address = 100 + index;
            @display("p=605,116");
        }

    connections:
        // Create a simple topology: 6 routers in a mesh
        // Router connections (bidirectional)
        router[0].port++ <--> {  delay = 10ms; datarate = 1Mbps; } <--> router[1].port++;
        router[0].port++ <--> {  delay = 15ms; datarate = 1Mbps; } <--> router[2].port++;
        router[1].port++ <--> {  delay = 12ms; datarate = 1Mbps; } <--> router[2].port++;
        router[1].port++ <--> {  delay = 20ms; datarate = 1Mbps; } <--> router[3].port++;
        router[2].port++ <--> {  delay = 18ms; datarate = 1Mbps; } <--> router[4].port++;
        router[3].port++ <--> {  delay = 10ms; datarate = 1Mbps; } <--> router[4].port++;
        router[3].port++ <--> {  delay = 25ms; datarate = 1Mbps; } <--> router[5].port++;
        router[4].port++ <--> {  delay = 15ms; datarate = 1Mbps; } <--> router[5].port++;

        // Device connections
        device[0].port <--> {  delay = 2ms; datarate = 1Mbps; } <--> router[0].port++;
        device[1].port <--> {  delay = 2ms; datarate = 1Mbps; } <--> router[3].port++;
        device[2].port <--> {  delay = 2ms; datarate = 1Mbps; } <--> router[5].port++;
}
