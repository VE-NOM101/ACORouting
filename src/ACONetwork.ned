// AcoNetwork.ned
package acorouting;

// Simple router node module
simple RouterNode
{
    parameters:
        int address;  // unique router ID
        int routingMethod = default(0);  // 0=Greedy, 1=MultiHop, 2=Dijkstra
        @display("i=abstract/router;is=l");  // Larger router icons
    gates:
        inout port[];  // variable number of ports
        input directIn @directIn;  // dedicated gate for sendDirect()
}

// Device (host) module
simple Device
{
    parameters:
        int address;  // unique device ID
        int connectedRouter;  // router this device connects to
        @display("i=device/pc;is=l");  // Larger device icons
    gates:
        inout port;  // single connection to router
}

// Network module defining topology
network AcoSimpleNetwork
{
    parameters:
        @display("bgb=900,700;bgg=100,1,grey95");  // Larger canvas with grid

    submodules:
        // Routers with individual positions
        router[6]: RouterNode {
            address = index;
            @display("p=200,300,ring,150,150");  // Ring layout, 150px radius
        }

        // Devices positioned near their routers
        device[3]: Device {
            address = 100 + index;
        }

    connections:
        // Router connections (bidirectional) with thicker blue lines
        router[0].port++ <--> {delay = 10ms; datarate = 1Mbps; @display("ls=blue,2");} <--> router[1].port++;
        router[0].port++ <--> {delay = 15ms; datarate = 1Mbps; @display("ls=blue,2");} <--> router[2].port++;
        router[1].port++ <--> {delay = 12ms; datarate = 1Mbps; @display("ls=blue,2");} <--> router[2].port++;
        router[1].port++ <--> {delay = 20ms; datarate = 1Mbps; @display("ls=blue,2");} <--> router[3].port++;
        router[2].port++ <--> {delay = 18ms; datarate = 1Mbps; @display("ls=blue,2");} <--> router[4].port++;
        router[3].port++ <--> {delay = 10ms; datarate = 1Mbps; @display("ls=blue,2");} <--> router[4].port++;
        router[3].port++ <--> {delay = 25ms; datarate = 1Mbps; @display("ls=blue,2");} <--> router[5].port++;
        router[4].port++ <--> {delay = 15ms; datarate = 1Mbps; @display("ls=blue,2");} <--> router[5].port++;

        // Device connections (thinner green lines)
        device[0].port <--> {delay = 2ms; datarate = 1Mbps; @display("ls=green,1");} <--> router[0].port++;
        device[1].port <--> {delay = 2ms; datarate = 1Mbps; @display("ls=green,1");} <--> router[3].port++;
        device[2].port <--> {delay = 2ms; datarate = 1Mbps; @display("ls=green,1");} <--> router[5].port++;
}
