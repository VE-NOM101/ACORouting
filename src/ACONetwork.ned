// AcoNetwork.ned
package acorouting;

simple RouterNode
{
    parameters:
        int address;
        int routingMethod = default(0);
        @display("i=abstract/router;is=l");
    gates:
        inout port[];
        input directIn @directIn;
}

simple Device
{
    parameters:
        int address;
        int connectedRouter;
        @display("i=device/pc;is=l");
    gates:
        inout port;
}

// ═══════════════════════════════════════════════════════════
// LARGE NETWORK: 6 Routers, 3 Devices
// ═══════════════════════════════════════════════════════════
network AcoLargeNetwork
{
    parameters:
        @display("bgb=900,700;bgg=100,1,grey95");

    submodules:
        router[6]: RouterNode {
            address = index;
        }

        device[3]: Device {
            address = 100 + index;
        }

    connections:
        router[0].port++ <--> {delay = 10ms; datarate = 1Mbps; @display("ls=blue,2");} <--> router[1].port++;
        router[0].port++ <--> {delay = 15ms; datarate = 1Mbps; @display("ls=blue,2");} <--> router[2].port++;
        router[1].port++ <--> {delay = 12ms; datarate = 1Mbps; @display("ls=blue,2");} <--> router[2].port++;
        router[1].port++ <--> {delay = 20ms; datarate = 1Mbps; @display("ls=blue,2");} <--> router[3].port++;
        router[2].port++ <--> {delay = 18ms; datarate = 1Mbps; @display("ls=blue,2");} <--> router[4].port++;
        router[3].port++ <--> {delay = 10ms; datarate = 1Mbps; @display("ls=blue,2");} <--> router[4].port++;
        router[3].port++ <--> {delay = 25ms; datarate = 1Mbps; @display("ls=blue,2");} <--> router[5].port++;
        router[4].port++ <--> {delay = 15ms; datarate = 1Mbps; @display("ls=blue,2");} <--> router[5].port++;

        device[0].port <--> {delay = 2ms; datarate = 1Mbps; @display("ls=green,2");} <--> router[0].port++;
        device[1].port <--> {delay = 2ms; datarate = 1Mbps; @display("ls=green,2");} <--> router[3].port++;
        device[2].port <--> {delay = 2ms; datarate = 1Mbps; @display("ls=green,2");} <--> router[5].port++;
}

// ═══════════════════════════════════════════════════════════
// SMALL NETWORK: 3 Routers, 3 Devices (CLEAN LAYOUT)
// ═══════════════════════════════════════════════════════════
network AcoSmallNetwork
{
    parameters:
        @display("bgb=800,600;bgg=100,1,grey95");

    submodules:
        router[3]: RouterNode {
            address = index;
        }

        device[3]: Device {
            address = 100 + index;
        }

    connections:
        // Router connections (triangle topology)
        router[0].port++ <--> {delay = 25ms; datarate = 1Mbps; @display("ls=blue,3");} <--> router[1].port++;
        router[0].port++ <--> {delay = 10ms; datarate = 1Mbps; @display("ls=blue,3");} <--> router[2].port++;
        router[1].port++ <--> {delay = 5ms; datarate = 1Mbps; @display("ls=blue,3");} <--> router[2].port++;

        // Device connections
        device[0].port <--> {delay = 2ms; datarate = 1Mbps; @display("ls=green,2");} <--> router[0].port++;
        device[1].port <--> {delay = 2ms; datarate = 1Mbps; @display("ls=green,2");} <--> router[2].port++;
        device[2].port <--> {delay = 2ms; datarate = 1Mbps; @display("ls=green,2");} <--> router[1].port++;
}
