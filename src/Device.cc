/*
 * Device.cc
 *
 *  Created on: Oct 18, 2025
 *      Author: Choyan Barua
 */



#include "Device.h"

Define_Module(Device);

// Static counters initialization
int Device::packetsReceived = 0;
int Device::packetsSent = 0;

void Device::initialize()
{
    address = par("address");
    connectedRouter = par("connectedRouter");

    EV << "Device " << address << " initialized\n";

    // Get the connected router info
    cGate *outGate = gate("port$o");
    if (outGate->isConnected()) {
        cModule *routerModule = outGate->getPathEndGate()->getOwnerModule();
        EV << "  Connected to: " << routerModule->getFullName() << "\n";
    }

    // Schedule data transmission SEQUENTIALLY after ACO completes (5 seconds each)
    // Device 100: t=10s, Device 101: t=15s, Device 102: t=20s
    double sendTime = 10.0 + (address - 100) * 5.0;

    cMessage *sendData = new cMessage("SendData");
    scheduleAt(simTime() + sendTime, sendData);

    EV << "  Will send data packet at t=" << sendTime << "s\n";
}

void Device::sendDataPacket()
{
    // Determine destination device (ring pattern)
    int destDevice = -1;

    if (address == 100) {
        destDevice = 101;  // Device 0 → Device 1
    } else if (address == 101) {
        destDevice = 102;  // Device 1 → Device 2
    } else if (address == 102) {
        destDevice = 100;  // Device 2 → Device 0
    }

    // Create data packet
    DataMsg *dataPacket = new DataMsg();
    dataPacket->setSrcAddress(address);
    dataPacket->setDestAddress(destDevice);
    dataPacket->setHopCount(0);

    // Initialize empty visited routers list
    dataPacket->setVisitedRoutersArraySize(0);

    char payload[100];
    sprintf(payload, "Hello from Device %d to Device %d", address, destDevice);
    dataPacket->setPayload(payload);

    char pktName[50];
    sprintf(pktName, "Data-%d->%d", address, destDevice);
    dataPacket->setName(pktName);

    packetsSent++;

    EV << "\n╔════════════════════════════════════════╗\n";
    EV << "║ Device " << address << " SENDING DATA (" << packetsSent << "/3)     ║\n";
    EV << "╚════════════════════════════════════════╝\n";
    EV << "Destination: Device " << destDevice << "\n";
    EV << "Payload: " << payload << "\n";
    EV << "Send time: t=" << simTime() << "s\n\n";

    // Send to connected router
    send(dataPacket, "port$o");
}

void Device::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) {
        if (strcmp(msg->getName(), "SendData") == 0) {
            sendDataPacket();
            delete msg;
        }
    } else if (DataMsg *dataMsg = dynamic_cast<DataMsg*>(msg)) {
        // Data packet received successfully!
        packetsReceived++;

        EV << "\n╔════════════════════════════════════════╗\n";
        EV << "║ ✓ SUCCESS! Device " << address << " RECEIVED (" << packetsReceived << "/3) ║\n";
        EV << "╚════════════════════════════════════════╝\n";
        EV << "From: Device " << dataMsg->getSrcAddress() << "\n";
        EV << "Payload: " << dataMsg->getPayload() << "\n";
        EV << "Hop Count: " << dataMsg->getHopCount() << " hops\n";
        EV << "Path taken: ";

        // Print the complete path
        for (unsigned int i = 0; i < dataMsg->getVisitedRoutersArraySize(); i++) {
            if (i > 0) EV << " → ";
            EV << "R" << dataMsg->getVisitedRouters(i);
        }
        EV << " → Device " << address << "\n\n";

        delete msg;
    } else {
        EV << "Device " << address << " received: " << msg->getName() << "\n";
        delete msg;
    }
}
