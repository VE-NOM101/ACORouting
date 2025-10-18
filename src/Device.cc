/*
 * Device.cc
 *
 *  Created on: Oct 18, 2025
 *      Author: Choyan Barua
 */



// Device.cc
#include "Device.h"

Define_Module(Device);

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

    // Send a test message at simulation start (only from device[0])
    if (address == 100) {
        cMessage *testMsg = new cMessage("TestMessage");
        scheduleAt(simTime() + 1.0, testMsg);
    }
}

void Device::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) {
        EV << "Device " << address << " sending test message to connected router\n";
        cMessage *outMsg = new cMessage("TestFromDevice");
        send(outMsg, "port$o");
        delete msg;
    } else {
        EV << "Device " << address << " received message: " << msg->getName() << "\n";
        delete msg;
    }
}


