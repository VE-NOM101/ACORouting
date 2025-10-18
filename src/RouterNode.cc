/*
 * RouterNode.cc
 *
 *  Created on: Oct 18, 2025
 *      Author: Choyan Barua
 */




// RouterNode.cc
#include "RouterNode.h"

Define_Module(RouterNode);

void RouterNode::initialize()
{
    address = par("address");
    numPorts = gateSize("port");

    EV << "Router " << address << " initialized with " << numPorts << " ports\n";

    // Print neighbor information
    for (int i = 0; i < numPorts; i++) {
        cGate *outGate = gate("port$o", i);
        if (outGate->isConnected()) {
            cChannel *channel = outGate->getTransmissionChannel();
            cModule *neighborModule = outGate->getPathEndGate()->getOwnerModule();
            EV << "  Port " << i << " connected to " << neighborModule->getName()
               << "[" << neighborModule->getIndex() << "]";

            if (channel) {
                cDatarateChannel *drChannel = check_and_cast<cDatarateChannel *>(channel);
                EV << " (delay: " << drChannel->getDelay() << ")";
            }
            EV << "\n";
        }
    }
}

void RouterNode::handleMessage(cMessage *msg)
{
    EV << "Router " << address << " received message: " << msg->getName() << "\n";

    // For now, just delete the message
    delete msg;
}
