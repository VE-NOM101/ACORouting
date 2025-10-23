/*
 * Device.h
 *
 *  Created on: Oct 18, 2025
 *      Author: Choyan Barua
 */

// Device.h


#ifndef DEVICE_H
#define DEVICE_H

#include <omnetpp.h>
#include "AcoMessages_m.h"

using namespace omnetpp;

class Device : public cSimpleModule
{
private:
    int address;
    int connectedRouter;

    static int packetsReceived;  // Count successful deliveries
    static int packetsSent;      // Count sent packets

    void sendDataPacket();

protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
};

#endif

