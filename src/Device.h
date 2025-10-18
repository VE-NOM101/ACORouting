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

using namespace omnetpp;

class Device : public cSimpleModule
{
private:
    int address;
    int connectedRouter;

protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
};

#endif
