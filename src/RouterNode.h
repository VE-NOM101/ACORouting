/*
 * RouterNode.h
 *
 *  Created on: Oct 18, 2025
 *      Author: Choyan Barua
 */

// RouterNode.h
#ifndef ROUTERNODE_H
#define ROUTERNODE_H

#include <omnetpp.h>

using namespace omnetpp;

class RouterNode : public cSimpleModule
{
private:
    int address;
    int numPorts;

protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
};

#endif
