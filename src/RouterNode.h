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
#include <map>
#include <vector>

using namespace omnetpp;

// Structure to store neighbor information
struct NeighborInfo {
    int neighborAddress;
    int gateIndex;
    double linkCost;  // based on delay or distance
};

class RouterNode : public cSimpleModule
{
private:
    int address;
    int numPorts;
    std::vector<NeighborInfo> neighbors;

    // Centralized cost table (shared across all routers)
    static std::map<std::pair<int, int>, double> globalCostTable;
    static int routersInitialized;
    static int totalRouters;

    void discoverNeighbors();
    void addNeighborsToGlobalTable();
    void printNeighbors();
    void printGlobalCostTable();

protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;

public:
    static double getLinkCost(int nodeA, int nodeB);
    std::vector<NeighborInfo>& getNeighbors() { return neighbors; }
};

#endif


