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
#include "AcoMessages_m.h"
#include <map>
#include <vector>
#include <cmath>
#include <algorithm>

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

    // ACO Parameters
    static std::map<std::pair<int, int>, double> pheromoneTable;
    static const double ALPHA;  // pheromone importance
    static const double BETA;   // heuristic importance (visibility)
    static const double RHO;    // evaporation rate
    static const double INITIAL_PHEROMONE;
    static const double Q;      // pheromone deposit factor
    static const int MAX_ITERATIONS;
    static const int NUM_ANTS;  // equal to number of routers

    // ACO state
    static int currentIteration;
    static int antsCompleted;

    void discoverNeighbors();
    void addNeighborsToGlobalTable();
    void initializePheromones();
    void printNeighbors();
    void printGlobalCostTable();
    void printPheromoneTable();

    double getVisibility(int nodeA, int nodeB);
    double getPheromone(int nodeA, int nodeB);
    void updatePheromone(int nodeA, int nodeB, double delta);
    void evaporatePheromones();

    // Ant handling
    void startAcoAlgorithm();
    void launchAnts();
    void handleForwardAnt(AntMsg *ant);
    void handleBackwardAnt(AntMsg *ant);
    int selectNextHop(AntMsg *ant);
    double calculateProbability(int currentNode, int nextNode, AntMsg *ant);
    bool isNodeVisited(AntMsg *ant, int node);
    int getGateIndexToNeighbor(int neighborAddr);

protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;

public:
    static double getLinkCost(int nodeA, int nodeB);
    std::vector<NeighborInfo>& getNeighbors() { return neighbors; }
};

#endif
