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

// Enum for routing table construction methods
enum RoutingMethod {
    GREEDY_LOCAL = 0,           // Current method (greedy, local decision)
    MULTIHOP_PHEROMONE = 1,     // Multi-hop pheromone path analysis
    DIJKSTRA_PHEROMONE = 2      // Dijkstra-like with pheromone weights
};

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
    static const double ALPHA;
    static const double BETA;
    static const double RHO;
    static const double INITIAL_PHEROMONE;
    static const double Q;
    static const int MAX_ITERATIONS;
    static int NUM_ANTS;  // equal to number of routers (dynamic)

    // ACO state
    static int currentIteration;
    static int antsCompleted;

    // Routing method selection
    static int routingMethod;  // Selected routing algorithm

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

    // Routing table construction
    static std::map<std::pair<int, int>, int> routingTable;
    void buildRoutingTable();
    void printRoutingTable();

    // Multiple routing algorithms
    int getBestNextHop_GreedyLocal(int from, int to);
    int getBestNextHop_MultiHop(int from, int to);
    int getBestNextHop_Dijkstra(int from, int to);

    // Data packet forwarding
    void forwardDataPacket(DataMsg *dataMsg);
    int getRouterForDevice(int deviceAddress);


    static bool iterationScheduled;

protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;  // ADD THIS LINE

public:
    static double getLinkCost(int nodeA, int nodeB);
    std::vector<NeighborInfo>& getNeighbors() { return neighbors; }
};

#endif

