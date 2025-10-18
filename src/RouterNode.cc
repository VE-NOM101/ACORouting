/*
 * RouterNode.cc
 *
 *  Created on: Oct 18, 2025
 *      Author: Choyan Barua
 */



// RouterNode.cc
#include "RouterNode.h"

Define_Module(RouterNode);

// Static member initialization
std::map<std::pair<int, int>, double> RouterNode::globalCostTable;
std::map<std::pair<int, int>, double> RouterNode::pheromoneTable;
int RouterNode::routersInitialized = 0;
int RouterNode::totalRouters = 6;

// ACO Parameters
const double RouterNode::ALPHA = 1.0;  // pheromone importance
const double RouterNode::BETA = 2.0;   // heuristic importance
const double RouterNode::RHO = 0.1;    // evaporation rate (10%)
const double RouterNode::INITIAL_PHEROMONE = 1.0;
const double RouterNode::Q = 100.0;    // pheromone deposit factor

void RouterNode::initialize()
{
    address = par("address");
    numPorts = gateSize("port");

    EV << "Router " << address << " initialized with " << numPorts << " ports\n";

    // Discover neighbors from network topology
    discoverNeighbors();
    printNeighbors();

    // Add this router's neighbors to global table
    addNeighborsToGlobalTable();

    // Increment counter
    routersInitialized++;

    // Print table immediately after last router initializes
    if (routersInitialized == totalRouters) {
        EV << "\nAll routers initialized. Building ACO data structures...\n\n";
        printGlobalCostTable();
        initializePheromones();
        printPheromoneTable();
    }
}

void RouterNode::discoverNeighbors()
{
    neighbors.clear();

    for (int i = 0; i < numPorts; i++) {
        cGate *outGate = gate("port$o", i);
        if (outGate->isConnected()) {
            cModule *neighborModule = outGate->getPathEndGate()->getOwnerModule();

            // Only consider router neighbors (not devices)
            if (strcmp(neighborModule->getName(), "router") == 0) {
                NeighborInfo info;
                info.neighborAddress = neighborModule->par("address");
                info.gateIndex = i;

                // Get link cost from channel delay (in milliseconds)
                cChannel *channel = outGate->getTransmissionChannel();
                if (channel) {
                    cDatarateChannel *drChannel = check_and_cast<cDatarateChannel *>(channel);
                    info.linkCost = drChannel->getDelay().dbl() * 1000.0;  // convert to ms
                } else {
                    info.linkCost = 1.0;  // default cost
                }

                neighbors.push_back(info);

                EV << "  Neighbor found: Router " << info.neighborAddress
                   << " via port " << info.gateIndex
                   << " (cost: " << info.linkCost << " ms)\n";
            }
        }
    }
}

void RouterNode::addNeighborsToGlobalTable()
{
    // Add costs for all neighbors of this router
    for (const auto &neighbor : neighbors) {
        std::pair<int, int> link(address, neighbor.neighborAddress);
        globalCostTable[link] = neighbor.linkCost;

        EV << "Adding to global table: " << address << " -> "
           << neighbor.neighborAddress << " = " << neighbor.linkCost << " ms\n";
    }
}

void RouterNode::initializePheromones()
{
    EV << "\n=== Initializing Pheromone Table ===\n";

    // Initialize pheromones for all links in global cost table
    for (const auto &entry : globalCostTable) {
        pheromoneTable[entry.first] = INITIAL_PHEROMONE;
    }

    EV << "Initialized " << pheromoneTable.size() << " pheromone entries with value "
       << INITIAL_PHEROMONE << "\n";
}

void RouterNode::printNeighbors()
{
    EV << "Router " << address << " neighbors: ";
    if (neighbors.empty()) {
        EV << "none";
    } else {
        for (size_t i = 0; i < neighbors.size(); i++) {
            if (i > 0) EV << ", ";
            EV << neighbors[i].neighborAddress << "(cost:" << neighbors[i].linkCost << ")";
        }
    }
    EV << "\n";
}

void RouterNode::printGlobalCostTable()
{
    EV << "\n========================================\n";
    EV << "=== GLOBAL COST TABLE (ALL LINKS) ===\n";
    EV << "========================================\n";
    EV << "Link (i->j) | Cost (ms)\n";
    EV << "------------------------\n";

    for (const auto &entry : globalCostTable) {
        EV << "  " << entry.first.first << " -> " << entry.first.second
           << "  |  " << entry.second << "\n";
    }
    EV << "------------------------\n";
    EV << "Total links: " << globalCostTable.size() << "\n";
    EV << "========================================\n\n";
}

void RouterNode::printPheromoneTable()
{
    EV << "\n========================================\n";
    EV << "=== PHEROMONE TABLE (INITIAL) ===\n";
    EV << "========================================\n";
    EV << "Link (i->j) | Pheromone | Visibility\n";
    EV << "----------------------------------------\n";

    for (const auto &entry : pheromoneTable) {
        int from = entry.first.first;
        int to = entry.first.second;
        double pheromone = entry.second;
        double visibility = getVisibility(from, to);

        EV << "  " << from << " -> " << to
           << "  |  " << pheromone
           << "  |  " << visibility << "\n";
    }
    EV << "========================================\n\n";

    EV << "ACO Parameters:\n";
    EV << "  ALPHA (pheromone importance): " << ALPHA << "\n";
    EV << "  BETA (visibility importance): " << BETA << "\n";
    EV << "  RHO (evaporation rate): " << RHO << "\n";
    EV << "  Q (deposit factor): " << Q << "\n";
    EV << "========================================\n\n";
}

double RouterNode::getVisibility(int nodeA, int nodeB)
{
    double cost = getLinkCost(nodeA, nodeB);
    if (cost > 0) {
        return 1.0 / cost;  // visibility (eta) = 1 / cost
    }
    return 0.0;
}

double RouterNode::getPheromone(int nodeA, int nodeB)
{
    std::pair<int, int> link(nodeA, nodeB);
    auto it = pheromoneTable.find(link);
    if (it != pheromoneTable.end()) {
        return it->second;
    }
    return 0.0;
}

void RouterNode::updatePheromone(int nodeA, int nodeB, double delta)
{
    std::pair<int, int> link(nodeA, nodeB);
    pheromoneTable[link] += delta;

    // Ensure pheromone doesn't go below minimum threshold
    if (pheromoneTable[link] < 0.01) {
        pheromoneTable[link] = 0.01;
    }
}

void RouterNode::evaporatePheromones()
{
    for (auto &entry : pheromoneTable) {
        entry.second *= (1.0 - RHO);

        // Ensure minimum pheromone level
        if (entry.second < 0.01) {
            entry.second = 0.01;
        }
    }
}

double RouterNode::getLinkCost(int nodeA, int nodeB)
{
    std::pair<int, int> link(nodeA, nodeB);
    auto it = globalCostTable.find(link);
    if (it != globalCostTable.end()) {
        return it->second;
    }
    return -1.0;  // no direct link
}

void RouterNode::handleMessage(cMessage *msg)
{
    EV << "Router " << address << " received message: " << msg->getName() << "\n";
    delete msg;
}
