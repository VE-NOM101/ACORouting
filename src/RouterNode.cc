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
std::map<std::pair<int, int>, int> RouterNode::routingTable;
int RouterNode::routersInitialized = 0;
int RouterNode::totalRouters = 6;

// Add with other static initializations:
bool RouterNode::iterationScheduled = false;

// ACO Parameters
const double RouterNode::ALPHA = 1.0;  // pheromone importance
const double RouterNode::BETA = 2.0;   // heuristic importance
const double RouterNode::RHO = 0.1;    // evaporation rate (10%)
const double RouterNode::INITIAL_PHEROMONE = 1.0;
const double RouterNode::Q = 100.0;    // pheromone deposit factor
const int RouterNode::MAX_ITERATIONS = 5;
const int RouterNode::NUM_ANTS = 6;    // equal to number of routers

int RouterNode::currentIteration = 0;
int RouterNode::antsCompleted = 0;

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

    // Router 0 schedules ACO start (checks after all routers initialized)
    if (address == 0) {
        cMessage *startAco = new cMessage("StartACO");
        scheduleAt(simTime() + 0.5, startAco);
        EV << "\n*** Router 0: Scheduled ACO to start at t=0.5s ***\n\n";
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
    EV << "  MAX_ITERATIONS: " << MAX_ITERATIONS << "\n";
    EV << "  NUM_ANTS: " << NUM_ANTS << " (equal to number of routers)\n";
    EV << "========================================\n\n";
}

void RouterNode::startAcoAlgorithm()
{
    iterationScheduled = false;  // Reset flag for the new iteration!
       if (address == 0) {  // Only Coordinator Router 0 launches ants
           EV << "\n*****************************************\n";
           EV << "*** STARTING ACO ITERATION " << (currentIteration + 1) << " ***\n";
           EV << "*****************************************\n\n";
           antsCompleted = 0;
           launchAnts();
       }
}

void RouterNode::launchAnts()
{
    if (address != 0) return;  // Only router 0 launches ants

    EV << "Launching " << NUM_ANTS << " ants (one from each router)...\n\n";

    // Launch one ant from each router to explore paths
    cModule *network = getParentModule();
    int antId = 0;

    for (cModule::SubmoduleIterator it(network); !it.end(); ++it) {
        cModule *mod = *it;
        if (strcmp(mod->getName(), "router") == 0) {
            RouterNode *router = check_and_cast<RouterNode *>(mod);
            int srcRouter = router->address;

            // For simplicity, each ant explores to a different destination
            // Ant from router i goes to router (i+3) % totalRouters
            int destRouter = (srcRouter + 3) % totalRouters;

            if (srcRouter != destRouter) {
                AntMsg *ant = new AntMsg();
                ant->setSrcAddress(srcRouter);
                ant->setDestAddress(destRouter);
                ant->setCurrentNode(srcRouter);
                ant->setIsForwardAnt(true);
                ant->setHopCount(0);
                ant->setPathCost(0.0);
                ant->setIteration(currentIteration);

                // Initialize visited nodes array
                ant->setVisitedNodesArraySize(1);
                ant->setVisitedNodes(0, srcRouter);

                char antName[50];
                sprintf(antName, "Ant-%d-Iter%d(%d->%d)", antId++, currentIteration, srcRouter, destRouter);
                ant->setName(antName);

                EV << "Launching " << ant->getName() << "\n";

                // Send ant to the source router
                sendDirect(ant, router, "directIn");
            }
        }
    }
}

int RouterNode::getGateIndexToNeighbor(int neighborAddr)
{
    for (const auto &neighbor : neighbors) {
        if (neighbor.neighborAddress == neighborAddr) {
            return neighbor.gateIndex;
        }
    }
    return -1;
}

bool RouterNode::isNodeVisited(AntMsg *ant, int node)
{
    for (unsigned int i = 0; i < ant->getVisitedNodesArraySize(); i++) {
        if (ant->getVisitedNodes(i) == node) {
            return true;
        }
    }
    return false;
}

double RouterNode::calculateProbability(int currentNode, int nextNode, AntMsg *ant)
{
    // Get pheromone and visibility
    double tau = getPheromone(currentNode, nextNode);
    double eta = getVisibility(currentNode, nextNode);

    if (tau <= 0 || eta <= 0) return 0.0;

    // P = (tau^alpha * eta^beta)
    double probability = pow(tau, ALPHA) * pow(eta, BETA);

    return probability;
}

int RouterNode::selectNextHop(AntMsg *ant)
{
    int currentNode = ant->getCurrentNode();

    // Get all unvisited neighbors
    std::vector<int> unvisitedNeighbors;
    std::vector<double> probabilities;
    double sumProbability = 0.0;

    for (const auto &neighbor : neighbors) {
        if (!isNodeVisited(ant, neighbor.neighborAddress)) {
            double prob = calculateProbability(currentNode, neighbor.neighborAddress, ant);
            if (prob > 0) {
                unvisitedNeighbors.push_back(neighbor.neighborAddress);
                probabilities.push_back(prob);
                sumProbability += prob;
            }
        }
    }

    if (unvisitedNeighbors.empty()) {
        return -1;  // No valid next hop (dead end)
    }

    // Normalize probabilities
    for (auto &prob : probabilities) {
        prob /= sumProbability;
    }

    // Roulette wheel selection
    double randomValue = uniform(0, 1);
    double cumulativeProbability = 0.0;

    for (size_t i = 0; i < unvisitedNeighbors.size(); i++) {
        cumulativeProbability += probabilities[i];
        if (randomValue <= cumulativeProbability) {
            EV << "  Selected next hop: " << unvisitedNeighbors[i]
               << " (probability: " << probabilities[i] << ")\n";
            return unvisitedNeighbors[i];
        }
    }

    // Fallback (should not reach here)
    return unvisitedNeighbors[0];
}

void RouterNode::handleForwardAnt(AntMsg *ant)
{
    int currentNode = ant->getCurrentNode();
    int destNode = ant->getDestAddress();

    EV << "\n--- Forward Ant at Router " << address << " ---\n";
    EV << "Ant: " << ant->getName() << "\n";
    EV << "Path so far: ";
    for (unsigned int i = 0; i < ant->getVisitedNodesArraySize(); i++) {
        if (i > 0) EV << " -> ";
        EV << ant->getVisitedNodes(i);
    }
    EV << "\n";
    EV << "Total cost: " << ant->getPathCost() << " ms\n";

    if (currentNode == destNode) {
        EV << "DESTINATION REACHED! Converting to backward ant...\n\n";

        // Convert to backward ant
        ant->setIsForwardAnt(false);

        // Send back along the path (to previous node)
        int pathLength = ant->getVisitedNodesArraySize();
        if (pathLength > 1) {
            int prevNode = ant->getVisitedNodes(pathLength - 2);

            // Find the router module for previous node
            cModule *network = getParentModule();
            for (cModule::SubmoduleIterator it(network); !it.end(); ++it) {
                cModule *mod = *it;
                if (strcmp(mod->getName(), "router") == 0 && mod->par("address").intValue() == prevNode) {
                    sendDirect(ant->dup(), mod, "directIn");
                    delete ant;
                    return;
                }
            }
        } else {
            // Path too short, count as completed
            antsCompleted++;

            if (antsCompleted >= NUM_ANTS && !iterationScheduled) {
                EV << "\n*** ALL ANTS COMPLETED ITERATION " << (currentIteration + 1) << " ***\n";
                iterationScheduled = true;  // Prevent other routers from scheduling

                // Evaporate pheromones
                evaporatePheromones();

                currentIteration++;

                if (currentIteration < MAX_ITERATIONS) {
                    // ANY router can schedule, but only first one due to flag
                    cMessage *nextIter = new cMessage("StartACO");

                    // Send the message directly to Router 0
                    cModule *network = getParentModule();
                    for (cModule::SubmoduleIterator it(network); !it.end(); ++it) {
                        cModule *mod = *it;
                        if (strcmp(mod->getName(), "router") == 0 && mod->par("address").intValue() == 0) {
                            sendDirect(nextIter, 1.0, 0.0, mod, "directIn");  // delay of 1.0 second
                            EV << "Router " << address << " scheduling next iteration on Router 0 at t="
                               << (simTime() + 1.0) << "\n\n";
                            break;
                        }
                    }
                } else {
                    // Algorithm complete - build routing table
                    EV << "\n*** ACO ALGORITHM COMPLETED ***\n";
                    printPheromoneTable();
                    buildRoutingTable();
                    printRoutingTable();
                }
            }


        }

        delete ant;
        return;
    }

    // Select next hop using ACO probability
    int nextHop = selectNextHop(ant);

    if (nextHop == -1) {
        EV << "Dead end! Discarding ant.\n\n";

        // Count this as completed (even though failed)
        antsCompleted++;

        if (antsCompleted >= NUM_ANTS && !iterationScheduled) {
            EV << "\n*** ALL ANTS COMPLETED ITERATION " << (currentIteration + 1) << " ***\n";
            iterationScheduled = true;  // Prevent other routers from scheduling

            // Evaporate pheromones
            evaporatePheromones();

            currentIteration++;

            if (currentIteration < MAX_ITERATIONS) {
                // ANY router can schedule, but only first one due to flag
                cMessage *nextIter = new cMessage("StartACO");

                // Send the message directly to Router 0
                cModule *network = getParentModule();
                for (cModule::SubmoduleIterator it(network); !it.end(); ++it) {
                    cModule *mod = *it;
                    if (strcmp(mod->getName(), "router") == 0 && mod->par("address").intValue() == 0) {
                        sendDirect(nextIter, 1.0, 0.0, mod, "directIn");  // delay of 1.0 second
                        EV << "Router " << address << " scheduling next iteration on Router 0 at t="
                           << (simTime() + 1.0) << "\n\n";
                        break;
                    }
                }
            } else {
                // Algorithm complete - build routing table
                EV << "\n*** ACO ALGORITHM COMPLETED ***\n";
                printPheromoneTable();
                buildRoutingTable();
                printRoutingTable();
            }
        }

        delete ant;
        return;
    }
    // Update ant information
    double linkCost = getLinkCost(currentNode, nextHop);
    ant->setPathCost(ant->getPathCost() + linkCost);
    ant->setHopCount(ant->getHopCount() + 1);
    ant->setCurrentNode(nextHop);

    // Add next hop to visited nodes
    int newSize = ant->getVisitedNodesArraySize() + 1;
    ant->setVisitedNodesArraySize(newSize);
    ant->setVisitedNodes(newSize - 1, nextHop);

    EV << "Moving to router " << nextHop << " (added cost: " << linkCost << " ms)\n\n";

    // Forward ant to next router
    cModule *network = getParentModule();
    for (cModule::SubmoduleIterator it(network); !it.end(); ++it) {
        cModule *mod = *it;
        if (strcmp(mod->getName(), "router") == 0 && mod->par("address").intValue() == nextHop) {
            sendDirect(ant->dup(), mod, "directIn");
            delete ant;
            return;
        }
    }

    delete ant;
}

void RouterNode::handleBackwardAnt(AntMsg *ant)
{
    EV << "\n--- Backward Ant at Router " << address << " ---\n";
    EV << "Ant: " << ant->getName() << " returning\n";
    EV << "Complete path cost: " << ant->getPathCost() << " ms\n";

    // Update pheromones along the path
    double deltaPheromone = Q / ant->getPathCost();

    for (unsigned int i = 0; i < ant->getVisitedNodesArraySize() - 1; i++) {
        int from = ant->getVisitedNodes(i);
        int to = ant->getVisitedNodes(i + 1);
        updatePheromone(from, to, deltaPheromone);
        EV << "Updated pheromone: " << from << " -> " << to << " (delta: " << deltaPheromone << ")\n";
    }

    // Check if we're back at source
    int pathLength = ant->getVisitedNodesArraySize();
    int currentPosInPath = -1;

    for (unsigned int i = 0; i < pathLength; i++) {
        if (ant->getVisitedNodes(i) == address) {
            currentPosInPath = i;
            break;
        }
    }

    if (currentPosInPath == 0) {
        // Back at source
        EV << "Ant completed journey back to source!\n\n";
        antsCompleted++;

        // Check if all ants completed
        if (antsCompleted >= NUM_ANTS && !iterationScheduled) {
            EV << "\n*** ALL ANTS COMPLETED ITERATION " << (currentIteration + 1) << " ***\n";
            iterationScheduled = true;  // Prevent other routers from scheduling

            // Evaporate pheromones
            evaporatePheromones();

            currentIteration++;

            if (currentIteration < MAX_ITERATIONS) {
                // ANY router can schedule, but only first one due to flag
                cMessage *nextIter = new cMessage("StartACO");

                // Send the message directly to Router 0
                cModule *network = getParentModule();
                for (cModule::SubmoduleIterator it(network); !it.end(); ++it) {
                    cModule *mod = *it;
                    if (strcmp(mod->getName(), "router") == 0 && mod->par("address").intValue() == 0) {
                        sendDirect(nextIter, 1.0, 0.0, mod, "directIn");  // delay of 1.0 second
                        EV << "Router " << address << " scheduling next iteration on Router 0 at t="
                           << (simTime() + 1.0) << "\n\n";
                        break;
                    }
                }
            } else {
                // Algorithm complete - build routing table
                EV << "\n*** ACO ALGORITHM COMPLETED ***\n";
                printPheromoneTable();
                buildRoutingTable();
                printRoutingTable();
            }
        }

        delete ant;
        return;
    }

    // Continue backward to previous node in path
    if (currentPosInPath > 0) {
        int prevNode = ant->getVisitedNodes(currentPosInPath - 1);

        cModule *network = getParentModule();
        for (cModule::SubmoduleIterator it(network); !it.end(); ++it) {
            cModule *mod = *it;
            if (strcmp(mod->getName(), "router") == 0 && mod->par("address").intValue() == prevNode) {
                sendDirect(ant->dup(), mod, "directIn");
                delete ant;
                return;
            }
        }
    }

    delete ant;
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
    EV << "\n=== Evaporating Pheromones (rate: " << RHO << ") ===\n";

    for (auto &entry : pheromoneTable) {
        entry.second *= (1.0 - RHO);

        // Ensure minimum pheromone level
        if (entry.second < 0.01) {
            entry.second = 0.01;
        }
    }

    EV << "Pheromone evaporation complete.\n\n";
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
    if (msg->isSelfMessage()) {
        if (strcmp(msg->getName(), "StartACO") == 0) {
            startAcoAlgorithm();
            delete msg;
        }
    } else if (dynamic_cast<AntMsg*>(msg)) {
        AntMsg *ant = check_and_cast<AntMsg*>(msg);

        if (ant->isForwardAnt()) {
            handleForwardAnt(ant);
        } else {
            handleBackwardAnt(ant);
        }
    } else if (strcmp(msg->getName(), "StartACO") == 0) {  // ADD THIS LINE
        startAcoAlgorithm();  // ADD THIS LINE
        delete msg;  // ADD THIS LINE
    } else {
        EV << "Router " << address << " received message: " << msg->getName() << "\n";
        delete msg;
    }
}


int RouterNode::getBestNextHop(int from, int to)
{
    // Find neighbor of 'from' with highest pheromone toward 'to'
    double maxProbability = -1.0;
    int bestNextHop = -1;

    cModule *network = getParentModule();
    RouterNode *fromRouter = nullptr;

    // Find the router module for 'from'
    for (cModule::SubmoduleIterator it(network); !it.end(); ++it) {
        cModule *mod = *it;
        if (strcmp(mod->getName(), "router") == 0 && mod->par("address").intValue() == from) {
            fromRouter = check_and_cast<RouterNode *>(mod);
            break;
        }
    }

    if (!fromRouter) return -1;

    // Check all neighbors of 'from'
    for (const auto &neighbor : fromRouter->neighbors) {
        int nextNode = neighbor.neighborAddress;

        // Calculate attractiveness: pheromone^alpha * visibility^beta
        double tau = getPheromone(from, nextNode);
        double eta = getVisibility(from, nextNode);
        double probability = pow(tau, ALPHA) * pow(eta, BETA);

        if (probability > maxProbability) {
            maxProbability = probability;
            bestNextHop = nextNode;
        }
    }

    return bestNextHop;
}

void RouterNode::buildRoutingTable()
{
    EV << "\n========================================\n";
    EV << "=== BUILDING CENTRALIZED ROUTING TABLE ===\n";
    EV << "========================================\n\n";

    routingTable.clear();

    // For each router pair, determine best next hop based on pheromones
    for (int src = 0; src < totalRouters; src++) {
        for (int dest = 0; dest < totalRouters; dest++) {
            if (src != dest) {
                int nextHop = getBestNextHop(src, dest);

                if (nextHop != -1) {
                    std::pair<int, int> route(src, dest);
                    routingTable[route] = nextHop;

                    EV << "Route: " << src << " -> " << dest
                       << " | Next hop: " << nextHop << "\n";
                }
            }
        }
    }

    EV << "\nRouting table built with " << routingTable.size() << " entries\n";
    EV << "========================================\n\n";
}

void RouterNode::printRoutingTable()
{
    EV << "\n========================================\n";
    EV << "=== FINAL CENTRALIZED ROUTING TABLE ===\n";
    EV << "========================================\n";
    EV << "Source | Destination | Next Hop\n";
    EV << "------------------------------------\n";

    for (int src = 0; src < totalRouters; src++) {
        for (int dest = 0; dest < totalRouters; dest++) {
            if (src != dest) {
                std::pair<int, int> route(src, dest);
                auto it = routingTable.find(route);

                if (it != routingTable.end()) {
                    EV << "   " << src << "   |      " << dest
                       << "      |    " << it->second << "\n";
                }
            }
        }
    }

    EV << "========================================\n\n";
}
