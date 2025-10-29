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

int RouterNode::totalRouters = 0;  // Will be set dynamically

bool RouterNode::iterationScheduled = false;
int RouterNode::routingMethod = GREEDY_LOCAL;  // Default method

// ACO Parameters
const double RouterNode::ALPHA = 1.0;
const double RouterNode::BETA = 2.0;
const double RouterNode::RHO = 0.1;
const double RouterNode::INITIAL_PHEROMONE = 1.0;
const double RouterNode::Q = 100.0;
const int RouterNode::MAX_ITERATIONS = 5;
int RouterNode::NUM_ANTS = 0;  // Will be set dynamically

int RouterNode::currentIteration = 0;
int RouterNode::antsCompleted = 0;

void RouterNode::initialize()
{
    address = par("address");
    numPorts = gateSize("port");


    // ═══════════════════════════════════════════════════════
      // RESET ALL STATIC VARIABLES ON FIRST ROUTER (address==0)
      // ═══════════════════════════════════════════════════════
      if (address == 0 && routersInitialized > 0) {
          // This is a RESTART - clear everything!
          EV << "*** RESETTING STATIC STATE FROM PREVIOUS RUN ***\n";
          globalCostTable.clear();
          pheromoneTable.clear();
          routingTable.clear();
          routersInitialized = 0;
          totalRouters = 0;
          NUM_ANTS = 0;
          currentIteration = 0;
          antsCompleted = 0;
          iterationScheduled = false;
      }

      // Dynamically detect how many routers in the network
      if (totalRouters == 0) {
          cModule *network = getParentModule();
          int count = 0;
          for (cModule::SubmoduleIterator it(network); !it.end(); ++it) {
              if (strcmp((*it)->getName(), "router") == 0) {
                  count++;
              }
          }
          totalRouters = count;
          NUM_ANTS = totalRouters;
          EV << "Network detected: " << totalRouters << " routers\n";
          EV << "NUM_ANTS set to: " << NUM_ANTS << "\n";
      }



    EV << "Router " << address << " initialized with " << numPorts << " ports\n";

    // Read routing method from configuration (only once)
    if (address == 0) {
        routingMethod = par("routingMethod").intValue();
        const char* methodNames[] = {"Greedy Local", "Multi-hop Pheromone", "Dijkstra with Pheromone"};
        EV << "\n*** ROUTING METHOD SELECTED: " << methodNames[routingMethod] << " ***\n\n";
    }

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

    // Router 0 schedules ACO start
    if (address == 0) {
        cMessage *startAco = new cMessage("StartACO");
        scheduleAt(simTime() + 0.5, startAco);
        EV << "\n*** Router 0: Scheduled ACO to start at t=0.5s ***\n\n";
    }
}

// Add this NEW function after initialize()
void RouterNode::finish()
{
    // Reset ALL static variables when simulation ends
    if (address == 0) {
        globalCostTable.clear();
        pheromoneTable.clear();
        routingTable.clear();
        routersInitialized = 0;
        totalRouters = 0;
        NUM_ANTS = 0;
        currentIteration = 0;
        antsCompleted = 0;
        iterationScheduled = false;

        EV << "Static variables reset for next simulation\n";
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
    EV << "=== PHEROMONE TABLE===\n";
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
    if (address != 0) return;

    EV << "Launching " << NUM_ANTS << " ants (one from each router)...\n\n";

    cModule *network = getParentModule();
    int antId = 0;

    for (cModule::SubmoduleIterator it(network); !it.end(); ++it) {
        cModule *mod = *it;
        if (strcmp(mod->getName(), "router") == 0) {
            RouterNode *router = check_and_cast<RouterNode *>(mod);
            int srcRouter = router->address;

            // Smart destination mapping that works for both 3 and 6 router networks
            int destRouter;
            if (totalRouters == 3) {
                // For 3 routers: 0→1, 1→2, 2→0 (circular)
                destRouter = (srcRouter + 1) % totalRouters;
            } else {
                // For 6 routers: use diagonal routing (0→3, 1→4, 2→5, 3→0, etc.)
                destRouter = (srcRouter + totalRouters/2) % totalRouters;
            }

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
    printPheromoneTable();
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
    } else if (DataMsg *dataMsg = dynamic_cast<DataMsg*>(msg)) {
        // Handle data packet forwarding
        forwardDataPacket(dataMsg);
    } else if (strcmp(msg->getName(), "StartACO") == 0) {
        startAcoAlgorithm();
        delete msg;
    } else {
        EV << "Router " << address << " received message: " << msg->getName() << "\n";
        delete msg;
    }
}



void RouterNode::buildRoutingTable()
{
    const char* methodNames[] = {"Greedy Local", "Multi-hop Pheromone", "Dijkstra with Pheromone"};

    EV << "\n========================================\n";
    EV << "=== BUILDING CENTRALIZED ROUTING TABLE ===\n";
    EV << "=== METHOD: " << methodNames[routingMethod] << " ===\n";
    EV << "========================================\n\n";

    routingTable.clear();

    // For each router pair, determine best next hop
    for (int src = 0; src < totalRouters; src++) {
        for (int dest = 0; dest < totalRouters; dest++) {
            if (src != dest) {
                int nextHop = -1;

                // Call the selected routing method
                switch (routingMethod) {
                    case GREEDY_LOCAL:
                        nextHop = getBestNextHop_GreedyLocal(src, dest);
                        break;
                    case DIJKSTRA_PHEROMONE:
                        nextHop = getBestNextHop_Dijkstra(src, dest);
                        break;
                }

                if (nextHop != -1) {
                    std::pair<int, int> route(src, dest);
                    routingTable[route] = nextHop;

                    double pheromone = getPheromone(src, nextHop);
                    EV << "Route: " << src << " -> " << dest
                       << " | Next hop: " << nextHop
                       << " (pheromone: " << pheromone << ")\n";
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


// NEW: Method 1 - Greedy Local (your current implementation)
int RouterNode::getBestNextHop_GreedyLocal(int from, int to)
{
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

    double maxProbability = -1.0;
    int bestNextHop = -1;

    // Check all neighbors - pure greedy based on immediate link
    for (const auto &neighbor : fromRouter->neighbors) {
        int nextNode = neighbor.neighborAddress;

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

// Method 3 - CORRECTED Dijkstra with Pheromone (OPTIMAL!)
int RouterNode::getBestNextHop_Dijkstra(int from, int to)
{
    std::map<int, double> minCost;
    std::map<int, int> prevRouter;
    std::set<int> visited;

    for (int i = 0; i < totalRouters; i++) {
        minCost[i] = 1e9;
    }
    minCost[from] = 0.0;

    // Dijkstra's main loop
    for (int i = 0; i < totalRouters; i++) {
        int current = -1;
        double minDist = 1e9;

        for (int j = 0; j < totalRouters; j++) {
            if (!visited.count(j) && minCost[j] < minDist) {
                minDist = minCost[j];
                current = j;
            }
        }

        if (current == -1 || current == to) break;

        visited.insert(current);

        cModule *network = getParentModule();
        RouterNode *currRouter = nullptr;

        for (cModule::SubmoduleIterator it(network); !it.end(); ++it) {
            cModule *mod = *it;
            if (strcmp(mod->getName(), "router") == 0 && mod->par("address").intValue() == current) {
                currRouter = check_and_cast<RouterNode *>(mod);
                break;
            }
        }

        if (!currRouter) continue;

        // For each neighbor, calculate edge weight
        for (const auto &neighbor : currRouter->neighbors) {
            int neighborAddr = neighbor.neighborAddress;
            double linkCost = neighbor.linkCost;

            // Get pheromone and visibility
            double tau = getPheromone(current, neighborAddr);
            double eta = getVisibility(current, neighborAddr);

            // CORRECTED: Edge weight = actual cost / pheromone quality
            // But use a LOWER multiplier so pheromone actually helps
            double pheromoneQuality = pow(tau, ALPHA) * pow(eta, BETA);

            double edgeWeight;
            if (pheromoneQuality > 0.001) {
                // Penalize cost by pheromone: low pheromone = high penalty
                edgeWeight = linkCost * (1.0 / pheromoneQuality);
            } else {
                edgeWeight = linkCost * 1000.0;  // Big penalty for no pheromone
            }

            double newCost = minCost[current] + edgeWeight;

            // Update if better path found
            if (newCost < minCost[neighborAddr]) {
                minCost[neighborAddr] = newCost;
                prevRouter[neighborAddr] = current;
            }
        }
    }

    // Backtrack to find first hop
    if (minCost[to] == 1e9) {
        return -1;
    }

    int current = to;
    while (prevRouter.find(current) != prevRouter.end() && prevRouter[current] != from) {
        current = prevRouter[current];
    }

    if (prevRouter.find(current) != prevRouter.end()) {
        return current;
    }

    return -1;
}



int RouterNode::getRouterForDevice(int deviceAddress)
{
    // For 6-router network (Large)
    if (totalRouters == 6) {
        if (deviceAddress == 100) return 0;  // Device 0 → Router 0
        if (deviceAddress == 101) return 3;  // Device 1 → Router 3
        if (deviceAddress == 102) return 5;  // Device 2 → Router 5
    }
    // For 3-router network (Small)
    else if (totalRouters == 3) {
        if (deviceAddress == 100) return 0;  // Device 0 → Router 0
        if (deviceAddress == 101) return 2;  // Device 1 → Router 2
        if (deviceAddress == 102) return 1;  // Device 2 → Router 1
    }
    return -1;
}

void RouterNode::forwardDataPacket(DataMsg *dataMsg)
{
    int srcDevice = dataMsg->getSrcAddress();
    int destDevice = dataMsg->getDestAddress();
    int hopCount = dataMsg->getHopCount();

    // Path tracking for visited routers
    int newSize = dataMsg->getVisitedRoutersArraySize() + 1;
    dataMsg->setVisitedRoutersArraySize(newSize);
    dataMsg->setVisitedRouters(newSize - 1, address);

    EV << "\n┌─────────────────────────────────────┐\n";
    EV << "│ Router " << address << " Processing packet      │\n";
    EV << "└─────────────────────────────────────┘\n";
    EV << "Path: ";
    for (unsigned int i = 0; i < dataMsg->getVisitedRoutersArraySize(); i++) {
        if (i > 0) EV << " → ";
        EV << "R" << dataMsg->getVisitedRouters(i);
    }
    EV << "\n";

    // TTL protection
    if (hopCount > 15) {
        EV << "✗ TTL EXCEEDED (>15 hops)! Dropping packet.\n\n";
        delete dataMsg;
        return;
    }

    // Get destination router
    int destRouter = getRouterForDevice(destDevice);
    if (destRouter == -1) {
        EV << "✗ Unknown destination device: " << destDevice << "\n\n";
        delete dataMsg;
        return;
    }

    // ═══════════════════════════════════════════════════════
    // Check if DESTINATION REACHED
    // ═══════════════════════════════════════════════════════
    if (address == destRouter) {
        // Deliver locally to device
        for (int i = 0; i < numPorts; i++) {
            cGate *outGate = gate("port$o", i);
            if (outGate->isConnected()) {
                cModule *neighbor = outGate->getPathEndGate()->getOwnerModule();
                if (strcmp(neighbor->getName(), "device") == 0 &&
                    neighbor->par("address").intValue() == destDevice) {
                    EV << "✓ Destination reached! Delivering to Device " << destDevice << "\n\n";
                    send(dataMsg, "port$o", i);
                    return;
                }
            }
        }
        EV << "✗ ERROR: Device " << destDevice << " not found on this router!\n\n";
        delete dataMsg;
        return;
    }

    // ═══════════════════════════════════════════════════════
    // Get ALL UNVISITED NEIGHBORS (not in visited path)
    // ═══════════════════════════════════════════════════════
    struct NeighborRanking {
        int routerAddress;
        double pheromoneQuality;
        int gateIndex;
    };

    std::vector<NeighborRanking> unvisitedNeighborsRanked;

    for (const auto& neighbor : neighbors) {
        // Check if already visited
        bool alreadyVisited = false;
        for (unsigned int i = 0; i < dataMsg->getVisitedRoutersArraySize(); i++) {
            if (dataMsg->getVisitedRouters(i) == neighbor.neighborAddress) {
                alreadyVisited = true;
                break;
            }
        }

        if (!alreadyVisited) {
            // Calculate pheromone quality for this neighbor
            double tau = getPheromone(address, neighbor.neighborAddress);
            double eta = getVisibility(address, neighbor.neighborAddress);
            double quality = pow(tau, ALPHA) * pow(eta, BETA);

            NeighborRanking nr;
            nr.routerAddress = neighbor.neighborAddress;
            nr.pheromoneQuality = quality;
            nr.gateIndex = neighbor.gateIndex;

            unvisitedNeighborsRanked.push_back(nr);

            EV << "  Unvisited neighbor: R" << neighbor.neighborAddress
               << " | Pheromone: " << tau << " | Quality: " << quality << "\n";
        }
    }

    // ═══════════════════════════════════════════════════════
    // SORT NEIGHBORS BY PHEROMONE QUALITY (BEST FIRST)
    // ═══════════════════════════════════════════════════════
    std::sort(unvisitedNeighborsRanked.begin(), unvisitedNeighborsRanked.end(),
        [](const NeighborRanking& a, const NeighborRanking& b) {
            return a.pheromoneQuality > b.pheromoneQuality;  // Descending order
        });

    // ═══════════════════════════════════════════════════════
    // If NO UNVISITED NEIGHBORS: BACKTRACK
    // ═══════════════════════════════════════════════════════
    if (unvisitedNeighborsRanked.empty()) {
        EV << "✗ No more unvisited neighbors at Router " << address << "\n";

        // If source router has no way forward
        if (dataMsg->getVisitedRoutersArraySize() == 1) {
            EV << "✗✗✗ NO PATH AVAILABLE ✗✗✗\n";
            EV << "Source Router " << address << " cannot reach Device " << destDevice << "\n\n";
            delete dataMsg;
            return;
        }

        // Backtrack to previous router
        int prevRouter = dataMsg->getVisitedRouters(dataMsg->getVisitedRoutersArraySize() - 2);
        EV << "→ Backtracking to previous router: R" << prevRouter << "\n";
        EV << "  Will try NEXT BEST unvisited neighbor there.\n\n";

        // Remove this router from path
        dataMsg->setVisitedRoutersArraySize(dataMsg->getVisitedRoutersArraySize() - 1);

        // Send back to previous router
        cModule *network = getParentModule();
        for (cModule::SubmoduleIterator it(network); !it.end(); ++it) {
            cModule *mod = *it;
            if (strcmp(mod->getName(), "router") == 0 &&
                mod->par("address").intValue() == prevRouter) {
                sendDirect(dataMsg, 0.01, 0.0, mod, "directIn");
                return;
            }
        }
        delete dataMsg;
        return;
    }

    // ═══════════════════════════════════════════════════════
    // TRY NEIGHBORS IN ORDER OF QUALITY (BEST FIRST)
    // ═══════════════════════════════════════════════════════
    for (const auto& neighborRanked : unvisitedNeighborsRanked) {
        int nextHop = neighborRanked.routerAddress;

        EV << "→ Trying neighbor R" << nextHop
           << " | Quality: " << neighborRanked.pheromoneQuality << "\n";

        // If this neighbor IS the destination router, forward directly
        if (nextHop == destRouter) {
            dataMsg->setHopCount(hopCount + 1);
            EV << "✓ Next hop is DESTINATION ROUTER! Forwarding directly.\n\n";
            send(dataMsg, "port$o", neighborRanked.gateIndex);
            return;
        }

        // Check if this neighbor is in routing table for destination
        std::pair<int, int> route(address, destRouter);
        auto it = routingTable.find(route);

        if (it != routingTable.end() && it->second == nextHop) {
            // This is the routing table's choice
            dataMsg->setHopCount(hopCount + 1);
            EV << "✓ Routing table recommends R" << nextHop << "\n\n";
            send(dataMsg, "port$o", neighborRanked.gateIndex);
            return;
        }

        // Otherwise, this neighbor is an exploration candidate
        // We'll forward to the BEST RANKED unvisited neighbor
    }

    // ═══════════════════════════════════════════════════════
    // If no neighbor matched routing table, forward to BEST RANKED neighbor
    // ═══════════════════════════════════════════════════════
    if (!unvisitedNeighborsRanked.empty()) {
        const auto& bestNeighbor = unvisitedNeighborsRanked[0];  // First is best
        dataMsg->setHopCount(hopCount + 1);

        EV << "→ No routing table match. Forwarding to BEST neighbor:\n";
        EV << "  R" << bestNeighbor.routerAddress
           << " (Quality: " << bestNeighbor.pheromoneQuality << ")\n\n";

        send(dataMsg, "port$o", bestNeighbor.gateIndex);
        return;
    }

    // ═══════════════════════════════════════════════════════
    // FALLBACK: Should not reach here, but just in case
    // ═══════════════════════════════════════════════════════
    EV << "✗ No valid path forward! Dropping packet.\n\n";
    delete dataMsg;
}
