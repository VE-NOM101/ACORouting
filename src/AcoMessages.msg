// AcoMessages.msg
// Message definitions for ACO routing simulation

// Data packet sent between devices
packet DataMsg {
    int srcAddress;
    int destAddress;
    int hopCount = 0;
    string payload;
}

// Ant packet for ACO algorithm
packet AntMsg {
    int srcAddress;
    int destAddress;
    int hopCount = 0;
    bool isForwardAnt = true;  // true for forward ant, false for backward ant
    int visitedNodes[];  // stores path traversed
    double pathCost = 0.0;  // accumulated cost along path
}
