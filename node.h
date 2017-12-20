//#include "point.h"

class Node {
public:
    int cost;
    int priority;
    bool visited;
    Point location;
    Node* from;
    Node(Point loc);
    Node();
};

Node::Node() {
    cost = 10000;
    priority = 10000;
    from = NULL;
    visited = false;
}

Node::Node(Point loc) {
    cost = 10000;
    priority = 10000;
    from = NULL;
    location = loc;
    visited = false;
}
