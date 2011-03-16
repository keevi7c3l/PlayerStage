#include "astarImpTest.h"
#include <iostream>

Node::Node() {
}

Node::Node(int x, int y) : x(x), y(y), inOpen(false), inClosed(false) {
};

bool Node::operator<(const Node& other) {
    float f = other.heuristic + other.cost;
    float of = this->heuristic + this->cost;
    if (f > of) {
        return true;
    } else if (f < of) {
        return false;
    } else {
        return 0;
    }
}

bool Node::operator==(const Node& other) const {
    if ((other.x == this->x) && (other.y == this->y)) {
        return true;
    }
    return false;
}

int Node::setParent(Node *par) {
    depth = par->depth + 1;
    this->parent = par;
    return depth;
}

bool blocked(int x, int y) {
    return isObst(x, y);
}

bool isValidLocation(int sx, int sy, int x, int y) {
    bool invalid = (x < 0) || (y < 0) || (x >= MAPSIZE_X) || (y >= MAPSIZE_Y);

    if ((!invalid) && ((sx != x) || (sy != y))) {
        invalid = blocked(x, y);
    }

    return !invalid;
}

double getMovCost(int currX, int currY, int x, int y) {
    return 1;
}

double getHeuCost(int x, int y, int tx, int ty) {
    float dx = tx - x;
    float dy = ty - y;

    return (float) (sqrt((dx * dx)+(dy * dy)));
}

bool inList(std::list<Node> &list, Node &node) {
    std::list<Node>::iterator i;
    for (i = list.begin(); i != list.end(); ++i) {
        if (i->x == node.x && i->y == node.y) {
            return true;
        }
    }
    return false;
}

bool findPath(int sx, int sy, int tx, int ty, std::vector<player_pose2d_t> *path) {

    std::list<Node> closed;
    std::list<Node> open;

    for (int x = 0; x < MAPSIZE_X; x++) {
        for (int y = 0; y < MAPSIZE_Y; y++) {
            nodes[x][y] = Node(x, y);
        }
    }

    nodes[sx][sy].cost = 0;
    nodes[sx][sy].depth = 0;

    closed.clear();
    open.clear();
    open.push_back(nodes[sx][sy]);
    nodes[sx][sy].inOpen = true;

    nodes[tx][ty].parent = NULL;

    int maxDepth = 0;

    while ((maxDepth < MAX_DIST) && (open.size() != 0)) {
        Node *current = &(open.front());
        if ((current->x == nodes[tx][ty].x) && (current->y == nodes[tx][ty].y)) {
            break;
        }
        open.remove(*current);
        closed.push_back(*current);
        current->inOpen = false;
        current->inClosed = true;

        for (int x = -1; x < 2; x++) {
            for (int y = -1; y < 2; y++) {
                if ((x == 0) && (y == 0)) {
                    continue;
                }

                int xp = x + current->x;
                int yp = y + current->y;

                if (isValidLocation(sx, sy, xp, yp)) {
                    float nextStepCost = current->cost + getMovCost(current->x, current->y, xp, yp);
                    Node *neighbour = &nodes[xp][yp];
                    visited[xp][yp] = true;


                    if (nextStepCost < neighbour->cost) {
                        if (inList(open, *neighbour)) {
                            open.remove(*neighbour);
                            neighbour->inOpen = false;
                        }
                        if (inList(closed, *neighbour)) {
                            closed.remove(*neighbour);
                            neighbour->inClosed = false;
                        }
                    }

                    if (!(neighbour-> inOpen) && !(neighbour->inClosed)) {
                        neighbour->cost = nextStepCost;
                        neighbour->heuristic = getHeuCost(xp, yp, tx, ty);
                        maxDepth = std::max(maxDepth, neighbour->setParent(current));
                        open.push_back(*neighbour);
                        neighbour->inOpen = true;
                        open.sort();
                    }
                }
            }
        }
    }

    if (nodes[tx][ty].parent == NULL) {
        std::cout << "NO PATH FOUND" << std::endl;
        return false;
    }

    Node *target = &nodes[tx][ty];
    while ((target->x != nodes[sx][sy].x) || (target->y != nodes[sx][sy].y)) {

        path->push_back((player_pose2d_t) {
                        getCoorValue((double) target->x), getCoorValue((double) target->y), 0.0
        });
        target = target->parent;
    }

    path->push_back((player_pose2d_t) {
                    getCoorValue((double) sx), getCoorValue((double) sy), 0.0
    });
    return true;
}