#ifndef ASTARIMPSEARCH_H
#define	ASTARIMPSEARCH_H
#include <list>
#include <vector>
#include "laserReader.h"

#define MAX_DIST 10000

using namespace std;

class Node {
public:
    int x, y, depth;
    float cost, heuristic;
    Node *parent;
    bool inClosed;
    bool inOpen;

    Node() {
    }

    Node(int x, int y) : x(x), y(y), inOpen(false), inClosed(false), cost(0), heuristic(0), depth(0) {
    }

    bool operator<(const Node& other) {
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

    bool operator==(const Node& other) const {
        if ((other.x == this->x) && (other.y == this->y)) {
            return true;
        }
        return false;
    }

    int setParent(Node *par) {
        depth = par->depth + 1;
        this->parent = par;
        return depth;
    }
};

class Path {
public:
    int x, y;
    double cost;

    Path(int x, int y, double cost) : x(x), y(y), cost(cost) {

    }
};

class Astar {
public:

    Astar(LaserReader *lr) : lr(lr) {

    }
    bool findPath(double sx, double sy, double tx, double ty, vector<player_pose2d_t> *path); // Method Overload
    player_pose2d_t findClosest(double currX, double currY); // Method Overload
private:
    LaserReader *lr;
    Node nodes[MAPSIZE_X][MAPSIZE_Y];
    bool visited[MAPSIZE_X][MAPSIZE_Y];

    player_pose2d_t findClosest(int x, int y);
    bool findPath(int sx, int sy, int tx, int ty, vector<player_pose2d_t> *path);
    bool isValidLocation(int sx, int sy, int x, int y);
};

#endif