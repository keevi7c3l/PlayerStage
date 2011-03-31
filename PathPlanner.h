/*
 * File:   Astar.h
 * Author: Pit Apps
 *
 * Created on March 26, 2011, 4:43 PM
 *
 * This Class Calculates the next best path using the A* algorithm.
 * The findClosest method computes the next best goal to go to by looking at the
 * closest unseen points.
 */

#ifndef ASTARIMPSEARCH_H
#define	ASTARIMPSEARCH_H
#include <list>
#include <vector>
#include <queue>
#include "DataReader.h"

#define MAX_DIST 10000

using namespace std;

class Node {
public:
    int x, y, depth;
    float cost, heuristic;
    Node *parent;
    bool inClosed;
    bool inOpen;

    Node() : depth(0), cost(0), heuristic(0), inClosed(false), inOpen(false) {
    }

    Node(int x, int y) : x(x), y(y), depth(0), cost(0), heuristic(0), inClosed(false), inOpen(false) {
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

class Astar {
public:

    Astar(DataReader *dr) : dr(dr) {

    }
    int findPath(double sx, double sy, double tx, double ty, vector<player_pose2d_t> *path); // Method Overload
    player_pose2d_t findClosest(double currX, double currY); // Method Overload
    void findClosest2(int x, int y, player_pose2d_t *path, bool paths[MAPSIZE_X][MAPSIZE_Y]);
    void findClosest2(int x, int y, player_pose2d_t *path);
private:
    DataReader *dr;
    Node nodes[MAPSIZE_X][MAPSIZE_Y];

    player_pose2d_t findClosest(int x, int y);
    bool isInProximity(int x, int y, int tx, int ty);
    bool isInvalid(int sx, int sy, int x, int y);
    int findPath(int sx, int sy, int tx, int ty, vector<player_pose2d_t> *path);
};

#endif