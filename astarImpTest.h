#ifndef ASTARIMPSEARCH_H
#define	ASTARIMPSEARCH_H
#include <list>
#include <vector>
#include <math.h>
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

    Node();

    Node(int x, int y);

    bool operator<(const Node& other);

    bool operator==(const Node& other) const;

    int setParent(Node *par);
};

class Path {
public:
    int x, y;
    double cost;
    Path(int x, int y, double cost);
};

bool findPath(int sx, int sy, int tx, int ty, vector<player_pose2d_t> *path);
player_pose2d_t findClosest(double currX, double currY);

static Node nodes[MAPSIZE_X][MAPSIZE_Y];
static bool visited[MAPSIZE_X][MAPSIZE_Y] = {false};

#endif