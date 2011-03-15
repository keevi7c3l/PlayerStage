#ifndef ASTARIMPSEARCH_H
#define	ASTARIMPSEARCH_H
#include <list>
#include <vector>
#include <algorithm>
#include <set>
#include <math.h>

#include <libplayerinterface/player.h>
#include <libplayercore/playercore.h>
#include <libplayerc/playerc.h>

#include "laserReader.h"

#define MAX_DIST 10000

using namespace std;

class Node {
public:
    int x, y, depth;
    float cost, heuristic;
    Node *parent;

    Node();

    Node(int x, int y);

    bool operator<(const Node& other);

    bool operator==(const Node& other) const;

    int setParent(Node *par);
};

class Path {
public:
    int x, y;

    Path(int x, int y);
};

bool blocked(int x, int y);

bool isValidLocation(int sx, int sy, int x, int y);

double getMovCost(int currX, int currY, int x, int y);

double getHeuCost(int x, int y, int tx, int ty);

bool inList(std::list<Node> &list, Node &node);
bool findPath(int sx, int sy, int tx, int ty, std::vector<player_pose2d_t> *path);
#endif