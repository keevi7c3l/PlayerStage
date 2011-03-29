#include "PathPlanner.h"
#include <queue>

/*
 * Returns false if the location is out of bounds or an obstacle.
 */
bool Astar::isValidLocation(int sx, int sy, int x, int y) {
    return (x < 0) || (y < 0) || (x >= MAPSIZE_X) || (y >= MAPSIZE_Y) || lr->isObst(x, y) || ((sx == x) && (sy == y));
}

/*
 * Returns the cost of moving.
 */
double getMovCost(int currX, int currY, int x, int y) {
    if (currX != x && currY != y) {
        return 1.4;
    } else {
        return 1.0;
    }
}

/*
 * Returns an estimate of the distance between (x,y) and (tx,ty).
 */
float getHeuCost(int x, int y, int tx, int ty) {
    //float dx = tx - x;
    //float dy = ty - y;

    //return sqrt((dx * dx)+(dy * dy));
    return hypot(tx - x, ty - y);
}

/*
 * Returns true if the Node is in the specified list.
 */
bool inList(list<Node> &list, Node & node) {
    std::list<Node>::iterator i;
    for (i = list.begin(); i != list.end(); ++i) {
        if (i->x == node.x && i->y == node.y) {
            return true;
        }
    }
    return false;
}

/*
 * Overloading for finding the closest point (converts the coordinates to
 * matrix values).
 */
player_pose2d_t Astar::findClosest(double currX, double currY) {
    return findClosest2(lr->getMatrixValue(currX), lr->getMatrixValue(currY));
}

/*
 * Returns the closest unseen, accessible, point to the robot's current position.
 */
player_pose2d_t Astar::findClosest(int x, int y) {
    Path *mainPath = new Path(-1, -1, 1000000); // If no path found, returned path will be smaller than the XY_BOUND
    for (int i = 0; i < MAPSIZE_X; i++) {
        for (int j = 0; j < MAPSIZE_Y; j++) {
            if (lr->getCoorValue(i) <= -(X_BOUND - 0.2) || lr->getCoorValue(i) >= (X_BOUND - 0.2) || lr->getCoorValue(j) <= -(Y_BOUND - 0.2) || lr->getCoorValue(j) >= (Y_BOUND - 0.2)) { //hack
                continue;
            }
            if (!lr->isSeen(i, j) && i != x && j != y && !lr->isObst(i, j)) {
                double currentHeu = getHeuCost(x, y, i, j);
                if (currentHeu < mainPath->cost) {
                    mainPath->x = i;
                    mainPath->y = j;
                    mainPath->cost = currentHeu;
                }
            }
        }
    }
    player_pose2d_t path;
    path.px = lr->getCoorValue(mainPath->x);
    path.py = lr->getCoorValue(mainPath->y);
    //free(mainPath);
    cout << "Closest path is: (" << path.px << ", " << path.py << ")" << endl;
    return path;
}

struct Paath {
    int x, y;
    double cost;
};

bool operator<(const Paath &a, const Paath &b) {
    return a.cost > b.cost;
}

bool Astar::isInProximity(int x, int y, int tx, int ty) {
    double sx = lr->getCoorValue(x);
    double sy = lr->getCoorValue(y);
    double tsx = lr->getCoorValue(tx);
    double tsy = lr->getCoorValue(ty);
    int range = 1;
    bool isit = (((sx + range >= tsx && tsx >= sx) || (sx - range <= tsx && tsx <= sx)) && ((sy + range >= tsy && tsy >= sy) || (sy - range <= tsy && tsy <= sy)));
    //printf("(%f,%f) and (%f,%f) is %d\n", sx, sy, tsx, tsy, isit);
    return isit;
}

player_pose2d_t Astar::findClosest2(int x, int y) {
    std::priority_queue<Paath> currPath;
    for (int i = 0; i < MAPSIZE_X; i++) {
        for (int j = 0; j < MAPSIZE_Y; j++) {
            if (lr->getCoorValue(i) <= -(X_BOUND - 0.2) || lr->getCoorValue(i) >= (X_BOUND - 0.2) || lr->getCoorValue(j) <= -(Y_BOUND - 0.2) || lr->getCoorValue(j) >= (Y_BOUND - 0.2)) { //hack
                continue;
            }
            if (!lr->isSeen(i, j) && i != x && j != y && !lr->isObst(i, j)) {
                if (lr->getCoorValue(i)<-11 || lr->getCoorValue(i) > 11 || lr->getCoorValue(j)<-7 || lr->getCoorValue(j) > 7) { //hackety hack

                    currPath.push((Paath) {
                        i, j, getHeuCost(x, y, i, j) + 100
                    });
                } else {

                    currPath.push((Paath) {
                        i, j, getHeuCost(x, y, i, j)
                    });
                }
            }
        }
    }
    if (currPath.empty()) {
        cout << "empty queue" << endl;

        return ((player_pose2d_t) {
            -X_BOUND-0.1, -Y_BOUND-0.1, 0
        });
    }
    Paath closest = currPath.top();
    Paath secondC;
    bool isSec = false;
    currPath.pop();
    while (!currPath.empty()) {
        Paath temp = currPath.top();
        if (!isInProximity(closest.x, closest.y, temp.x, temp.y)) {
            secondC = temp;
            isSec = true;
            break;
        }
        currPath.pop();
    }
    Paath mainPath;
    if (isSec) {
       // printf("First has cost: %f, second: %f\n", closest.cost, secondC.cost);
        vector<player_pose2d_t> temp;
        int firstSize = 100000;
        int secondSize = 100000;
        if (findPath(x, y, closest.x, closest.y, &temp)) {
            firstSize = temp.size();
        }
        if (findPath(x, y, secondC.x, secondC.y, &temp)) {
            secondSize = temp.size();
        }
       // printf("First has size: %d, second: %d\n", firstSize, secondSize);
        firstSize < secondSize ? mainPath = closest : mainPath = secondC;
    } else mainPath = closest;
    player_pose2d_t path;
    path.px = lr->getCoorValue(mainPath.x);
    path.py = lr->getCoorValue(mainPath.y);
    cout << "Closest path is: (" << path.px << ", " << path.py << ")" << endl;
    return path;
}

/*
 * Overloading for the findPath method. Converts the coordinates to matrix values.
 */
int Astar::findPath(double sx, double sy, double tx, double ty, vector<player_pose2d_t> *path) {
    return findPath(lr->getMatrixValue(sx), lr->getMatrixValue(sy), lr->getMatrixValue(tx), lr->getMatrixValue(ty), path);
}

/*
 * Main A* algorithm, takes a start coordinate (sx,sy) and a goal (tx,ty) and updates
 * the path vector to the best path it could find.
 * Returns false if no valid path was found.
 */
int Astar::findPath(int sx, int sy, int tx, int ty, vector<player_pose2d_t> *path) {
    list<Node> closed;
    list<Node> open;
    int maxDepth = 0;
    open.clear();
    closed.clear();

    for (int x = 0; x < MAPSIZE_X; x++) {
        for (int y = 0; y < MAPSIZE_Y; y++) {
            nodes[x][y] = Node(x, y);
        }
    }

    for (int x = 0; x < MAPSIZE_X; x++) {
        for (int y = 0; y < MAPSIZE_Y; y++) {
            visited[x][y] = false;
        }
    }

    open.push_back(nodes[sx][sy]);
    nodes[sx][sy].inOpen = true;
    nodes[tx][ty].parent = NULL;

    //cout << "Findpath while loop started" << endl;
    while ((maxDepth < MAX_DIST) && (open.size() != 0)) {
        open.sort();
        Node *current = &(open.front());
        if ((current->x == nodes[tx][ty].x) && (current->y == nodes[tx][ty].y)) {
            break;
        }
        open.remove(*current);
        closed.push_back(*current);
        current->inOpen = false;
        current->inClosed = true;

        //if (visited[current->x][current->y]) continue;
        //visited[current->x][current->y] = true;

        for (int x = -1; x < 2; x++) {
            for (int y = -1; y < 2; y++) {
                if ((x == 0) && (y == 0)) {
                    continue;
                }

                int xp = x + current->x;
                int yp = y + current->y;

                if (!isValidLocation(sx, sy, xp, yp)) {
                    // if (visited[xp][yp]) continue;
                    float nextStepCost = current->cost + getMovCost(current->x, current->y, xp, yp);
                    Node *neighbour = &nodes[xp][yp];
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
                        maxDepth = max(maxDepth, neighbour->setParent(current));
                        open.push_back(*neighbour);
                        neighbour->inOpen = true;
                    }
                }
            }
        }
    }
    //cout << "Findpath while loop finished; depth: " << maxDepth << endl;

    /* Player sometimes confuses the robot's position and A* thus thinks every cell around it is an obstacle */
    if (maxDepth == 0) {
        return 0;
    }

    /* No Path found*/
    if (nodes[tx][ty].parent == NULL) {
        return -1;
    }

    Node *target = &nodes[tx][ty];
    int count = 0;
    path->clear();
    while ((target->x != nodes[sx][sy].x) || (target->y != nodes[sx][sy].y)) {

        /* An infinite loop SOMETIMES occurs here for unknown reasons */
        if (count > maxDepth) {
            cout << "Infinite Loop" << endl;
            break;
        }

        path->push_back((player_pose2d_t) {
            lr->getCoorValue((double) target->x), lr->getCoorValue((double) target->y), 0.0
        });
        target = target->parent;
        count++;
    }

    path->push_back((player_pose2d_t) {
        lr->getCoorValue((double) sx), lr->getCoorValue((double) sy), 0.0
    });
    return maxDepth;
}