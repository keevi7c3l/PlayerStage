#include "PathPlanner.h"

/*
 * Returns false if the location is out of bounds or an obstacle.
 */
bool Astar::isInvalid(int sx, int sy, int x, int y) {
    return !dr->isInMap(x, y) || dr->isObst(x, y) || ((sx == x) && (sy == y));
}

/*
 * Returns the cost of moving.
 */
double getMovCost(int currX, int currY, int x, int y) {
    if (currX != x && currY != y) return 1.4; // this makes the paths straighter
    else return 1.0;
}

/*
 * Returns an estimate of the distance between (x,y) and (tx,ty).
 */
float getHeuCost(int x, int y, int tx, int ty) {
    return hypot(tx - x, ty - y);
}

/*
 * Returns true if the Node is in the specified list.
 * (why does c++ not have a method like this in the std?)
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
 * This is a much better way to find the closest path than the older method at the bottom.
 * It will ignore all points outside the map making the pathplanning much more
 * effective.
 *
 * It looks at the neighbouring (x,y) until it finds smth that is not an obstacle
 * and has not been seen yet.
 * This code is very similar to the astar and could probably be optimized further.
 */
int Astar::findClosest2(int x, int y, player_pose2d_t *path) {
    list<Node> closed;
    list<Node> open;
    int depth = 0;

    open.push_back(Node(x, y));

    while (open.size() != 0) {
        open.sort();
        Node *current = &(open.front());
        int lx = current->x;
        int ly = current->y;
        if (!dr->isSeen(lx, ly) && dr->isInMap(lx, ly) && !dr->isObst(lx, ly) && (lx != x && ly != y)) {
            cout << "Found something" << endl;
            path->px = dr->getCoorValue(current->x);
            path->py = dr->getCoorValue(current->y);
            break;
        }
        open.remove(*current);
        closed.push_back(*current);

        for (int x = -1; x < 2; x++) {
            for (int y = -1; y < 2; y++) {
                if ((x == 0) && (y == 0)) {
                    continue;
                }

                int xp = x + current->x;
                int yp = y + current->y;

                if (!isInvalid(x, y, xp, yp)) {
                    Node *neighbour = new Node(xp, yp);
                    if (!inList(open, *neighbour) && !inList(closed, *neighbour)) {
                        depth++;
                        open.push_back(*neighbour);
                    }
                }
            }
        }
    }
    return depth;
}

/*
 * Overloading for the findPath method. Converts the coordinates to matrix values.
 */
int Astar::findPath(double sx, double sy, double tx, double ty, vector<player_pose2d_t> *path) {
    return findPath(dr->getMatrixValue(sx), dr->getMatrixValue(sy), dr->getMatrixValue(tx), dr->getMatrixValue(ty), path);
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

    for (int x = 0; x < MAPSIZE_X; x++) {
        for (int y = 0; y < MAPSIZE_Y; y++) {
            nodes[x][y] = Node(x, y);
        }
    }

    open.push_back(nodes[sx][sy]);
    nodes[sx][sy].inOpen = true;
    nodes[tx][ty].parent = NULL;


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

        for (int x = -1; x < 2; x++) {
            for (int y = -1; y < 2; y++) {
                if ((x == 0) && (y == 0)) {
                    continue;
                }

                int xp = x + current->x;
                int yp = y + current->y;

                if (!isInvalid(sx, sy, xp, yp)) {
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
            dr->getCoorValue((double) target->x), dr->getCoorValue((double) target->y), 0.0
        });
        target = target->parent;
        count++;
    }

    path->push_back((player_pose2d_t) {
        dr->getCoorValue((double) sx), dr->getCoorValue((double) sy), 0.0
    });
    return maxDepth;
}


/*
 * The lines under here are old unused methods. They are kept in the code just in case
 * they are needed, and because they took a lot of work to code and it would be sad
 * to delete them :)
 */

/*
 * Used in depreciated findClosest method only!
 *
 * Returns true if (tx,ty) is within a certain diameter (int range) of (x,y).
 */
//bool Astar::isInProximity(int x, int y, int tx, int ty) {
//    double sx = dr->getCoorValue(x);
//    double sy = dr->getCoorValue(y);
//    double tsx = dr->getCoorValue(tx);
//    double tsy = dr->getCoorValue(ty);
//    int range = 1;
//    bool isit = (((sx + range >= tsx && tsx >= sx) || (sx - range <= tsx && tsx <= sx)) && ((sy + range >= tsy && tsy >= sy) || (sy - range <= tsy && tsy <= sy)));
//    return isit;
//}

/*
 * Used in depreciated findClosest method only!
 */
//struct Path {
//    int x, y;
//    double cost;
//};
//
//bool operator<(const Path &a, const Path &b) {
//    return a.cost > b.cost;
//}

/*
 * DEPRECIATED METHOD !!!
 *
 * Overloading for finding the closest point (converts the coordinates to
 * matrix values).
 */
//player_pose2d_t Astar::findClosest(double currX, double currY) {
//    return findClosest(dr->getMatrixValue(currX), dr->getMatrixValue(currY));
//}

/*
 * DEPRECIATED METHOD !!!
 *
 * Kept in the code for old times sake. (and because it sort of works)
 *
 * Returns the closest unseen, accessible, point to the robot's current position.
 * It will compare the two closest points in terms of direct distance and then determine which one is
 * effectively closer by taking obstacles into consideration.
 * We only compare two points because otherwise the computation would be too slow.
 */
//player_pose2d_t Astar::findClosest(int x, int y) {
//    priority_queue<Path> currPath;
//    for (int i = 0; i < MAPSIZE_X; i++) {
//        for (int j = 0; j < MAPSIZE_Y; j++) {
//            if (dr->getCoorValue(i) <= -(X_BOUND - 0.2) || dr->getCoorValue(i) >= (X_BOUND - 0.2)
//                    || dr->getCoorValue(j) <= -(Y_BOUND - 0.2) || dr->getCoorValue(j) >= (Y_BOUND - 0.2)) { // Don't consider points close to the boundaries of the map
//                continue;
//            }
//            if (!dr->isSeen(i, j) && i != x && j != y && !dr->isObst(i, j)) {
//                if (dr->getCoorValue(i)<-11 || dr->getCoorValue(i) > 11 ||
//                        dr->getCoorValue(j)<-7 || dr->getCoorValue(j) > 7) {
//
//                    currPath.push((Path) {
//                        i, j, getHeuCost(x, y, i, j) + 100 // prioritize points closer to the centre of the map.
//                    });
//                } else {
//
//                    currPath.push((Path) {
//                        i, j, getHeuCost(x, y, i, j)
//                    });
//                }
//            }
//        }
//    }
//
//    /* No Path found */
//    if (currPath.empty()) {
//
//        return ((player_pose2d_t) {
//            -X_BOUND - 0.1, -Y_BOUND - 0.1, 0
//        });
//    }
//
//    Path closest = currPath.top();
//    Path secondC;
//    currPath.pop();
//    bool isSec = false;
//
//    /* Look for a point to compare the first to */
//    while (!currPath.empty()) {
//        Path temp = currPath.top();
//        if (!isInProximity(closest.x, closest.y, temp.x, temp.y)) {
//            secondC = temp;
//            isSec = true;
//            break;
//        }
//        currPath.pop();
//    }
//
//    /* If a second point exists, compare its real length to the first point */
//    if (isSec) {
//        vector<player_pose2d_t> temp;
//        int firstSize = 100000;
//        int secondSize = 100000;
//        if (findPath(x, y, closest.x, closest.y, &temp)) firstSize = temp.size();
//        if (findPath(x, y, secondC.x, secondC.y, &temp)) secondSize = temp.size();
//        if (firstSize > secondSize) closest = secondC; // is the second point actually closer?
//    }
//
//    player_pose2d_t path = (player_pose2d_t){dr->getCoorValue(closest.x), dr->getCoorValue(closest.y), 0};
//    cout << "Closest path is: (" << path.px << ", " << path.py << ")" << endl;
//    return path;
//}
