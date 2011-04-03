/*
 * Team Charlie's Search & Rescue Mapping Robot v1.0
 * Produced for King's College London
 * 
 * Authors: Pit Apps,
 *          Paul Fontenay,
 *          Vlad-Ovidiu Tutunea,
 *          Erleene Lyder,
 *          Nathaniel Ghilazghi,
 *          Oliver Kelly
 *
 * Copyright 2011
 */
#include "DataReader.h"
#include "PathPlanner.h"
#include "PlayerWrapper.h"
#include "Mapper.h"

using namespace std;

PlayerWrapper *pw;
DataReader *dr;
Mapper *mp;
Astar *as;

vector<player_pose2d_t> path;

/*
 * Debug method for printing the current path (unused)
 */
//void printPath() {
//    vector<player_pose2d_t>::iterator it = path.begin();
//    while (it != path.end()) {
//        cout << "(" << it->px << ", " << it->py << ")" << endl;
//        it++;
//    }
//}

/*
 * Returns true if the robot has arrived at a destination (tx,ty).
 * We use an approximate value because player's "goto" command is not very precise.
 */
bool isArrived(double tx, double ty) {
    pw->readClient();
    return ((abs(pw->getRobX() - tx) <= 0.30) && (abs(pw->getRobY() - ty) <= 0.30));
}

/*
 * Improves the path by simplifying it.
 */
player_pose2d_t calcChange() {
    int deltaX, deltaY;
    int deltas = 0;
    int counter = 0;
    vector<player_pose2d_t>::iterator it = path.end();
    int i = path.size() - 2;
    while (it != path.begin()) {
        player_pose2d_t thisPose = path[i];
        player_pose2d_t lastPose = path[i + 1];
        deltaX = (int) (abs(thisPose.px - lastPose.px)*10);
        deltaY = (int) (abs(thisPose.py - lastPose.py)*10);
        int newDeltas = deltaX + deltaY;
        if (newDeltas != deltas && counter > 4) {
            return path[i + 1];
        }
        deltas = newDeltas;
        float dx = path[i].px - path[path.size() - 1].px;
        float dy = path[i].py - path[path.size() - 1].py;
        if (sqrt((dx * dx)+(dy * dy)) > 5) { // don't simplify over 5 meters
            return path[i + 1];
        }
        it--;
        i--;
        counter++;
    }
    return ( player_pose2d_t{
        -X_BOUND, -Y_BOUND, 0
    });
}

player_pose2d_t findClosestPoint() {
    player_pose2d_t tempPath;
    tempPath.px = -X_BOUND - 1;
    tempPath.py = -Y_BOUND - 1;
    /* This is a similar check as found in the main aStar, player sometimes "barricades" itself in */
    while (as->findClosest2(dr->getMatrixValue(pw->getRobX()), dr->getMatrixValue(pw->getRobY()), &tempPath) == 0) {
        cout << "Player messed up, rereading position" << endl;
        pw->goTo(pw->getRobX() + 0.1, pw->getRobY() + 0.1); // Move away from object
        pw->readClient();
    }
    return tempPath;
}

int main(int argc, char* argv[]) {

    cout << "Starting up robot" << endl;
    pw = new PlayerWrapper(6665);
    pw->readClient();

    cout << "Creating DataReader" << endl;
    dr = new DataReader(pw);

    cout << "Creating Mapper" << endl;
    mp = new Mapper(1000, 1200, dr);

    cout << "Creating PathFinder" << endl;
    as = new Astar(dr);

    player_pose2d_t nextDest = (player_pose2d_t){
        pw->getRobX(), pw->getRobY(), pw->getRobA()
    };
    int oldDepth = 10000;
    int newDepth = 100000;

    timeval start, end;
    gettimeofday(&start, NULL);

    cout << "Starting Main Loop" << endl;
    while (true) {
        dr->readLaser();
        dr->readFid();

        /* This is to avoid the robot going back and forth between two equidistant points */
        if (dr->isObst(nextDest) || dr->isSeen(nextDest) || oldDepth < newDepth) {
            cout << "Looking for new path" << endl;
            nextDest = findClosestPoint();
            oldDepth = 10000;
        }

        /* Robot has finished */
        if ((nextDest.px < -X_BOUND) && (nextDest.py <-Y_BOUND)) {
            cout << "Whole Map has been traversed; depth:" << nextDest.pa << endl;
            break;
        }

        cout << "Looking for path between: (" << pw->getRobX() << ", " << pw->getRobY() << ") and (" << nextDest.px << ", " << nextDest.py << ")" << endl;

        if ((newDepth = as->findPath(pw->getRobX(), pw->getRobY(), nextDest.px, nextDest.py, &path)) == -1) {
            cout << "No Path found from (" << pw->getRobX() << ", " << pw->getRobY() << ") to (" << nextDest.px << ", " << nextDest.py << ")" << endl;
            /* Not used anymore */
            //dr->setIsland(nextDest.px, nextDest.py); // Trying to go somewhere that is unreachable
            continue;
        } else if (newDepth == 0) {
            cout << "Player messed up, rereading position" << endl;
            pw->goTo(pw->getRobX() + 0.1, pw->getRobY() + 0.1); // Move away from object
            continue;
        } else if (newDepth > oldDepth) { // The goal is further away than we first estimated, let's look for a closer point
            continue;
        }

        oldDepth = newDepth;

        player_pose2d_t nextPoint = calcChange();

        if (nextPoint.px == -X_BOUND && nextPoint.py == -Y_BOUND) {
            cout << "calcChanged messed up" << endl; // This should not happen
            continue;
        }

        cout << "Going to:" << "(" << nextPoint.px << ", " << nextPoint.py << ")" << endl;

        mp->drawInternalMap(path);

        pw->goTo(nextPoint);
        while (!isArrived(nextPoint.px, nextPoint.py)) {
            dr->readLaser();
            dr->readFid();
            //mp->drawMap(pw); // this does not work on older OpenCV versions
        }
    }

    /* Let's find out how long it took */
    gettimeofday(&end, NULL);
    int secs = end.tv_sec - start.tv_sec;
    int mins = secs / 60;
    secs -= (60 * mins);

    cout << "Mapping took: " << mins << " minutes, " << secs << " seconds." << endl;

    cout << "Saving map" << endl;
    path.clear();
    mp->drawInternalMap(path);
    mp->saveMap();

    cout << "Disconnecting Player" << endl;
    delete as;
    delete mp;
    delete dr;
    delete pw;

    return 0;
}