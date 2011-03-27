#include "LaserReader.h"
#include "Astar.h"
#include "PlayerWrapper.h"
#include "Mapper.h"

using namespace std;

PlayerWrapper *pw;
LaserReader *lr;
Mapper *mp;
Astar *as;

vector<player_pose2d_t> path;

bool isArrived(double tx, double ty) {
    pw->readClient();
    return ((abs(pw->getRobX() - tx) <= 0.15) && (abs(pw->getRobY() - ty) <= 0.15));
}

void printPath() {
    std::vector<player_pose2d_t>::iterator it = path.begin();
    while (it != path.end()) {
        cout << "(" << it->px << ", " << it->py << ")" << endl;
        it++;
    }
}

player_pose2d_t calcChange() {
    double deltaX, deltaY;
    double deltas = 0;
    int counter = 0;
    std::vector<player_pose2d_t>::iterator it = path.end();
    int i = path.size() - 2;
    while (it != path.begin()) {
        player_pose2d_t thisPose = path[i];
        player_pose2d_t lastPose = path[i + 1];
        deltaX = abs(thisPose.px - lastPose.px);
        deltaY = abs(thisPose.py - lastPose.py);
        double newDeltas = deltaX + deltaY;
        if (newDeltas != deltas && counter > 2) {
            return path[i + 1];
        }
        deltas = newDeltas;
        float dx = path[i].px - path[path.size() - 1].px;
        float dy = path[i].py - path[path.size() - 1].py;
        if (sqrt((dx * dx)+(dy * dy)) > 1) {
            return path[i + 1];
        }
        it--;
        i--;
        counter++;
    }
}

int main() {

    cout << "Starting up robot" << endl;
    pw = new PlayerWrapper(6665);

    cout << "Creating LaserReader" << endl;
    lr = new LaserReader(pw);

    cout << "Creating Mapper" << endl;
    mp = new Mapper(500, 500);

    cout << "Creating PathFinder" << endl;
    as = new Astar(lr);

    cout << "Starting Main Loop" << endl;
    while (true) {
        lr->readLaser();

        player_pose2d_t nextDest = as->findClosest(pw->getRobX(), pw->getRobY());
        printf("Looking for path between: (%f, %f) and (%f, %f)\n", pw->getRobX(), pw->getRobY(), nextDest.px, nextDest.py);

        while (!as->findPath(pw->getRobX(), pw->getRobY(), nextDest.px, nextDest.py, &path)) {
            //while (!findPath(pw->getRobX(), pw->getRobY(), 8.0, 8.0, &path)) {
            cout << "No Path found from " << "(" << pw->getRobX() << ", " << pw->getRobY() << ")" << " to " << "(" << nextDest.px << ", " << nextDest.py << ")" << endl;
            pw->readClient();
            lr->setIsland(nextDest.px, nextDest.py);
            break;
        }

        player_pose2d_t nextPoint = calcChange();

        cout << "Going to:" << "(" << nextPoint.px << ", " << nextPoint.py << ")" << endl;
        pw->goTo(nextPoint);
        while (!isArrived(nextPoint.px, nextPoint.py)) {
            lr->readLaser();
            mp->drawInternalMap(&path, lr);
            //mp->drawMap(&pw);
        }
    }

    cout << "Saving map" << endl;
    mp->saveMap();

    cout << "Disconnecting Player" << endl;
    delete as;
    delete mp;
    delete lr;
    delete pw;

    return 0;
}