#include "DataReader.h"

/*
 * Converts a cartesian coordinate to its array equivalent integer.
 */
int DataReader::getMatrixValue(double i) {
    return (int) ((i + X_BOUND) * SCALE); // relying on X and Y bound being the same for simplicity
}

/*
 * Converts an array number to its equivalent coordinate value.
 */
double DataReader::getCoorValue(int i) {
    return ((double) i / SCALE)-X_BOUND;
}

/*
 * Returns true if x and y are in the array.
 */
bool DataReader::isInMap(int x, int y) {
    return (x >= 0 && x < MAPSIZE_X && y >= 0 && y < MAPSIZE_X);
}

/*
 * Overloaded method.
 */
bool DataReader::isObst(player_pose2d_t pose) {
    return isObst(getMatrixValue(pose.px), getMatrixValue(pose.py));
}

/*
 * Returns false if the given array value is not registered as an obstacle.
 */
bool DataReader::isObst(int x, int y) {
    if (isInMap(x, y)) {
        return obstacle[x][y];
    }
    return true;
}

/*
 * Overloaded method.
 */
bool DataReader::isSeen(player_pose2d_t pose) {
    return isSeen(getMatrixValue(pose.px), getMatrixValue(pose.py));
}

/*
 * Returns false if the given array value is not registered as having been seen by the robot.
 */
bool DataReader::isSeen(int x, int y) {
    if (isInMap(x, y)) {
        return seen[x][y];
    }
    return true;
}

int DataReader::returnFid(int x, int y) {
    if (isInMap(x, y)) {
        return fiducials[x][y].getId();
    }
    return NONE;
}

/*
 * Sets the array equivalent to the given x and y coordinates to being an obstacle.
 * It also sets those points around it as obstacles. This is to give the robot a certain
 * safety distance from obstacles.
 */
void DataReader::setObst(double x, double y) {
    // add 0.2 padding
    for (double i = -PADDING; i <= PADDING; i += (1.0 / SCALE)) {
        for (double j = -PADDING; j <= PADDING; j += (1.0 / SCALE)) {
            double xn = x + i;
            double yn = y + j;
            int newX = getMatrixValue(xn);
            int newY = getMatrixValue(yn);
            if (isInMap(newX, newY)) {
                obstacle[newX][newY] = true;
            }
        }
    }
}

/*
 * Sets all points on the line between the robot's position and a laser reading
 * to seen.
 */
void DataReader::setSeen(double robX, double robY, double dist, double angle) {
    double x, y;
    while (dist >= 0) {
        x = robX + (cos(angle) * dist);
        y = robY + (sin(angle) * dist);
        int newX = getMatrixValue(x);
        int newY = getMatrixValue(y);
        if (isInMap(newX, newY)) {
            seen[newX][newY] = true;
        }
        dist -= (1.0 / SCALE);
    }
}

/*
 * Overloaded method for setIsland (converts the coordinates to matrix equivalents)
 */
void DataReader::setIsland(double sx, double sy) {
    return setIsland(getMatrixValue(sx), getMatrixValue(sy));
}

/*
 * Recursive function that turns unreachable "islands" into objects (i.e. intraverisble).
 * It takes on point within the island and then recursively sets all the points around it
 * to obstacles until it reaches the walls of the island.
 *
 * Thanks to the new findClosest method, this should not be needed anymore.
 */
void DataReader::setIsland(int mx, int my) {
    for (int x = -1; x < 2; x++) {
        for (int y = -1; y < 2; y++) {
            int newX = mx + x;
            int newY = my + y;
            if (isInMap(newX, newY)) {
                if (!isObst(newX, newY) && !isSeen(newX, newY)) {
                    obstacle[newX][newY] = true; // we don't want any padding
                    return setIsland(newX, newY);
                }
            }
        }
    }
}

void DataReader::setFid(int id, double x, double y) {
    int newX = getMatrixValue(x);
    int newY = getMatrixValue(y);
    if (isInMap(newX, newY)) {
        fiducials[newX][newY] = Fiducial(id, x, y);
    }
}

/*
 * Converts the laser readings into coordinates and then executes the setObst and
 * setSeen methods to set the internal map of the environment.
 */
void DataReader::readLaser() {
    pw->readClient();
    double x, y, angle, dist;

    for (int i = 0; i < pw->getLaserCount(); i++) {
        dist = pw->getRange(i);
        angle = pw->getRobA() + DTOR(i - (pw->getLaserCount() / 2.0));

        x = pw->getRobX() + (cos(angle) * dist);
        y = pw->getRobY() + (sin(angle) * dist);

        if (x<-X_BOUND - 0.5 || x > X_BOUND + 0.5 || y<-Y_BOUND - 0.5 || y > Y_BOUND + 0.5) {
            std::cout << "Out of Map: (" << x << ", " << y << ")" << std::endl;
        } else if (dist < pw->getMaxRange()) {
            setObst(x, y);
        }
        setSeen(pw->getRobX(), pw->getRobY(), dist, angle);
    }
}

/*
 * Converts the fiducial readings into RELEVANT coordinates and then executes setFid
 * to add the detected fiducial to the array
 */
void DataReader::readFid() {
    pw->readClient();
    double x, y, angle, dist;

    for (int i = 0; i < pw->getFidCount(); i++) {
        int id = pw->getFidID(i);
        if (id == DEAD || id == FIRE || id == LIVING) { // Ignore all other IDs
            x = pw->getFidX(i);
            y = pw->getFidY(i);
            angle = pw->getRobA() + atan2(y, x);
            dist = sqrt(x * x + y * y);
            x = pw->getRobX() + (cos(angle) * dist);
            y = pw->getRobY() + (sin(angle) * dist);
            setFid(id, x, y);
        }
    }
}