#include "LaserReader.h"

int LaserReader::getMatrixValue(double i) {
    return (int) ((i + X_BOUND) * SCALE); // relying on X and Y bound being the same
}

double LaserReader::getCoorValue(int i) {
    return ((double) i / SCALE)-X_BOUND;
}

bool LaserReader::isObst(int x, int y) {
    if (x >= 0 && x < MAPSIZE_X && y >= 0 && y < MAPSIZE_X) {
        return obstacle[x][y];
    }
    return true;
}

bool LaserReader::isSeen(int x, int y) {
    if (x >= 0 && x < MAPSIZE_X && y >= 0 && y < MAPSIZE_X) {
        return seen[x][y];
    }
    return true;
}

void LaserReader::setObst(double x, double y) {
    // add 0.2 padding
    for (double i = -PADDING; i <= PADDING; i += (1.0 / SCALE)) {
        for (double j = -PADDING; j <= PADDING; j += (1.0 / SCALE)) {
            double xn = x + i;
            double yn = y + j;
            int newX = getMatrixValue(xn);
            int newY = getMatrixValue(yn);
            if (newX >= 0 && newX < MAPSIZE_X && newY >= 0 && newY < MAPSIZE_X) {
                obstacle[newX][newY] = true;
            }
        }
    }
}

void LaserReader::setSeen(double robX, double robY, double dist, double angle) {
    double x, y;
    while (dist >= 0) {
        x = robX + (cos(angle) * dist);
        y = robY + (sin(angle) * dist);
        int newX = getMatrixValue(x);
        int newY = getMatrixValue(y);
        if (newX >= 0 && newX < MAPSIZE_X && newY >= 0 && newY < MAPSIZE_X) {
            seen[newX][newY] = true;
        }
        dist -= (1.0 / SCALE);
    }
}

void LaserReader::setIsland(double sx, double sy) {
    return setIsland(getMatrixValue(sx), getMatrixValue(sy));
}

void LaserReader::setIsland(int mx, int my) {
    for (int x = -1; x < 2; x++) {
        for (int y = -1; y < 2; y++) {
            int newX = mx + x;
            int newY = my + y;
            if (!isObst(newX, newY)) {
                obstacle[newX][newY] = true; // we don't want any padding
                return setIsland(newX, newY);
            }
        }
    }
}

void LaserReader::setSeen(double x, double y) {
    int newX = getMatrixValue(x);
    int newY = getMatrixValue(y);
    if (newX >= 0 && newX < MAPSIZE_X && newY >= 0 && newY < MAPSIZE_X) {
        seen[newX][newY] = true;
    }
}

void LaserReader::readLaser() {
    pw->readClient();
    double x, y, angle, dist;

    for (int i = 0; i < pw->getLaserCount(); i++) {
        dist = pw->getRange(i);
        angle = pw->getRobA() + DTOR(i - (pw->getLaserCount() / 2.0));

        x = pw->getRobX() + (cos(angle) * dist);
        y = pw->getRobY() + (sin(angle) * dist);

        if (x<-X_BOUND - 0.5 || x > X_BOUND + 0.5 || y<-Y_BOUND - 0.5 || y > Y_BOUND + 0.5) {
            printf("Out of Map: (%f,%f)\n", x, y);
        } else if (dist < pw->getMaxRange()) {
            setObst(x, y);
        }
        setSeen(pw->getRobX(), pw->getRobY(), dist, angle);
    }
}