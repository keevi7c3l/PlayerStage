#include <libplayerc/playerc.h>
#include "laserReader.h"
#include <cmath>

LaserReader::LaserReader(playerc_client_t *client,
                         playerc_laser_t *laser,
                         playerc_position2d_t *position2d) : client(client), laser(laser), position2d(position2d) {
}

extern int getMatrixValue(double i) {
    return (int) ((i + X_BOUND) * SCALE); // relying on X and Y bound being the same
}

extern double getCoorValue(int i) {
    return ((double) i / SCALE)-X_BOUND;
}

extern int isObst(int x, int y) {
    if (x >= 0 && x < MAPSIZE_X && y >= 0 && y < MAPSIZE_X) {
        return obstacle[x][y];
    }
    return true;
}

void setObst(double x, double y) {
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

void LaserReader::readLaser() {
    playerc_client_read(client);
    double x, y, angle, dist, robAng, xPos, yPos;

    for (int i = 0; i < laser->scan_count; i++) {
        robAng = position2d->pa;
        xPos = position2d->px;
        yPos = position2d->py;
        dist = laser->ranges[i];
        angle = robAng + DTOR(i - 90);

        y = yPos + (sin(angle) * dist);
        x = xPos + (cos(angle) * dist);

        if (x<-X_BOUND - 0.5 || x > X_BOUND + 0.5 || y<-Y_BOUND - 0.5 || y > Y_BOUND + 0.5) {
            printf("Out of Map: (%f,%f)\n", x, y);
        } else if (dist < laser->max_range) {
            setObst(x, y);
        }
    }
}