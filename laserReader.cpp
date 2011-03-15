#include <libplayerc/playerc.h>
#include "laserReader.h"
#include <math.h>

#define DEG2RAD(a) ((a * M_PI) / 180)
#define X_BOUND 8
#define Y_BOUND 8
#define MAPSIZE_X (X_BOUND*2*10)
#define MAPSIZE_Y (X_BOUND*2*10)

static int obstacle[MAPSIZE_X][MAPSIZE_Y];

LaserReader::LaserReader(playerc_client_t *client,
                         playerc_laser_t *laser,
                         playerc_position2d_t *position2d) : client(client), laser(laser), position2d(position2d) {

}

int getMatrixValue(double i) {
    return (int) ((i + 8)*10);
}

extern int isObst(int x, int y) {
    int newX = getMatrixValue(x);
    int newY = getMatrixValue(y);
    //printf("Checking: %d,%d\n", x, y);
    if (newX >= 0 && newX < MAPSIZE_X && newY >= 0 && newY < MAPSIZE_X) {
        //printf("obstacle is: %d\n", obstacle[newX][newY]);
        return obstacle[newX][newY];
    }
    return 0;
}

extern int isObst(double x, double y) {
    int newX = getMatrixValue(x);
    int newY = getMatrixValue(y);
    //printf("Checking: %f,%f\n", x, y);
    if (newX >= 0 && newX < MAPSIZE_X && newY >= 0 && newY < MAPSIZE_X) {
        //printf("obstacle is: %d\n", obstacle[newX][newY]);
        return obstacle[newX][newY];
    }
    return 0;
}

void LaserReader::readLaser() {
    double x, y, angle, dist;
    double robAng = position2d->pa;

    int i = 0;
    for (; i <= 360; i += 2) {
        dist = laser->ranges[i];

        angle = ((1.5 * M_PI) + DEG2RAD(i / 2) + robAng);
        //TODO: error here.
        x = (position2d->px + (cos(angle) * dist));
        y = (position2d->py + (sin(angle) * dist));

        if (x<-X_BOUND || x > X_BOUND || y<-Y_BOUND || y > Y_BOUND) {
            //printf("Out of Map: (%f,%f)\n", x, y);
        } else {
            if (dist < 8) {
                //printf("Obstacle at: (%f,%f)\n", x, y);
                //printf("Obstacle at: (%d,%d)\n", getMatrixValue(x), getMatrixValue(y));
                int obstX = getMatrixValue(x);
                int obstY = getMatrixValue(y);
                if (obstX < 0 || obstX > MAPSIZE_X || obstY < 0 || obstY > MAPSIZE_Y) {
                } else {
                    obstacle[obstX][obstY] = 1;
                }
            }
        }
    }

}
