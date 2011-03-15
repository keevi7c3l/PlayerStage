#include <libplayerc/playerc.h>
#include "laserReader.h"
#include <math.h>

static int obstacle[MAPSIZE_X][MAPSIZE_Y];

LaserReader::LaserReader(playerc_client_t *client,
                         playerc_laser_t *laser,
                         playerc_position2d_t *position2d) : client(client), laser(laser), position2d(position2d) {

}

// relying on X and Y bound being the same

extern int getMatrixValue(double i) {
    return (int) ((i + X_BOUND) * SCALE);
}

extern int isObst(int x, int y) {
    int newX = getMatrixValue(x);
    int newY = getMatrixValue(y);
    if (newX >= 0 && newX < MAPSIZE_X && newY >= 0 && newY < MAPSIZE_X) {
        //printf("obstacle is: %d\n", obstacle[newX][newY]);
        return obstacle[newX][newY];
    }
    return 0;
}

void LaserReader::readLaser() {
    playerc_client_read(client);
    double x, y, angle, dist;
    double robAng = position2d->pa;

    double xPos = position2d->px;
    double yPos = position2d->py;


    int i = 0;
    for (; i <= 360; i += 2) {
        dist = laser->ranges[i];

        angle = 1.5 * M_PI + robAng + DTOR(i / 2.0);

        if (xPos < 0 && yPos < 0 || xPos > 0 && yPos > 0) {
            y = xPos + (sin(angle) * dist);
            x = yPos + (cos(angle) * dist);
        } else {
            y = yPos + (sin(angle) * dist);
            x = xPos + (cos(angle) * dist);
        }

        if (x<-X_BOUND - 0.5 || x > X_BOUND + 0.5 || y<-Y_BOUND - 0.5 || y > Y_BOUND + 0.5) {
            printf("Out of Map: (%f,%f)\n", x, y);
        } else if (dist < 8) {
            //printf("Angle: %d Obstacle at: (%f,%f); distance: %f\n", i / 2, x, y, dist);
            obstacle[getMatrixValue(x)][getMatrixValue(y)] = true;
        }
    }
}