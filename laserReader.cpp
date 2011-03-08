#include <libplayerc/playerc.h>
#include "laserReader.h"
#include <math.h>
#include <unordered_map>

#define DEG2RAD(a) ((a * M_PI) / 180)
#define X_BOUND 8
#define Y_BOUND 8
#define MAPSIZE_X (X_BOUND*2*10)
#define MAPSIZE_Y (X_BOUND*2*10)
#define SCALE 100

static int obstacle[MAPSIZE_X][MAPSIZE_Y];
std::unordered_map<uint64_t, bool> obstacle2;

LaserReader::LaserReader(playerc_client_t *client,
                         playerc_ranger_t *laser,
                         playerc_position2d_t *position2d) : client(client), laser(laser), position2d(position2d) {

}

extern uint64_t astar_pose2d_hash(const astar_pose2d_t &p) {
    return (((uint64_t) p.x << 32) | (uint64_t) p.y);


}

extern double astar_scaler(const double xy) {
    int temp = (int) (xy * SCALE);
    return (double) (temp / SCALE);
}

int getMatrixValue(double i) {
    return (int) ((i + 8)*10);
}

/*
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
    return 1;
}*/

extern bool isObst(astar_pose2d_t pose) {
    return obstacle2[astar_pose2d_hash(pose)];
}

void LaserReader::readLaser() {
    double x, y, angle, dist;
    double robAng = position2d->pa;

    //int xPos = position2d->px;
    //int yPos = position2d->py;
    const astar_pose2d_t currPos = {position2d->px, position2d->py};
    astar_pose2d_t obsPos;

    int i = 0;
    for (; i <= 360; i += 2) {
        dist = laser->ranges[i];

        angle = 1.5 * M_PI + robAng + DTOR(i / 2.0);

        if (currPos.x < 0 && currPos.y < 0 || currPos.x > 0 && currPos.y > 0) {
            obsPos.y = astar_scaler(currPos.x + (sin(angle) * dist));
            obsPos.x = astar_scaler(currPos.y + (cos(angle) * dist));
        } else {
            obsPos.y = astar_scaler(currPos.y + (sin(angle) * dist));
            obsPos.x = astar_scaler(currPos.x + (cos(angle) * dist));
        }

        if (obsPos.x<-X_BOUND || obsPos.x > X_BOUND || obsPos.y<-Y_BOUND || obsPos.y > Y_BOUND) {
            //printf("Out of Map: (%f,%f)\n", x, y);
        } else {
            if (dist < 8) {
                //printf("Obstacle at: (%f,%f)\n", x, y);
                //printf("Obstacle at: (%d,%d)\n", getMatrixValue(x), getMatrixValue(y));
                //int obstX = getMatrixValue(x);
                //int obstY = getMatrixValue(y);
                // if (obstX < 0 || obstX > MAPSIZE_X || obstY < 0 || obstY > MAPSIZE_Y) {
                //} else {
                //obstacle[obstX][obstY] = 1;
                obstacle2[astar_pose2d_hash(obsPos)] = true;
                //}
            }
        }
    }

}
