/* 
 * File:   laserReader.h
 * Author: koy
 *
 * Created on February 16, 2011, 10:54 PM
 */

#ifndef LASERREADER_H
#define	LASERREADER_H
#include <libplayerc/playerc.h>

#define X_BOUND 9
#define Y_BOUND 9
#define SCALE 100
#define MAPSIZE_X (X_BOUND*2*SCALE)
#define MAPSIZE_Y (X_BOUND*2*SCALE)

class LaserReader {
public:
    LaserReader(playerc_client_t *client,
            playerc_laser_t *laser,
            playerc_position2d_t *position2d);
    void readLaser();
private:
    playerc_client_t *client;
    playerc_laser_t *laser;
    playerc_position2d_t *position2d;
};

extern int isObst(int x, int y);
extern int getMatrixValue(double i);
extern double getCoorValue(int i);

#endif	/* LASERREADER_H */

