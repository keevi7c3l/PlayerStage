/* 
 * File:   laserReader.h
 * Author: koy
 *
 * Created on February 16, 2011, 10:54 PM
 */

#ifndef LASERREADER_H
#define	LASERREADER_H
#include <libplayerc/playerc.h>

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
extern int isObst(double x, double y);

#endif	/* LASERREADER_H */

