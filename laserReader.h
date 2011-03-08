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
            playerc_ranger_t *laser,
            playerc_position2d_t *position2d);
    void readLaser();
private:
    playerc_client_t *client;
    playerc_ranger_t *laser;
    playerc_position2d_t *position2d;
};

typedef struct astar_pose2d {
    double x, y;
} astar_pose2d_t;

//extern int isObst(int x, int y);
//extern int isObst(double x, double y);
extern bool isObst(astar_pose2d_t pose);

extern uint64_t astar_pose2d_hash(const astar_pose2d_t &p);
extern double astar_scaler(const double xy);


#endif	/* LASERREADER_H */

