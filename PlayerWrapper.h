/*
 * File:   PlayerWrapper.h
 * Author: Pit Apps
 *
 * Created on March 26, 2011, 4:43 PM
 *
 * This Class wraps a handful of useful player commands and initializes the robot.
 * This makes it simpler to change global access to player commands whenever command
 * names change when updating to newer versions of player.
 *
 * This header also has some constants which determine the precision of the
 * algorithms used (SCALE, PADDING etc.)
 */

#ifndef PLAYERWRAPPER_H
#define	PLAYERWRAPPER_H

#include <iostream>
#include <libplayerc/playerc.h>
#define X_BOUND 13 // For eval
#define Y_BOUND 13
#define SCALE 10
#define MAPSIZE_X (X_BOUND*2*SCALE)
#define MAPSIZE_Y (Y_BOUND*2*SCALE)
#define PADDING 0.39 // Padding used around obstacles

class PlayerWrapper {
public:
    PlayerWrapper(int port);
    ~PlayerWrapper();
    void readClient();

    // Laser Methods
    double getRange(int);
    int getLaserCount();
    double getMaxRange();

    // P2D Methods
    void goTo(player_pose2d_t pose);
    void goTo(double x, double y);
    double getRobX();
    double getRobY();
    double getRobA();

    // Fiducial Methods
    int getFidCount();
    int getFidID(int i);
    double getFidX(int i);
    double getFidY(int i);
private:
    playerc_client_t *client;
    playerc_laser_t *laser;
    playerc_position2d_t *position2d;
    playerc_fiducial_t *fiducial;
    
    int createClient(int);
    int createLaser();
    int createP2D();
    int createFid();
    int createRest();
};
#endif	/* PLAYERWRAPPER_H */



