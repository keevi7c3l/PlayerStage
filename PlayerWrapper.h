/*
 * File:   PlayerWrapper.h
 * Author: koy
 *
 * Created on February 3, 2011, 9:06 PM
 * This Class creates a robot using the playerc libraries and attaches
 * laser ans position2d proxies to it.
 * It also simplifies access to frequently used methods.
 */

#ifndef PLAYERWRAPPER_H
#define	PLAYERWRAPPER_H

#include <iostream>
#include <libplayerc/playerc.h>

class PlayerWrapper {
public:
    PlayerWrapper(int port);
    ~PlayerWrapper();
    void readClient();

    double getRange(int);
    int getLaserCount();
    double getMaxRange();

    void goTo(player_pose2d_t pose);
    double getRobX();
    double getRobY();
    double getRobA();
private:
    playerc_client_t *client;
    playerc_laser_t *laser;
    playerc_position2d_t *position2d;
    int createClient(int);
    int createLaser();
    int createP2D();
    int createRest();
};
#endif	/* PLAYERWRAPPER_H */



