#ifndef PLAYERWRAPPER_H
#define	PLAYERWRAPPER_H

#include <iostream>
#include <libplayerc/playerc.h>
#define X_BOUND 12 // For building3
#define Y_BOUND 12
#define SCALE 10
#define MAPSIZE_X (X_BOUND*2*SCALE)
#define MAPSIZE_Y (Y_BOUND*2*SCALE)
#define PADDING 0.2

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



