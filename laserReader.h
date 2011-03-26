#ifndef LASERREADER_H
#define	LASERREADER_H
#include <libplayerc/playerc.h>

#define X_BOUND 12 // For building3
#define Y_BOUND 12
#define SCALE 10
#define MAPSIZE_X (X_BOUND*2*SCALE)
#define MAPSIZE_Y (X_BOUND*2*SCALE)
#define PADDING 0.2

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
extern int isSeen(int x, int y);
void setSeen(double x, double y);
extern int getMatrixValue(double i);
extern double getCoorValue(int i);

static bool obstacle[MAPSIZE_X][MAPSIZE_Y] = {false};
static bool seen[MAPSIZE_X][MAPSIZE_Y] = {false};

#endif	/* LASERREADER_H */

