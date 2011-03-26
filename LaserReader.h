/*
 * File:   LaserReader.h
 * Author: Pit Apps
 *
 * Created on March 26, 2011, 4:43 PM
 *
 * This Class deals with creating an internal map of the environment in real
 * time using the laser data.
 *
 * It puts a virtual grid on the map (whose realitve coordinates are calculated
 * using the getMatrixValue and getCoorValue methods).
 * 
 * It stores the known obstacles in the obstacle array and the known seen points
 * in the seen array.
 */

#ifndef LASERREADER_H
#define	LASERREADER_H
#include <libplayerc/playerc.h>
#include <cmath>
#include "PlayerWrapper.h"

class LaserReader {
public:

    LaserReader(PlayerWrapper *pw) : pw(pw) {
    }
    void readLaser();
    bool isObst(int x, int y);
    bool isSeen(int x, int y);
    int getMatrixValue(double i);
    double getCoorValue(int i);
    void setSeen(double x, double y);
private:
    PlayerWrapper *pw;

    bool obstacle[MAPSIZE_X][MAPSIZE_Y];
    bool seen[MAPSIZE_X][MAPSIZE_Y];

    void setObst(double x, double y);
    void setSeen(double robX, double robY, double dist, double angle);
};

#endif	/* LASERREADER_H */

