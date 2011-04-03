/*
 * File:   DataReader.h
 * Author: Pit Apps
 *
 * Created on March 26, 2011, 4:43 PM
 *
 * This Class deals with creating an internal map of the environment in real
 * time using the laser and fiducial data.
 *
 * It puts a virtual grid on the map (whose realitve coordinates are calculated
 * using the getMatrixValue and getCoorValue methods).
 * 
 * It stores the known obstacles in the obstacle array, the known seen points
 * in the seen array and the known fiducials in the fiducial array.
 */

#ifndef DATAREADER_H
#define	DATAREADER_H
#include <cmath>
#include "PlayerWrapper.h"

/* Fidcuials ID list */
enum FIDID {
    FIRE = 105, DEAD = 19, LIVING = 40, NONE = 0
};

/* Class that holds all information about fiducials */
class Fiducial {
public:

    Fiducial() : id(NONE) {

    }

    Fiducial(int id, double x, double y) : id(id), x(x), y(y) {

    }

    int getId() {
        return id;
    }

    double getX() {
        return x;
    }

    double gety() {
        return y;
    }

private:

    int id;
    double x, y;
};

/* Main DR class which holds all datastructers concerning proxy readings */
class DataReader {
public:

    DataReader(PlayerWrapper *pw) : pw(pw) {
    }
    void readLaser();
    void readFid();

    bool isObst(int x, int y);
    bool isObst(player_pose2d_t pose);
    bool isSeen(int x, int y);
    bool isSeen(player_pose2d_t pose);
    int returnFid(int x, int y);
    bool isInMap(int x, int y);

    int getMatrixValue(double i);
    double getCoorValue(int i);

    void setIsland(double sx, double sy);
private:
    PlayerWrapper *pw;

    bool obstacle[MAPSIZE_X][MAPSIZE_Y];
    bool seen[MAPSIZE_X][MAPSIZE_Y];
    Fiducial fiducials[MAPSIZE_X][MAPSIZE_Y]; // We use one array for all fids because we suppose no two fiducials occupy the same coordinates

    void setObst(double x, double y);
    void setSeen(double robX, double robY, double dist, double angle);
    void setIsland(int mx, int my);
    void setFid(int id, double x, double y);
};

#endif	/* LASERREADER_H */

