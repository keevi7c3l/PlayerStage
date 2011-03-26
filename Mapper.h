/* 
 * File:   Mapper.h
 * Author: koy
 *
 * Created on March 26, 2011, 4:43 PM
 */

#ifndef MAPPER_H
#define	MAPPER_H
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "laserReader.h"
#include "PlayerWrapper.h"

class Mapper {
public:
    Mapper(int width = 500, int height = 500);
    void drawInternalMap(std::vector<player_pose2d_t> *path, LaserReader *lr);
    void drawMap(PlayerWrapper *pw);
    void saveMap();
private:
    bool isMap, isIntMap;
    int width, height, centreX, centreY;
    CvScalar bckgrndCol, objCol, freeCol;
    CvPoint pt, pt1;
    IplImage* image;
    IplImage* internalImage;
    std::string map_window_name;
    std::string internal_window_name;

    void mapInit();
    void intMapInit();
    void drawPath(std::vector<player_pose2d_t> *path);
};


#endif	/* MAPPER_H */

