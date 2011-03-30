/* 
 * File:   Mapper.h
 * Author: Pit Apps
 *
 * Created on March 26, 2011, 4:43 PM
 *
 * This Class deals with creating a human-readable Map using OpenCV.
 */

#ifndef MAPPER_H
#define	MAPPER_H
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "DataReader.h"
#include "PlayerWrapper.h"

class Mapper {
public:
    Mapper(int width, int height, DataReader *dr);
    void drawInternalMap(std::vector<player_pose2d_t> path);
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

    DataReader *dr;

    void mapInit();
    void intMapInit();
    void drawPath(std::vector<player_pose2d_t> path);
    void drawFid(double x, double y,int fid, IplImage *image);
};


#endif	/* MAPPER_H */

