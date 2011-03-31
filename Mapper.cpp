#include "Mapper.h"

/*
 * Constructor for the Mapper class; takes the desired width and height of the OpenCv
 * Map.
 */
Mapper::Mapper(int width, int height, DataReader *dr) : isMap(false), isIntMap(false), width(width), height(height) {
    this->centreX = (width / 8);
    this->centreY = (height / 8);
    this->bckgrndCol = CV_RGB(100, 100, 100);
    this->objCol = CV_RGB(0, 0, 0);
    this->freeCol = CV_RGB(225, 225, 225);

    this->map_window_name = "Map";
    this->image = cvCreateImage(cvSize(width, height), 8, 3);

    this->internal_window_name = "Internal Map";
    this->internalImage = cvCreateImage(cvSize(500, 500), 8, 3);

    this->dr = dr;
}

/* 
 * Convert floating point to integer and round up.
 */
int floatToInt(float num) {
    int temp = (int) (num * 100);
    temp %= 10;
    if (temp >= 5) return (int) ((num * 10) + 1);
    else return (int) (num * 10);
}

/*
 * Initializes the external map.
 */
void Mapper::mapInit() {
    //cvNamedWindow(map_window_name.c_str(), 1);
    cvSet(image, bckgrndCol, 0);
    isMap = true;
}

/*
 * Initializes the internal map.
 */
void Mapper::intMapInit() {
    cvNamedWindow(internal_window_name.c_str(), 1);
    cvSet(internalImage, bckgrndCol, 0);
    isIntMap = true;
}

/*
 * Draws the current projected path to the internal map.
 */
void Mapper::drawPath(std::vector<player_pose2d_t> path) {
    if (!isIntMap) intMapInit();
    CvPoint first, second;
    if (path.empty()) return;
    std::vector<player_pose2d_t>::iterator it = path.end() - 1;
    while (it > path.begin() + 1) {
        first.x = (*it).px * 15 + X_BOUND * 15;
        first.y = (*it).py * 15 + Y_BOUND * 15;
        it--;
        second.x = (*it).px * 15 + X_BOUND * 15;
        second.y = (*it).py * 15 + Y_BOUND * 15;
        it--;
        cvLine(internalImage, first, second, CV_RGB(0, 0, 255), 1, 4, 0);
    }
}

/*
 * Draws the external map.
 */
void Mapper::drawMap(PlayerWrapper *pw) {
    if (!isMap) mapInit();
    pt.x = centreX;
    pt.y = centreY;
    float dist, angle;
    for (int i = 0; i < pw->getLaserCount(); i++) {
        dist = pw->getRange(i);
        if (dist < pw->getMaxRange()) {
            angle = DTOR(i - 180) + pw->getRobA();
            pt1.x = (centreX + floatToInt(pw->getRobX()))*4;
            pt1.y = (centreY - floatToInt(pw->getRobY()))*4;
            pt.x = (int) (pt1.x + (cos(angle) * dist * 40));
            pt.y = (int) (pt1.y - (sin(angle) * dist * 40));
            cvLine(image, pt1, pt, freeCol, 1, 4, 0); //free
            cvLine(image, pt, pt, objCol, 2, 4, 0); //object
           // cvShowImage(map_window_name.c_str(), image);
            //cvWaitKey(10);
        }
    }
//        for (int x = 0; x < MAPSIZE_X; x++) {
//            for (int y = 0; y < MAPSIZE_Y; y++) {
//                drawFid(dr->getCoorValue(x), dr->getCoorValue(y), dr->returnFid(x, y), image);
//            }
//        }
}

/*
 * Draws fiducials onto a given IplImage (image of a map)
 */
void Mapper::drawFid(double x, double y, int fid, IplImage *image) {
    if (fid != 0) {
        /* Drawing on the big Map */
        if (image->height > 500) {
            /* Needs fixing */
            pt.x = (x * 40) + 4 * centreX;
            pt.y = (y * 40);
        } else {
            pt.x = (x * 15 + X_BOUND * 15);
            pt.y = (y * 15 + Y_BOUND * 15);
        }
        if (fid == FIRE) {
            cvCircle(image, pt, 10, CV_RGB(255, 215, 0), -1);
        } else {
            /* Needs improving */
            CvPoint one = (CvPoint){pt.x - 10, pt.y - 10};
            CvPoint two = (CvPoint){pt.x + 10, pt.y + 10};
            if (fid == DEAD) {
                cvRectangle(image, one, two, CV_RGB(255, 0, 0), -1);
            } else if (fid == LIVING) {
                cvRectangle(image, one, two, CV_RGB(222, 184, 135), -1);
            }
        }
    }
}

/*
 * Draws the internal map.
 */
void Mapper::drawInternalMap(std::vector<player_pose2d_t> path) {
    if (!isIntMap) intMapInit();
    cvSet(internalImage, bckgrndCol);
    for (int x = 0; x < MAPSIZE_X; x++) {
        for (int y = 0; y < MAPSIZE_Y; y++) {
            double newX = dr->getCoorValue(x);
            double newY = dr->getCoorValue(y);
            pt.x = (newX * 15 + X_BOUND * 15);
            pt.y = (newY * 15 + Y_BOUND * 15);
            /* Check for Seen */
            if (dr->isSeen(x, y)) {
                cvLine(internalImage, pt, pt, freeCol, 2, 4, 0);
            }
            /* Check for obstacles */
            if (dr->isObst(x, y)) {
                cvLine(internalImage, pt, pt, objCol, 2, 4, 0);
            }
            drawFid(newX, newY, dr->returnFid(x, y), internalImage);
        }
    }
    drawPath(path);
    cvFlip(internalImage, NULL, 0);
    cvShowImage(internal_window_name.c_str(), internalImage);
    cvWaitKey(10);
}

/*
 * Saves both the internal and external maps as .jpg files.
 */
void Mapper::saveMap() {

    /* Adding fiducials to Main Map */
    for (int x = 0; x < MAPSIZE_X; x++) {
        for (int y = 0; y < MAPSIZE_Y; y++) {
            drawFid(dr->getCoorValue(x), dr->getCoorValue(y), dr->returnFid(x, y), image);
        }
    }
    std::string jpg = ".jpg";
    std::string mapExt = map_window_name + jpg;
    std::string mapInt = internal_window_name + jpg;
    cvSaveImage(mapExt.c_str(), image, 0);
    cvSaveImage(mapInt.c_str(), internalImage, 0);
}