#include "Mapper.h"

/*
 * Constructor for the Mapper class; takes the desired width and height of the OpenCv
 * Map.
 */
Mapper::Mapper(int width, int height) : isMap(false), isIntMap(false), width(width), height(height) {
    this->centreX = (width / 8);
    this->centreY = (height / 8);
    this->bckgrndCol = CV_RGB(100, 100, 100);
    this->objCol = CV_RGB(0, 0, 0);
    this->freeCol = CV_RGB(225, 225, 225);

    this->map_window_name = "Map";
    this->image = cvCreateImage(cvSize(width, height), 8, 3);

    this->internal_window_name = "Internal Map";
    this->internalImage = cvCreateImage(cvSize(width, height), 8, 3);
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
    std::vector<player_pose2d_t>::iterator it = path.end()-1;
    while (it > path.begin() + 1) {
        first.x = (*it).px * 15 + X_BOUND * 15;
        first.y = (*it).py * 15 + Y_BOUND * 15;
        it--;
        second.x = (*it).px * 15 + X_BOUND * 15;
        second.y = (*it).py * 15 + Y_BOUND * 15;
        it--;
        cvLine(internalImage, first, second, cvScalar(255, 0, 0), 1, 4, 0);
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
            //cvShowImage(map_window_name.c_str(), image);
            //cvWaitKey(10);
        }
    }
}

/*
 * Draws the internal map.
 */
void Mapper::drawInternalMap(std::vector<player_pose2d_t> path, LaserReader *lr) {
    if (!isIntMap) intMapInit();
    cvSet(internalImage, bckgrndCol);
    for (int x = 0; x < MAPSIZE_X; x++) {
        for (int y = 0; y < MAPSIZE_Y; y++) {
            double newX = lr->getCoorValue(x);
            double newY = lr->getCoorValue(y);
            if (lr->isSeen(x, y)) {
                pt.x = (newX * 15 + X_BOUND * 15);
                pt.y = (newY * 15 + Y_BOUND * 15);
                cvLine(internalImage, pt, pt, freeCol, 2, 4, 0);
            }
            if (lr->isObst(x, y)) {
                pt.x = (newX * 15 + X_BOUND * 15);
                pt.y = (newY * 15 + Y_BOUND * 15);
                cvLine(internalImage, pt, pt, objCol, 2, 4, 0);
            }
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
    std::string jpg = ".jpg";
    std::string mapExt = map_window_name + jpg;
    std::string mapInt = internal_window_name + jpg;
    cvSaveImage(mapExt.c_str(), image, 0);
    cvSaveImage(mapInt.c_str(), internalImage, 0);
}