#include <iostream>
#include <cmath>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "laserReader.h"
#include "astarImpTest.h"
#include "PlayerWrapper.h"

using namespace std;

PlayerWrapper *pw;

vector<player_pose2d_t> path;
LaserReader *lr;

//OPENCV Map
bool isMap = false;
bool isIntMap = false;
int width = 500, height = 500; //Map size in pixels
int centreX = (width / 2), centreY = (height / 2);
CvPoint pt, pt1;
CvScalar bckgrndCol = CV_RGB(92, 92, 92);
CvScalar objCol = CV_RGB(0, 0, 0);
CvScalar freeCol = CV_RGB(150, 150, 150);

char map_window_name[] = "Map";
IplImage* image = cvCreateImage(cvSize(width, height), 8, 3);

char internal_window_name[] = "Internal Map";
IplImage* internalImage = cvCreateImage(cvSize(width, height), 8, 3);

bool isArrived(double tx, double ty) {
    pw->readClient();
    return ((abs(pw->getRobX() - tx) <= 0.15) && (abs(pw->getRobY() - ty) <= 0.15));
}

void printPath() {
    std::vector<player_pose2d_t>::iterator it = path.begin();
    while (it != path.end()) {
        cout << "(" << it->px << ", " << it->py << ")" << endl;
        it++;
    }
}

/* Convert floating point to integer and round up*/
int floatToInt(float num) {
    int temp = (int) (num * 100);
    temp = temp % 10;
    if (temp >= 5) return (int) ((num * 10) + 1);
    else return (int) (num * 10);
}

void mapInit() {
    pt.x = centreX;
    pt.y = centreY;
    cvNamedWindow(map_window_name, 1);
    cvSet(image, bckgrndCol, 0);
    isMap = true;
}

void intMapInit() {
    cvNamedWindow(internal_window_name, 1);
    cvSet(internalImage, bckgrndCol, 0);
    isIntMap = true;
}

void drawMap() {
    if (!isMap) mapInit();
    float dist, angle;
    for (int i = 0; i < pw->getLaserCount(); i++) {
        dist = pw->getRange(i);
        if (dist < pw->getMaxRange()) {
            angle = (1.5 * M_PI) + DTOR(i) + pw->getRobA();
            pt1.x = (centreX + floatToInt(pw->getRobX()));
            pt1.y = (centreY - floatToInt(pw->getRobY()));
            pt.y = (int) (pt1.y - (sin(angle) * dist * 10));
            pt.x = (int) (pt1.x + (cos(angle) * dist * 10));
            cvLine(image, pt1, pt, freeCol, 1, 4, 0); //free
            cvLine(image, pt, pt, objCol, 1, 4, 0); //object
            cvShowImage(map_window_name, image);
            cvWaitKey(10);
        }
    }
}

void drawPath() {
    if (!isIntMap) intMapInit();
    CvPoint first, second;
    std::vector<player_pose2d_t>::iterator it = path.end();
    while (it > path.begin() + 1) {
        first.x = it->px * 10 + X_BOUND * 10;
        first.y = it->py * 10 + Y_BOUND * 10;
        it--;
        second.x = it->px * 10 + X_BOUND * 10;
        second.y = it->py * 10 + Y_BOUND * 10;
        it--;
        cvLine(internalImage, first, second, cvScalar(255, 0, 0), 1, 4, 0);
    }
    cvShowImage(internal_window_name, image);
    cvWaitKey(10);
}

void drawInternalMap() {
    if (!isIntMap) intMapInit();
    cvSet(internalImage, bckgrndCol);
    for (int x = 0; x < MAPSIZE_X; x++) {
        for (int y = 0; y < MAPSIZE_Y; y++) {
            if (isSeen(x, y)) {
                pt.x = (getCoorValue(x)*10 + X_BOUND * 10);
                pt.y = (getCoorValue(y)*10 + Y_BOUND * 10);
                cvLine(internalImage, pt, pt, freeCol, 1, 4, 0);
            }
            if (isObst(x, y)) {
                pt.x = (getCoorValue(x)*10 + X_BOUND * 10);
                pt.y = (getCoorValue(y)*10 + Y_BOUND * 10);
                cvLine(internalImage, pt, pt, objCol, 1, 4, 0);
            }
        }
    }
    drawPath();
    cvFlip(internalImage, NULL, 0);
    cvShowImage(internal_window_name, internalImage);
    cvWaitKey(10);
}

player_pose2d_t calcChange() {
    double deltaX, deltaY;
    double deltas = 0;
    int counter = 0;
    std::vector<player_pose2d_t>::iterator it = path.end();
    int i = path.size() - 2;
    while (it != path.begin()) {
        player_pose2d_t thisPose = path[i];
        player_pose2d_t lastPose = path[i + 1];
        deltaX = abs(thisPose.px - lastPose.px);
        deltaY = abs(thisPose.py - lastPose.py);
        double newDeltas = deltaX + deltaY;
        if (newDeltas != deltas && counter > 2) {
            printf("Returining Path: %f, %f\n", path[i + 1].px, path[i + 1].py);
            return path[i + 1];
        }
        deltas = newDeltas;
        float dx = path[i].px - path[path.size() - 1].px;
        float dy = path[i].py - path[path.size() - 1].py;
        if (sqrt((dx * dx)+(dy * dy)) > 1) {
            return path[i + 1];
        }
        it--;
        i--;
        counter++;
    }
}

int main() {

    cout << "Starting up robot" << endl;
    pw = new PlayerWrapper(6665);

    cout << "Creating LaserReader" << endl;
    lr = new LaserReader(pw);
    lr->readLaser();
    printf("Current position is: %f,%f\n", pw->getRobX(), pw->getRobY());

    cout << "Starting Main Loop" << endl;
    while (true) {
        lr->readLaser();

        player_pose2d_t nextDest = findClosest(pw->getRobX(), pw->getRobY());
        printf("Looking for path between: (%f, %f) and (%f, %f)\n", pw->getRobX(), pw->getRobY(), nextDest.px, nextDest.py);

        while (!findPath(getMatrixValue(pw->getRobX()), getMatrixValue(pw->getRobY()), getMatrixValue(nextDest.px), getMatrixValue(nextDest.py), &path)) {
            //while (!findPath(getMatrixValue(position2d->px), getMatrixValue(position2d->py), getMatrixValue(8), getMatrixValue(8), &path)) {
            cout << "No Path found from " << "(" << pw->getRobX() << ", " << pw->getRobY() << ")" << " to " << "(" << nextDest.px << ", " << nextDest.py << ")" << endl;
            pw->readClient();
            setSeen(nextDest.px, nextDest.py);
            break;
        }

        player_pose2d_t nextPoint = calcChange();

        cout << "Going to:" << "(" << nextPoint.px << ", " << nextPoint.py << ")" << endl;
        pw->goTo(nextPoint);
        while (!isArrived(nextPoint.px, nextPoint.py)) {
            lr->readLaser();
            drawInternalMap();
            //drawMap();
        }
    }


    cout << "Arrived at Destination" << endl;
    //cvSaveImage("mappppa.jpg", image, 0);

    //Disconnect player
    delete pw;

    return 0;
}