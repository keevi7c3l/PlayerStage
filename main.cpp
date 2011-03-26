#include <iostream>
#include <cmath>
#include "laserReader.h"
#include "astarImpTest.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <fstream>

#define X_GOAL 8
#define Y_GOAL 8

using namespace std;

playerc_client_t *client;
playerc_laser_t *laser;
playerc_position2d_t *position2d;

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
    playerc_client_read(client);
    return ((abs(position2d->px - tx) <= 0.15) && (abs(position2d->py - ty) <= 0.15));
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

int init() {
    /* Set up player/stage connectivity*/
    client = playerc_client_create(NULL, "localhost", 6665);
    if (playerc_client_connect(client) != 0)
        return -1;

    /* Set up laser connectivity*/
    laser = playerc_laser_create(client, 0);
    if (playerc_laser_subscribe(laser, PLAYERC_OPEN_MODE))
        return -1;

    /* Set up position2d connectivity*/
    position2d = playerc_position2d_create(client, 0);
    if (playerc_position2d_subscribe(position2d, PLAYERC_OPEN_MODE) != 0) {
        fprintf(stderr, "error: %s\n", playerc_error_str());
        return -1;
    }

    if (playerc_client_datamode(client, PLAYERC_DATAMODE_PULL) != 0) {
        fprintf(stderr, "error: %s\n", playerc_error_str());
        return -1;
    }

    if (playerc_client_set_replace_rule(client, -1, -1, PLAYER_MSGTYPE_DATA, -1, 1) != 0) {
        fprintf(stderr, "error: %s\n", playerc_error_str());
        return -1;
    }

    playerc_position2d_enable(position2d, 1);
    playerc_position2d_set_odom(position2d, 0, 0, 0);
    playerc_client_read(client);

    return 0;
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
    for (int i = 0; i < laser->scan_count; i++) {
        dist = laser->ranges[i];
        if (dist < laser->max_range) {
            angle = (1.5 * M_PI) + DTOR(i) + position2d->pa;
            pt1.y = (centreY - floatToInt(position2d->py));
            pt1.x = (centreX + floatToInt(position2d->px));
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
        first.x = it->px * 10 + 80;
        first.y = it->py * 10 + 80;
        it--;
        second.x = it->px * 10 + 80;
        second.y = it->py * 10 + 80;
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
                pt.x = (getCoorValue(x)*10 + 80);
                pt.y = (getCoorValue(y)*10 + 80);
                cvLine(internalImage, pt, pt, freeCol, 1, 4, 0);
            }
            if (isObst(x, y)) {
                pt.x = (getCoorValue(x)*10 + 80);
                pt.y = (getCoorValue(y)*10 + 80);
                cvLine(internalImage, pt, pt, objCol, 1, 4, 0);
            }
        }
    }
    drawPath();
    cvFlip(internalImage, NULL, 0);
    cvShowImage(internal_window_name, internalImage);
    cvWaitKey(10);
}

std::vector<player_pose2d_t> calcChange() {
    double deltaX, deltaY;
    double deltas = 0;
    std::vector<player_pose2d_t> newPath;
    std::vector<player_pose2d_t>::iterator it = path.end();
    int i = path.size() - 2;
    while (it != path.begin()) {
        player_pose2d_t thisPose = path[i];
        player_pose2d_t lastPose = path[i + 1];
        deltaX = abs(thisPose.px - lastPose.px);
        deltaY = abs(thisPose.py - lastPose.py);
        double newDeltas = deltaX + deltaY;
        if (newDeltas != deltas) {
            newPath.push_back(path[i + 1]);
        }
        deltas = newDeltas;
        float dx = path[i].px - path[path.size() - 1].px;
        float dy = path[i].py - path[path.size() - 1].py;
        if (sqrt((dx * dx)+(dy * dy)) > 1) {
            break;
        }
        it--;
        i--;
    }
    return newPath;
}

int main() {

    cout << "Starting up robot" << endl;
    if (init() == -1) return -1;

    cout << "Creating LaserReader" << endl;
    lr = new LaserReader(client, laser, position2d);
    lr->readLaser();
    printf("Current position is: %f,%f\n", position2d->px, position2d->py);

    cout << "Starting Main Loop" << endl;
    while (true) {
        lr->readLaser();

        player_pose2d_t nextDest = findClosest(position2d->px, position2d->py);
        printf("Looking for path between: (%f, %f) and (%f, %f)\n", position2d->px, position2d->py, nextDest.px, nextDest.py);

        //while (!findPath(getMatrixValue(position2d->px), getMatrixValue(position2d->py), getMatrixValue(nextDest.px), getMatrixValue(nextDest.py), &path)) {
        while (!findPath(getMatrixValue(position2d->px), getMatrixValue(position2d->py), getMatrixValue(8), getMatrixValue(8), &path)) {
            cout << "No Path found from " << "(" << position2d->px << ", " << position2d->py << ")" << " to " << "(" << nextDest.px << ", " << nextDest.py << ")" << endl;
            playerc_client_read(client);
            setSeen(nextDest.px, nextDest.py);
            break;
        }

        if (path.empty()) continue;
        if (path.size() > 2) path.pop_back();
        path.pop_back(); // popping origin
        player_pose2d_t nextPoint = path.back();

        cout << "Going to:" << "(" << nextPoint.px << ", " << nextPoint.py << ")" << endl;
        playerc_position2d_set_cmd_pose(position2d, nextPoint.px, nextPoint.py, 0, position2d->pa);
        while (!isArrived(nextPoint.px, nextPoint.py)) {
            lr->readLaser();
            //drawMap();
            drawInternalMap();
        }
    }


    cout << "Arrived at Destination" << endl;
    //cvSaveImage("mappppa.jpg", image, 0);

    //Disconnect player
    playerc_laser_unsubscribe(laser);
    playerc_laser_destroy(laser);
    playerc_client_disconnect(client);
    playerc_client_destroy(client);

    return 0;
}