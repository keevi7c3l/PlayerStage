#include <iostream>
#include <cmath>
#include "laserReader.h"
#include "astarImpTest.h"

using namespace std;

playerc_client_t *client;
playerc_laser_t *laser;
playerc_position2d_t *position2d;

vector<player_pose2d_t> path;
LaserReader *lr;

bool isArrived(double tx, double ty) {
    playerc_client_read(client);
    return ((abs(position2d->px - tx) <= 0.15) && (abs(position2d->py - ty) <= 0.15));
}

int main() {

    cout << "Starting up robot" << endl;

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

    playerc_position2d_enable(position2d, 1); // Turn on Motors
    playerc_position2d_set_odom(position2d, 0, 0, 0); // Set odometer to zero
    playerc_client_read(client);

    cout << "Creating LaserReader" << endl;
    lr = new LaserReader(client, laser, position2d);
    usleep(10000);

    vector<player_pose2d_t> path;
    cout << "Starting Main Loop" << endl;
    for (;;) {
        lr->readLaser();

        while (!findPath(getMatrixValue(position2d->px), getMatrixValue(position2d->py), getMatrixValue(8), getMatrixValue(8), &path)) {
            cout << "No Path found from Main" << endl;
        }
        path.pop_back();
        path.pop_back();
        player_pose2d_t next = path.back();

        cout << "Going to:" << next.px", " << next.py << endl;
        playerc_position2d_set_cmd_pose(position2d, next.px, next.py, 0, position2d->pa);
        while (!isArrived(next.px, next.py)) {
        }
    }

    //Disconnect player
    playerc_laser_unsubscribe(laser);
    playerc_laser_destroy(laser);
    playerc_client_disconnect(client);
    playerc_client_destroy(client);

    return 0;
}