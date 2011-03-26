#include "PlayerWrapper.h"

PlayerWrapper::PlayerWrapper(int port) {
    createClient(port);
    createLaser();
    createP2D();
    createRest();
}

PlayerWrapper::~PlayerWrapper() {
    playerc_laser_unsubscribe(laser);
    playerc_laser_destroy(laser);
    playerc_position2d_unsubscribe(position2d);
    playerc_position2d_destroy(position2d);
    playerc_client_disconnect(client);
    playerc_client_destroy(client);
}

int PlayerWrapper::createClient(int port) {
    client = playerc_client_create(NULL, "localhost", port);
    if (playerc_client_connect(client) != 0) {
        return -1;
    }
    return 0;
}

int PlayerWrapper::createLaser() {
    laser = playerc_laser_create(client, 0);
    if (playerc_laser_subscribe(laser, PLAYERC_OPEN_MODE)) {
        return -1;
    }
    return 0;
}

int PlayerWrapper::createP2D() {
    position2d = playerc_position2d_create(client, 0);
    if (playerc_position2d_subscribe(position2d, PLAYERC_OPEN_MODE) != 0) {
        fprintf(stderr, "error: %s\n", playerc_error_str());
        return -1;
    }
    return 0;
}

int PlayerWrapper::createRest() {
    if (playerc_client_datamode(client, PLAYERC_DATAMODE_PULL) != 0) {
        fprintf(stderr, "error: %s\n", playerc_error_str());
        return -1;
    }

    if (playerc_client_set_replace_rule(client, -1, -1, PLAYER_MSGTYPE_DATA, -1, 1) != 0) {
        fprintf(stderr, "error: %s\n", playerc_error_str());
        return -1;
    }

    return 0;
}

void PlayerWrapper::goTo(double x, double y) {
    return goTo(player_pose2d_t{x, y, 0});
}

void PlayerWrapper::goTo(player_pose2d_t pose) {
    playerc_position2d_set_cmd_pose(position2d, pose.px, pose.py, 0, pose.pa);
}

double PlayerWrapper::getRange(int i) {
    return laser->ranges[i];
}

int PlayerWrapper::getLaserCount() {
    return laser->scan_count;
}

double PlayerWrapper::getMaxRange() {
    return laser->max_range;
}

void PlayerWrapper::readClient() {
    playerc_client_read(client);
}

double PlayerWrapper::getRobX() {
    return position2d->px;
}

double PlayerWrapper::getRobY() {
    return position2d->py;
}

double PlayerWrapper::getRobA() {
    return position2d->pa;
}