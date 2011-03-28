#include "PlayerWrapper.h"

/*
 * Contructor for PlayerWrapper. Takes the port stage is listening on.
 */
PlayerWrapper::PlayerWrapper(int port) {
    createClient(port);
    createLaser();
    createP2D();
    createFid();
    createRest();
}

/*
 * Destructor for PlayerWrapper. Destroys the player proxies.
 */
PlayerWrapper::~PlayerWrapper() {
    playerc_laser_unsubscribe(laser);
    playerc_laser_destroy(laser);
    playerc_position2d_unsubscribe(position2d);
    playerc_position2d_destroy(position2d);
    playerc_fiducial_unsubscribe(fiducial);
    playerc_fiducial_destroy(fiducial);
    playerc_client_disconnect(client);
    playerc_client_destroy(client);
}

/*
 * Creates the robot client.
 */
int PlayerWrapper::createClient(int port) {
    client = playerc_client_create(NULL, "localhost", port);
    if (playerc_client_connect(client) != 0) {
        return -1;
    }
    return 0;
}

/*
 * Creates the laser proxy.
 */
int PlayerWrapper::createLaser() {
    laser = playerc_laser_create(client, 0);
    if (playerc_laser_subscribe(laser, PLAYERC_OPEN_MODE)) {
        return -1;
    }
    return 0;
}

/*
 * Creates the Position2D proxy.
 */
int PlayerWrapper::createP2D() {
    position2d = playerc_position2d_create(client, 0);
    if (playerc_position2d_subscribe(position2d, PLAYERC_OPEN_MODE) != 0) {
        fprintf(stderr, "error: %s\n", playerc_error_str());
        return -1;
    }
    return 0;
}

/*
 * Creates the Fiducial proxy.
 */
int PlayerWrapper::createFid() {
    fiducial = playerc_fiducial_create(client, 0);
    if (playerc_fiducial_subscribe(fiducial, PLAYERC_OPEN_MODE) != 0) {
        fprintf(stderr, "error: %s\n", playerc_error_str());
        return -1;
    }
    return 0;
}

/*
 * Misc commands.
 */
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

/*
 * Overloaded method taking x and y coordinates and command the robot to go there.
 */
void PlayerWrapper::goTo(double x, double y) {
    return goTo(player_pose2d_t{x, y, 0});
}

/*
 * Takes a player2d pose and tells the robot to go there.
 */
void PlayerWrapper::goTo(player_pose2d_t pose) {
    playerc_position2d_set_cmd_pose(position2d, pose.px, pose.py, 0, pose.pa);
}

/*
 * Returns the distance in meters of a specified laser beam.
 */
double PlayerWrapper::getRange(int i) {
    return laser->ranges[i];
}

/*
 * Returns the number of laser beams the robot is equipped with.
 */
int PlayerWrapper::getLaserCount() {
    return laser->scan_count;
}

/*
 * Returns the maximum range the laser will read to.
 */
double PlayerWrapper::getMaxRange() {
    return laser->max_range;
}

/*
 * Updates the various proxy data structures with new data.
 */
void PlayerWrapper::readClient() {
    playerc_client_read(client);
}

/*
 * Gets the robot's current X coordinate in the map.
 */
double PlayerWrapper::getRobX() {
    return position2d->px;
}

/*
 * Gets the robot's current Y coordinate in the map.
 */
double PlayerWrapper::getRobY() {
    return position2d->py;
}

/*
 * Gets the robot's current angle coordinate in the map.
 */
double PlayerWrapper::getRobA() {
    return position2d->pa;
}

/*
 * Gets the current number of detected fiducials.
 */
int PlayerWrapper::getFidCount() {
    return fiducial->fiducials_count;
}

/*
 * Gets the ID of a fiducial.
 */
int PlayerWrapper::getFidID(int i) {
    return fiducial->fiducials[i].id;
}

/*
 * Gets the X coordinate of a fiducial.
 */
double PlayerWrapper::getFidX(int i) {
    return fiducial->fiducials[i].pose.px;
}

/*
 * Gets the Y coordinate of a fiducial.
 */
double PlayerWrapper::getFidY(int i) {
    return fiducial->fiducials[i].pose.py;
}

/*
 * Gets the angle of a fiducial.
 */
double PlayerWrapper::getFidYAW(int i) {
    return fiducial->fiducials[i].pose.pyaw;
}