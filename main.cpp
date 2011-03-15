#include <stdio.h>
#include <libplayerc/playerc.h>
#include <math.h>
#include <string>
#include <queue>
#include <iostream>
#include <cstdlib>
#include <map>
#include <unordered_map>
#include <assert.h>
#include <limits>
#include <vector>
#include "astar.h"
#include "laserReader.h"


playerc_client_t *client;
playerc_laser_t *laser;
playerc_position2d_t *position2d;

using namespace std;

player_planner_data_t planner;
std::vector<player_pose2d_t> path;
AStarThread *astar;
LaserReader *lr;


#define PLANNER_PATH_DEVIATION_LIMIT 3.0
#define PLANNER_NEXT_WAYPOINT_DISTANCE 1.5
#define PLANNER_GOAL_DISTANCE 0.75

void plan() {
    planner.waypoint_idx = -1;
    planner.valid = false;
    planner.done = false;
    planner.waypoint = planner.pos;
    planner.pos.px = position2d->px;
    planner.pos.py = position2d->py;
    planner.pos.pa = position2d->pa;

    const double goal_distance = hypot(planner.goal.px - planner.pos.px, planner.goal.py - planner.pos.py);

    // are we done?
    if ((planner.done = goal_distance < PLANNER_GOAL_DISTANCE)) {
        // stop astar
        astar->set(planner.pos, planner.pos);

        // no path
        path.clear();
        planner.waypoints_count = 0;

        printf("Path Done\n");
        return;
    }

    // are we really close to our goal ready?
    /*if (goal_distance < PLANNER_NEXT_WAYPOINT_DISTANCE)
    {
            // stop astar
            astar->set(planner.pos, planner.pos);

            // now the only thing in the path is our goal
            path.clear();
            path.push_back(planner.goal);
            planner.waypoints_count = 1;
            planner.waypoint_idx = 0;
            planner.waypoint = planner.goal;
            planner.valid = true;
            return;
    }*/

    // talk to the astar thread
    astar->set(planner.pos, planner.goal);
    astar->get(&path);
    planner.waypoints_count = path.size();

    // do we have a path?
    if (!path.size()) {
        // no path found or not ready yet
        printf("No Path\n");
        return;
    }

    // find closest waypoint to our location
    double min = std::numeric_limits<double>::infinity();
    for (uint32_t i = 0; i < planner.waypoints_count; i++) {
        const double d = hypot(path[i].px - planner.pos.px, path[i].py - planner.pos.py);
        if (d <= min) {
            min = d;
            planner.waypoint_idx = i;
        }
    }

    // are we deviated from path?
    if (min > PLANNER_PATH_DEVIATION_LIMIT) {
        // too much deviation
        PLAYER_WARN("planner: path deviation detected");
        return;
    }

    // then find a waypoint that is far away enough
    //uint32_t i;
    /*for (i = planner.waypoint_idx; i < planner.waypoints_count; i++)
    {
            // current waypoint is set to some waypoint ahead of the current position
            if (hypot(path[i].px - planner.pos.px, path[i].py - planner.pos.py) > PLANNER_NEXT_WAYPOINT_DISTANCE)
            {
                    break;
            }
    }*/
    const uint32_t next_waypoint_dist = (uint32_t) ceil(PLANNER_NEXT_WAYPOINT_DISTANCE / 0.1);
    planner.waypoint_idx += next_waypoint_dist;
    if (planner.waypoint_idx >= (int) planner.waypoints_count)
        planner.waypoint_idx = planner.waypoints_count - 1;
    planner.waypoint = path[planner.waypoint_idx];
    planner.valid = true;
}

int main() {

    printf("Starting up robot\n");

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

    printf("Creating LaserReader\n");
    lr = new LaserReader(client, laser, position2d);

    printf("Creating AstarTread\n");
    astar = new AStarThread(PLANNER_NEXT_WAYPOINT_DISTANCE);
    printf("Starting AstarTread\n");
    astar->start();

    planner.goal.px = 8;
    planner.goal.py = 8;
    printf("Starting Main Loop\n");
    for (;;) {
        playerc_client_read(client);
        //printf("Starting Main Loop\n");
        plan();

        //update laser
       // printf("Reading Laser Command\n");
        lr->readLaser();

        player_position2d_cmd_pos_t cmd;
        if (planner.valid && !planner.done) {
            // tell local planner to go towards next waypoint
            cmd.pos = planner.waypoint;
            cmd.state = true;

        } else {
            // tell local planner to stop
            cmd.pos = planner.pos;
            cmd.state = false;
        }

        printf("Going to: %f,%f\n", cmd.pos.px, cmd.pos.py);

        playerc_position2d_set_cmd_pose_with_vel(position2d, cmd.pos, cmd.vel, cmd.state);

        usleep(10000);

    }

    //Disconnect player
    playerc_laser_unsubscribe(laser);
    playerc_laser_destroy(laser);
    playerc_client_disconnect(client);
    playerc_client_destroy(client);

    return 0;
}