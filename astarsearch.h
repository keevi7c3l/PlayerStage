/*
 * File:   astarsearch.h
 * Author: koy
 *
 * Created on February 16, 2011, 5:36 PM
 */

#ifndef ASTARSEARCH_H
#define	ASTARSEARCH_H
#include <libplayercore/playercore.h>
#include <libplayerc/playerc.h>
#include <vector>
#include "laserReader.h"



bool astar_search(
        const player_pose2d_t &begin,
        const player_pose2d_t &end,
        std::vector<player_pose2d_t> *path = 0,
        double *cost = 0,
        const double accuracy = 0.0
        );

#endif	/* ASTARSEARCH_H */
