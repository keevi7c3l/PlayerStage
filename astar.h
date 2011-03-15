/*
 * File:   astar.h
 * Author: koy
 *
 * Created on February 16, 2011, 5:29 PM
 */

#ifndef ASTAR_H
#define	ASTAR_H

#include <vector>
#include <libplayercore/playercore.h>
#include <libplayerc/playerc.h>
#include "thread.h"
#include "mutex.h"
#include "laserReader.h"
#include "astarImpTest.h"

class AStarThread : public Thread {
public:
    AStarThread(const double accuracy = 0.0);
    virtual ~AStarThread();
    virtual void set(const player_pose2d_t &begin, const player_pose2d_t &end);
    virtual void get(std::vector<player_pose2d_t> *path);

protected:
    virtual void run();

    const double accuracy;
    player_pose2d_t begin, end;
    std::vector<player_pose2d_t> path;
    Mutex mutex;
};

#endif	/* ASTAR_H */
