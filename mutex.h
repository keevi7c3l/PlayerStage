/*
 * File:   mutex.h
 * Author: koy
 *
 * Created on February 16, 2011, 5:30 PM
 */

#ifndef MUTEX_H
#define	MUTEX_H

#include <pthread.h>
#include <assert.h>

class Mutex {
public:
    Mutex();
    virtual ~Mutex();

    virtual void lock();
    virtual void unlock();

protected:
    pthread_mutex_t mutex;
};

class MutexLock {
public:

    MutexLock(Mutex *mutex) {
        assert(mutex);
        this->mutex = mutex;
        this->mutex->lock();
    }

    virtual ~MutexLock() {
        this->mutex->unlock();
    }
protected:
    Mutex *mutex;
};

#endif	/* MUTEX_H */
