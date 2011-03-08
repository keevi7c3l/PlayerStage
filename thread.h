/*
 * File:   thread.h
 * Author: koy
 *
 * Created on February 16, 2011, 5:31 PM
 */

#ifndef THREAD_H
#define	THREAD_H
#include <pthread.h>

class Thread {
public:
    Thread();
    virtual ~Thread();
    virtual void start();
    virtual void stop();

protected:
    virtual void run() = 0;
    virtual void testcancel();

    static void *main(void *object);
    pthread_t thread;
};
#endif	/* THREAD_H */
