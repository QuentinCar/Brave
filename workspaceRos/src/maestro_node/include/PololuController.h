/* 
 * File:   PololuController.h
 * Author: raffaello
 *
 * Created on 25 November 2013, 10:14
 */

#ifndef POLOLUCONTROLLER_H
#define	POLOLUCONTROLLER_H

#include "TimeoutSerial.h"

#include <boost/thread.hpp>

class PololuController {
public:
    PololuController(TimeoutSerial* serial);
    virtual ~PololuController();
    void maestroSetAngle(int channel, double target);
    double maestroGetAngle(int channel);
private:
    TimeoutSerial* serial;
    boost::mutex pololuMutex;
    
    void maestroSetTarget(int channel, double target);
    double maestroGetTarget(int channel);
};

#endif	/* POLOLUCONTROLLER_H */

