/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2020 German Aerospace Center (DLR) and others.
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// https://www.eclipse.org/legal/epl-2.0/
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    MSCFModel_BicycleModel.cpp
/// @author  Nico Ostendorf
/// @date    Tue, 10 Dec 2024
/****************************************************************************/
#include <config.h>

#include <microsim/MSVehicle.h>
#include <microsim/MSLane.h>
#include "MSCFModel_BicycleModel.h"
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include <utils/common/RandHelper.h>
#include <microsim/MSGlobals.h>


MSCFModel_BicycleModel::MSCFModel_BicycleModel(const MSVehicleType* vtype) :
    MSCFModel(vtype),
    avgSpeed((myType->getMaxSpeed() - 1.5)),
    myDawdle(vtype->getParameter().getCFParam(SUMO_ATTR_SIGMA, SUMOVTypeParameter::getDefaultImperfection(vtype->getParameter().vehicleClass))),
    myTauDecel(myDecel * myHeadwayTime){
}

MSCFModel_BicycleModel::~MSCFModel_BicycleModel() {}

double 
MSCFModel_BicycleModel::avgDecelForXBreakingSteps(double breakingSteps, const MSVehicle* const veh) const {
    int i = 0;
    double sum = 0.0;
    std::vector<double> stoppingPattern = veh->getStoppingPattern();

    while(i<breakingSteps) {
        // If there are not enough elements in the stoppingPattern vector, append a default deceleration
        if (i >= stoppingPattern.size()) {
            veh->calcStoppingList();
            stoppingPattern = veh->getStoppingPattern();
        }

        sum += stoppingPattern[i];
        i++;
    }

    return sum / i;
}

double 
MSCFModel_BicycleModel::calculate_braking_distance(double initial_speed, double predSpeed, const MSVehicle* const veh) const {
    double total_distance = 0.0;
    int i = 0;
    double sum = 0.0;
    std::vector<double> stoppingPattern = veh->getStoppingPattern();

    while(true) {
        // If there are not enough elements in the stoppingPattern vector, append a default deceleration
        if (i >= stoppingPattern.size()) {
            veh->calcStoppingList();
            stoppingPattern = veh->getStoppingPattern();
        }

        double decel = stoppingPattern[i];
        sum += decel;

        // Calculate the speed after the time step
        double speed_after_time_step = initial_speed - ACCEL2SPEED(decel);

        // Calculate the distance traveled in this time step
        double distance = ((initial_speed + speed_after_time_step) / 2) * TS;

        // Update the total distance and the initial speed
        total_distance += distance;
        initial_speed = speed_after_time_step;

        // If the speed after the time step is less than or equal to 0, stop
        if (speed_after_time_step <= predSpeed) {
            break;
        }
        i++;
    }

    avgStoppingPatternValue = sum / i;

    return total_distance;
}

MSCFModel*
MSCFModel_BicycleModel::duplicate(const MSVehicleType* vtype) const {
    return new MSCFModel_BicycleModel(vtype);
}

double
MSCFModel_BicycleModel::finalizeSpeed(MSVehicle* const veh, double vPos) const {
    
    // save old v for optional acceleration computation
    const double oldV = veh->getSpeed();
    // apply deceleration bounds
    const double vMinEmergency = minNextSpeedEmergency(oldV, veh);
    // vPos contains the uppper bound on safe speed. allow emergency braking here
    const double vMin = MIN2(minNextSpeed(oldV, veh), MAX2(vPos, vMinEmergency));
    // aMax: Maximal admissible acceleration until the next action step, such that the vehicle's maximal
    // desired speed on the current lane will not be exceeded when the
    // acceleration is maintained until the next action step.
    double maximumBikeSpeed = avgSpeed*veh->getChosenBikeFactor();
    double aMax = MIN2((maximumBikeSpeed - oldV) / veh->getActionStepLengthSecs(),veh->getVariableAccel());
    // apply planned speed constraints and acceleration constraints
    double maxSpeed = maxNextSpeed(oldV,veh);

    double vMax = MIN3(oldV + ACCEL2SPEED(aMax), maxSpeed, vPos);

    vMax = MAX2(vMin, vMax);
    // apply further speed adaptations
    double vNext = vMax;

    assert(vNext >= vMin);
    assert(vNext <= vMax);
    // apply lane-changing related speed adaptations
    vNext = veh->getLaneChangeModel().patchSpeed(vMin, vNext, vMax, *this);
    assert(vNext >= vMin);
    assert(vNext <= vMax);

    return vNext;
}

double
MSCFModel_BicycleModel::followSpeed(const MSVehicle* const veh, double speed, double gap, double predSpeed, double predMaxDecel, const MSVehicle* const /*pred*/) const {
    return vsafe(gap, predSpeed, predMaxDecel, speed, veh);
}

double
MSCFModel_BicycleModel::followSpeedTransient(double duration, const MSVehicle* const veh, double /*speed*/, double gap2pred, double predSpeed, double predMaxDecel) const {
    // minimium distance covered by the leader if braking
    double leaderMinDist = gap2pred + distAfterTime(duration, predSpeed, -predMaxDecel);
    // if ego would not brake it could drive with speed leaderMinDist / duration
    // due to potentential ego braking it can safely drive faster

    // number of potential braking steps
    int a = (int)ceil(duration / TS - TS);
    // can we brake for the whole time?

    double avgDecelForDuration = avgDecelForXBreakingSteps(a, veh);

    const double bg =  brakeGap(a * avgDecelForDuration, avgDecelForDuration, 0);
    if (bg <= leaderMinDist) {
        // braking continuously for duration
        // distance reduction due to braking
        double b = TS * veh->getCurrentDecel() * 0.5 * (a * a - a);

        return (b + leaderMinDist) / duration;
    } else {
        double bg = 0;
        double speed = 0;
        while (bg < leaderMinDist) {
            speed += ACCEL2SPEED(avgDecelForDuration);
            bg += SPEED2DIST(speed);
        }
        speed -= DIST2SPEED(bg - leaderMinDist);
        return MIN2(speed, avgSpeed*veh->getChosenBikeFactor());
    }

}

double
MSCFModel_BicycleModel::freeSpeed(const MSVehicle* const veh, double speed, double seen, double maxSpeed, const bool onInsertion) const {
    if (maxSpeed < 0.) {
        // can occur for ballistic update (in context of driving at red light)
        return maxSpeed;
    }
    maxSpeed = MIN2(maxSpeed, avgSpeed*veh->getChosenBikeFactor());
    double vSafe = freeSpeed(speed, getMaxDecel(), seen, maxSpeed, onInsertion, veh->getActionStepLengthSecs());
    return vSafe;
}

double
MSCFModel_BicycleModel::freeSpeed(const double /*currentSpeed*/, const double decel, const double dist, const double targetSpeed, const bool onInsertion, const double /*actionStepLength*/) {
    const double v = SPEED2DIST(targetSpeed);
    if (dist < v) {
        return targetSpeed;
    }
    const double b = ACCEL2DIST(decel);
    const double y = MAX2(0.0, ((sqrt((b + 2.0 * v) * (b + 2.0 * v) + 8.0 * b * dist) - b) * 0.5 - v) / b);
    const double yFull = floor(y);
    const double exactGap = (yFull * yFull + yFull) * 0.5 * b + yFull * v + (y > yFull ? v : 0.0);
    const double fullSpeedGain = (yFull + (onInsertion ? 1. : 0.)) * ACCEL2SPEED(decel);
    return DIST2SPEED(MAX2(0.0, dist - exactGap) / (yFull + 1)) + fullSpeedGain + targetSpeed;
}

double
MSCFModel_BicycleModel::maxNextSpeed(double speed, const MSVehicle* const veh) const {
    if(speed > avgSpeed*veh->getChosenBikeFactor()){
        return MAX2(speed - ACCEL2SPEED(veh->getCurrentDecel()),0.0);
    }

    return MAX2(speed + ACCEL2SPEED(veh->getVariableAccel()),0.0);
}

double
MSCFModel_BicycleModel::minNextSpeed(double speed, const MSVehicle* const veh) const {
    return MAX2(speed - ACCEL2SPEED(veh->getCurrentDecel()), 0.);
}

double
MSCFModel_BicycleModel::patchSpeedBeforeLC(const MSVehicle* veh, double vMin, double vMax) const {
    UNUSED_PARAMETER(veh);
    const double vDawdle = MAX2(vMin, dawdle(vMax, veh->getRNG()));
    return vDawdle;
}

double
MSCFModel_BicycleModel::stopSpeed(const MSVehicle* const veh, const double speed, double gap) const {
    return vsafe(gap, 0., 0., speed, veh);
}

double
MSCFModel_BicycleModel::dawdle(double speed, std::mt19937* /*rng*/) const {
    if (!MSGlobals::gSemiImplicitEulerUpdate) {
        // in case of the ballistic update, negative speeds indicate
        // a desired stop before the completion of the next timestep.
        // We do not allow dawdling to overwrite this indication
        if (speed < 0) {
            return speed;
        }
    }
    return MAX2(0., speed);
}

double 
MSCFModel_BicycleModel::vsafe(double gap, double predSpeed, double /* predMaxDecel */, double speed, const MSVehicle* const veh) const {
    double necessaryDeceleration = (speed * speed - predSpeed * predSpeed) / (2 * (gap));
    double breakDistance = calculate_braking_distance(speed, predSpeed, veh);

    if (predSpeed == 0 && gap < 0.01) {
        return 0;
    } else if (necessaryDeceleration >= avgStoppingPatternValue+0.5){
        double randomNum = 0.2 * RandHelper::rand((double)1.0,veh->getRNG());
        return MAX2(speed - ACCEL2SPEED(necessaryDeceleration+randomNum),0.0);
    } else if (avgStoppingPatternValue <= necessaryDeceleration+0.2) {
        return MAX2(speed - ACCEL2SPEED(veh->getCurrentDecel()),0.0);
    } else {
        return MAX2(maxNextSpeed(speed, veh),0.0);
    }
}
/****************************************************************************/
