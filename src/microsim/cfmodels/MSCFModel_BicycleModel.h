/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2020 German Aerospace Center (DLR) and others.
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// https://www.eclipse.org/legal/epl-2.0/
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    MSCFModel_BicycleModel.h
/// @author  Nico Ostendorf
/// @date    Tue, 10 Dec 2024
/****************************************************************************/
#pragma once
#include <config.h>

#include "MSCFModel.h"
#include <utils/xml/SUMOXMLDefinitions.h>

#include <queue>
#include <fstream>


// ===========================================================================
// class definitions
// ===========================================================================
/** @class MSCFModel_BicycleModel
 * @brief The BicycleModel implements realistic speed, acceleration, and deceleration behavior of bicycles in urban scenarios. 
 * @see MSCFModel
 */
class MSCFModel_BicycleModel : public MSCFModel {
public:

    /** @brief Constructor
     *  @param[in] vtype the type for which this model is built and also the parameter object to configure this model
     */
    MSCFModel_BicycleModel(const MSVehicleType* vtype);


    /// @brief Destructor
    ~MSCFModel_BicycleModel();

    /** @brief Returns the average deceleration for a given number of breaking steps
     * Calculates also additional decelerations for the stopping pattern for the given vehicle
     * if vector is not long enough
     * @param[in] breakingSteps The number of breaking steps
     * @param[in] veh The vehicle itself, for obtaining other values
     * @return The average deceleration for the given number of breaking steps
    */
    double avgDecelForXBreakingSteps(double breakingSteps, const MSVehicle* const veh) const;

    /** @brief Return the braking distance for a given speed
     * @param[in] initial_speed The vehicle's current speed
     * @param[in] predSpeed The speed of the vehicle in front
     * @param[in] veh The vehicle itself, for obtaining other values
     * @return The braking distance for the given speed
    */
    double calculate_braking_distance(double initial_speed, double predSpeed, const MSVehicle* const veh) const;

    /** @brief Duplicates the car-following model
     * @param[in] vtype The vehicle type this model belongs to (1:1)
     * @return A duplicate of this car-following model
    */
    MSCFModel* duplicate(const MSVehicleType* vtype) const;

    /** @brief Applies interaction with stops and lane changing model
     * influences. Called at most once per simulation step (exactcly once per action step)
     * @param[in] veh The ego vehicle
     * @param[in] vPos The possible velocity
     * @return The velocity after applying interactions with stops and lane change model influences
    */
    double finalizeSpeed(MSVehicle* const veh, double vPos) const override;

    /** @brief Computes the vehicle's safe speed (no dawdling)
     * @param[in] veh The vehicle (EGO)
     * @param[in] speed The vehicle's speed
     * @param[in] gap2pred The (netto) distance to the LEADER
     * @param[in] predSpeed The speed of LEADER
     * @return EGO's safe speed
     * @see MSCFModel::ffeV
    */
    double followSpeed(const MSVehicle* const veh, double speed, double gap2pred, double predSpeed, double predMaxDecel, const MSVehicle* const pred = 0) const override;

    /** @brief Computes the vehicle's follow speed that avoids a collision for the given amount of time
     *
     * Returns the velocity of the vehicle in dependence to the vehicle's and its leader's values and the distance between them.
     * @param[in] veh The vehicle (EGO)
     * @param[in] speed The vehicle's speed
     * @param[in] gap2pred The (netto) distance to the LEADER
     * @param[in] predSpeed The speed of LEADER
     * @param[in] predMaxDecel The maximum leader decelration
     * @return EGO's safe speed
    */
    double followSpeedTransient(double duration, const MSVehicle* const veh, double speed, double gap2pred, double predSpeed, double predMaxDecel) const;

    /** @brief Computes the vehicle's safe speed without a leader
     *
     * Returns the velocity of the vehicle in dependence to the length of the free street and the target
     *  velocity at the end of the free range. If onInsertion is true, the vehicle may still brake
     *  before the next movement.
     * @param[in] veh The vehicle (EGO)
     * @param[in] speed The vehicle's speed
     * @param[in] seen The look ahead distance
     * @param[in] maxSpeed The maximum allowed speed
     * @param[in] onInsertion whether speed at insertion is asked for
     * @return EGO's safe speed
    */
    double freeSpeed(const MSVehicle* const veh, double speed, double seen, double maxSpeed, const bool onInsertion = false) const;

    static double freeSpeed(const double currentSpeed, const double decel, const double dist, const double maxSpeed, const bool onInsertion, const double actionStepLength);

    /** @brief Get the vehicle type's apparent deceleration [m/s^2] (the one regarded by its followers
     * @return The apparent deceleration (in m/s^2) of vehicles of this class
    */
    inline double getApparentDecel() const {
        return getMaxDecel();
    }

    /** @brief Get the bike type's average speed
     * @return The avg speed of bikes of this class
    */
    double getAvgSpeed() const {
        return avgSpeed;
    }
    
    /** @brief Get the driver's imperfection
     * @return The imperfection of drivers of this class
    */
    double getImperfection() const {
        return myDawdle;
    }
    
    /** @brief Returns the model's name
     * @return The model's name
     * @see MSCFModel::getModelName
    */
    int getModelID() const {
        return SUMO_TAG_CF_BICYCLE_MODEL;
    }

    /** @brief Returns the maximum speed given the current speed
     * @param[in] speed The vehicle's current speed
     * @param[in] veh The vehicle itself, for obtaining other values
     * @return The maximum possible speed for the next step
    */
    double maxNextSpeed(double speed, const MSVehicle* const /*veh*/) const override;

    /** @brief Returns the minimum speed given the current speed
     * (depends on the numerical update scheme and its step width)
     * Note that it wouldn't have to depend on the numerical update
     * scheme if the semantics would rely on acceleration instead of velocity.
     *
     * @param[in] speed The vehicle's current speed
     * @param[in] speed The vehicle itself, for obtaining other values, if needed as e.g. road conditions.
     * @return The minimum possible speed for the next step
    */
    double minNextSpeed(double speed, const MSVehicle* const veh = 0) const override;

    double patchSpeedBeforeLC(const MSVehicle* veh, double vMin, double vMax) const override;

    /** @brief Sets a new value for desired headway [s]
     * @param[in] headwayTime The new desired headway (in s)
    */
    void setHeadwayTime(double headwayTime) {
        myHeadwayTime = headwayTime;
        myTauDecel = myDecel * headwayTime;
    }

    /** @brief Sets a new value for driver imperfection
     * @param[in] accel The new driver imperfection
    */
    void setImperfection(double imperfection) {
        myDawdle = imperfection;
    }

    /** @brief Sets a new value for maximum deceleration [m/s^2]
     * @param[in] accel The new deceleration in m/s^2
    */
    void setMaxDecel(double decel) {
        myDecel = decel;
        myTauDecel = myDecel * myHeadwayTime;
    }

    /** @brief Computes the vehicle's safe speed for approaching a non-moving obstacle (no dawdling)
     * @param[in] veh The vehicle (EGO)
     * @param[in] gap2pred The (netto) distance to the the obstacle
     * @return EGO's safe speed for approaching a non-moving obstacle
     * @see MSCFModel::ffeS
     * @todo generic Interface, models can call for the values they need
    */
    double stopSpeed(const MSVehicle* const veh, const double speed, double gap2pred) const override;

protected:

    /** @brief Applies driver imperfection (dawdling / sigma)
     * @param[in] speed The speed with no dawdling
     * @return The speed after dawdling
    */
    double dawdle(double speed, std::mt19937* rng) const;

    /** @brief Returns the "safe" velocity
     * @param[in] gap2pred The (netto) distance to the LEADER
     * @param[in] predSpeed The LEADER's speed
     * @param[in] predMaxDecel The LEADER's maximum deceleration
     * @return the safe velocity
    */
    double vsafe(double gap, double predSpeed, double predMaxDecel, double speed, const MSVehicle* const veh) const;

protected:

    /// @brief the overall average speed of the vehicle (-1.5 of sumo max value)
    double avgSpeed;

    /// @brief the overall average stopping pattern value of the vehicle
    mutable double avgStoppingPatternValue;

    /// @brief the maximimum speed of a medium bike
    double mediumSpeed = 9.0; 

    /// @brief The vehicle's dawdle-parameter. 0 for no dawdling, 1 for max.
    double myDawdle;

    /// @brief The precomputed value for myDecel*myTau
    mutable double myTauDecel;

    /// @brief the maximimum speed of a slow bike
    double slowSpeed = 7.2;
};