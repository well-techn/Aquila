/**
 * @file FusionAhrs_t.h
 * @author Seb Madgwick
 * @brief Attitude and Heading Reference System (AHRS) algorithm.
 */

#ifndef FUSION_AHRS_H
#define FUSION_AHRS_H

//------------------------------------------------------------------------------
// Includes

#include "FusionConvention.h"
#include "FusionMath.h"
#include <stdbool.h>

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief Settings.
 */
typedef struct {
    FusionConvention convention;
    float gain;
    float gyroscopeRange;
    float accelerationRejection;
    float magneticRejection;
    unsigned int recoveryTriggerPeriod;
} FusionAhrsSettings_t;

/**
 * @brief AHRS structure. All members are private.
 */
typedef struct {
    FusionAhrsSettings_t settings;
    FusionQuaternion_t quaternion;
    FusionVector_t accelerometer;
    bool initialising;
    float rampedGain;
    float rampedGainStep;
    bool angularRateRecovery;
    FusionVector_t halfAccelerometerFeedback;
    FusionVector_t halfMagnetometerFeedback;
    bool accelerometerIgnored;
    int accelerationRecoveryTrigger;
    int accelerationRecoveryTimeout;
    bool magnetometerIgnored;
    int magneticRecoveryTrigger;
    int magneticRecoveryTimeout;
} FusionAhrs_t;

/**
 * @brief Internal states.
 */
typedef struct {
    float accelerationError;
    bool accelerometerIgnored;
    float accelerationRecoveryTrigger;
    float magneticError;
    bool magnetometerIgnored;
    float magneticRecoveryTrigger;
} FusionAhrsInternalStates_t;

/**
 * @brief Flags.
 */
typedef struct {
    bool initialising;
    bool angularRateRecovery;
    bool accelerationRecovery;
    bool magneticRecovery;
} FusionAhrsFlags_t;

//------------------------------------------------------------------------------
// Variable declarations

extern const FusionAhrsSettings_t fusionAhrsDefaultSettings;

//------------------------------------------------------------------------------
// Function declarations

void FusionAhrsInitialise(FusionAhrs_t *const ahrs);

void FusionAhrsReset(FusionAhrs_t *const ahrs);

void FusionAhrsSetSettings(FusionAhrs_t *const ahrs, const FusionAhrsSettings_t *const settings);

void FusionAhrsUpdate(FusionAhrs_t *const ahrs, const FusionVector_t gyroscope, const FusionVector_t accelerometer, const FusionVector_t magnetometer, const float deltaTime);

void FusionAhrsUpdateNoMagnetometer(FusionAhrs_t *const ahrs, const FusionVector_t gyroscope, const FusionVector_t accelerometer, const float deltaTime);

void FusionAhrsUpdateExternalHeading(FusionAhrs_t *const ahrs, const FusionVector_t gyroscope, const FusionVector_t accelerometer, const float heading, const float deltaTime);

FusionQuaternion_t FusionAhrsGetQuaternion(const FusionAhrs_t *const ahrs);

void FusionAhrsSetQuaternion(FusionAhrs_t *const ahrs, const FusionQuaternion_t quaternion);

FusionVector_t FusionAhrsGetGravity(const FusionAhrs_t *const ahrs);

FusionVector_t FusionAhrsGetLinearAcceleration(const FusionAhrs_t *const ahrs);

FusionVector_t FusionAhrsGetEarthAcceleration(const FusionAhrs_t *const ahrs);

FusionAhrsInternalStates_t FusionAhrsGetInternalStates(const FusionAhrs_t *const ahrs);

FusionAhrsFlags_t FusionAhrsGetFlags(const FusionAhrs_t *const ahrs);

void FusionAhrsSetHeading(FusionAhrs_t *const ahrs, const float heading);

#endif

//------------------------------------------------------------------------------
// End of file
