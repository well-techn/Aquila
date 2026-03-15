/**
 * @file FusionBias_t.h
 * @author Seb Madgwick
 * @brief Run-time estimation and compensation of gyroscope offset.
 */

#ifndef FUSION_BIAS_H
#define FUSION_BIAS_H

//------------------------------------------------------------------------------
// Includes

#include "FusionMath.h"

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief Bias structure. All members are private.
 */
typedef struct {
    float filterCoefficient;
    unsigned int timeout;
    unsigned int timer;
    FusionVector_t gyroscopeOffset;
} FusionBias_t;

//------------------------------------------------------------------------------
// Function declarations

void FusionBiasInitialise(FusionBias_t *const bias, const unsigned int sampleRate);

FusionVector_t FusionBiasUpdate(FusionBias_t *const bias, FusionVector_t gyroscope);

#endif

//------------------------------------------------------------------------------
// End of file
