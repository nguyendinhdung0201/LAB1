/*
 * SimpleKalmanFilter - a Kalman Filter implementation for single variable models.
 * Created by Denys Sene, January, 1, 2017.
 * Released under MIT License - see LICENSE file for details.
 */
#include "stm32h5xx.h"
#include "math.h"
#ifndef SimpleKalmanFilter_h
#define SimpleKalmanFilter_h

typedef struct {
    float err_measure;
    float err_estimate;
    float q;
    float current_estimate;
    float last_estimate;
    float kalman_gain;
} SimpleKalmanFilter;

void 	SimpleKalmanFilterInit(SimpleKalmanFilter *filter, float mea_e, float est_e, float q);
uint16_t 	SimpleKalmanFilter_updateEstimate(SimpleKalmanFilter *filter, float mea);
void 	SimpleKalmanFilter_setMeasurementError(SimpleKalmanFilter *filter, float mea_e);
void 	SimpleKalmanFilter_setEstimateError(SimpleKalmanFilter *filter, float est_e);
void 	SimpleKalmanFilter_setProcessNoise(SimpleKalmanFilter *filter, float q);
float 	SimpleKalmanFilter_getKalmanGain(SimpleKalmanFilter *filter);

#endif
