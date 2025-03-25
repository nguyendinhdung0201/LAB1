/*
 * Input_parameters.c
 *
 *  Created on: Nov 7, 2024
 *      Author: PC
 */
#include "Input_parameters.h"
#include "math.h"
#define SERIESRESISTOR 10000
const float A = 1.129241e-3f;
const float B = 2.341077e-4f;
const float C = 8.775468e-8f;

Temperature_Sensors temperature_sensors;
Pressure_Sensors pressure_sensors;
Vrefint vrefint;

ADC_Temperature_Sensors adc_temperature_sensors;
ADC_Pressure_Sensors adc_pressure_sensors;
ADC_Vrefint adc_vrefint;

Filter_Temperature_Sensors filter_temperature_sensors;
Filter_Pressure_Sensors filter_pressure_sensors;
Filter_Vrefint filter_vrefint;

void Filter_Input_Init()
{
	SimpleKalmanFilterInit(&filter_temperature_sensors.filter_temperature_sensor_hoi_ve, 2.0f, 2.0f, 0.001f);
	SimpleKalmanFilterInit(&filter_temperature_sensors.filter_temperature_sensor_dau_day, 2.0f, 2.0f, 0.001f);
	SimpleKalmanFilterInit(&filter_pressure_sensors.filter_high_pressure_sensor, 2.0f, 2.0f, 0.001f);
	SimpleKalmanFilterInit(&filter_pressure_sensors.filter_low_pressure_sensor, 2.0f, 2.0f, 0.001f);
	SimpleKalmanFilterInit(&filter_vrefint.filter_vref, 2.0f, 2.0f, 0.001f);
}
static float calcular_temperature(uint16_t adc_value)
{
    float resistance = (4095*SERIESRESISTOR)/(float)adc_value - SERIESRESISTOR;
    float steinhart = 1.0f/(A + B*log(resistance) + C*pow(log(resistance),3)) - 273.15f;
    return steinhart;
}
static float calcular_pressure_voltage(uint16_t adc_value, float sample_pressure, float vref)
{
    float voltage = (float)(adc_value)*vref/4095.0f;
    float pressure = ((voltage - 0.5f)*(sample_pressure - 0.0f))/(3.5f-0.5f);
    if(pressure < 0.0f){pressure = 0.0f;}
    return pressure;
}
static float calcular_vref(uint16_t adc_value){
	uint16_t vrefint_cal = *((uint16_t*)0x08FFF810);
	float vref = 3300*vrefint_cal/(float)adc_value/1000.0f;
	return vref;
}
static float calcular_pressure_voltage_H(uint16_t adc_value, float sample_pressure, float vref)
{
    float voltage = (float)(adc_value)*vref/4095.0f;
    float pressure = ((voltage - 0.5f)*(sample_pressure - 0.0f))/(4.5f-0.5f);
    if(pressure < 0.0f){pressure = 0.0f;}
    return pressure;
}

static float calcular_pressure_current(uint16_t adc_value, float sample_res, float sample_pressure_low, float sample_pressure_high, float vref)
{
    float voltage = (float)(adc_value)*vref/4095.0f;
    float curent = (voltage/sample_res)*1000.0f;
    float pressure = ((curent - 4.0f)*(sample_pressure_high - sample_pressure_low))/(20.0f - 4.0f) + sample_pressure_low;
    return pressure;
}
static void read_adc_sensor(ADC_HandleTypeDef* hadc1){
    adc_vrefint.ADC_vref = SimpleKalmanFilter_updateEstimate(&filter_vrefint.filter_vref, ADC_ReadChannel(hadc1, ADC_CHANNEL_VREFINT));
	adc_temperature_sensors.ADC_hoi_ve = SimpleKalmanFilter_updateEstimate(&filter_temperature_sensors.filter_temperature_sensor_hoi_ve, ADC_ReadChannel(hadc1, ADC_CHANNEL_0));
	adc_temperature_sensors.ADC_dau_day = SimpleKalmanFilter_updateEstimate(&filter_temperature_sensors.filter_temperature_sensor_dau_day, ADC_ReadChannel(hadc1, ADC_CHANNEL_1));
	adc_pressure_sensors.ADC_low_pressure = SimpleKalmanFilter_updateEstimate(&filter_pressure_sensors.filter_low_pressure_sensor, ADC_ReadChannel(hadc1, ADC_CHANNEL_14));
	adc_pressure_sensors.ADC_high_pressure = SimpleKalmanFilter_updateEstimate(&filter_pressure_sensors.filter_high_pressure_sensor, ADC_ReadChannel(hadc1, ADC_CHANNEL_15));
}
void Calcular_Input(ADC_HandleTypeDef* hadc1){
	read_adc_sensor(hadc1);
	vrefint.vref = calcular_vref(adc_vrefint.ADC_vref);
	temperature_sensors.hoi_ve = calcular_temperature(adc_temperature_sensors.ADC_hoi_ve);
	temperature_sensors.dau_day = calcular_temperature(adc_temperature_sensors.ADC_dau_day);
	pressure_sensors.low_pressure_sensor = calcular_pressure_voltage(adc_pressure_sensors.ADC_low_pressure, 20.0f, vrefint.vref);
	pressure_sensors.high_pressure_sensor = calcular_pressure_voltage_H(adc_pressure_sensors.ADC_high_pressure, 50.0f, vrefint.vref);
}



