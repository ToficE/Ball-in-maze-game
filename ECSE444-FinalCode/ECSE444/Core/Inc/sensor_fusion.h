/**
  ******************************************************************************
  * @file           : sensor_fusion.c
  * @author			: Tofic Esses
  * @brief          : This file provides a set of functions to measure STM32
  * 				  board orientation.
  ******************************************************************************
  * @attention
  *
  * This software has been written for a final class project at McGill University.
  * Class: ECSE 444
  *
  ******************************************************************************
  */


typedef struct _kalman_state{
    float q; //process noise covariance
    float r; //measurement noise covariance
    float x; //estimated value
    float p; //estimation error covariance
    float k; // adaptive Kalman filter gain.
}kalman_state;

// kalman: inner function
// @RETURN: 0 (if no error), -1 (if error)
extern int kalman(kalman_state* kstate, float measurement);

/* Functions ------------------------------------------------------------------*/

/*
 *  @brief  Initialize the ACCELERO and GYRO.
  * @retval None.
 */
void Sensors_Init(void);

/*
 *  @brief  DeInitialize the ACCELERO and GYRO.
  * @retval None.
 */
void Sensors_DeInit(void);

/*
 *  @brief  Set sensor sampling frequency and sampling period.
  * @retval None.
 */
void Set_Sampling_Frequency(int frequency);

/*
 *  @brief  Measure GYRO bias
  * @retval None.
 */
void Measure_Gyro_Bias(void);

/*
 *  @brief  Measure STM32 Board Orientation
  * @retval 2-element orientation array (x, y).
 */
void Measure_Orientation(float* orientation);
