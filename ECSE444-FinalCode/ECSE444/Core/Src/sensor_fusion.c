/**
  ******************************************************************************
  * @file           : sensor_fusion.c
  * @author			: Tofic Esses
  * @brief          : This file provides a set of functions to measure STM32
  * 				  board orientation in the x and y direction only.
  * 				  Uses gyroscope and accelerometer of the LSM6DSL package.
  ******************************************************************************
  * @attention
  *
  * This software has been written for a final class project at McGill University.
  * Class: ECSE 444
  * Final project: Ball-in-a maze system simulated (on an embedded system)
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#define ARM_MATH_CM4
#include "arm_math.h"
#include <stdio.h>
#include "stm32l4s5i_iot01.h"
#include "stm32l4s5i_iot01_gyro.h"
#include "stm32l4s5i_iot01_accelero.h"

#include "sensor_fusion.h"

/* Defines --------------------------------------------------------------------*/
// Gyroscope scaling factors
// predetermined values
#define GYRO_X_SENS (1/(890193.688-(-900583.5)))
#define GYRO_Y_SENS (1/(873963.938-(-1290208.62)))

// Complementary filter constants
#define GYRO_ALPHA 0.90
#define ACCEL_MAG_ALPHA 0.10

/* Variables ------------------------------------------------------------------*/
int SAMPLING_FREQUENCY;
float DT; // sampling period

float gyro_x_bias;
float gyro_y_bias;

// kalman states for filtering gyroscope readings
kalman_state gyro_x_state = {.q = 0.01, .r = 0.1, .x = 0.0, .p = 0.5, .k = 0};
kalman_state gyro_y_state = {.q = 0.01, .r = 0.1, .x = 0.0, .p = 0.5, .k = 0};

// kalman states for filtering angle computation
kalman_state angle_x_state = {.q = 0.01, .r = 0.1, .x = 0.0, .p = 0.01, .k = 0};
kalman_state angle_y_state = {.q = 0.01, .r = 0.1, .x = 0.0, .p = 0.01, .k = 0};

// Initialize memory for storing sensor data
float gyro_xyz[3];
float gyro_x, gyro_y, gyro_z;

int16_t accel_xyz[3];
float accel_norm_xyz[3];
float accel_x, accel_y, accel_z;

// Gyroscope and accelerometer orientations
float roll_gyro, pitch_gyro, roll_acc, pitch_acc;

// Variables for storing current angle readings
float angle_x = 0.0;
float angle_y = 0.0;


/* Functions ------------------------------------------------------------------*/

/*
 *  @brief  Initialize the ACCELERO and GYRO.
  * @retval None.
 */
void Sensors_Init() {
	 BSP_GYRO_Init();
	 BSP_ACCELERO_Init();

	 /* Set gyroscope to normal power mode. */
	 BSP_GYRO_LowPower(1);
}

/*
 *  @brief  DeInitialize the ACCELERO and GYRO.
  * @retval None.
 */
void Sensors_DeInit() {
	BSP_GYRO_DeInit();
	BSP_ACCELERO_DeInit();
	//reset angles
	angle_x = 0;
	angle_y = 0;
	// reset kalman state
	gyro_x_state.x = 0.0;
	gyro_y_state.x = 0.0;
	angle_x_state.x = 0.0;
	angle_y_state.x = 0.0;
}

/*
 *  @brief  Set sensor sampling frequency and sampling period.
  * @retval None.
 */
void Set_Sampling_Frequency(int frequency) {
	SAMPLING_FREQUENCY = frequency;
	DT = 1.0/SAMPLING_FREQUENCY;
}

/*
 *  @brief  Measure GYRO bias
  * @retval None.
 */
void Measure_Gyro_Bias() {
	float sum_x, sum_y = 0;
	for (int i = 0; i < 10000 ;i++){
		BSP_GYRO_GetXYZ(gyro_xyz);
		sum_x += gyro_xyz[0];
		sum_y += gyro_xyz[1];
	}
	gyro_x_bias = sum_x/10000;
	gyro_y_bias = sum_y/10000;
}

/*
 *  @brief  Measure STM32 Board Orientation
  * @retval 2-element orientation array (x, y).
 */
void Measure_Orientation(float* orientation) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// sensor measurements
	BSP_ACCELERO_AccGetXYZ(accel_xyz);
    BSP_GYRO_GetXYZ(gyro_xyz);

	// acceleration magnitude
	float accel_mag = sqrt(accel_xyz[0] * accel_xyz[0] + accel_xyz[1] * accel_xyz[1] + accel_xyz[2] * accel_xyz[2]);


	// normalization of acceleration
	accel_x = accel_xyz[0]/accel_mag;
	accel_y = accel_xyz[1]/accel_mag;
	accel_z = accel_xyz[2]/accel_mag;

	// calculate roll and pitch
	roll_acc = atan2(accel_y, accel_z) * 180.0 / M_PI;
	pitch_acc = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / M_PI;

	// remove bias from gyroscope measurements
	gyro_x = gyro_xyz[0] - gyro_x_bias;
	gyro_y = gyro_xyz[1] - gyro_y_bias;

	/* LOW-PASS FILTER: remove noise from gyroscope measurements */
	kalman(&gyro_x_state, gyro_x);
	kalman(&gyro_y_state, gyro_y);

	/* obtain filtered results */
	gyro_x = gyro_x_state.x*GYRO_X_SENS;
	gyro_y = gyro_y_state.x*GYRO_Y_SENS;

	/* Used to find the operating scale and to
	  	 calculate GYRO_X_SENS and GYRO_Y_SENS. */
//	  if (gyro_x > max_gyro_x) {
//		  max_gyro_x = gyro_x;
//	  }
//
//	  if (gyro_y > max_gyro_y) {
//	  		  max_gyro_y = gyro_y;
//	  }
//
//	  if (gyro_x < min_gyro_x) {
//	  		  min_gyro_x = gyro_x;
//	  }
//
//	  if (gyro_y < min_gyro_y) {
//	  	  	  min_gyro_y = gyro_y;
//	  }

	/* integrate to obtain roll and pitch from gyroscope */
	roll_gyro = angle_x + gyro_x * DT;
	pitch_gyro = angle_y + gyro_y * DT;

	// complementary filter
	angle_x = GYRO_ALPHA * roll_gyro + ACCEL_MAG_ALPHA * roll_acc;
	angle_y = GYRO_ALPHA * pitch_gyro + ACCEL_MAG_ALPHA * pitch_acc;

	orientation[0] = angle_x;
	orientation[1] = angle_y;

}





