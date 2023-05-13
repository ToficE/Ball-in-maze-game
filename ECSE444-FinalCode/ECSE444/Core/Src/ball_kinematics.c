/**
  ******************************************************************************
  * @file           : ball_kinematics.c
  * @author			: Tofic Esses
  * @brief          : This file provides a set of functions to determine the
  * 				  ball's displacement in a maze.
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
#include "ssd1306.h"
#include "maze.h"
#include <time.h>


/* Defines --------------------------------------------------------------------*/

#define BALL_MASS 1.0
#define g 10.0
#define uk 0.5
#define DISP_SCALE (1.0/7000.0)
#define BALL_RADIUS 1


/* Variables ------------------------------------------------------------------*/

// ball position
int x_pos;
int y_pos;

// orientation of board
float x_angle;
float y_angle;

// discrete timestep
float dt;

/* analog velocity */
float v_x;
float v_x_prev;

float a_x, a_y;

float v_y;
float v_y_prev;

// displacement
float d_x;
float d_y;
float Dx;
float Dy;

int x_ball_pos;
int y_ball_pos;

int x_hole_pos;
int y_hole_pos;

int win; //win boolean



/*
 *  @brief  Initialize ball kinematics and timestep
 *  @params timestep
  * @retval None.
 */
void Initialize_Game(float timestep) {
	// dt for numerical integration
	dt = timestep;


	// initialize ball params
	x_angle = 0.0;
	y_angle = 0.0;

	d_x = 0;
	d_y = 0;

	Dx = 0;
	Dy = 0;

	v_x = 0.0;
	v_x_prev = 0.0;

	v_y = 0.0;
	v_y_prev = 0.0;

	win = 0; //set win boolean to 0
}

/*
 * @brief initialize ball on LCD display
 * @params (x, y) position
 */
void Init_Ball(int x_pos, int y_pos){
	x_ball_pos = x_pos;
	y_ball_pos = y_pos;
	ssd1306_DrawCircle(x_pos, y_pos, BALL_RADIUS, White);
	ssd1306_UpdateScreen();
}


/*
 * @brief updates ball on LCD display based on change
 * @params (dx, dy)
 */
void Update_Position(int dx, int dy){
	ssd1306_DrawCircle(x_ball_pos, y_ball_pos, BALL_RADIUS, Black);
	x_ball_pos += dx;
	y_ball_pos += dy;
	ssd1306_DrawCircle(x_ball_pos, y_ball_pos, BALL_RADIUS, White);
}

/**
 * @brief display the ball entering the win area and display a win string
 * 		  and plays sounds
 */
void winSequence(){
	char winString[] = "Win!";
	for (int i = 0; i<BALL_RADIUS+2;i++){ //ball animation entering the exit
		Update_Position(0,-1);
		ssd1306_UpdateScreen();
		HAL_Delay(300);
	}
	ssd1306_SetCursor(42,23); //center string
	ssd1306_WriteString(winString, Font_11x18, White); //write winning string
	ssd1306_UpdateScreen();
	playSounds();

	HAL_Delay(500);
}

/**
 * @brief displays a "GAME OVER" string and plays sounds
 */
void gameOver(){
	char lossString[] = "GAME OVER!";
	ssd1306_SetCursor(14,23); //center string
	ssd1306_WriteString(lossString, Font_11x18, White); //write winning string
	ssd1306_UpdateScreen();
	//playSounds();
	playGameOverSound();
	HAL_Delay(500);
}


/**
 * @brief calculates the ball's kinematics
 * @params pointers to the (theta_x, theta_y) STM32 orientation angle
 */
void Calculate_Ball_Kinematics(float* theta_x_in, float* theta_y_in) {

	// transform angles to match simulation
	x_angle = -(*theta_y_in);
	y_angle = -(*theta_x_in);

	// X-DIRECTION
	if (x_angle < 2 && x_angle > -2) {
		// check if angle is negligible small
		a_x = 0;
		v_x_prev = v_x_prev*0.99;
	} else {
		a_x = g*arm_sin_f32(x_angle*M_PI/180.0);
	}

	if ((a_x > 0 && v_x_prev < 0) || (a_x < 0 && v_x_prev > 0)) {
		// if they have opposite directions
		v_x_prev = v_x_prev*0.0001;
	}

	v_x = v_x_prev + a_x*dt;

	// Y-DIRECTION
	if (y_angle < 2 && y_angle > -2) {
		// check if angle is negligible small
		a_y = 0;
		v_y_prev = v_y_prev*0.99;
	} else {
		a_y = g*arm_sin_f32(y_angle*M_PI/180.0);
	}

	if ((a_y > 0 && v_y_prev < 0) || (a_y < 0 && v_y_prev > 0)) {
		// if they have opposite directions
		v_y_prev = v_y_prev*0.0001;
	}


	v_y = v_y_prev + a_y*dt;


}

/**
 * @brief checks for collisions
 */
void Calculate_Ball_Velocity() {

	if (v_x > 0) {
		if (x_ball_pos >= SSD1306_WIDTH -1- BALL_RADIUS) {
			v_x_prev = 0;
			v_x = 0;
		}
		// avoid collision with wall directly
		if (mazeDisplayed[y_ball_pos][x_ball_pos+1+BALL_RADIUS] == 1) {
			v_x_prev = 0;
			v_x = 0;
		}
	}
	else if (v_x < 0) { // negative x velocity
		if (x_ball_pos <= 0 + BALL_RADIUS + 1) {
			v_x_prev = 0;
			v_x = 0;
		}
		// avoid collision with wall directly
		if (mazeDisplayed[y_ball_pos][x_ball_pos-1-BALL_RADIUS] == 1) {
			v_x_prev = 0;
			v_x = 0;
		}
	}

	// positive y velocity
	if (v_y > 0) {
		if (y_ball_pos >= SSD1306_HEIGHT -1- BALL_RADIUS) {
			v_y_prev = 0;
			v_y = 0;
		}
		// avoid collision with wall direclty
		if (mazeDisplayed[y_ball_pos+1+BALL_RADIUS][x_ball_pos] == 1) {
			v_y_prev = 0;
			v_y = 0;
		}
	}
	else if (v_y < 0) { // negative y velocity
		if (y_ball_pos <= 0 + BALL_RADIUS + 1) {
			v_y_prev = 0;
			v_y = 0;
			//upper collision => check for win
			if (mazeDisplayed[0][x_ball_pos-1-BALL_RADIUS] == 0 && mazeDisplayed[0][x_ball_pos+1+BALL_RADIUS] == 0){ //check if opening has been found
				win = 1; //set win boolean to 1 and return from function
				return;
			}
		}
		// avoid collision with wall directly
		if (mazeDisplayed[y_ball_pos-1-BALL_RADIUS][x_ball_pos] == 1) {
			v_y_prev = 0;
			v_y = 0;
		}
	}
}

///*
// *  @brief  Calculate ball's change of displacement
//  * @retval None.
// */
void Calculate_Delta_Displacement() {
	// calculate displacement over timestep
	// AREA UNDER CURVE SEGMENT (Trapezium)
	d_x = (0.5)*(v_x_prev + v_x)*dt;
	d_y = (0.5)*(v_y_prev + v_y)*dt;

	// update velocity
	v_x_prev = v_x;
	v_y_prev = v_y;
}


/**
 * @brief Accumulates the ball's displacement vectors
 */
void Accumulate_Displacement() {
	Dx += d_x;
	Dy += d_y;
}

/**
 * @brief Wrapper function that determines the ball's movement and updates the screen accordingly
 * @params pointers to the (theta_x, theta_y) STM32 orientation angle, returns a 1 if a win has been detected a 0 otherwise
 */
int Determine_Ball_Movement(float* theta_x_in, float* theta_y_in) {
	Calculate_Ball_Kinematics(theta_x_in, theta_y_in);
	Calculate_Ball_Velocity();
	//check for win
	if (win == 1){
		return 1; //if win has been detected, righturn right away
	}
	Calculate_Delta_Displacement();
	Accumulate_Displacement();
	// assume that each pixel represents 1mm
	if (Dx >= DISP_SCALE) {
		Update_Position(1, 0);
		Dx = 0;
	}
	else if (Dx <= -DISP_SCALE) {
		Update_Position(-1, 0);
		Dx = 0;
	}

	if (Dy >= DISP_SCALE) {
		Update_Position(0, 1);
		Dy = 0;
	}
	else if (Dy <= -DISP_SCALE) {
		Update_Position(0, -1);
		Dy = 0;
	}
	ssd1306_UpdateScreen();
	return 0;
}
