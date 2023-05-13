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

void Initialize_Game(float timestep);
void Init_Ball(int x_pos, int y_pos);
void Init_Hole(int x_pos, int y_pos);
void Update_Position(int dx, int dy);

void Calculate_Ball_Kinematics(float* theta_x_in, float* theta_y_in);

void Calculate_Delta_Displacement(void);

void Accumulate_Displacement(void);

void Calculate_Ball_Velocity();

int Determine_Ball_Movement(float* theta_x_in, float* theta_y_in);

void winSequence();

void gameOver();
