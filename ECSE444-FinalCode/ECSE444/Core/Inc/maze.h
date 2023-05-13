/*
 * maze.h
 *
 *  Created on: Apr 4, 2023
 *      Author: mariobouzakhm
 */


#ifndef INC_MAZE_H_
#define INC_MAZE_H_

#include <stdio.h>
#include <stdlib.h>
#include "ssd1306.h"
#include "stm32l4s5i_iot01_qspi.h"
#include <time.h>
#include "main.h"

extern uint8_t rMaze[128];
extern uint8_t mazeDisplayed[64][128];

extern uint32_t sineValue[500];
extern uint32_t sine2Value[200];

void initializeSounds();
void writeMazesToFlash();
int determineStartPosition();
void playSounds();
void playGameOverSound();
void mazeDisplayInit();
void selectRandomMaze();
void clearScreen();
void displayMaze1();
void convertLoadedMazeToDisplayedMaze(uint8_t *loadedMaze);
void writeMazeToSectors(uint16_t mazeNumber, uint8_t *mazeToSave);
void readMazeFromSector(uint16_t mazeNumber, uint8_t *mazeBuffer);
void updateTime();

#endif /* INC_MAZE_H_ */
