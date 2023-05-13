/*
 * maze.c
 *
 *  Created on: Apr 4, 2023
 *      Author: mariobouzakhm
 */
#include "maze.h"
#include "audio_file.h"
#include "gameover_signal.h"

uint8_t rMaze[128];
uint8_t mazeDisplayed[64][128];

uint8_t toLoadMaze4[128] = {
		3, 9, 3, 1, 5, 5, 9, 18, 5, 9, 3, 5, 5, 9, 3, 9,
		10, 10, 10, 6, 9, 7, 12, 6, 13,  4, 4, 1, 9, 6, 12, 10,
		10, 14, 10, 11, 2, 1, 5, 5, 5, 5, 9, 14, 10, 7, 5, 12,
		10, 3, 12, 10, 14, 6, 1, 3, 5, 9, 6, 9, 10, 3, 5, 9,

		10, 2, 5, 4, 5, 9, 2, 12, 3, 12, 7, 4, 8, 10, 3, 12,
		10, 2, 1, 9, 7, 12, 6, 13, 6, 9, 3, 5, 12, 2, 6, 13,
		6, 12, 14, 6, 4, 9, 3, 5, 5, 12, 10, 7, 9, 6, 5, 9,
		7, 5, 5, 5, 5, 12, 6, 5, 41, 7, 4, 5, 4, 5, 5, 12
};

uint8_t toLoadMaze5[128] = {
		7, 5, 5, 9, 7, 9, 3, 20, 9, 3, 13, 3, 9, 3, 5, 13,
		3, 5, 9, 10, 3, 4, 12, 3, 12, 2, 9, 10, 6, 4, 1, 9,
		10, 3, 12, 6, 12, 3, 5, 4, 5, 12, 10, 6, 5, 9, 14, 10,
		10, 2, 13, 3, 9, 6, 9, 7, 9, 3, 12, 9, 5, 12, 3, 8,

		10, 6, 9, 10, 10, 3, 12, 3, 12, 6, 5, 12, 3, 5, 12, 10,
		6, 13, 6, 8, 6, 12, 3, 12, 3, 5, 5, 9, 6, 5, 9, 14,
		3, 5, 5, 12, 3, 9, 6, 5, 12, 3, 5, 12, 3, 5, 4, 9,
		6, 5, 5, 5, 12, 6, 5, 5, 41, 6, 5, 5, 12, 7, 5, 12
};

uint8_t toLoadMaze6[128] = {
		7, 5, 1, 9, 7, 9, 11, 18, 5, 5, 9, 3, 5, 9, 3, 13,
		3, 9, 6, 6, 9, 2, 12, 6, 5, 9, 10, 10, 3, 12, 2, 9,
		10, 6, 5, 5, 12, 6, 5, 5, 5, 12, 6, 8, 3, 9, 14, 10,
		2, 5, 5, 5, 1, 1, 5, 5, 7, 5, 9, 6, 12, 10, 3, 8,

		6, 9, 7, 5, 12, 10, 11, 6, 5, 9, 6, 9, 11, 10, 14, 10,
		3, 8, 3, 9, 7, 12, 6, 5, 9, 6, 9, 2, 4, 4, 5, 12,
		10, 6, 12, 10, 3, 9, 7, 5, 4, 5, 8, 2, 9, 7, 5, 9,
		6, 5, 13, 6, 12, 6, 5, 5, 41, 7, 4, 12, 6, 5, 5, 12
};

uint8_t toLoadMaze7[128] = {
		11, 3, 1, 5, 5, 9, 7, 24, 3, 5, 1, 5, 5, 5, 5, 9,
		2, 12, 6, 1, 13, 6, 5, 12, 6, 9, 2, 5, 5, 9, 3, 12,
		10, 3, 9, 6, 5, 5, 5, 5, 9, 10, 2, 5, 13, 10, 6, 9,
		6, 12, 6, 5, 1, 9, 7, 5, 12, 10, 14, 3, 5, 12, 3, 12,

		3, 1, 5, 5, 12, 6, 5, 5, 5, 8, 3, 12, 7, 1, 12, 11,
		10, 2, 9, 3, 5, 9, 7, 5, 9, 14, 6, 5, 9, 2, 9, 10,
		10, 10, 6, 12, 7, 4, 5, 5, 12, 3, 5, 5, 12, 10, 10, 10,
		14, 6, 5, 5, 5, 5, 5, 13, 35, 12, 7, 5, 5, 12, 6, 12
};

uint32_t sineValue[500];
uint32_t sine2Value[200];

#define MAZE_NUM 4; //number of mazes currently loaded to QPSI

//Initializes the screen and the QSPI
void mazeDisplayInit() {
	  BSP_QSPI_Init();
	  srand(time(NULL));  // seed the random number generator
	  ssd1306_Init();

	  //writeMazesToFlash();

	  readMazeFromSector(0, rMaze); //read initial maze
	  convertLoadedMazeToDisplayedMaze(rMaze);
}

//Function that writes mazes to the QSPI - Only one time use, Mazes need to be written to the QSPI only once
void writeMazesToFlash() {
	writeMazeToSectors(0, toLoadMaze4);
	writeMazeToSectors(1, toLoadMaze5);
	writeMazeToSectors(2, toLoadMaze6);
	writeMazeToSectors(3, toLoadMaze7);
}

//Function that determines the start position on the ball based on the maze loaded from the QSPI Flash
int determineStartPosition() {
	int result = -1;
	for(int i = 0; i < 128; i++) {
		if(rMaze[i] & 32) {
			return i;
		}
	}

	return result;
}

//Function that plays the win sounds using the DAC Speaker.
void playSounds() {
	HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, (uint32_t *) audio_signal, SIGNAL_LENGTH, DAC_ALIGN_12B_R);

	HAL_Delay(1800);

	HAL_DAC_Stop_DMA(&hdac1, DAC1_CHANNEL_1);
	HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, (uint32_t *) audio_signal, SIGNAL_LENGTH, DAC_ALIGN_12B_R);

	HAL_Delay(1800);

	HAL_DAC_Stop_DMA(&hdac1, DAC1_CHANNEL_1);
	HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, (uint32_t *) audio_signal, SIGNAL_LENGTH, DAC_ALIGN_12B_R);

	HAL_Delay(1800);

	HAL_DAC_Stop_DMA(&hdac1, DAC1_CHANNEL_1);
	HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, (uint32_t *) audio_signal, SIGNAL_LENGTH, DAC_ALIGN_12B_R);

	HAL_Delay(1800);

	HAL_DAC_Stop_DMA(&hdac1, DAC1_CHANNEL_1);
	HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, (uint32_t *) audio_signal, SIGNAL_LENGTH, DAC_ALIGN_12B_R);

	HAL_Delay(1800);

	HAL_DAC_Stop_DMA(&hdac1, DAC1_CHANNEL_1);
}

//Function that plays the game over sound when the player runs out of time.
void playGameOverSound() {
	HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, (uint32_t *) gameover_sig, SIGNAL_LENGTH_GO, DAC_ALIGN_12B_R);
	HAL_Delay(1800);
	HAL_DAC_Stop_DMA(&hdac1, DAC1_CHANNEL_1);
}

/**
function to update and displat the timer on the screen, takes as input the elapsed time and the total time permitted for the gaem*/
void updateTime(int i, int maxtime){
	ssd1306_SetCursor(115,2); //top right
	int num = maxtime-i; //get remaining time
	char str[20]; 
	if (num<10){ //single digit
		sprintf(str, "%02d", num); //format string to display on the screen
	}else{
		sprintf(str, "%d", num);
	}
	ssd1306_WriteString(str, Font_6x8, White); //write winning string
	ssd1306_UpdateScreen();
}


/*
 * fct to randomely select mazes from the number of mazes currently loaded to flash
 */
void selectRandomMaze(){

	int sel = rand() % MAZE_NUM;	//select random index from total number of mazes
	readMazeFromSector(sel, rMaze); //dispaly on  screen
	convertLoadedMazeToDisplayedMaze(rMaze);
}

/**
fct to clar screen*/
void clearScreen(){
	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();
}

// Function that displays the maze on the LCD Display
void displayMaze1() {
	  for(int i = 0; i < 64; i++) {
		  for(int j = 0; j < 128; j++) {
			  if(mazeDisplayed[i][j] == 1) {
				  ssd1306_DrawPixel(j, i, White);
			  }
		  }
	  }

	  ssd1306_UpdateScreen();
}

//Function that convert the maze loaded from the Flash Memory (128 8-bit unsigned ints) to an array of 64x128 to be displayed on the screen.
void convertLoadedMazeToDisplayedMaze(uint8_t *loadedMaze) {
	//Initialize the Array with Zeros
		for(int i = 0; i < 64; i++) {
			for(int j = 0; j < 128; j++) {
				mazeDisplayed[i][j] = 0;
			}
		}

		//Iterate over all the cells in loaded Maze
		for(int i = 0; i < 128; i++) {
			uint8_t cellRow = i / 16;
			uint8_t cellColumn = i %16;

			uint8_t cellNumber  = loadedMaze[i];
			uint8_t topWall = cellNumber  & 1;
			uint8_t leftWall = cellNumber & 2;
			uint8_t bottomWall = cellNumber & 4;
			uint8_t rightWall = cellNumber & 8;

			uint8_t leftWallIndex = cellColumn * 8;
			uint8_t rightWallIndex = leftWallIndex + 7;
			uint8_t topWallIndex = cellRow * 8;
			uint8_t bottomWallIndex = topWallIndex + 7;

			for(int k = 0; k < 8; k++) {
				if(topWall != 0) {
					mazeDisplayed[topWallIndex][leftWallIndex+k]= 1;
				}

				if(bottomWall != 0) {
					mazeDisplayed[bottomWallIndex][leftWallIndex+k] = 1;
				}

				if(leftWall != 0) {
					mazeDisplayed[topWallIndex+k][leftWallIndex] = 1;
				}

				if(rightWall != 0) {
					mazeDisplayed[topWallIndex+k][rightWallIndex] = 1;
				}
			}
		}
}

//Write a maze to a speicific sector in the QSPI
void writeMazeToSectors(uint16_t mazeNumber, uint8_t *mazeToSave) {
	uint16_t firstSector = mazeNumber;

	if(BSP_QSPI_Erase_Sector(firstSector) != QSPI_OK) {
//		UART_TX("Error Erase Sector 1\n", 21);
	}

	while(BSP_QSPI_GetStatus() != QSPI_OK);

//	UART_TX("Sector Erase OK!\n", 17);

	uint32_t firstStartAddress = firstSector * 4096;

	if(BSP_QSPI_Write(mazeToSave, firstStartAddress, 128) != QSPI_OK) {
//		UART_TX("Write Error\n", 12);
	}


//	UART_TX("Maze Write OK\n", 14);
}

//Reads a maze from a specific sector in the QSPi.
void readMazeFromSector(uint16_t mazeNumber, uint8_t *mazeBuffer) {
	uint16_t firstSector = mazeNumber;
	uint32_t firstStartAddress = firstSector * 4096;

	if(BSP_QSPI_Read(mazeBuffer, firstStartAddress, 128) != QSPI_OK) {
//		UART_TX("Read Error\n", 11);
	}


//	UART_TX("Maze Read OK\n", 13);
}
