// Copyright 2015, 2016 Andrew Williams
/*
    This file is part of Instruction Robot.

    Instruction Robot is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Instruction Robot is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Instruction Robot.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "bit.h"
#include "lcd.h"
#include "scheduler.h"
#include "helpers.h"

// Enumerates a type to indicate which cell to move into. 
// i.e. cT_NORTH indicates a movement into the cell in the direction of the
// world's "north" direction (has nothing to do with actual, geographical
// north. This north is whatever direction the robot is facing when it is
// activated.
typedef enum cellType {cT_NORTH, cT_EAST, cT_SOUTH, cT_WEST, cT_STAY} cellType;
// Determines the type of movement that should be executed.
// i.e.,  mTYPE_LEFT indicates the robot should be turning to the left.
typedef enum moveType {mTYPE_STAY, mTYPE_FORWARD, mTYPE_LEFT, mTYPE_RIGHT} moveType;
// Indicates a facing direction
typedef enum facingDirection {dir_NORTH, dir_EAST, dir_SOUTH, dir_WEST} facingDirection;

// defines the number of orders to be carried out. For example,
// moving north, then south, then west is 3 orders. To add more
// orders, change this value to the desired amount and copy and
// paste the 3 lines below the "2nd order" comment in the
// function "init_orders" below until the number of orders
// matches.
#define orderAmt 6

struct Order{
	facingDirection direction;
	cellType cell;
};
struct Order orders[orderAmt];
void init_orders(){
	unsigned char j=0;
	// 1st order
	orders[j].direction = dir_NORTH; // initial direction in relation to the world
	orders[j].cell = cT_WEST;
	j++;
	// 2nd order
	orders[j].direction = (cellType) orders[j-1].cell;
	orders[j].cell = cT_SOUTH;
	j++;
	// 3rd order
	orders[j].direction = (cellType) orders[j-1].cell;
	orders[j].cell = cT_EAST;
	j++;
	// 4th order
	orders[j].direction = (cellType) orders[j-1].cell;
	orders[j].cell = cT_SOUTH;
	j++;
	// 5th order
	orders[j].direction = (cellType) orders[j-1].cell;
	orders[j].cell = cT_SOUTH;
	j++;
	// 6th order
	orders[j].direction = (cellType) orders[j-1].cell;
	orders[j].cell = cT_NORTH;
	j++;
}

// This variable is set by the "Exec_Sequence" state machine and read by the "MoveTime"
// state machine. It represents the current movement that should be executed. For
// example, if it is set to the value representing a left turn, then the "MoveTime" SM
// will read it and execute a single 90 degree left turn.
unsigned char finalInstruct = 4;
// This variable is used to communicate between the "MoveTime" state machine and the "Movement" 
// state machine. Specifically, it indicates the type of movement that the robot should be
// making at any given moment. If it is "mTYPE_LEFT", the robot should be pivoting/turning
// left. If it is "mTYPE_FORWARD", the robot should be moving forward. If it is "mTYPE_STAY",
// it should not be moving at all. The "MoveType" state machine changes this variable
// accordingly to make sure that the robot makes the correct moves at the correct durations.
moveType curMoveType = mTYPE_STAY;
// Contains the sequence of instructions to move 1 cell
signed char instructions[4] = {-1, -1, -1, -1};
// Used to communicate between the Exec_sequence state machine and the
// MoveTime state machine. When true, it signals to the MoveTime SM
// that the next command is ready. When it is false, it signals to the
// Exec_sequence that MoveTime is ready for the next command.
bool isReady = false;

// Determines whether the robot is moving forward, backward, turning left, turning right,
// or staying still. This state machine does NOT control the duration for which it moves.
// That functionality is controlled by the "MoveTime" state machine. This is strictly
// controlling which direction the wheels turn. It goes when MoveTime tells it to,
// and it stops when MoveTime tells it to.
enum Movement_States {M_Wait, M_Forward, M_TurnLeft, M_TurnRight};
int Movement( int state ) {
	switch( state ) {
		case M_Wait:
			if(curMoveType==mTYPE_LEFT)
				state = M_TurnLeft;
			else if(curMoveType==mTYPE_RIGHT)
				state = M_TurnRight;
			else if(curMoveType==mTYPE_FORWARD)
				state = M_Forward;
			else
				state = M_Wait;
			break;
		case M_Forward:
			if(curMoveType==mTYPE_LEFT)
				state = M_TurnLeft;
			else if(curMoveType==mTYPE_RIGHT)
				state = M_TurnRight;
			else if(curMoveType==mTYPE_FORWARD)
				state = M_Forward;
			else
				state = M_Wait;
			break;
		case M_TurnLeft:
			if(curMoveType==mTYPE_LEFT)
				state = M_TurnLeft;
			else if(curMoveType==mTYPE_RIGHT)
				state = M_TurnRight;
			else if(curMoveType==mTYPE_FORWARD)
				state = M_Forward;
			else
				state = M_Wait;
			break;
		case M_TurnRight:
			if(curMoveType==mTYPE_LEFT)
				state = M_TurnLeft;
			else if(curMoveType==mTYPE_RIGHT)
				state = M_TurnRight;
			else if(curMoveType==mTYPE_FORWARD)
				state = M_Forward;
			else
				state = M_Wait;
			break;
		default:
			state = M_Wait;
			break;
	}
	switch( state ) {
		case M_Wait:
			PORTC |= 0x03;
			_delay_us(1500);
			PORTC &= 0xFC;
			break;
		case M_Forward:
			PORTC |= 0x03;
			_delay_us(1000);
			PORTC &= 0xFE;
			_delay_us(1100);
			PORTC &= 0xFC;
			break;
		case M_TurnLeft:
			PORTC |= 0x03;
			_delay_us(1000);
			PORTC &= 0xFC;
			break;
		case M_TurnRight:
			PORTC |= 0x03;
			_delay_us(2000);
			PORTC &= 0xFC;
			break;
		default:
			break;
	}
	
	return state;
}

// Determines how long the wheels spin for each type of movement. This
// state machine sends commands to the "Movement" state machine to make
// turn its wheels and stop its wheels. So if this state machine sends
// the command to turn to the left, the Movement SM will make the left
// wheel spin backwards and the right wheel spin forwards to make the
// robot pivot to the left. However, MoveTime also has to send the
// command to stop it at the appropriate time, otherwise, Movement
// will keep spinning its wheels. The "maxTime" variable used below
// sets the number of state machine cycles will execute before the
// "stop" command is sent to the Movement SM. Increasing this value
// makes the robot turn further before stopping. Decreasing it makes
// it turn less before stopping.
enum MoveTime_States {MT_Wait, MT_Forward, MT_Left, MT_Right};
int MoveTime( int state ) {
	// A counter to keep track of the amount of state machine
	// cycles that have occurred.
	static unsigned char runTime = 0;
	// Indicates the amount of state machine
	// cycles a movement should last for.
	static unsigned char maxTime = 250;
	switch(state){
		case MT_Wait:
			if((finalInstruct == 0) && (isReady == true)){
				curMoveType = mTYPE_FORWARD;
				maxTime = 112;
				state = MT_Forward;
			}
			else if((finalInstruct == 3) && (isReady == true)){
				curMoveType = mTYPE_LEFT;
				maxTime = 50;
				state = MT_Left;
			}
			else if((finalInstruct == 1) && (isReady == true)){
				curMoveType = mTYPE_RIGHT;
				maxTime = 60;
				state = MT_Right;
			}
			else{
				curMoveType = mTYPE_STAY;
				state = MT_Wait;
			}
			break;
		case MT_Forward:
			if(runTime < maxTime){
				state = MT_Forward;
				runTime++;
			}
			else{
				state = MT_Wait;
				curMoveType = mTYPE_STAY;
				isReady = false;
			}
			break;
		case MT_Left:
			if(runTime < maxTime){
				state = MT_Left;
				runTime++;
			}
			else{
				state = MT_Wait;
				curMoveType = mTYPE_STAY;
				isReady = false;
			}
			break;
		case MT_Right:
			if(runTime < maxTime){
				state = MT_Right;
				runTime++;
			}
			else{
				state = MT_Wait;
				curMoveType = mTYPE_STAY;
				isReady = false;
			}
			break;
		default:
			state = MT_Wait;
			curMoveType = mTYPE_STAY;
			break;
	}
	switch(state){
		case MT_Wait:
			runTime=0;
			break;
		case MT_Forward:
			break;
		case MT_Left:
			break;
		case MT_Right:
			break;
		default:
			curMoveType=mTYPE_STAY;
			break;
	}
	return state;
}

// Runs through the individual instructions required to move the robot
// into a different cell. The "individual instructions" are pivot 90
// degrees left, pivot 90 degrees right, and go straight. Using
// different combinations of these, the robot can move into any
// adjacent cell (north, south, west, east). These individual commands
// are saved in the "instructions" array declared above. When the
// robot has finished these instructions and entered a new cell, the
// index of the "orders" array, which stores the "big picture" instructions 
// (i.e., the sequence of cells to move into, like NORTH, then SOUTH,
//  etc.) is incremented so that the instructions for the next cell
// destination are determined and executed. This continues until
// there are no more elements in the "orders" array.
enum Exec_Sequence_States {ES_Init, ES_Wait, ES_Command, ES_NextCell, ES_Done};
int Exec_Sequence(int state){
	static unsigned char i=0;
	static unsigned char n=0;
	switch(state){
		case ES_Init:
			state = ES_NextCell;
			i=0;
			break;
		case ES_Wait:
			if(isReady == true){
				state = ES_Wait;
			}
			else{
				state = ES_Command;
			}
			break;
		case ES_Command:
			if((i>=4) || (instructions[i]<0)){
				state = ES_NextCell;
			}
			else{
				state = ES_Wait;
			}
			break;
		case ES_NextCell:
			if(n >= orderAmt){
				state = ES_Done;
			}
			else{
				state = ES_Wait;
				n++;
			}
			break;
		case ES_Done:
			state = ES_Done;
			break;
		default:
			state = ES_Init;
			break;
	}
	switch(state){
		case ES_Init:
			break;
		case ES_Wait:
			break;
		case ES_Command:
			i++;
			if((instructions[i]>=0) && (i<4)){
				finalInstruct = instructions[i];
				isReady = true;
			}
			break;
		case ES_NextCell:
			for(i=0; i<4; i++){
				instructions[i] = -1;
			}
			if(n < orderAmt){
				instruct_sequence(instructions, orders[n].direction, orders[n].cell);
				i=0;
				finalInstruct = instructions[i];
				isReady = true;
			}
			break;
		case ES_Done:
			break;
		default:
			break;
	}
	return state;
}

// Controls the range finder at the front of the robot. This does not, in any way, affect the
// robot's movement.
enum RangeFind_States {RF_Read};
int Range_Finder(int state) {
	static unsigned int range=0;
	static char buf[10];
	switch (state) {
		case RF_Read:
			state = RF_Read;
			range = 0;
			break;
		default:
			state = RF_Read;
			break;
	}
	switch (state) {
		case RF_Read:
			PORTD &= 0xFD;
			_delay_ms(2);
			PORTD |= 0x02;
			_delay_ms(10);
			PORTD &= 0xFD;
			for(unsigned short r=0; r<16000; r++){
				if(GET_BIT(PIND, 0))
				++range;
			}
			range /= 70; // saves distance in inches
			sprintf(buf, "%d", range);
			LCD_DisplayString(1, buf);
			break;
		default:
			break;
	}

	return state;
}

int main(void)
{
	DDRA=0xFF; PORTA=0x00;
	DDRB=0x03; PORTB=0x00;
	DDRC=0xFF; PORTC=0x00;
	DDRD=0xAA; PORTD=0x00;
	
	LCD_init();
	init_orders();
	instruct_sequence(instructions, 0, 4);
    
    tasksNum = 4; // declare number of tasks
    task tsks[4]; // initialize the task array
    tasks = tsks; // set the task array
    
    // define tasks
    unsigned char i=0; // task counter
    tasks[i].state = M_Wait;
    tasks[i].period = 20;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &Movement;
    i++;
    
    tasks[i].state = MT_Wait;
    tasks[i].period = 20;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &MoveTime;
    i++;
	
	tasks[i].state = ES_Init;
	tasks[i].period = 100;
	tasks[i].elapsedTime = tasks[i].period;
	tasks[i].TickFct = &Exec_Sequence;
	i++;
    
	tasks[i].state = RF_Read;
	tasks[i].period = 200;
	tasks[i].elapsedTime = tasks[0].period;
	tasks[i].TickFct = &Range_Finder;
	
    TimerSet(20);
    TimerOn();
    
    while (1)
    {
    }
}

