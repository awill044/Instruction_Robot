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

#include <stdlib.h>

// Fills the instruction sequence array will proper values
void instruct_sequence(signed char *list, unsigned char dir, unsigned char instruct){
	unsigned char i=0;
	if(instruct >= 4)
		return; // case in which robot has reached goal
	// If the robot is not facing the right direction.
	if(dir != instruct){
		// if facing opposite direction (must turn twice)
		if(abs(dir-instruct) == 2){
			list[i] = 1; // turn right 90 degrees
			i++;
			list[i] = 1;
			i++;
		}
		else if(((signed char)dir-(signed char)instruct == 1) || ((dir==0)&&(instruct==3))){
			list[i] = 3; // turn left 90 degrees
			i++;
		}
		else{
			list[i] = 1;
			i++;
		}
	}
	list[i] = 0;
}