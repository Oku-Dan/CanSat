#include "machine.h"

#define GOAL_LAT 100
#define GOAL_LON 100

Machine machine;

void setup(){
	machine.Initialize();
}
void loop(){
	machine.Update();
	
}
