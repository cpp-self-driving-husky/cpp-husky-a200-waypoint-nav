/*

	simulation latitude longitude

	top left
		49.8999160171 8.90012732623

	top right
		49.8999237892 8.89987663986

	bottom left
		49.9000753632 8.90012531813

	bottom right
		49.9000770425 8.89988743462
	
	49.8999237892 8.89987663986 49.8999160171 8.90012732623 49.9000753632 8.90012531813 49.9000770425 8.89988743462

*/
#include "global_checkpoints.h"


int main(int argc, char* argv[]) {
	if (argc >= 3 && (argc-1) % 2 == 0) {
		PathGenerator path_gen = PathGenerator();
		return path_gen.run(argc,argv);
	} else {
		printf("Invalid number of arguments!");
		return -1;
	}
}
