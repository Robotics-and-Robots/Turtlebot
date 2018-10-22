#include <iostream>

#include <OccupationGrid.hpp>


int main(int argc, char** argv){

	OccupationGrid* g = new OccupationGrid();

	g->Set(7, 7, 54.5f);

	std::cout << g->Get(7, 7) << std::endl;
	
	return 0;
}