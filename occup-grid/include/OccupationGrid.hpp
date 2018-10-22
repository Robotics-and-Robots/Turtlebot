#ifndef _OCCUPATION_GRID_HPP
#define _OCCUPATION_GRID_HPP

#include <cmath>

#define OG_WIDTH  4 
#define OG_HEIGTH 5

#define OG_SEC_W (OG_WIDTH  / 2)
#define OG_SEC_H (OG_HEIGTH / 2)

//Cell elements are float-typed values
typedef float OGCellType;

//Defines a class to hold values for an
//occupation grid. 
class OccupationGrid{

private:
	OGCellType _m_pospos[OG_SEC_W][OG_SEC_H]; //X is positive, Y is positive.
	OGCellType _m_posneg[OG_SEC_W][OG_SEC_H]; //X is positive, Y is negative.
	OGCellType _m_negpos[OG_SEC_W][OG_SEC_H]; //X is negative, Y is positive.
	OGCellType _m_negneg[OG_SEC_W][OG_SEC_H]; //X is negative, Y is negetive.

public:
	
	//ctor. and dtor.
	OccupationGrid();
	~OccupationGrid();
	
	//overload [] to access as an array
	OGCellType Get(int x, int y);
	
	//set some value to a cell
	OGCellType Set(int x, int y, OGCellType value);
};

OGCellType OccupationGrid::Get(int x, int y){

	int a = std::abs(x);
	int b = std::abs(y);

	if(x >=0 && y >=0) return _m_pospos[a][b];
	if(x >=0 && y < 0) return _m_posneg[a][b];
	if(x < 0 && y >=0) return _m_negpos[a][b];
	else return _m_negneg[a][b];
}

OGCellType OccupationGrid::Set(int x, int y, OGCellType value){
	
	int a = std::abs(x);
	int b = std::abs(y);

	if(x >=0 && y >=0) return _m_pospos[a][b] = value;
	if(x >=0 && y < 0) return _m_posneg[a][b] = value;
	if(x < 0 && y >=0) return _m_negpos[a][b] = value;
	else return _m_negneg[a][b] = value;
}

OccupationGrid::OccupationGrid(){}
OccupationGrid::~OccupationGrid(){}

#endif /* _OCCUPATION_GRID_HPP */