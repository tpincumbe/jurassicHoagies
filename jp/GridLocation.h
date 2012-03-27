#pragma once
#include <vector>
#include "Grid.h"

using namespace std;

class Grid;

class GridLocation {
	int row, col, value, pathCost;
	Grid *grid;
	GridLocation* cameFrom;

public:
	GridLocation(Grid*,int,int,int);
	GridLocation::GridLocation();
	bool isEdge();	// return true iff value=0 (not wall) & has neighboring walls
	void setValue(int);
	int getValue();
	vector<GridLocation*> getValidMoves(int);
	vector<GridLocation*> getNeighbors();
	int getScore();
	void setPathCost(int);
	int getPathCost();
	int heuristic();
	vector<int> getLocation();

	void setCameFrom(GridLocation*);
	GridLocation* getCameFrom();

	Grid* getGrid();
	void setGrid(Grid*);
};