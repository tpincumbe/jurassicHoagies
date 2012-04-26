#pragma once
#include <vector>
#include <unordered_set> 
#include "GridLocation.h"

using namespace std;
using namespace std::tr1;

class Grid {
	vector<vector<GridLocation>> map;	// 100x100 default ?
	vector<int> start, goal;

public:
	int numRows, numCols;
	Grid(vector<vector<GridLocation>>);
	Grid(int**,int,int);
	Grid(vector<int>, Grid*);
	void enlargeObstacles(int);
	void setLocation(int,int,int);
	vector<vector<GridLocation>>* getMap();
	vector<GridLocation> search(vector<int>,vector<int>);
	vector<int> getGoalLocation();
	GridLocation* lowestScore(unordered_set<GridLocation*> theSet);
	int round(float in);
};
