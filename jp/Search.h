#pragma once
#include <math.h>
#include <iostream>
#include <vector>
#include "Grid.h"
#include "GridLocation.h"

using namespace std;

class Search {

public:
	Search();
	Grid getGrid(vector<vector<int>> pixelArray);
	vector<int> gridLocToActual(vector<int> gridLoc);
	vector<int> actualLocToGrid(vector<int> actualLoc);
	vector<vector<int>> findPath(vector<int> pixelArray, vector<int> start, vector<int> end);
private:
	float conversionFactor;	// TODO: conversion factor of px to grid spaces
};