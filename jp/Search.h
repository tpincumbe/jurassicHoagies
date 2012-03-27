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
	vector<int> gridLocToActual(vector<int> gridLoc);
	vector<int> actualLocToGrid(vector<int> actualLoc);
	void findPath(vector<vector<int>> vec);
private:
	float conversionFactor;	// TODO: conversion factor of px to grid spaces
};