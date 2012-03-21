#include <math.h>
#include <iostream>
#include <vector>

using namespace std;

class Grid;


class GridLocation {
	int row, col, value;
	Grid* grid;

public:
	GridLocation(Grid*,int,int,int);
	bool isEdge();	// return true iff value=0 (not wall) & has neighboring walls
	void setValue(int);
	vector<GridLocation> getNeighbors() {		// TODO: change to getValidMoves(int/float pleoOrientation) --> return only valid moves he can make.. ?
		vector<GridLocation> out;
		if (row>0) {
			if (grid->getMap()[row-1][col].value==0) {
				out.push_back(grid->getMap()[row-1][col]);
			}
			if (col>0) {
				if (grid->getMap()[row-1][col-1].value==0) {
					out.push_back(grid->getMap()[row-1][col-1]);
				}
			}
		}
		if (row<grid->numRows-1) {
			if (grid->getMap()[row+1][col].value==0) {
				out.push_back(grid->getMap()[row+1][col]);
			}
			if (col<grid->numCols-1) {
				if (grid->getMap()[row+1][col+1].value==0) {
					out.push_back(grid->getMap()[row+1][col+1]);
				}
			}
		}
		if (col>0) {
			if (grid->getMap()[row][col-1].value==0) {
				out.push_back(grid->getMap()[row][col-1]);
			}
			if (row<grid->numRows-1) {
				if (grid->getMap()[row+1][col-1].value==0) {
					out.push_back(grid->getMap()[row+1][col-1]);
				}
			}
		}
		if (col<grid->numCols-1) {
			if (grid->getMap()[row][col+1].value==0) {
				out.push_back(grid->getMap()[row][col+1]);
			}
			if (row>0) {
				if (grid->getMap()[row-1][col+1].value==0) {
					out.push_back(grid->getMap()[row-1][col+1]);
				}
			}
		}
		return out;
	}
};

GridLocation::GridLocation(Grid* g,int r, int c, int v) {
	grid = g;
	row=r;	col=c;	value=v;
}
void GridLocation::setValue(int v) { value=v; }
bool GridLocation::isEdge() {
	if (value == 1)	return false;	// already an edge
	if (row==0 || col==0 || row==grid->numRows-1 || col==grid->numCols-1) {		// boundary edge
		return true;
	}
	if (grid->getMap()[row-1][col].value==1 || grid->getMap()[row-1][col-1].value==1 || 
		grid->getMap()[row][col-1].value==1 || grid->getMap()[row+1][col-1].value==1 || 
		grid->getMap()[row+1][col].value==1 || grid->getMap()[row+1][col+1].value==1 || 
		grid->getMap()[row][col+1].value==1 || grid->getMap()[row-1][col+1].value==1) {
		return true;
	}
	return false;
}


class Grid {
	GridLocation** map;		// 100 x 100 default

public:
	int numRows, numCols;
	Grid(GridLocation**);
	Grid(int**,int,int);
	void enlargeObstacles(int);
	GridLocation* getEdgeLocations();
	void setLocation(int,int,int);
	GridLocation** getMap();
	vector<GridLocation> search(int*,int*);
};

Grid::Grid(GridLocation** gl) {
	map = gl;
	numRows=100; numCols=100;
}
Grid::Grid(int** pixels, int numPx, int numPy) {
	float xFactor = numPx/100.0;
	float yFactor = numPy/100.0;
	for (int i=0; i<100; i++) {
		for (int j=0; j<100; j++) {
			map[i][j] = GridLocation(this,i,j,pixels[round(i*xFactor)][round(j*yFactor)]);
		}
	}
}
void Grid::enlargeObstacles(int gridSpaces) {
	for (int i=0; i<gridSpaces; i++) {
		GridLocation** tmp = map;		// TODO: needz deep copy !!!!!!!!!!!!!!!
		for (int i=0; i<numRows; i++) {
			for (int j=0; j<numCols; j++) {
				if (map[i][j].isEdge())		tmp[i][j].setValue(1);
			}
		}
		map = tmp;
	}
}
void Grid::setLocation(int r, int c, int v) {
	map[r][c] = GridLocation(this,r,c,v);
}
GridLocation** Grid::getMap() {
	return map;
}
vector<GridLocation> Grid::search(int* start, int* goal) {
	vector<GridLocation> out;	// will return vector of GridLocations or vector of Pleo movement primitives

	// TODO: 

	return out;
}


int round(float in) {
	if (in > 0)
		return floor(in + 0.5);
	else
		return floor(in - 0.5);
}




int main() {

	// TODO: test after fixes

	//Grid theTestGridThatIAmMakingJustToTestTheFunctionalityOfWhatIHaveJustWritten = Grid

	//return 0;
}