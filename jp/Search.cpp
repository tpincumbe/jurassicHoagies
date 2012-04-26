#include "Search.h"
#include <vector>
#include <unordered_set>
#include <algorithm>

using namespace std;
using namespace std::tr1;

int manhattanDistance(vector<int>, vector<int>);
vector<GridLocation> reconstructPath(GridLocation*);
int pixelRound(float);

float pixelsPerGrid = 16;


// GRID FUNCTION DECLARATIONS //
Grid::Grid(vector<vector<int>> pixels, Grid* g) {
	numRows = 480/pixelsPerGrid;	numCols = 640/pixelsPerGrid;

	cout << "Making grid of size " << numRows << " x " << numCols << endl;

	int ps = pixels.size();

	float xFactor = 640/numCols;
	float yFactor = 480/numRows;

	for (int i=0; i<numRows; i++) {
		vector<GridLocation> newRow;
		for (int j=0; j<numCols; j++) {
			int px = pixelRound(i*yFactor + (pixelsPerGrid/2));
			int py = pixelRound(j*xFactor + (pixelsPerGrid/2));
					//row,col
			if (pixels[px][py] == 1) {
				cout << "Found an obstacle in grid @ (" << i << ", " << j << ")" << " pixel (" << px << ", " << py << ")" << endl;
			}

			newRow.push_back(GridLocation(g, i, j, pixels[px][py]));
		}
		map.push_back(newRow);
	}
}
void Grid::enlargeObstacles(int gridSpaces) {

	vector<vector<GridLocation>> tmp;
	for (int i=0; i<gridSpaces; i++) {
		
		tmp = map;
		for (int i=0; i<numRows; i++) {
			for (int j=0; j<numCols; j++) {
				if (map[i][j].isEdge()) {
					tmp[i][j].setValue(1);
				}
			}
		}
		map = tmp;
	}
	//~tmp();		// destructor
}
void Grid::setLocation(int r, int c, int v) {
	map[r][c] = GridLocation(this,r,c,v);
}
vector<vector<GridLocation>>* Grid::getMap() {
	return &map;
}
vector<GridLocation> Grid::search(vector<int> startLoc, vector<int> goalLoc) {	// implements A* search algorithm
	start = startLoc;	goal = goalLoc;
	int tmp = start[0];
	start[0] = (480-start[1])/pixelsPerGrid;	// start row = (480-pixel_y)/pixelsPerGrid
	start[1] = tmp/pixelsPerGrid;				// start col = pixel_x/pixelsPerGrid
	tmp = goal[0];
	goal[0] = (480-goal[1])/pixelsPerGrid;
	goal[1] = tmp/pixelsPerGrid;

	if (start[0] >= numRows)	start[0] = numRows-1;
	if (start[1] >= numCols)	start[0] = numCols-1;
	if (goal[0] >= numRows)		goal[0] = numRows-1;
	if (goal[1] >= numCols)		goal[0] = numCols-1;

	cout << "Searching from (" << start[0] << ", " << start[1] << ") to (" << goal[0] << ", " << goal[1] << ")" << endl;

	unordered_set<GridLocation*> closed, open;
	//priority_queue<GridLocation> open;

	GridLocation* current = &map[start[0]][start[1]];

	GridLocation curGridLoc = *current;

	current->setPathCost(0);

	open.insert(current);

	while (!open.empty()) {
		current = lowestScore(open);
		if (current->getLocation() == getGoalLocation())	break;			// TODO: will this ever be equal?
		open.erase(current);
		closed.insert(current);

		vector<GridLocation*> neighbors = current->getNeighbors();
		//vector<GridLocation*> neighbors = current->getValidMoves(0);
		for (unsigned int i=0; i<neighbors.size(); i++) {
			GridLocation* neighbor = neighbors[i];

			if (!closed.empty() && closed.find(neighbor) != closed.end())	continue;	// neighbor in closed set; continue
			int newPathCost = current->getPathCost() + manhattanDistance(current->getLocation(), neighbor->getLocation());
			if (open.empty() || open.find(neighbor) == open.end()) {	// neighbor not in open set
				neighbor->setPathCost(newPathCost);
				neighbor->setCameFrom(current);
				open.insert(neighbor);
			}
		}
	}

	return reconstructPath(current);
}
vector<int> Grid::getGoalLocation() {
	return goal;
}
GridLocation* Grid::lowestScore(unordered_set<GridLocation*> theSet) {
	int lowest = 32767;
	GridLocation *loc, *current;
	for (unordered_set<GridLocation*>::iterator it=theSet.begin(); it!=theSet.end(); it++) {
		current = (GridLocation *) *it;
		int a = current->getScore();
		if (current->getScore() < lowest) {
			lowest = current->getScore();
			loc = current;
		}
	}

	return loc;
}


/*
// GRIDLOCATION FUNCTION DECLARATIONS //
*/
GridLocation::GridLocation(Grid* g,int r, int c, int v) {
	grid = g;
	row=r;	col=c;	value=v;

	cameFrom = (GridLocation *) 0;
}
GridLocation::GridLocation(){ cameFrom = (GridLocation *) 0; }
void GridLocation::setValue(int v) { value=v; }
int GridLocation::getValue() { return value; }

vector<GridLocation*> GridLocation::getValidMoves(int orientation) {
	vector<GridLocation*> out;
	vector<vector<GridLocation>>* theMap = grid->getMap();

	switch(orientation) {
	case 0:	// forward

	if (row>0) {
		if ((&(&theMap->at(row-1))->at(col))->getValue()==0) {
			GridLocation* tmp = (&(&theMap->at(row-1))->at(col));
			out.push_back(tmp);
		}
		if (col>0) {
			if ((&(&theMap->at(row-1))->at(col-1))->getValue()==0) {
				GridLocation* tmp = (&(&theMap->at(row-1))->at(col-1));
				out.push_back(tmp);
			}
		}
	}
	if (col<grid->numCols-1) {
		if (row>0) {
			if ((&(&theMap->at(row-1))->at(col+1))->getValue()==0) {
				GridLocation* tmp = (&(&theMap->at(row-1))->at(col+1));
				out.push_back(tmp);
			}
		}
	}

	// TODO: align with Pleo movement primitives
	// TODO: other orientations

	}

	return out;
}

vector<GridLocation*> GridLocation::getNeighbors() {		// TODO: change to getValidMoves(int/float pleoOrientation) --> return only valid moves he can make based on orientation
	vector<GridLocation*> out;
	vector<vector<GridLocation>>* theMap = grid->getMap();
	if (row>0) {
		if ((&(&theMap->at(row-1))->at(col))->getValue()==0) {
			GridLocation* tmp = (&(&theMap->at(row-1))->at(col));
			out.push_back(tmp);
		}
		if (col>0) {
			if ((&(&theMap->at(row-1))->at(col-1))->getValue()==0) {
				GridLocation* tmp = (&(&theMap->at(row-1))->at(col-1));
				out.push_back(tmp);
			}
		}
	}
	if (row<grid->numRows-1) {
		if ((&(&theMap->at(row+1))->at(col))->getValue()==0) {
			GridLocation* tmp = (&(&theMap->at(row+1))->at(col));
			out.push_back(tmp);
		}
		if (col<grid->numCols-1) {
			if ((&(&theMap->at(row+1))->at(col+1))->getValue()==0) {
				GridLocation* tmp = (&(&theMap->at(row+1))->at(col+1));
				out.push_back(tmp);
			}
		}
	}
	if (col>0) {
		if ((&(&theMap->at(row))->at(col-1))->getValue()==0) {
			GridLocation* tmp = (&(&theMap->at(row))->at(col-1));
			out.push_back(tmp);
		}
		if (row<grid->numRows-1) {
			if ((&(&theMap->at(row+1))->at(col-1))->getValue()==0) {
				GridLocation* tmp = (&(&theMap->at(row+1))->at(col-1));
				out.push_back(tmp);
			}
		}
	}
	if (col<grid->numCols-1) {
		if ((&(&theMap->at(row))->at(col+1))->getValue()==0) {
			GridLocation* tmp = (&(&theMap->at(row))->at(col+1));
			out.push_back(tmp);
		}
		if (row>0) {
			if ((&(&theMap->at(row-1))->at(col+1))->getValue()==0) {
				GridLocation* tmp = (&(&theMap->at(row-1))->at(col+1));
				out.push_back(tmp);
			}
		}
	}
	return out;
}
bool GridLocation::isEdge() {
	if (value == 1)	return false;	// already an edge
	if (row==0 || col==0 || row==(grid->numRows-1) || col==(grid->numCols-1)) {		// boundary edge
		return true;
	}

	vector<vector<GridLocation>>* theMap = grid->getMap();

	if ((&(&theMap->at(row-1))->at(col))->getValue()==1 || (&(&theMap->at(row-1))->at(col-1))->getValue()==1 || 
		(&(&theMap->at(row))->at(col-1))->getValue()==1 || (&(&theMap->at(row+1))->at(col-1))->getValue()==1 || 
		(&(&theMap->at(row+1))->at(col))->getValue()==1 || (&(&theMap->at(row+1))->at(col+1))->getValue()==1 || 
		(&(&theMap->at(row))->at(col+1))->getValue()==1 || (&(&theMap->at(row-1))->at(col+1))->getValue()==1) {
		return true;
	}
	return false;
}
int GridLocation::heuristic() {
	return (manhattanDistance(getLocation(),grid->getGoalLocation()));
}
int GridLocation::getScore() {
	return (pathCost + heuristic());
}
void GridLocation::setPathCost(int g) {	pathCost = g;	}
int GridLocation::getPathCost() {	return pathCost;	}
vector<int> GridLocation::getLocation() {
	vector<int> out;
	out.push_back(row);
	out.push_back(col);
	return out;
}
GridLocation* GridLocation::getCameFrom() { return cameFrom; }
void GridLocation::setCameFrom(GridLocation* from) {
				//cout << "came from (" << from->getLocation()[0] << ", " << from->getLocation()[1] << ")" << endl;
	cameFrom = from; }
Grid* GridLocation::getGrid() { return grid; }
void GridLocation::setGrid(Grid* newGrid) {	grid = newGrid; }


Search::Search(){
	conversionFactor = 5.0;
}

/*
// OTHER fuction declarations
*/
vector<GridLocation> reconstructPath(GridLocation *current) {	// TODO: modify to return Pleo movements
	vector<GridLocation> out;
	out.push_back(*current);
	GridLocation* from = current->getCameFrom();
	if (from == NULL)	return out;
	vector<GridLocation> more = reconstructPath(from);
	out.insert(out.begin(), more.begin(), more.end());
	return out;
}
int manhattanDistance(vector<int> a, vector<int> b) {
	return (abs(b[0]-a[0]) + abs(b[1]-a[1]));
}
int pixelRound(float in) {
	if (in >= 0)
		return static_cast<int>(floor(in + 0.5));
	else
		return static_cast<int>(floor(in - 0.5));
}
vector<vector<int>> gridPathtoPixels(vector<GridLocation> gridPath) {
	vector<vector<int>> out;
	for (int i=0; i<gridPath.size(); i++) {
		vector<int> next;
		vector<int> current = gridPath[i].getLocation();
		next.push_back(current[1]*pixelsPerGrid+(pixelsPerGrid/2));
		next.push_back(480-(current[0]*pixelsPerGrid)-(pixelsPerGrid/2));
		out.push_back(next);
	}
	return out;
}


/*
 *	Compute and return pixel path from grid
 */
vector<vector<int>> Search::findPath(vector<vector<int>> pixelArray, vector<int> start, vector<int> end){

	Grid g = Grid(pixelArray, &g);
	g.enlargeObstacles(1);

	cout << "Searching from (" << start[0] << ", " << start[1] << ")"
		 << "to (" << end[0] << ", " << end[1] << ")" << endl;

	vector<GridLocation> optimalPath = g.search(start, end);

	
		cout << "grid path: ";
	for (int i=0; i<optimalPath.size(); i++) {
		cout << "(" << optimalPath.at(i).getLocation().at(0) << "," << optimalPath.at(i).getLocation().at(1) << ")  ";
		if (g.getMap()->at(optimalPath[i].getLocation()[0]).at(optimalPath[i].getLocation()[1]).getValue() == 1) {
			cout << "ERROR: path location (" << optimalPath[i].getLocation()[0] << ", " << optimalPath[i].getLocation()[1] << ") is inside obstacle" << endl;
		}
	}
		cout << endl;

	vector<vector<int>> pixelLocPath = gridPathtoPixels(optimalPath);
	
	// pixel path
	cout << endl;
	for (unsigned int i=0; i<pixelLocPath.size(); i++) {
		cout << "(" << pixelLocPath[i][0] << ", " << pixelLocPath[i][1] << ") ";
	}
	cout << endl;

	
	cout << endl << endl << "MAP:" << endl;
	vector<vector<GridLocation>>* theMap = g.getMap();
	for (int i=0; i<theMap->size(); i++) {
		for (int j=0; j<theMap->at(0).size(); j++) {
			cout << theMap->at(i).at(j).getValue();
		}
		cout << endl;
	}


	return pixelLocPath;
	//return obstacleLocs;
}




// TODO: check each grid space for if image contains obstacle anywhere in the space