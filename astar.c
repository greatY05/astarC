#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <float.h>
#include <stdio.h>
#include <string.h>
#include "astar.h"

// shorthand pair struct for shortcuts
typedef struct {
    int x;
    int y;
} Pair;

//shortcut of 2 pairs
typedef struct {
    Pair p1;
    Pair p2;
} pPair;

//cells in the aStar iteration
typedef struct {
   Pair parent;
   float g, h, f;
} Cell;

//value of cells in the aStar's openlist
typedef struct {
    float f;
    int x, y;
} openListCell;


//declarations
static bool isValid(Pair coord, Pair limit );
static bool isUnblocked(float **costmap, Pair coord);
static bool isDestination(Pair coord, Pair end_point);
static float calcHValue(Pair coord, Pair end_point);                                        //calculates the heurestics of a cell - most direct line from cell to end
static void tracePath(Cell **cellDetails, Pair end_point, int max_path_size, Path *path);  // creates path to return 

//              functions

//check if cell within user given limits ()
static bool isValid(Pair coord, Pair limit ) {
    return (coord.y >= 0)&& (coord.y < limit.y) && (coord.x >= 0) && (coord.x<limit.x);
}


//check if cell is passable and returns as such
static bool isUnblocked(float **costmap, Pair coord) {
    if (costmap[coord.x][coord.y] > 0.1f) return true; else return false; // TODO fit to our own cost
}

//check if we reached destination cell
static bool isDestination(Pair coord, Pair end_point) {
    if (coord.x == end_point.x && coord.y == end_point.y) return true; else return false;
}

// heuristic calculation - aka h, distance from cell to end node 
static float calcHValue(Pair coord, Pair end_point){
    //distance formula
    float dx = (float)(coord.x - end_point.x);
    float dy = (float)(coord.y - end_point.y);
    return sqrt((dx*dx) + (dy * dy));
}

//tracing path after finishing the calculation, from start to end
static void tracePath(Cell ** cellDetails, Pair end_point, int max_path_size, Path *path) {
    int row = end_point.x;
    int col = end_point.y;
    //allocate memory for the pathstack
    Pair *pathStack = (Pair *)malloc(max_path_size * sizeof(Pair));
    if (pathStack == NULL) {
        printf("error - memory allocation failed\n");
        return;
    }
    int top = 0;

    while (!(cellDetails[row][col].parent.x == row && cellDetails[row][col].parent.y == col)) {
        Pair coord = {row, col};
        pathStack[top++] = coord;
        int temp_row = cellDetails[row][col].parent.x;
        int temp_col = cellDetails[row][col].parent.y;
        row = temp_row;
        col = temp_col;
    }

    Pair coord = {row, col};
    pathStack[top++] = coord;
    
    // top - number of waypoints from end to start
    path->Length = top;
    path->coord_x = (int *)malloc(top * sizeof(int));
    path->coord_y = (int *)malloc(top * sizeof(int));
    
    //flip order of path - built end to start, this flip turns it start->end
    for (int i =0; i < top; i++) {
        Pair p = pathStack[top -1 - i];
        path->coord_x[i] = p.x;
        path->coord_y[i] = p.y;
    }
    
    free(pathStack);
}

//main function - a star algorithm to find teh shortest path between 2 given cells

Path aStarPathAlgo(float **costMap, int Width, int Height, int start_x, int start_y, int end_x, int end_y){
    Path result = { NULL, NULL, 0 }; // init path in case of early return
    
    // --------------------- validations --------------------
    
    //validate given cells - invalid/out of range
    if (isValid((Pair){start_x, start_y}, (Pair){Width, Height}) == false || isValid((Pair){end_x, end_y}, (Pair){Width, Height}) == false) {
        printf("start/end is invalid/out of range \n");
        return result;
    }
    //validate given cells - blocked
    if (isUnblocked(costMap, (Pair){start_x, start_y}) == false || isUnblocked(costMap, (Pair){end_x, end_y}) == false) {
        printf("start/end is blocked \n");
        return result;
    }
    
    //validate given cells - same cell/reached the end
    if (isDestination((Pair){start_x, start_y}, (Pair){end_x, end_y}) == true) {
        result.Length = 1;
        result.coord_x = (int *)malloc(sizeof(int));
        result.coord_y = (int *)malloc(sizeof(int));
        result.coord_x[0] = start_x;
        result.coord_y[0] = start_y;
        printf("already at destination \n");
        return result;
    }
    
    // -------------- list creation -------------------------
    
    //create closed list, initialize as false, which means no cell is incluided but implement as boolean 2D array
    bool **closedList = (bool **)malloc(Height * sizeof(bool *));
    for (int i = 0; i < Height; i++) {
        closedList[i] = (bool *)malloc(Width * sizeof(bool));
        memset(closedList[i], false, Width * sizeof(bool));  // initialize to false
    }
    
    //allocate array of pointers (one per row, 2d array)
    Cell **cellDetails = (Cell **)malloc(Height * sizeof(Cell *));
    
    //allocate the rows
    for (int i = 0; i < Height; i++){
        cellDetails[i] = (Cell *)malloc(Width * sizeof(Cell));
    }
    
    int x, y;
    
    for (x = 0; x < Width; x++){
        for (y = 0; y< Height; y++){
            cellDetails[x][y].h = FLT_MAX;             
            cellDetails[x][y].f = FLT_MAX;
            cellDetails[x][y].g = FLT_MAX;
            cellDetails[x][y].parent.x = x;
            cellDetails[x][y].parent.y = y;
        }
    }
    
    //init parameters of starting node
    x = start_x, y = start_y;
    cellDetails[x][y].f = 0;
    cellDetails[x][y].g = 0;
    cellDetails[x][y].h = 0;          
    cellDetails[x][y].parent.x = x;
    cellDetails[x][y].parent.y = y;
    
    
    //create open list, information such as f's xy (row and column) information, list is set of Pair of Pairs (created at top of code)
    int MAX_OPEN_LIST_SIZE = Width * Height;  // worst case: every cell
    openListCell *openList = (openListCell *)malloc(MAX_OPEN_LIST_SIZE * sizeof(openListCell));
    int openListCount = 0;  // track how many items are actually in it
    
    //put starting point in the open list
    openList[openListCount].f = 0;
    openList[openListCount].x = x;
    openList[openListCount].y = y;
    openListCount++;
    
    
    bool foundDest = false; // checker if goal reached
    
    //main loop - iterate through the list of open cells
    while(openListCount>0) 
    {
        // --------- iteration though openList to find lowest F cell----------------
        int minIndex = 0;
        float TempMinF = FLT_MAX;
        //sort the list by lowest F
        for (int i = 0; i < openListCount; i++){ 
            if (openList[i].f < TempMinF) {
                minIndex = i;
                TempMinF = openList[i].f;
            }
        }
        
        openListCell p = openList[minIndex]; //assign first cell of openlist
        
        //second loop to shift all cells into the min value, "popping" it out of hte list
        for (minIndex; minIndex < openListCount-1; minIndex++){
            openList[minIndex] = openList[minIndex+1];
        }
        openListCount--;
        
        
        
        // --------add to closed list------------
        x = p.x;
        y = p.y;
        closedList[x][y] = true;
        
        // generate all 8 neigbours
        float gNew, hNew, fNew;
        
        //loop through all directions and break on 0 0 which is the current one
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++){
                if (dx == 0 && dy == 0) continue;
                
                if (isValid((Pair){x+dx, y+dy}, (Pair){Width, Height}) == true) {
                    if (isDestination((Pair){x+dx, y+dy}, (Pair){end_x, end_y}) == true){
                        cellDetails[x+dx][y+dy].parent.x = x;
                        cellDetails[x+dx][y+dy].parent.y = y;
                        printf("destination cell found!\n");
                        tracePath(cellDetails, (Pair){end_x, end_y}, MAX_OPEN_LIST_SIZE, &result); // w*h longest theoretical path
                        foundDest = true;
                        
                        // memory freeing since return upcoming
                        for (int i = 0; i < Height; i++){
                            free(cellDetails[i]);
                            free(closedList[i]);
                        }
                        free(cellDetails);
                        free(closedList);
                        free(openList);
                        
                        return result;
                    }
                    //if neighbour is already in closed list or blocked, ignore it, else, calculate g,h,f
                    else if(closedList[x+dx][y+dy] == false && isUnblocked(costMap, (Pair){x+dx, y+dy}) == true) {
                        float moveCost = 1;
                        if (dx != 0 && dy != 0) moveCost = 1.414;
                        gNew = cellDetails[x][y].g + moveCost * costMap[x+dx][y+dy];
                        hNew = calcHValue((Pair){x+dx, y+dy}, (Pair){end_x, end_y});
                        fNew = gNew + hNew;
                        
                        //if neighbour isnt in open list, add it there, make current cell its parent, and record g h failed
                        //else if its in the open list already, check to see if the path cost is better using f
                        if (cellDetails[x+dx][y+dy].f == FLT_MAX ||cellDetails[x+dx][y+dy].f > fNew) {
                            openList[openListCount] = (openListCell){fNew ,x+dx,y+dy};
                            openListCount++;
                            //update details of Cell
                            cellDetails[x+dx][y+dy].f = fNew;
                            cellDetails[x+dx][y+dy].g = gNew;
                            cellDetails[x+dx][y+dy].h = hNew;
                            cellDetails[x+dx][y+dy].parent.x = x;
                            cellDetails[x+dx][y+dy].parent.y = y;
                        }
                    }
                }
            }
        }
        
    }
    
    //free dynamic lists since we allocated the memory for them
    for (int i = 0; i < Height; i++){
        free(cellDetails[i]);
        free(closedList[i]);
        
        
    }
    free(cellDetails);
    free(closedList);
    free(openList);
    //when destination cell is not found, and the open list is empty, we conclude failed to reach mostly do to it being blocked
    if (foundDest == false) printf("failed to reach/find destination\n");
    return result;
    
}


