#ifndef ASTAR_H
#define ASTAR_H

#include <stdbool.h>
// a* algorithm - specifically made for raylib case, but should work on every Caller
// was very much inspired and directed by geeksforgeeks article on it, took the mantle and 
// tried to translate it to c
// link to article: https://www.geeksforgeeks.org/dsa/a-search-algorithm/


// ---- structs -------
typedef struct {
    int *coord_x, *coord_y;  // arrays of x and y coordinates along the path
    int Length;               // number of waypoints in the path
} Path;


//func declarations

/*
 * aStarPathAlgo - Find the shortest path from start to end using A* algorithm
 *
 * Parameters:
 *   costMap  - 2D array of movement costs (float **), where 0.0 = blocked, higher = more expensive
 *   Width    - number of columns in the grid
 *   Height   - number of rows in the grid
 *   start_x  - starting x (column) coordinate
 *   start_y  - starting y (row) coordinate
 *   end_x    - ending x (column) coordinate
 *   end_y    - ending y (row) coordinate
 *
 * Returns:
 *   Path struct containing:
 *     - coord_x, coord_y: dynamically allocated arrays of waypoints
 *     - length: number of waypoints
 *   If no path is found, returns Path with length = 0 and NULL pointers.
 *   Caller is responsible for freeing coord_x and coord_y when done.
 */
Path aStarPathAlgo(float **costMap, int Width, int Height, int start_x, int start_y, int end_x, int end_y);

#endif // ASTAR_H