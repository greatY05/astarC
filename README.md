# astarC
a* algorithm for C, made for raylib but works elsewhere too!
inspired by geeksforgeeks article on a*, translated into c!


how to use:

the function aStarPathAlgo takes - allocated nested array for the map (held like a 2D array), Width, Height, start_x, start_y, end_x, end_y
it uses its type as a inner struct called path, taking int pointer to coord x and y, and a int of length
typedef struct {
    int *coord_x, *coord_y;  // arrays of x and y coordinates along the path
    int Length;               // number of waypoints in the path
} Path;

the full signature:
Path aStarPathAlgo(float **costMap, int Width, int Height, int start_x, int start_y, int end_x, int end_y);


for the nested array - you can initalize it like this:

float **costMap = (float **)malloc(Width * sizeof(float *));
for (int i = 0; i < Width; i++) 
{
  costMap[i] = (float *)malloc(Height * sizeof(float));
}

this allocates memory for the list for the algorithm to use

Width and Height- boundaries of the space your algorithm will take place in
start_x/y - starting position 
end_x/y - end position 

reading from the algorithm:
you can read the result directly using "result.coord_x[i]" or "result.coord_y[i]"
or just iterate through the path easily with for loop
for (int i = 0; i < result.Length; i++) {
        int row = result.coord_x[i];  
        int col = result.coord_y[i];  
}

IMPORTANT:
the coords are pointers, remembe to free them once youre done
free(result.coord_x);
free(result.coord_y);
