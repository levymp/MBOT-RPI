#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>

ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}

/*
 * Check if a cell is scored (e.g., something in the grid and in free-space, with a non-zero value)
 */
bool ObstacleDistanceGrid::isCellScored(const OccupancyGrid &map, int x, int y){
    return isCellInGrid(x,y) && map.logOdds(x,y) < 0 && distance(x,y) > 0;
}

/*
 * Check if a cell is unscored (e.g., something in the grid and in free-space, and not scored)
 */
bool ObstacleDistanceGrid::isCellUnscored(const OccupancyGrid &map, int x, int y){
    return isCellInGrid(x,y) && map.logOdds(x,y) < 0 && distance(x,y) == 0;
}

/*
 * Return the min score from all the neighbors
 * 
 * this is the distance from nearest obstacle
 */
float ObstacleDistanceGrid::getMinScore(const OccupancyGrid &map, int x, int y){
    
    // If the cell is an obstacle, it's score is 0
    if(map.logOdds(x,y) >= 0){
        return 0;
    }

    // If the cell hasn't been scored yet, mark it as something really high
    //  Otherwise, adjust its distance score to properly compare to scores of neighbor cells
    float min_score = (distance(x,y) > 0) ? distance(x,y)-metersPerCell() : 1000;

     /* Go through neighbors and skip current cell
     *  For a valid cell, that is in free space, and has been visited:
     *  update distance score IF it is closer to an obstacle than previous min
     */
    for(int row = y-1; row <= y+1; row++){
        for(int col = x-1; col <= x+1; col++){
            if(!(row == y && col == x) && isCellScored(map, col, row) && distance(col,row) < min_score){
                min_score = distance(col,row);
            }
        }
    }

    // If there are no scored neighbors, default to scoring this as close to an obstacles
    // o/w mark the distance as our best estimate + metersPerCell() offset
    min_score = (min_score == 1000) ? metersPerCell() : (min_score+metersPerCell()); // return at least metersPerCell() for a cell that is in free space
    return min_score;
}

/*
 *  Add all box neighbors of a given cell to the visit queue
 *  
 *  This checks to see that neighbors are in free-space, and haven't been scored already.
 *  Also, for the initial neighbors of obstacles, this will initialize their distance to metersPerCell()
 */
void ObstacleDistanceGrid::addNeighbors(const OccupancyGrid &map, int x, int y, std::queue<location>* visit){
    float prescore = (map.logOdds(x,y) >= 0) ? metersPerCell() : 0; // initialize obstacle neighbors to metersPerCell()

    /*  Go through neighbors and skip current cell
     *  For a valid cell, that is in free space, and has not been visited yet:
     *  update distance score IF it is right next to a obstacle
     *  add it to our visit queue
     */
    for(int row = y-1; row <= y+1; row++){
        for(int col = x-1; col <= x+1; col++){
            if(!(row == y && col == x) && isCellUnscored(map, col, row)){
                distance(col,row) = prescore;
                location tmp(col,row);
                visit->push(tmp);
            }
        }
    }

}

void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    // Reset visited and distance vectors
    for(int x = 0; x < width_; x++){
        for(int y = 0; y < height_; y++){
            distance(x,y) = 0.0f;
            visited(x,y) = false;
        }
    }
    
    std::queue<location> visit;    // locations to visit

    int w = map.widthInCells();
    int h = map.heightInCells();
    
    // Loop through all grid points, and mark if they are occupied
    // If something is occupied, add its freespace neighbors to a visit queue so we can come
    // back to them
    for(int x = 0; x < w; x++){
        for(int y = 0; y < h; y++){
            if(visited(x,y)){
                continue;
            }
            location cur_loc;
            cur_loc.x = x;
            cur_loc.y = y;
            if(map.logOdds(x,y) >= 0){
                distance(x,y) = 0.0f;
                visited(x,y) = true;
                addNeighbors(map,x,y,&visit);
            }
        }
    }

    // Now start to loop through all the free-space cells
    // This will start with neighbors of obstacles, and branch out from there
    while(!visit.empty()){

        // Get curent cell location data
        location cur_loc = visit.front();
        int x = cur_loc.x;
        int y = cur_loc.y;

        // Make sure that we haven't visited this cell before
        // if we have, skip this & remove it from the visit queue
        if(!visited(x,y)){
            visited(x,y) = true;

            // If we haven't scored this node already, calculate its score
            //      This mainly comes in for the initial set of obstacle neighbors
            distance(x,y) = (distance(x,y) > 0) ? distance(x,y) : getMinScore(map,x,y);
            visit.pop(); // we've visited this cell, so can remove
            addNeighbors(map, x, y, &visit); // add this cell's neighbors to our visit queue
        }
        else{
            visit.pop();
        }
    }

    // // Useful to debug; double checking we visited all the cells
    // int num_unvisited = 0;
    // for(int x = 0; x < w; x++){
    //     for(int y = 0; y < h; y++){
    //         if(visited(x,y) == false){
    //             num_unvisited++;
    //         }
    //     }
    // }
    // std::cout << num_unvisited << " unvisited cells found" << std:: endl;

}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
    visited_.resize(width_ * height_);
    
}
