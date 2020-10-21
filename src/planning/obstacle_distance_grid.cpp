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
 * Check if a cell is unscored (e.g., something in the grid and in free-space, and not visited)
 */
bool ObstacleDistanceGrid::isCellUnscored(const OccupancyGrid &map, int x, int y){
    return isCellInGrid(x,y) && map.logOdds(x,y) < 0 && distance(x,y) == 0;
}

/*
 * Return the min score from all the neighbors
 */
float ObstacleDistanceGrid::getMinScore(const OccupancyGrid &map, int x, int y){
    if(map.logOdds(x,y) >= 0){
        return 0;
    }
    float min_score = (distance(x,y) > 0) ? distance(x,y)-metersPerCell() : 1000;
    location north, south, east, west, northeast, southeast, southwest, northwest;
    north.x = x; north.y = y-1;
    south.x = x; south.y = y+1;
    east.x = x+1; east.y = y;
    west.x = x-1; west.y = y;
    northeast.x = east.x; northeast.y = north.y;
    southeast.x = east.x; southeast.y = south.y;
    southwest.x = west.x; southwest.y = south.y;
    northwest.x = west.x; northwest.y = north.y; // # Kanye 2020 
    // Check NSEW
    if(isCellScored(map, north.x,north.y) && distance(north.x,north.y) < min_score){
        min_score = distance(north.x,north.y);
    }
    if(isCellScored(map,south.x,south.y) && distance(south.x,south.y) < min_score){
        min_score = distance(south.x,south.y);
    }
    if(isCellScored(map,east.x,east.y) && distance(east.x,east.y) < min_score){;
        min_score = distance(east.x,east.y);
    }
    if(isCellScored(map,west.x,west.y) && distance(west.x,west.y) < min_score){
        min_score = distance(west.x,west.y);
    }
    // Check diagonals
    if(isCellScored(map,northeast.x,northeast.y) && distance(northeast.x,northeast.y) < min_score){
        min_score = distance(northeast.x,northeast.y);
    }
    if(isCellScored(map,southeast.x,southeast.y) && distance(southeast.x,southeast.y) < min_score){
        min_score = distance(southeast.x,southeast.y);
    }
    if(isCellScored(map,southwest.x,southwest.y) && distance(southwest.x,southwest.y) < min_score){;
        min_score = distance(southwest.x,southwest.y);
    }
    if(isCellScored(map,northwest.x,northwest.y) && distance(northwest.x,northwest.y) < min_score){
        min_score = distance(northwest.x,northwest.y);
    }
    min_score = (min_score == 1000) ? metersPerCell() : (min_score+metersPerCell()); // return at least metersPerCell() for a cell that is in free space
    return min_score;
}

void ObstacleDistanceGrid::addNeighbors(const OccupancyGrid &map, int x, int y, std::queue<location>* visit){
    float prescore = (map.logOdds(x,y) >= 0) ? metersPerCell() : 0;
    location north, south, east, west, northeast, southeast, southwest, northwest;
    north.x = x; north.y = y-1;
    south.x = x; south.y = y+1;
    east.x = x+1; east.y = y;
    west.x = x-1; west.y = y;
    northeast.x = east.x; northeast.y = north.y;
    southeast.x = east.x; southeast.y = south.y;
    southwest.x = west.x; southwest.y = south.y;
    northwest.x = west.x; northwest.y = north.y; // # Kanye 2020 
    // For a valid cell, that is in free space, and has not been visited yet:
    //  calculated distance score
    //  mark as visited
    //  visit its neighboring cells
    if(isCellUnscored(map, north.x,north.y)){
        distance(north.x,north.y) = prescore;
        visit->push(north);
    }
    if(isCellUnscored(map, south.x,south.y)){
        distance(south.x,south.y) = prescore;
        visit->push(south);
    }
    if(isCellUnscored(map, east.x,east.y)){
        distance(east.x,east.y) = prescore;
        visit->push(east);
    }
    if(isCellUnscored(map, west.x,west.y)){
        distance(west.x,west.y) = prescore;
        visit->push(west);
    }

    // Check diagonals
    if(isCellUnscored(map, northeast.x,northeast.y)){
        distance(northeast.x,northeast.y) = prescore;
        visit->push(northeast);
    }
    if(isCellUnscored(map, southeast.x,southeast.y)){
        distance(southeast.x,southeast.y) = prescore;
        visit->push(southeast);
    }
    if(isCellUnscored(map, southwest.x,southwest.y)){;
        distance(southwest.x,southwest.y) = prescore;
        visit->push(southwest);
    }
    if(isCellUnscored(map, northwest.x,northwest.y)){
        distance(northwest.x,northwest.y) = prescore;
        visit->push(northwest);
    }
    
}

void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
    
    std::queue<location> visit;    // locations to visit

    int w = map.widthInCells();
    int h = map.heightInCells();
    
    // Loop through all grid points, and mark if they are occupied or not
    // If not occupied, store them to visit later
    for(int x = 0; x < w; x++){
        for(int y = 0; y < h; y++){
            if(visited(x,y) == true){
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
    // std::cout << (visit.size()) << std::endl;
    while(!visit.empty()){
        location cur_loc = visit.front();
        int x = cur_loc.x;
        int y = cur_loc.y;

        // std::cout << "debug post location " << std::endl;
        if(!visited(x,y)){
            // std::cout << "debug inside if statement" << std::endl;
            visited(x,y) = true;
            distance(x,y) = (distance(x,y) > 0) ? distance(x,y) : getMinScore(map,x,y);
            visit.pop();
            addNeighbors(map, x, y, &visit);
        }
        else{
            visit.pop();
        }
    }
    int num_unvisited = 0;
    for(int x = 0; x < w; x++){
        for(int y = 0; y < h; y++){
            if(visited(x,y) == false){
                num_unvisited++;
            }
        }
    }
    std::cout << num_unvisited << " unvisited cells found" << std:: endl;

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
    
    // // If the grid is already the correct size, nothing needs to be done
    // if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    // {
    //     return;
    // }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
    visited_.resize(width_ * height_);
    for(int x = 0; x < width_; x++){
        for(int y = 0; y < height_; y++){
            distance(x,y) = 0.0f;
            visited(x,y) = false;
        }
    }
}
