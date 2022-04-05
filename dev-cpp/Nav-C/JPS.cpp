#include <iostream>
#include <queue>
#include <unordered_set>
#include <vector>
#include <math.h>
#include "JPS.hpp"
#include <unistd.h>
#include <cstdlib>

class Compare
{
public:
    bool operator()(Noeud &n1, Noeud &n2)
    {
        if (n1.cost + n1.h > n2.cost + n2.h || (n1.cost + n1.h == n2.cost + n2.h && n1.h >= n2.h))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
};

void Noeud::print_Noeud()
{
    cout << "x : " << this->x << " y : " << this->y << " cost : " << this->cost << " h : " << this->h << " id : " << this->id << " pid : " << this->pid << endl;
    //cout << "y : " << this->y << endl;
    // cout << "cost : " << this->cost << endl;
    // cout << "id : " << this->id << endl;
    // cout << "pid : " << this->pid << endl;
    // cout << "h : " << this->h << endl;
}
void Noeud::compute_heuristic(Noeud &goal)
{
    this->h = (abs(this->x - goal.x) + abs(this->y - goal.y)); // Manhatann heuristic
}

std::vector<Noeud> actions(Noeud &start)
{
    return {
        Noeud(start.x, start.y-1, start.cost + 1, 0, start.id),
        Noeud(start.x+1, start.y-1, start.cost + sqrt(2), 0, start.id),
        Noeud(start.x+1, start.y, start.cost + 1, 0, start.id),
        Noeud(start.x+1, start.y+1, start.cost + sqrt(2), 0, start.id),
        Noeud(start.x, start.y+1, start.cost + 1, 0, start.id),
        Noeud(start.x-1, start.y+1, start.cost + sqrt(2), 0, start.id),
        Noeud(start.x-1, start.y, start.cost + 1, 0, start.id),
        Noeud(start.x-1, start.y-1, start.cost + sqrt(2), 0, start.id)};
}

int direction(Noeud &parent, Noeud &child)
{
    int a = parent.x - child.x; // -1: droite 1: gauche
    int b = parent.y - child.y; // -1: bas 1:haut
    if (a == 0 && b == 1)
    {
        return 0;
    } // up move
    if (a == -1 && b == 1)
    {
        return 1;
    } // up/right move
    if (a == -1 && b == 0)
    {
        return 2;
    } // right move
    if (a == -1 && b == -1)
    {
        return 3;
    } // down/right move
    if (a == 0 && b == -1)
    {
        return 4;
    } // down move
    if (a == 1 && b == -1)
    {
        return 5;
    } // down/left move
    if (a == 1 && b == 0)
    {
        return 6;
    } // left move
    if (a == 1 && b == 1)
    {
        return 7;
    } // left/up move
    else
    {
        return 8;
    } // problem
}

Noeud step(Noeud &parent, int direction)
{
    if (direction == 0)
    {
        return Noeud(parent.x, parent.y - 1, parent.cost + 1, 0, parent.id);
    }
    if (direction == 1)
    {
        return Noeud(parent.x + 1, parent.y - 1, parent.cost + sqrt(2), 0, parent.id);
    }
    if (direction == 2)
    {
        return Noeud(parent.x + 1, parent.y, parent.cost + 1, 0, parent.id);
    }
    if (direction == 3)
    {
        return Noeud(parent.x + 1, parent.y + 1, parent.cost + sqrt(2), 0, parent.id);
    }
    if (direction == 4)
    {
        return Noeud(parent.x, parent.y + 1, parent.cost + 1, 0, parent.id);
    }
    if (direction == 5)
    {
        return Noeud(parent.x - 1, parent.y + 1, parent.cost + sqrt(2), 0, parent.id);
    }
    if (direction == 6)
    {
        return Noeud(parent.x - 1, parent.y, parent.cost + 1, 0, parent.id);
    }
    if (direction == 7)
    {
        return Noeud(parent.x - 1, parent.y - 1, parent.cost + sqrt(2), 0, parent.id);
    }
}

int hasForcedNeighbours(Noeud &current, int direction, std::vector<std::vector<int>> grid){
    int first_x;
    int second_one_x;
    int second_two_x;
    int third_one_x;
    int third_two_x;
    int first_y;
    int second_one_y;
    int second_two_y;
    int third_one_y;
    int third_two_y;
    int four_one_x;
    int four_one_y;
    int four_two_x;
    int four_two_y;
    if(direction == 1){
        return (hasForcedNeighbours(current,0,grid) ||  hasForcedNeighbours(current,2,grid));
    }
    if(direction == 3){
        return (hasForcedNeighbours(current,2,grid) ||  hasForcedNeighbours(current,4,grid));
    }
    if(direction == 5){
        return (hasForcedNeighbours(current,4,grid) ||  hasForcedNeighbours(current,6,grid));
    }
    if(direction == 7){
        return (hasForcedNeighbours(current,6,grid) ||  hasForcedNeighbours(current,0,grid));
    }
    if(direction == 0){ // up
        first_x = current.x;
        first_y = current.y + 1;
        second_one_x = current.x-1;
        second_one_y = current.y+1;
        second_two_x = current.x+1;
        second_two_y = current.y+1;
        third_one_x = current.x-1;
        third_one_y = current.y;
        third_two_x = current.x+1;
        third_two_y = current.y;
        four_one_x = current.x-1;
        four_one_y = current.y-1;
        four_two_x = current.x+1;
        four_two_y = current.y-1;
    }
    if(direction == 2){ // right
        first_x = current.x-1;
        first_y = current.y;
        second_one_x = current.x-1;
        second_one_y = current.y-1;
        second_two_x = current.x-1;
        second_two_y = current.y+1;
        third_one_x = current.x;
        third_one_y = current.y-1;
        third_two_x = current.x;
        third_two_y = current.y+1;
        four_one_x = current.x+1;
        four_one_y = current.y-1;
        four_two_x = current.x+1;
        four_two_y = current.y+1;
    }
    if(direction == 4){ // down
        first_x = current.x;
        first_y = current.y-1;
        second_one_x = current.x+1;
        second_one_y = current.y-1;
        second_two_x = current.x-1;
        second_two_y = current.y-1;
        third_one_x = current.x+1;
        third_one_y = current.y;
        third_two_x = current.x-1;
        third_two_y = current.y;
        four_one_x = current.x+1;
        four_one_y = current.y+1;
        four_two_x = current.x-1;
        four_two_y = current.y+1;
    }
    if(direction == 6){ // left
        first_x = current.x+1;
        first_y = current.y;
        second_one_x = current.x+1;
        second_one_y = current.y+1;
        second_two_x = current.x+1;
        second_two_y = current.y-1;
        third_one_x = current.x;
        third_one_y = current.y+1;
        third_two_x = current.x;
        third_two_y = current.y-1;
        four_one_x = current.x-1;
        four_one_y = current.y+1;
        four_two_x = current.x-1;
        four_two_y = current.y-1;
    }
    if(third_one_x >= 0 && third_one_x < grid[0].size() && third_one_y >= 0 && third_one_y < grid[0].size()){
        if(grid[third_one_x][third_one_y] == 1){
            if(four_one_x >= 0 && four_one_x < grid[0].size() && four_one_y >= 0 && four_one_y < grid[0].size()){
                if(grid[four_one_x][four_one_y] == 0){
                    return 1; // forced
                }
            }
        }
    }
    if(third_two_x >= 0 && third_two_x < grid[0].size() && third_two_y >= 0 && third_two_y < grid[0].size()){
        if(grid[third_two_x][third_two_y] == 1){
            if(four_two_x >= 0 && four_two_x < grid[0].size() && four_two_y >= 0 && four_two_y < grid[0].size()){
                if(grid[four_two_x][four_two_y] == 0){
                    return 1; // forced
                }
            }
        }
    }
    return 0; // Not forced
}

Noeud jump(Noeud &current, int direction, Noeud &start, Noeud &goal, std::vector<std::vector<int>> grid, int pid_parent, int depth)
{   
    Noeud new_node = step(current, direction);
    // Put one id/pid to the node and compute heuristic
    new_node.id = new_node.x * grid[0].size() + new_node.y;
    new_node.pid = pid_parent;
    new_node.compute_heuristic(goal);
    // outside the grid
    if (new_node.x < 0 || new_node.x >= grid[0].size() || new_node.y < 0 || new_node.y >= grid[0].size())
    {   
        return Noeud(-1, -1, -1, -1, -1);
    }
    // Obstacle
    if (grid[new_node.x][new_node.y] == 1)
    {
        return Noeud(-1, -1, -1, -1, -1);
    }
    // Goal
    if (new_node.x == goal.x && new_node.y == goal.y)
    {   
        return new_node;
    }
    // Forced neighbours
    int res = hasForcedNeighbours(new_node, direction, grid);
    if(res == 1){
        return new_node;
    }
    if(res == -1){
        return Noeud(-1, -1, -1, -1, -1);
    }
    // Diagonal move
    if (direction == 1 || direction == 3 || direction == 5 || direction == 7)
    {   
        if (direction == 7)
        {
            if (jump(new_node, direction - 1, start, goal, grid, pid_parent, 0).id != -1 || jump(new_node, 0, start, goal, grid, pid_parent, 0).id != -1)
            {
                return new_node;
            }
        }
        else
        {
            if (jump(new_node, direction - 1, start, goal, grid, pid_parent, 0).id != -1 || jump(new_node, direction + 1, start, goal, grid, pid_parent, 0).id != -1)
            {
                return new_node;
            }
        }
    }
    // Straight move and it is free to go --> recursive call to jump()
    return jump(new_node, direction, start, goal, grid, pid_parent, depth);
}

std::vector<Noeud> compute_path(std::vector<Noeud> closed_list, Noeud &goal, Noeud &start, std::vector<std::vector<int>> grid)
{   
    std::vector<Noeud> path;
    Noeud current = goal;
    path.push_back(current);
    int i = 0;
    while(current.id != 0){
        if(current.pid == closed_list[i].id){
            path.push_back(closed_list[i]);
            current = closed_list[i];
            i = 0;
        }
        else{
            i++;
        }
    }
    return path;
}

std::vector<Noeud> Jump_Search(Noeud &start, Noeud &goal, std::vector<std::vector<int>> grid)
{
    std::priority_queue<Noeud, std::vector<Noeud>, Compare> open_list;
    std::vector<Noeud> closed_list;
    std::vector<Noeud> path;
    std::vector<Noeud> moves;
    open_list.push(start);
    while (open_list.empty() != true)
    {   
        Noeud current = open_list.top();
        open_list.pop();
        // check if the current node is the goal
        if (current.x == goal.x && current.y == goal.y)
        {   
            closed_list.push_back(current);
            // Recompute the path from the nodes visited
            path = compute_path(closed_list, current, start, grid);
            return path;
        }
        else // Perform JPS algorithm
        {   
            moves = actions(current);
            for (int i = 0; i < moves.size(); i++)
            {   
                // Put one id to the node and Compute heuristic
                moves[i].id = moves[i].x * grid[0].size() + moves[i].y;
                moves[i].compute_heuristic(goal);
                // Check if outside the gird or in obstacle
                if (moves[i].x < 0 || moves[i].x >= grid[0].size() || moves[i].y < 0 || moves[i].y >= grid[0].size() || grid[moves[i].x][moves[i].y] == 1)
                {   
                    continue;
                }
                // Jump algorithm
                Noeud new_noeud = jump(current, direction(current, moves[i]), start, goal, grid, current.id, 0);
                new_noeud.cost = current.cost+sqrt(pow(current.x-new_noeud.x,2) + pow(current.y-new_noeud.y,2));
                if (new_noeud.id == -1) // No jump point
                {
                    continue;
                }
                if (new_noeud.x == goal.x && new_noeud.y == goal.y) // GOAL, we break the loop
                {
                    open_list.push(new_noeud);
                    break;
                }
                // Check if already in closed_list
                int k=0;
                for(int j=0; j<closed_list.size(); j++){
                    if(new_noeud.x == closed_list[j].x && new_noeud.y == closed_list[j].y){
                        k = k + 1;
                    }
                }
                if(k == 0){
                    open_list.push(new_noeud);
                }
            }
            //sleep(1);
            // Put the current node into the visited closed list
            closed_list.push_back(current);
        }
    }
    cout << "NO PATH";
}