#include <iostream>
#include <queue>
#include <unordered_set>
#include <vector>
#include <math.h>
#include "JPS.hpp"
#include <unistd.h>
#include <cstdlib>
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

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
}
void Noeud::compute_heuristic(Noeud& goal)
{
    this->h = (abs(this->x - goal.x) + abs(this->y - goal.y)); // Manhatann heuristic
}

std::vector<Noeud> JPS::actions(Noeud& current)
{
    return {
        Noeud(current.x, current.y-1, current.cost + 1, 0, current.id),
        Noeud(current.x+1, current.y-1, current.cost + sqrt(2), 0, current.id),
        Noeud(current.x+1, current.y, current.cost + 1, 0, current.id),
        Noeud(current.x+1, current.y+1, current.cost + sqrt(2), 0, current.id),
        Noeud(current.x, current.y+1, current.cost + 1, 0, current.id),
        Noeud(current.x-1, current.y+1, current.cost + sqrt(2), 0, current.id),
        Noeud(current.x-1, current.y, current.cost + 1, 0, current.id),
        Noeud(current.x-1, current.y-1, current.cost + sqrt(2), 0, current.id)};
}

int JPS::direction(Noeud &parent, Noeud &child)
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

Noeud JPS::step(Noeud &parent, int direction)
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
    return Noeud(-1,-1,-1,-1,-1);
}

int JPS::hasForcedNeighbours(Noeud &current, int direction){
    int third_one_x;
    int third_two_x;
    int third_one_y;
    int third_two_y;
    int four_one_x;
    int four_one_y;
    int four_two_x;
    int four_two_y;
    if(direction == 1){
        return (hasForcedNeighbours(current,0) ||  hasForcedNeighbours(current,2));
    }
    if(direction == 3){
        return (hasForcedNeighbours(current,2) ||  hasForcedNeighbours(current,4));
    }
    if(direction == 5){
        return (hasForcedNeighbours(current,4) ||  hasForcedNeighbours(current,6));
    }
    if(direction == 7){
        return (hasForcedNeighbours(current,6) ||  hasForcedNeighbours(current,0));
    }
    if(direction == 0){ // up
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
        third_one_x = current.x;
        third_one_y = current.y+1;
        third_two_x = current.x;
        third_two_y = current.y-1;
        four_one_x = current.x-1;
        four_one_y = current.y+1;
        four_two_x = current.x-1;
        four_two_y = current.y-1;
    }
    if(exist_in_the_map(third_one_x, third_one_y)){
        if(obstacle_in_the_map(third_one_x, third_one_y)){
            if(exist_in_the_map(four_one_x, four_one_y)){
                if(obstacle_in_the_map(four_one_x, four_one_y) == false){
                    return 1; // forced
                }
            }
        }
    }
    if(exist_in_the_map(third_two_x, third_two_y)){
        if(obstacle_in_the_map(third_two_x, third_two_y)){
            if(exist_in_the_map(four_two_x, four_two_y)){
                if(obstacle_in_the_map(four_two_x, four_two_y) == false){
                    return 1; // forced
                }
            }
        }
    }
    return 0; // Not forced
}

Noeud JPS::jump(Noeud &current, int direction, int pid_parent, int depth)
{   
    Noeud new_node = step(current, direction);
    // Put one id/pid to the node and compute heuristic
    new_node.id = new_node.x * this->n + new_node.y;
    new_node.pid = pid_parent;
    new_node.compute_heuristic(this->goal);
    // outside the this->map_normal
    if (exist_in_the_map(new_node.x, new_node.y) == false)
    {   
        return Noeud(-1, -1, -1, -1, -1);
    }
    // Obstacle
    if (obstacle_in_the_map(new_node.x, new_node.y))
    {
        return Noeud(-1, -1, -1, -1, -1);
    }
    // this->goal
    if (new_node.x == this->goal.x && new_node.y == this->goal.y)
    {   
        return new_node;
    }
    // Forced neighbours
    int res = hasForcedNeighbours(new_node, direction);
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
            if (jump(new_node, direction - 1,pid_parent, 0).id != -1 || jump(new_node, 0, pid_parent, 0).id != -1)
            {
                return new_node;
            }
        }
        else
        {
            if (jump(new_node, direction - 1, pid_parent, 0).id != -1 || jump(new_node, direction + 1, pid_parent, 0).id != -1)
            {
                return new_node;
            }
        }
    }
    // Straight move and it is free to go --> recursive call to jump()
    return jump(new_node, direction, pid_parent, depth);
}

std::vector<Noeud> JPS::compute_path(std::vector<Noeud> closed_list,  Noeud& current)
{   
    std::vector<Noeud> path;
    path.push_back(current);
    int i = 0;
    while(current.id != 0){
        if(current.pid == closed_list[i].id){
            path.push_back(closed_list[i]);
            current = closed_list[i];
            // current.print_Noeud();
            i = 0;
        }
        else{
            i++;
        }
    }
    return path;
}

bool JPS::exist_in_the_map(int x, int y){
    if(x < 0 || y < 0 || x>=this->n || y>=this->n){
        return false;
    }
    else{
        return true;
    }
}

bool JPS::obstacle_in_the_map(int x, int y){
    uint32_t word = this->map_normal[x][floor(y/32)];
    int item = (31-(y%31));
    return CHECK_BIT(word, item);
}

// CODE FROM https://www.geeksforgeeks.org/bresenhams-line-generation-algorithm/ 
bool JPS::Bresenham(int x1, int y1, int x2, int y2, int dx, int dy, int decide)
{   
	int pk = 2 * dy - dx;
	for (int i = 0; i < dx; i++)
	{   
		x1 < x2 ? x1++ : x1--;
		if (pk < 0)
		{
			if (decide == 0)
			{
			    if(obstacle_in_the_map(x1,y1)){
                    return true;
                }
				pk = pk + 2 * dy;
			}
			else
			{
                if(obstacle_in_the_map(y1,x1)){
                    return true;
                }
				pk = pk + 2 * dy;
			}
		}
		else
		{
			y1 < y2 ? y1++ : y1--;
			if (decide == 0)
			{
                if(obstacle_in_the_map(x1,y1)){
                    return true;
                }
			}
			else
			{
                if(obstacle_in_the_map(y1,x1)){
                    return true;
                }
			}
			pk = pk + 2 * dy - 2 * dx;
		}
	}
    return false;
}


std::vector<Noeud> JPS::Jump_Search()
{
    std::priority_queue<Noeud, std::vector<Noeud>, Compare> open_list;
    std::vector<Noeud> closed_list;
    std::vector<Noeud> path;
    std::vector<Noeud> moves;
    open_list.push(this->start);
    while (open_list.empty() != true)
    {   
        Noeud current = open_list.top();
        open_list.pop();
        // check if the current node is the this->goal
        if (current.x == this->goal.x && current.y == this->goal.y)
        {   
            closed_list.push_back(current);
            // Recompute the path from the nodes visited
            path = compute_path(closed_list, current);
            return path;
        }
        else // Perform JPS algorithm
        {   
            moves = actions(current);
            for (int i = 0; i < moves.size(); i++)
            {   
                // Put one id to the node and Compute heuristic
                moves[i].id = moves[i].x * this->map_normal[0].size() + moves[i].y;
                moves[i].compute_heuristic(this->goal);
                // Check if outside the gird or in obstacle
                if (exist_in_the_map(moves[i].x, moves[i].y) == false || obstacle_in_the_map(moves[i].x, moves[i].y))
                {   
                    continue;
                }
                // Jump algorithm
                Noeud new_noeud = jump(current, direction(current, moves[i]), current.id, 0);
                new_noeud.cost = current.cost+sqrt(pow(current.x-new_noeud.x,2) + pow(current.y-new_noeud.y,2));
                if (new_noeud.id == -1) // No jump point
                {
                    continue;
                }
                if (new_noeud.x == this->goal.x && new_noeud.y == this->goal.y) // this->goal, we break the loop
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
            // Put the current node into the visited closed list
            closed_list.push_back(current);
        }
    }
    cout << "NO PATH";
    return closed_list;
}