#include <iostream>
#include <queue>
#include <unordered_set>
#include <vector>
#include <math.h>

using namespace std;
class Noeud{
    public:
        int x;
        int y;
        int id;
        int pid;
        double cost;
        double h;
    Noeud(int x, int y, double cost, int id, int pid){
        this->x = x;
        this->y = y;
        this->id = id;
        this->pid = pid;
        this->cost = cost;
    }
    void print_Noeud();
    void compute_heuristic(Noeud& goal);
};

class JPS{
    public:
        std::vector<std::vector<uint32_t>> map_normal;
        std::vector<std::vector<uint32_t>> map_transpose;
        Noeud goal = Noeud(0,0,0,0,0);
        Noeud start = Noeud(0,0,0,0,0);
        int n;
    JPS(std::vector<std::vector<uint32_t>> normal, std::vector<std::vector<uint32_t>> transpose, Noeud& start, Noeud& goal, int n){
        this->map_normal = normal;
        this->map_transpose = transpose;
        this->goal = goal;
        this->start = start;
        this->n = n;
    }
    std::vector<Noeud> actions(Noeud& current);
    std::vector<Noeud> Jump_Search();
    int direction(Noeud &parent, Noeud &child);
    Noeud step(Noeud &parent, int direction);
    int hasForcedNeighbours(Noeud &current, int direction);
    Noeud jump(Noeud &current, int direction, int pid_parent, int depth);
    std::vector<Noeud> compute_path(std::vector<Noeud> closed_list, Noeud& current);
    bool exist_in_the_map(int x, int y);
    bool obstacle_in_the_map(int x, int y);
};