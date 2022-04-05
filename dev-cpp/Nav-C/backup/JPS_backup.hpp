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
std::vector<Noeud> actions(Noeud& start);
std::vector<Noeud> Jump_Search(Noeud& start, Noeud& goal, std::vector<std::vector<int>> grid);