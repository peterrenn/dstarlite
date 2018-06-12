#ifndef DSTAR_H
#define DSTAR_H

#include <math.h>
// #include <stack>
// #include <queue>
#include <list>
// #include <ext/hash_map>

#include <dstar_lite/graph.h>
#include <dstar_lite/heap.h>

using namespace std;
// using namespace __gnu_cxx;


class DSLPlanner {

public:

 DSLPlanner();
 void   init(int sX, int sY, int gX, int gY, int height, int width);
 void   updateCell(int x, int y, double newCost);
 void   updateStart(int x, int y);
 void   updateGoal(int x, int y);
 bool   replan();
 list<node*> getPath();

private:

 list<node*> path;
 double k_m;
 // state s_start, s_goal, s_last;
 int maxSteps;

 double   heuristic(node* nodeA, node* nodeB);
 double   euclideanDist(node* nodeA, node* nodeB);
 double   cost(node* nodeA, node* nodeB);
 int      keyLess(double* keyA, double* keyB);
 double*  topKey();
 void     insert(node* s, double* newKey);
 void     updateKey(node* s, double* newKey);
 double*  calculateKey(node* s);
 void     updateVertex(node* u);
 int      computeShortestPath();

};

#endif
