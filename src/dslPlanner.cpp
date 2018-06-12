#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
// #include <math.h>
// #include <list>

#include <dstar_lite/dsl_planner.h>

// #include "dslPlanner.h"
// #include "graph.h"
// #include "graph.c"
// #include "heap.h"
// #include "heap.c"

// #ifndef sqrt2
//   #define sqrt2 1.414213562373095
// #endif

// list<node*> path;
// double k_m;
// int maxSteps;

/* void DSLPlanner::DSLPlanner()
 * --------------------------
 * Constructor sets maxSteps.
 */
DSLPlanner::DSLPlanner() {
  maxSteps = 80000;  // node expansions before we give up
}

/* double DSLPlanner::heuristic(node* nodeA, node* nodeB)
 * --------------------------
 * returns heuristic (8-way distance between two nodes)
 */
double DSLPlanner::heuristic(node* nodeA, node* nodeB) {
  double tmp;
  double min = abs(nodeA->x - nodeB->x);
  double max = abs(nodeA->y - nodeB->y);
  if (min > max) {
    double tmp = min;
    min = max;
    max = tmp;
  }
  return ((M_SQRT2-1.0)*min + max);
}

/* double DSLPlanner::euclideanDist(node* nodeA, node* nodeB)
 * --------------------------
 * returns Euclidean distance between node a and node b.
 */
double DSLPlanner::euclideanDist(node* nodeA, node* nodeB) {
  float dx = nodeA->x - nodeB->x;
  float dy = nodeA->y - nodeB->y;
  return sqrt(dx*dx + dy*dy);
}

/* double DSLPlanner::cost(node* nodeA, node* nodeB)
 * --------------------------
 * Returns the cost of moving off node a. This is also the 8-way cost.
 * Method should only be called for unoccupied nodeA (nodeA->cost >= 0)
 */
double DSLPlanner::cost(node* nodeA, node* nodeB) {

  float dx = nodeA->x - nodeB->x;
  float dy = nodeA->y - nodeB->y;
  double scale = 1;

  if (dx+dy>1) scale = M_SQRT2;

  return scale*nodeA->cost;
}

/* int DSLPlanner::keyLess(double* keyA, double* keyB)
 * --------------------------
 * returns true if keyA < keyB based on the first two array values
 */
int DSLPlanner::keyLess(double* keyA, double* keyB)
{
  if(keyA[0] < keyB[0])
      return true;
  else if(keyA[0] == keyB[0] && keyA[1] < keyB[1])
      return true;
  return false;
}

/* double* DSLPlanner::topKey()
 * --------------------------
 * returns the key of the top node on the heap
 * using global topKeyKey (graph.h) to store the returned key for coding ease.
 */
double* DSLPlanner::topKey()
{
  node* top = topHeap();

  if(top == NULL) // I added this to fix a case that happens when s_start is the only node in the heap and then it is popped and no new nodes are added
  {
    topKeyKey[0] = LARGE;   // having LARGE here is good for this case because it forces the while loop check to fail in computShortestPath
    topKeyKey[1] = LARGE;
  }
  else
  {
    topKeyKey[0] = top->k[0];
    topKeyKey[1] = top->k[1];
  }

  return topKeyKey;
}


/* void DSLPlanner::insert(node* s, double* newKey)
 * --------------------------
 * inserts the node into the heap using the new key newKey
 */
void DSLPlanner::insert(node* s, double* newKey)
{
  s->k[0] = newKey[0];
  s->k[1] = newKey[1];
  addToHeap(s);
}

/* void DSLPlanner::updateKey(node* s, double* newKey)
 * --------------------------
 * changes the key of the node and then
 * re-orders the heap to account for the new key value
 */
void DSLPlanner::updateKey(node* s, double* newKey)
{
  s->k[0] = newKey[0];
  s->k[1] = newKey[1];
  updateHeapPositionOfNode(s);
}

/* void DSLPlanner::updateCell(int x, int y, double newCost)
 * --------------------------
 * changes the cost of the node and then
 * calls updateVertex for node (from paper)
 */
void DSLPlanner::updateCell(int x, int y, double newCost)
{
  node* s;
  if(y < 0 || y >= HEIGHT || x < 0 || x >= WIDTH)
      return;
  s = &graph[y][x];
  s->cost = newCost;
  updateVertex(s);
}


/* void Dstar::updateStart(int x, int y)
* --------------------------
* Update the position of the robot, this does not force a replan.
* increments k_m using s_last node and updates start key
*/
void DSLPlanner::updateStart(int x, int y) {
  s_start = &graph[y][x];

  k_m += heuristic(s_last,s_start);

  updateKey(s_start, calculateKey(s_start));
  s_last  = s_start;

}


/* void Dstar::updateGoal(int x, int y)
* --------------------------
* Example code basically reinitialized everything except the costs and s_start
* I'm only going to update the graph if the goal changed (first case after init)
*/
void DSLPlanner::updateGoal(int x, int y) {
  if(x != s_goal->x || y != s_goal->y){
    s_goal->rhs = LARGE;
    s_goal = &graph[y][x];
    s_goal->rhs = 0;
  }
}

/* double* DSLPlanner::calculateKey(node* s)
* --------------------------
* calculates the key of a node, and then returns the key.
* using the global calculateKeyKey to store the returned key for coding ease.
*/
double* DSLPlanner::calculateKey(node* s)
{
  calculateKeyKey[0] = min(s->g,s->rhs) + heuristic(s_start,s) + k_m;
  calculateKeyKey[1] = min(s->g,s->rhs);
  return calculateKeyKey;
}

/* void DSLPlanner::updateVertex(node* u)
* --------------------------
* function from paper: updates rhs for non-goal nodes
* if node is inconsistent, adds node to heap
* else, ensures its removed from heap
*/
void DSLPlanner::updateVertex(node* u)
{
//printf("updating node: %d \n",s);
  int n;
  node* s_prime;
  double minRhs = LARGE;
  double thisRhs;
  //if u isn't s_goal, rhs = min_(s' in successors) c(u,s') + s'->g
  if(u->x != s_goal->x || u->y != s_goal->y){
    // In order to have successors, u must be unoccupied
    if (u->cost >= 0){
      for(n = 0; n < NEIGHBORS; n++)
      {
        s_prime = u->neighbors[n];
        if(s_prime == NULL)
          continue;

        thisRhs = s_prime->g + cost(u, s_prime);
        if(thisRhs < minRhs)
          minRhs = thisRhs;
      }
    }
    u->rhs = minRhs;
  }
  if(u->g != u->rhs)
  {
    if(u->inHeap != true) // I added this check to increase speed
      insert(u,calculateKey(u));
    else
      updateKey(u,calculateKey(u));
  }
  else if(u->inHeap == true)
    deleteNodeFromHeap(u);
}

/* int DSLPlanner::computeShortestPath()
* --------------------------
* function from paper, does most of the planning
* returns 0 unless number of expansions exceeds maxsteps (returns -1)
* or priority heap is empty (returns 1)
* loop updates predecessors of top node in priority heap
*/
int DSLPlanner::computeShortestPath()
{
  node* u;
  node* s_prime;
  int n;
  int count;

  if (indexOfLast < 0) return 1;

  while(keyLess(topKey(),calculateKey(s_start)) || s_start->rhs != s_start->g)
  {
    if (count > maxsteps) return -1;
    keyOld[0] = topKey()[0];
    keyOld[1] = topKey()[1];
    u = popHeap(); // old code checks if u is valid
    if (keyLess(keyOld,calculateKey(u))) {
      insert(u, calculateKey(u));
    }
    else if(u->g > u->rhs)
    {
      u->g = u->rhs;

      // update predecessors
      // if node is occupied, not a valid predecessor
      for(n = 0; n < NEIGHBORS; n++)
      {
        s_prime = u->neighbors[n];
        if(s_prime == NULL)
          continue;
        if(s_prime->cost < 0)
          continue;
        updateVertex(s_prime);
      }
    }
    else
    {
      u->g = LARGE;
      // update predecessors and u
      for(n = 0; n < NEIGHBORS; n++)
      {
        s_prime = u->neighbors[n];
        if(s_prime == NULL)
          continue;
        if(s_prime->cost < 0)
          continue;
        updateVertex(s_prime);
      }
      updateVertex(u);
    }
    count++;
  }
  return 0;
}

/* void Dstar::init(int sX, int sY, int gX, int gY, int height, int width)
 * --------------------------
 * Initialize with start and goal coordinates, rest is as per
 * [S. Koenig, 2002]
 */
void DSLPlanner::init(int sX, int sY, int gX, int gY, int height, int width) {
  path.clear();
  while(indexOfLast < 0) popHeap();
  k_m = 0;
  if(heapNode != NULL)
     deleteHeap();
  if(graph != NULL)
     deleteGraph();
  buildGraph(height, width);
  buildHeap();
  s_start = &graph[sY][sX];
  s_goal = &graph[gY][gX];
  s_goal->rhs = 0;
  insert(s_goal,calculateKey(s_goal));
  s_last = s_start;
}

/* void Dstar::getPath()
 * --------------------------
 * Returns the path created by replan()
 */
list<node*> DSLPlanner::getPath() {
  return path;
}

/* bool Dstar::replan()
 * --------------------------
 * Updates the costs for all cells and computes the shortest path to
 * goal. Returns true if a path is found, false otherwise. The path is
 * computed by doing a greedy search over the cost+g values in each
 * cells. In order to get around the problem of the robot taking a
 * path that is near a 45 degree angle to goal we break ties based on
 * the metric euclidean(state, goal) + euclidean(state,start).
 */
bool DSLPlanner::replan() {
  path.clear();

  int res = computeShortestPath();
  //printf("res: %d ols: %d ohs: %d tk: [%f %f] sk: [%f %f] sgr: (%f,%f)\n",res,openList.size(),openHash.size(),openList.top().k.first,openList.top().k.second, s_start.k.first, s_start.k.second,getRHS(s_start),getG(s_start));
  if (res < 0) {
    fprintf(stderr, "NO PATH TO GOAL\n");
    return false;
  }

  node* cur = s_start;

  if (s_start->g == LARGE) {
    fprintf(stderr, "NO PATH TO GOAL\n");
    return false;
  }

  while(cur->x != s_goal->x || cur->y != s_goal->y) {

    path.push_back(cur);

    int nSuccessors = 0;
    double costmin = LARGE;
    double distmin;
    node* smin;
    node* s_prime;
    double val;
    double val2;
    int n;

    if (cur->cost >= 0){
      for(n = 0; n < NEIGHBORS; n++)
      {
        s_prime = cur->neighbors[n];
        if(s_prime == NULL)
          continue;
        nSuccessors++;
        val = cost(cur, s_prime) + s_prime->g;
        val2 = euclideanDist(s_start,s_prime) + euclideanDist(s_prime,s_goal);
        if (val < costmin ||(val == costmin && distmin > val2)){
          if (distmin > val2){
            distmin = val2;
            costmin = val;
            smin = s_prime;
          }
        }
      }
      if (nSuccessors == 0) {
        fprintf(stderr, "NO PATH TO GOAL\n");
        return false;
      }
    }
    cur = smin;
  }
  path.push_back(s_goal);
  return true;
}
