/*
 *
 *  Code modified from Base_Planner_CU.
 *  Copyright Michael Otte, University of Colorado, 9-22-2008, 2009
 */

#ifndef NEIGHBORS
  #define NEIGHBORS 8               // this is the N in N-graph, 4 or 8
  static int dx[] = {1, 0, -1, -1,  0,  0, 1, 1};
  static int dy[] = {0, 1,  0,  0, -1, -1, 0, 0};
#endif

#ifndef LARGE
  #define LARGE 100000000             // this is the 'infinity' cost
#endif

#ifndef SMALL
  #define SMALL .000001             // this is the 'epsilon' cost (things below this may be zero)
#endif

#ifndef false
  #define false 0                   // define false
#endif

#ifndef true
  #define true 1                    // define true
#endif

#ifndef closed
  #define closed -1                 // define closed
#endif

struct node;
typedef struct node node;

struct node
{
    int x, y;                       // this node's coordinates

    node* neighbors[NEIGHBORS];     // this holds pointers to neighbors (NULL if unused)
    //node* bestNeighbor;             // this is a pointer to the best neighbor (clockwise-most of the two neighbors used for the best edge)

    double g;                       // actual cost from start of path to this node
    //double h;                       // heuristic estimated cost from this node to end of path
    //double f;                       // actual + heuristic cost (g+h)
    double rhs;                     // this is the rhs value for field d* (lite)
    double cost;                    // cell cost
    double k[2];                    // this is the key value for field d* (lite)

    int inHeap;                     // is this node currently in the heap? (true, false, closed)
    int heapIndex;                  // this stores the heap index of the node
};

node** graph;                // used to create the graph as a 2D array of nodes, global for speed (note: 1 more in Height and Width than map)
int HEIGHT;                  // stores the height of the map (height of graph is one more)
int WIDTH;                   // stores the width of the map (width of graph is one more)
node* s_start;               // stores the current node associated with the robot's position
node* s_last;                // stores the current node associated with the robot's last position
node* s_goal;                // stores the current node associated with the goal position

double keyOld[] = {LARGE, LARGE};           // stores a key that is used in the function main (I did this to save on memory management)
double topKeyKey[] = {LARGE, LARGE};         // stores a key that is returned by the function topKey (I did this to save on memory management)
double calculateKeyKey[] = {LARGE, LARGE};   // stores a key that is returned by the function topKey (I did this to save on memory management)

// allocates the initial memory for the graph and map and sets up the graph
// structure. height and width refer to map, because graph is 1 more. this
// must be called before any of the next four functions
void buildGraph(int height, int width);

// this re-initializes the graph and map, saving cost values
void cleanGraph();

// this cleans up and deletes the memory associated with the global graph
// and map
void deleteGraph();

// returns true if nodeA < nodeB based on key vectors
int nodeLess(node* nodeA, node* nodeB);

// returns true if nodeA <= nodeB based on key vectors
int nodeLesseq(node* nodeA, node* nodeB);
