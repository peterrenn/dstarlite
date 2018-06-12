/*
 *  Code modified from Base_Planner_CU.
 *  Copyright Michael Otte, University of Colorado, 9-22-2008, 2009
 */

// allocates the initial memory for the graph and map and sets up the graph
// structure. height and width refer to map, this
// must be called before any of the next four functions

void buildGraph(int height, int width)
{
  int x, y, d, newx, newy;

  HEIGHT = height;  // set globals
  WIDTH = width;    // set globals

  if (graph == NULL)
  {
	graph = (node**)calloc(height, sizeof(node*));

  for (y = 0; y < height; ++y)
	   graph[y] = (node*)calloc(width, sizeof(node));

	for (y = 0; y < height; ++y)
  {
      for (x = 0; x < width; ++x)
      {
        graph[y][x].g = LARGE;
        //graph[y][x].h = 0;
        // graph[y][x].f = LARGE;
        graph[y][x].rhs = LARGE;
        graph[y][x].cost = 1;
        graph[y][x].k[0] = LARGE;
        graph[y][x].k[1] = LARGE;
        graph[y][x].x = x;
        graph[y][x].y = y;
        // graph[y][x].bestNeighbor = NULL;
        graph[y][x].inHeap = false;
        graph[y][x].heapIndex = -1;
		    for (d = 0; d < NEIGHBORS; ++d)
        {
    		  newy = y + dy[d];
    		  newx = x + dx[d];

          if(newy >= 0 && newy < height && newx >= 0 && newx < width)
            graph[y][x].neighbors[d] = &graph[newy][newx];
          else
            graph[y][x].neighbors[d] = NULL;
        }
	     }
    }
  }
}


#define TWOBYTE unsigned short
#define FOURBYTE unsigned long
#define LONG unsigned long


// this re-initializes the graph, saving cost values
void cleanGraph()
{
  int x, y;

  for (y = 0; y < HEIGHT; ++y)
  {
    for (x = 0; x < WIDTH; ++x)
    {
      graph[y][x].g = LARGE;
      graph[y][x].rhs = LARGE;
      graph[y][x].k[0] = LARGE;
      graph[y][x].k[1] = LARGE;
  	  graph[y][x].x = x;
  	  graph[y][x].y = y;
      graph[y][x].inHeap = false;
      graph[y][x].heapIndex = -1;
    }
  }
}

// this takes care of memory management when the graph is deleted -- M.O.
void deleteGraph()
{
  int y;
  for (y = 0; y < HEIGHT+1; ++y)
    free(graph[y]);
  free(graph);
  graph = NULL;
}

// returns true if nodeA < nodeB based on key vectors [k[0] k[1]]
int nodeLess(node* nodeA, node* nodeB)
{
  if(nodeA->k[0] < nodeB->k[0])
      return true;
  else if(nodeA->k[0] == nodeB->k[0] && nodeA->k[1] < nodeB->k[1])
      return true;
  return false;
}

// returns true if nodeA <= nodeB based on key vectors [k[0] k[1]]
int nodeLesseq(node* nodeA, node* nodeB)
{
  if(nodeA->k[0] <= nodeB->k[0])
      return true;
  else if(nodeA->k[0] == nodeB->k[0] && nodeA->k[1] <= nodeB->k[1])
      return true;
  return false;
}
