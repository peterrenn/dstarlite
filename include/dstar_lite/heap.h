/*
 *  Code modified from Base_Planner_CU.
 *  Copyright Michael Otte, University of Colorado, 9-22-2008, 2009
 */

double* heapCost;     // cost values
node** heapNode;      // pointers to the corresponding map nodes
int indexOfLast;      // the index of the last node in the heap array
int parentOfLast;     // stores the index of the parent of the last node
int tempInd;          // used to help swap nodes
node*  tempNode;      // used to help swap nodes

// sets up the memory for the heap
void buildHeap();

// compares a node n with its parent, and switches them if the parent's
// cost is more than the node's cost. Repeats if a switch happens.
void bubbleUp(int n);

// compares a node n with its children, and switches them if a child's cost
// is less than the node's cost. Repeats if a switch happens.
void bubbleDown(int n);

// add thisNode to the heap
void addToHeap(node* thisNode);

// returns a pointer to the node that is on the top of the heap
node* topHeap();

// removes the top valued node from the heap and returns a pointer to it
node* popHeap();

// deletes this_node from the heap, and then repairs the heap
void deleteNodeFromHeap(node* this_node);

// repairs the heap if this_node is in the wrong place in the heap
void updateHeapPositionOfNode(node* this_node);

// prints the heap values on the command line
void printHeap();

// returns 1 if heap is good, 0 if bad, also prints a command line message
int checkHeap();

// cleans up and deletes the memory used by the heap
void deleteHeap();
