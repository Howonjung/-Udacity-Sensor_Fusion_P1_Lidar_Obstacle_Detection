/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(std::vector<float> point, int id, unsigned char pointDim)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		if (pointDim == 2)
			insertHelper2D(&root,0, point, id);
		else
			insertHelper3D(&root,0, point, id);

	}

	void insertHelper2D(Node **node, uint depth, std::vector<float> point, int id)
    {
      if(*node == NULL)
      {		
		// root node!
		*node = new Node(point, id);
		
      }
      else{
		  uint currentDepth = depth % 2;
		  if ( (*node)->point[currentDepth] > point[currentDepth] ){
			  insertHelper2D( &((*node)->left) ,depth+1, point, id);
		  }
		  else{
			  insertHelper2D( &((*node)->right) ,depth+1, point, id);
		  }
      }
    }

	void insertHelper3D(Node **node, uint depth, std::vector<float> point, int id)
    {

	 if(*node == NULL){		
		// root node!
		*node = new Node(point, id);

	 }
     else{
		uint currentDepth = depth % 3;
		// split based on x value
		if (currentDepth == 0 && ((*node)->point[currentDepth] > point[currentDepth]) ){
			insertHelper3D( &((*node)->left) ,depth+1, point, id);
		}
		// split based on x value
		else if (currentDepth == 0 && ((*node)->point[currentDepth] < point[currentDepth]) ){
			insertHelper3D( &((*node)->right) ,depth+1, point, id);
		}

		// split based on y value
		else if(currentDepth == 1 && ((*node)->point[currentDepth] > point[currentDepth]) ){
			insertHelper3D( &((*node)->left) ,depth+1, point, id);
		}

		// split based on y value
		else if (currentDepth == 1 && ((*node)->point[currentDepth] < point[currentDepth]) ){
			insertHelper3D( &((*node)->right) ,depth+1, point, id);
		}
		// split based on z value
		else if(currentDepth == 2 && ((*node)->point[currentDepth] > point[currentDepth]) ){
			insertHelper3D( &((*node)->left) ,depth+1, point, id);
		}
		else{
			insertHelper3D( &((*node)->right) ,depth+1, point, id);
		}
	 }
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol,unsigned char pointDim)
	{	

		std::vector<int> ids;
		if (pointDim == 2)
			searchHelper2D(&root,0, &ids , target, distanceTol);
		else
			searchHelper3D(&root,0, &ids , target, distanceTol);
		return ids;
	}

	void searchHelper3D(Node **node, uint depth, std::vector<int> *ids, std::vector<float> target, float distanceTol)
	{	
		if((*node) != NULL)
		{
			uint currentDepth = depth % 3;
			if ( ((*node)->point[0] > (target[0] - distanceTol) && (*node)->point[0] < (target[0] + distanceTol)) &&
			((*node)->point[1] > (target[1] - distanceTol) && (*node)->point[1] < (target[1] + distanceTol)) &&
			((*node)->point[2] > (target[2] - distanceTol) && (*node)->point[2] < (target[2] + distanceTol)) ){
				float dist = sqrt(pow(((*node)->point[0]-target[0]),2)+pow(((*node)->point[1]-target[1]),2)+pow(((*node)->point[2]-target[2]),2));
				if (dist < distanceTol)
					ids->push_back((*node)->id);
			}

			// check across boundary
			// 1. When the (target[currentDepth] ± distanceTol) is located on the left side of (*node)->point[currentDepth]
			if ( ((*node)->point[currentDepth] > (target[currentDepth]-distanceTol)) && 
			((*node)->point[currentDepth] > (target[currentDepth]+distanceTol )) )
				searchHelper3D( &((*node)->left), depth+1, ids ,target, distanceTol);
			
			// 2. When the (target[currentDepth] - distanceTol) & (target[currentDepth] + distanceTol) are located on the left and right side of (*node)->point respectively
			// but (*node)->point[currentDepth] is larger than target[currentDepth]
			if ( ((*node)->point[currentDepth] > (target[currentDepth]-distanceTol)) && 
			((*node)->point[currentDepth] < (target[currentDepth]+distanceTol )) )
				searchHelper3D( &((*node)->left), depth+1, ids ,target, distanceTol);
			
			// 3. When the (target[currentDepth] - distanceTol) & (target[currentDepth] + distanceTol) are located on the left and right side of (*node)->point respectively
			// but target[currentDepth] is larger than (*node)->point[currentDepth]
			if( ((*node)->point[currentDepth]  < (target[currentDepth]+distanceTol)) &&
			((*node)->point[currentDepth]  < (target[currentDepth]-distanceTol)) )
				searchHelper3D( &((*node)->right), depth+1, ids ,target,distanceTol);

			// 4. When the (target[currentDepth] ± distanceTol) is located on the right side of (*node)->point[currentDepth]
			if( ((*node)->point[currentDepth]  < (target[currentDepth]+distanceTol)) &&
			((*node)->point[currentDepth]  > (target[currentDepth]-distanceTol)) )
				searchHelper3D( &((*node)->right), depth+1, ids ,target,distanceTol);

		}
	}

	void searchHelper2D(Node **node, uint depth, std::vector<int> *ids, std::vector<float> target, float distanceTol)
	{	
		if((*node) != NULL)
		{
			uint currentDepth = depth % 2;
			// float dist = calculateDist((*node)->point, target);
			if ( ((*node)->point[0] > (target[0] - distanceTol) && (*node)->point[0] < (target[0] + distanceTol)) &&
			((*node)->point[1] > (target[1] - distanceTol) && (*node)->point[1] < (target[1] + distanceTol)) ){
				float dist = sqrt(pow(((*node)->point[0]-target[0]),2)+pow(((*node)->point[1]-target[1]),2));
				if (dist < distanceTol)
					ids->push_back((*node)->id);
			}

			// check across boundary
			if ((*node)->point[currentDepth] > (target[currentDepth]-distanceTol ))
				searchHelper2D( &((*node)->left), depth+1, ids ,target, distanceTol);
			
			if((*node)->point[currentDepth]  < (target[currentDepth]+distanceTol ))
				searchHelper2D( &((*node)->right), depth+1, ids ,target,distanceTol);
			
		}
	}

};




