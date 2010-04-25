/*
* ECE 8853 - Autonomous Control of Robotic Systems
* Project - Tidybot
* Sethu Madhav Bhattiprolu && Arnaud BERNARD
* Spring 2010
*/

#ifndef DEF_CLASS_NODE
#define DEF_CLASS_NODE

#include <string>
#include <vector>

class Node
{
public:
	Node();
	Node(int x, int y);
	
	int x;
	int y;
	int costF;// Estimated total distance from start to goal through here.
	int costG;// Distance from start to here along optimal path.
	int costH;// Heuristic estimate of distance from here to goal.
	//std::string label;
	Node * parent;
	std::vector<Node *> Visible;
};

#endif
