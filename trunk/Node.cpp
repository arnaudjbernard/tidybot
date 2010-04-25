/*
* ECE 8853 - Autonomous Control of Robotic Systems
* Project - Tidybot
* Sethu Madhav Bhattiprolu && Arnaud BERNARD
* Spring 2010
*/

#include "Node.hpp"

Node::Node()
{
}

Node::Node(int x, int y)
{
	this->x = x;
	this->y = y;
	parent = 0;
}
