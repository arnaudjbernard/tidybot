/*
* ECE 8853 - Autonomous Control of Robotic Systems
* Project - Tidybot
* Sethu Madhav Bhattiprolu && Arnaud BERNARD
* Spring 2010
*/

#include "Obstacle.hpp"

Obstacle::Obstacle()
{
}

Obstacle::Obstacle(int x, int y, int w, int h)
{
	this->x = x;
	this->y = y;
	this->w = w;
	this->h = h;
}

void Obstacle::grow(int d, int mapW, int mapH)
{
	x = x - d;//std::max(x - d , 0);
	y = y - d;//std::max(y - d , 0);
	w = w + 2*d;//std::min(w + 2*d, mapW - x );
	h = h + 2*d;//std::min(h + 2*d, mapH - y );
}
