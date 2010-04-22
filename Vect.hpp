/*
* ECE 8853 - Autonomous Control of Robotic Systems
* Project - Tidybot
* Sethu Madhav Bhattiprolu && Arnaud BERNARD
* Spring 2010
*/

//Vector

#ifndef DEF_CLASS_VECT
#define DEF_CLASS_VECT

#include <cmath>

class Vect
{
public:
	double rho;
	double theta; //in rad in ]-pi; pi] (can set out of bound, will be corrected if use operator+)
	
	Vect();
	
	Vect(double rho, double theta);
	
	Vect operator+(const Vect &vect);
	
	Vect operator*(double factor);
};

#endif
