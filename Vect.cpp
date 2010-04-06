/*
* ECE 8853 - Autonomous Control of Robotic Systems
* Project - Tidyb
* Sethu Madhav Bhattiprolu && Arnaud BERNARD
* Spring 2010
*/

#include "Vect.hpp"
	
Vect::Vect()
{
	this->rho = 0;
	this->theta = 0;
}

Vect::Vect(double rho, double theta)
{
	this->rho = rho;
	this->theta = theta;
}

Vect::Vect Vect::operator+(const Vect &vect)
{
	Vect result;
	double resx, resy;
	resx = this->rho*cos(this->theta) +  vect.rho*cos(vect.theta);
	resy = this->rho*sin(this->theta) +  vect.rho*sin(vect.theta);
	result.rho = sqrt(pow(resx,2)+pow(resy,2));
	if(resx == 0 && resy == 0)
		result.theta = 0;
	else
		result.theta = 2*atan(resy/(resx+sqrt(pow(resx,2)+pow(resy,2))));
	return result;
}

Vect::Vect Vect::operator*(double factor)
{
	Vect result;
	result.rho = this->rho*factor;
	result.theta = this->theta;
	return result;
}
