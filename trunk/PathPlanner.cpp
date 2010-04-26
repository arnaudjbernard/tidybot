/*
* ECE 8853 - Autonomous Control of Robotic Systems
* Project - Tidybot
* Sethu Madhav Bhattiprolu && Arnaud BERNARD
* Spring 2010
*/

#include "PathPlanner.hpp"
	
PathPlanner::PathPlanner(int VERBOSITY)
{
	this->VERBOSITY = VERBOSITY;
	map = 0;
	mapH = 100;
	mapW = 100;
	robotSize = 1;
	originX = 0;
	originY = 0;
	goalX = 0;
	goalY = 0;
	readRoomFile();
	growObstacles();
}

std::vector<std::pair<int, int> > PathPlanner::getWayPoints(std::pair<int, int> origin, std::pair<int, int> goal)
{
	std::vector<std::pair<int, int> > result;
	originX = origin.first;
	originY = origin.second;
	goalX = goal.first;
	goalY = goal.second;
	if(VERBOSITY & 2)printf("Computing waypoints form [%d	%d] to [%d	%d].\n", originX, originY, goalX, goalY);
	if(map[mapW*originY + originX] == 1)
	{
		if(VERBOSITY & 2)printf("Origin [%d	%d] out of bound, calculating closest inbound point.\n", originX, originY);
		std::pair<int, int> newOrigin = findInbound(origin);
		originX = newOrigin.first;
		originY = newOrigin.second;
		if(VERBOSITY & 4)printf("New origin set to [%d	%d].\n", originX, originY);
	}
	if(map[mapW*goalY + goalX] == 1)
	{
		if(VERBOSITY & 2)printf("Goal [%d	%d] out of bound, calculating closest inbound point.\n", goalX, goalY);
		std::pair<int, int> newGoal = findInbound(goal);
		goalX = newGoal.first;
		goalY = newGoal.second;
		if(VERBOSITY & 4)printf("New goal set to [%d	%d].\n", goalX, goalY);
	}
	if(VERBOSITY & 2)printf("Computing waypoints form [%d	%d] to [%d	%d].\n", originX, originY, goalX, goalY);
	createVisibilityGraph();
	applyAStar();
	result = displayResult();
#ifdef OPENCV
	if(VERBOSITY & 1)printf("Select a display window and press any key to exit.\n");
	cv::waitKey(0);
#endif
	return result;
}

std::pair<int, int> PathPlanner::findInbound(std::pair<int, int> point)
{
	std::pair<int, int> result(point.first, point.second);
	int radius, x, y;
	for(radius = 0; radius < mapH; radius++)
	{
		y = -radius;
		for(x = -radius; x <= radius; x++)
		{
			if(map[mapW*(point.second + y) + (point.first + x)] == 0)
			{
				result.first = point.first + x;
				result.second = point.second + y;
				return result;
			}
		}
		x = radius;
		for(y = -radius; y <= radius; y++)
		{
			if(map[mapW*(point.second + y) + (point.first + x)] == 0)
			{
				result.first = point.first + x;
				result.second = point.second + y;
				return result;
			}
		}
		y = radius;
		for(x = radius; x >= -radius; x--)
		{
			if(map[mapW*(point.second + y) + (point.first + x)] == 0)
			{
				result.first = point.first + x;
				result.second = point.second + y;
				return result;
			}
		}
		x = -radius;
		for(y = radius; y > -radius; y--)
		{
			if(map[mapW*(point.second + y) + (point.first + x)] == 0)
			{
				result.first = point.first + x;
				result.second = point.second + y;
				return result;
			}
		}
	}
	return result;
}

void PathPlanner::readRoomFile()
{
	if(VERBOSITY & 8)printf("Reading Room.wld file.\n");
	
	std::ifstream file("map/Room.wld");
	if(file)
	{
		std::string line;
		while(std::getline(file, line))
		{
			if(VERBOSITY & 8)printf(".");
			std::vector<std::string> tokens;
			if(Split(tokens, line, ' '))
			{
				if(tokens[0] == "Obstacle")
				{
					Obstacle tmp(atoi(tokens[1].c_str())*100, atoi(tokens[2].c_str())*100, atoi(tokens[3].c_str())*100, atoi(tokens[4].c_str())*100);
					Obstacles.push_back(tmp);
				}
				else if(tokens[0] == "MapSize")
				{
					mapW = atoi(tokens[1].c_str())*100;
					mapH = atoi(tokens[2].c_str())*100;
				}
				else if(tokens[0] == "RobotSize")
				{
					robotSize = (int)(atof(tokens[1].c_str())*100);
				}
			}
		}
	}
	
	if(VERBOSITY & 8)printf("Summary:\n");
	if(VERBOSITY & 8)printf("Map size:		(%d,	%d)\n", mapW, mapH);
	if(VERBOSITY & 8)printf("Robot size:		%d	\n", robotSize);
	if(VERBOSITY & 8)printf("Nb of obstacles:	%d\n", (int)Obstacles.size());
	if(VERBOSITY & 8)printf("\n");
}

void PathPlanner::growObstacles()
{
	if(VERBOSITY & 4)printf("Growing obstacles.\n");
	
	if(VERBOSITY & 8)printf("Expanding obstacles by %d.\n", robotSize);
	
#ifdef OPENCV
	imgOri = cv::Mat::zeros(mapH,mapW,CV_8UC3);
	imgMap = cv::Mat::zeros(mapH,mapW,CV_8UC3);
	cv::Mat_<cv::Vec3b>& imgMapP = (cv::Mat_<cv::Vec3b>&)imgMap;
#endif
	
	if(VERBOSITY & 8)printf("Obstacles old dimensions		->	Obstacles new dimensions\n");
	if(VERBOSITY & 8)printf("(x,	y,	w,	h	)	->	(x,	y,	w,	h	)\n");
	for(int i = 0; i < Obstacles.size(); i++)
	{
#ifdef OPENCV
		cv::rectangle(imgOri,	cv::Point(Obstacles[i].x,Obstacles[i].y),\
						 		cv::Point(Obstacles[i].x+Obstacles[i].w,Obstacles[i].y+Obstacles[i].h),\
						 		cv::Scalar(255,255,255),-1);
#endif
		if(VERBOSITY & 16)printf("(%d,	%d,	%d,	%d	)	->	", Obstacles[i].x,Obstacles[i].y,Obstacles[i].w,Obstacles[i].h);
		Obstacles[i].grow(robotSize, mapW, mapH);
		if(VERBOSITY & 16)printf("(%d,	%d,	%d,	%d	)\n", Obstacles[i].x,Obstacles[i].y,Obstacles[i].w,Obstacles[i].h);
	}
	
	if(VERBOSITY & 8)printf("Creating bitmap of size %dx%d.\n", mapW, mapH);
	map = new bool[mapW * mapH];
	for(int y = 0; y < mapH; y++)
	{
		for(int x = 0; x < mapW; x++)
		{
			map[mapW*y + x] = 0;
		}
	}
	if(VERBOSITY & 8)printf("Inserting %d obstacles.\n", (int)Obstacles.size());
	for(int i = 0; i < Obstacles.size(); i++)
	{
		if(VERBOSITY & 8)printf(".");
		for(int y = Obstacles[i].y+1; y < Obstacles[i].y + Obstacles[i].h-1; y++)
		{
			for(int x = Obstacles[i].x+1; x < Obstacles[i].x + Obstacles[i].w-1; x++)
			{
				if(x>=0 && y>=0 && x<mapW && y<mapH)
				{
					map[mapW*y + x] = 1;
#ifdef OPENCV
					imgMapP(y,x)[0] = 255;
					imgMapP(y,x)[1] = 255;
					imgMapP(y,x)[2] = 255;
#endif
				}
			}
		}
	}
	if(VERBOSITY & 8)printf("\n");
#ifdef OPENCV
	cv::Mat imgOriR;
	cv::resize(imgOri, imgOriR, cv::Size(imgOri.cols/SCALE,imgOri.rows/SCALE));
	cv::imshow("imgOri", imgOriR);
	imgOri.copyTo(imgOriSol);
	cv::Mat imgMapR;
	cv::resize(imgMap, imgMapR, cv::Size(imgMap.cols/SCALE,imgMap.rows/SCALE));
	cv::imshow("imgMap", imgMapR);
	imgMap.copyTo(imgVis);
	cv::waitKey(1);
#endif
	if(VERBOSITY & 4)printf("\n");
}

void PathPlanner::createVisibilityGraph()
{
	if(VERBOSITY & 4)printf("Creating visibility graph.\n");
	
	if(VERBOSITY & 8)printf("Creating nodes vector.\n");
	Nodes.clear();

	if(VERBOSITY & 8)printf("Adding origin.\n");
	Node * tmp1 = new Node(originX,originY);
	Nodes.push_back(tmp1);

	if(VERBOSITY & 8)printf("Adding goal.\n");
	Node * tmp2 = new Node(goalX,goalY);
	Nodes.push_back(tmp2);
	
	if(VERBOSITY & 8)printf("Adding corners of the map.\n");
	Node * tmp3 = new Node(robotSize,robotSize);
	Nodes.push_back(tmp3);
	Node * tmp4 = new Node(mapW-1-robotSize,robotSize);
	Nodes.push_back(tmp4);
	Node * tmp5 = new Node(robotSize,mapH-1-robotSize);
	Nodes.push_back(tmp5);
	Node * tmp6 = new Node(mapW-1-robotSize,mapH-1-robotSize);
	Nodes.push_back(tmp6);
	
	if(VERBOSITY & 4)printf("Adding corners of the obstacles.\n");
	for(int i = 0; i < Obstacles.size(); i++)
	{
		if(VERBOSITY & 8)printf(".");
		//printf("%d	%d\n",Obstacles[i].x,Obstacles[i].y);
		Node * tmp11 = new Node(std::max(Obstacles[i].x,robotSize),std::max(Obstacles[i].y,robotSize));
		Nodes.push_back(tmp11);
		Node * tmp12 = new Node(std::min(Obstacles[i].x+Obstacles[i].w,mapW-1-robotSize),std::max(Obstacles[i].y,robotSize));
		Nodes.push_back(tmp12);
		Node * tmp13 = new Node(std::max(Obstacles[i].x,robotSize),std::min(Obstacles[i].y+Obstacles[i].h,mapH-1-robotSize));
		Nodes.push_back(tmp13);
		Node * tmp14 = new Node(std::min(Obstacles[i].x+Obstacles[i].w,mapW-1-robotSize),std::min(Obstacles[i].y+Obstacles[i].h,mapH-1-robotSize));
		Nodes.push_back(tmp14);
	}
	if(VERBOSITY & 8)printf("\n");
	//note: could check for duplicates nodes but since it does not really affect the algorithm, it can stay this way
	
	if(VERBOSITY & 4)printf("Updating %d nodes visibility.\n",Nodes.size());
	for(int i = 0; i < Nodes.size() - 1; i++)
	{
		if(VERBOSITY & 8)printf(".");
		for(int j = i + 1; j < Nodes.size(); j++)
		{
			//printf("%d	%d	%d	%d	%d	%d\n",i,j,Nodes[i]->x,Nodes[i]->y,Nodes[j]->x,Nodes[j]->y);
			if(isVisible(*Nodes[i], *Nodes[j]))
			{
				Nodes[i]->Visible.push_back(Nodes[j]);
				Nodes[j]->Visible.push_back(Nodes[i]);
#ifdef OPENCV
				cv::line(imgVis, cv::Point(Nodes[i]->x,Nodes[i]->y), cv::Point(Nodes[j]->x,Nodes[j]->y), cv::Scalar(0,255,0));
#endif
			}
		}
		//Nodes[i].sortVisible();
	}
	if(VERBOSITY & 8)printf("\n");
	
#ifdef OPENCV
	cv::Mat imgVisR;
	cv::resize(imgVis, imgVisR, cv::Size(mapW/SCALE,mapH/SCALE));
	cv::imshow("imgVis", imgVisR);
	imgVis.copyTo(imgSol);
	cv::waitKey(300);
#endif
	if(VERBOSITY & 4)printf("\n");
}

void PathPlanner::applyAStar()
{
#ifdef OPENCV
		cv::Mat imgVisD;
		imgVis.copyTo(imgVisD);
		cv::circle(imgVisD, cv::Point(originX, originY), 25, cv::Scalar(0,255,255), -2);
		cv::circle(imgVisD, cv::Point(goalX, goalY), 25, cv::Scalar(0,155,255), -2);
#endif
	if(VERBOSITY & 4)printf("Applying A* algorithm.\n");
	//This is an originial code. I have read several descriptions of the algorithm:
	//http://khayyam.developpez.com/articles/algo/astar/
	//http://en.wikipedia.org/wiki/A*_search_algorithm
	//http://de.wikipedia.org/wiki/A*-Algorithmus
	//http://fr.wikipedia.org/wiki/Algorithme_A*
	//the english article of wikipedia seems to be wrong about 'elseif tentative_g_score < g_score[y]',
	//other articles use f_score instead
	currentNode = Nodes[0];
	goalNode = Nodes[1];

	openList.push_back(currentNode);
	currentNode->costG = 0;
	currentNode->costH = evalDistance(currentNode, goalNode);
	currentNode->costF = currentNode->costG + currentNode->costH;
	currentNode->parent = 0;
	
	while(openList.size() != 0)
	{
		if(VERBOSITY & 8)printf("-");
		currentNode = bestNode(openList);
		
#ifdef OPENCV
		cv::circle(imgVisD, cv::Point(currentNode->x,currentNode->y), 9, cv::Scalar(255,0,0), 3);
		cv::Mat imgVisR;
		cv::resize(imgVisD, imgVisR, cv::Size(mapW/SCALE,mapH/SCALE));
		cv::imshow("imgVis", imgVisR);
		cv::waitKey(200);
#endif
		
		if(currentNode == goalNode)
			break;

		if(!removeFromList(openList, currentNode))
			return;

		closedList.push_back(currentNode);

		for(int i  = 0; i < currentNode->Visible.size(); i++)
		{
			if(VERBOSITY & 8)printf(".");
			bool tentativeIsBetter = true;
			Node * nextNode = currentNode->Visible[i];
			if(isIn(closedList, nextNode))
				continue;
		
			int tentativeCostF = currentNode->costG + evalDistance(currentNode, nextNode) + evalDistance(nextNode, goalNode);
			
			if(!isIn(openList, nextNode))
			{
				openList.push_back(nextNode);
				tentativeIsBetter = true;
			}
			else if(tentativeCostF < nextNode->costF)
			{
				tentativeIsBetter = true;
			}
			else
			{
				tentativeIsBetter = false;
			}
			
			if(tentativeIsBetter)
			{
				nextNode->parent = currentNode;
				nextNode->costG = currentNode->costG + evalDistance(currentNode, nextNode);
				nextNode->costH = evalDistance(nextNode, goalNode);
				nextNode->costF = tentativeCostF;
			}
#ifdef OPENCV
			cv::circle(imgVisD, cv::Point(nextNode->x,nextNode->y), 7, cv::Scalar(120,0,255), -1);
			cv::Mat imgVisR;
			cv::resize(imgVis, imgVisR, cv::Size(mapW/SCALE,mapH/SCALE));
			cv::imshow("imgVis", imgVisR);
			cv::waitKey(1);
#endif
		}
	}
	if(VERBOSITY & 8)printf("\n");
	if(openList.size() == 0)
	{
		if(VERBOSITY & 2)printf("No path found. :'(\n");
		pathFound = false;
	}
	else
	{
		pathFound = true;
	}
	if(VERBOSITY & 4)printf("\n");
}

std::vector<std::pair<int, int> > PathPlanner::displayResult()
{
	std::vector<std::pair<int, int> > result;
#ifdef OPENCV
		cv::Mat imgSolD;
		imgSol.copyTo(imgSolD);
		cv::Mat imgOriSolD;
		imgOriSol.copyTo(imgOriSolD);
#endif
	if(!pathFound)
	{
		if(VERBOSITY & 8)printf("No result to display.\n\n");
		return result;
	}
	if(VERBOSITY & 4)printf("Displaying result.\n");
	if(VERBOSITY & 8)printf("Waypoints from goal to start (x, y):\n");
	if(VERBOSITY & 8)printf("[%d, %d]",currentNode->x,currentNode->y);
	
	Node * nextNode = currentNode->parent;//to inverse the linked list
	Node * pastNode = 0;
	
	while(currentNode->parent)
	{
#ifdef OPENCV
		cv::line(imgSolD, cv::Point(currentNode->x,currentNode->y), cv::Point(currentNode->parent->x,currentNode->parent->y), cv::Scalar(0,0,255), 3);
		cv::line(imgOriSolD, cv::Point(currentNode->x,currentNode->y), cv::Point(currentNode->parent->x,currentNode->parent->y), cv::Scalar(0,0,255), 3);
#endif
		currentNode->parent = pastNode;
		pastNode = currentNode;
		currentNode = nextNode;
		nextNode = currentNode->parent;
		if(VERBOSITY & 8)printf("	[%d, %d]",currentNode->x,currentNode->y);
	}
	currentNode->parent = pastNode;
	if(VERBOSITY & 8)printf("\n");
	if(VERBOSITY & 8)printf("\n");
	
	if(VERBOSITY & 2)printf("Waypoints from start to goal in .wld file dimensions (x, y):\n");
	if(VERBOSITY & 2)printf("[%0.2f, %0.2f]",((float)currentNode->x)/100,((float)currentNode->y)/100);
	result.push_back(std::pair<int, int>(currentNode->x,currentNode->y));
	//note: it would be actually more convenient to set result in reverse order to use pop back in main
	while(currentNode->parent)
	{
		currentNode = currentNode->parent;
		if(VERBOSITY & 2)printf("	[%0.2f, %0.2f]",((float)currentNode->x)/100,((float)currentNode->y)/100);
		result.push_back(std::pair<int, int>(currentNode->x,currentNode->y));
	}
	if(VERBOSITY & 2)printf("\n");
	if(VERBOSITY & 2)printf("\n");
#ifdef OPENCV
	cv::Mat imgSolR;
	cv::resize(imgSolD, imgSolR, cv::Size(mapW/SCALE,mapH/SCALE));
	cv::imshow("imgSol", imgSolR);
	cv::Mat imgOriSolR;
	cv::resize(imgOriSolD, imgOriSolR, cv::Size(mapW/SCALE,mapH/SCALE));
	cv::imshow("imgOriSol", imgOriSolR);
	cv::waitKey(1);
#endif
	return result;
}

int PathPlanner::Split(std::vector<std::string>& vecteur, std::string chaine, char separateur)
{
	vecteur.clear();

	std::string::size_type stTemp = chaine.find(separateur);
	
	while(stTemp != std::string::npos)
	{
		vecteur.push_back(chaine.substr(0, stTemp));
		chaine = chaine.substr(stTemp + 1);
		stTemp = chaine.find(separateur);
	}
	
	vecteur.push_back(chaine);

	return vecteur.size();
}

inline bool PathPlanner::checkpixel(int x, int y)
{
	if(INVERSE)
	{
		if(SIGN)
		{
			//printf("1:%d	%d\n",x, y);
			return map[-mapW*x - y];//putpixel(320-y, 240+x, MYCOLOR);
		}
		else
		{
			//printf("2:%d	%d\n",x, y);
			return map[mapW*x + y];//putpixel(320+y, 240-x, MYCOLOR);
		}
	}
	else
	{
		//printf("3:%d	%d\n",x, y);
		return map[mapW*y + x];//putpixel(320+x, 240-y, MYCOLOR);
	}
}

bool PathPlanner::BHM(int x0, int y0, int x1, int y1)
{
	int twoDY = 2*(y1-y0),	twoDX = 2*(x1-x0),	DX = x1-x0;
	int incY = ((twoDY < 0) ? -1 : 1);
	twoDY = abs(twoDY);
	int p = twoDY - DX;	int x = x0, y = y0;

	//setpixel(x, y);

	while(x < x1)
	{
		++x;
		if(!(p<0))
		{
			y += incY;
			p -= twoDX;
		}
		p += twoDY;
		if(checkpixel(x, y))
			return false;
	}
	return true;
}

bool PathPlanner::isVisible(int x0, int y0, int x1, int y1)
{
	//Bresenham's line algorithm inspired by:
	//http://rooparam.blogspot.com/2009/09/bresenhams-line-drawing-algorithm.html
	//the algorithm is detailed at:
	//http://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
	INVERSE = 0;
	SIGN = 0;
	float m = 0;
	int t;
	int infinity = (x1 == x0);
	
	if(!infinity)
		m = (y1-y0) / (float)(x1-x0);
	if(m < 0)
		SIGN = 1;

	if(infinity || fabs(m) > 1) {
		INVERSE = 1;

		if(SIGN)
			{ t = -x0;  x0 = -y0;   y0 = t;
			  t = -x1;  x1 = -y1;   y1 = t; } 
		else
			{ t = x0;   x0 = y0;	y0 = t;
			  t = x1;   x1 = y1;	y1 = t; } 
	}

	if(x1<x0)
		return BHM(x1, y1, x0, y0);
	else
		return BHM(x0, y0, x1, y1);
}

bool PathPlanner::isVisible(Node n1, Node n2)
{
	//printf("%d	%d	%d	%d\n",n1.x, n1.y, n2.x, n2.y);
	return isVisible(n1.x, n1.y, n2.x, n2.y);
}

int PathPlanner::evalDistance(Node * n1, Node * n2)
{
	return (int)std::sqrt((n1->x - n2->x)*(n1->x - n2->x) + (n1->y - n2->y)*(n1->y - n2->y));
}

Node * PathPlanner::bestNode(std::vector<Node *>& list)
{
	int minD = list[0]->costF;
	Node * minN = list[0];
	for(int i = 1; i < list.size(); i++)
	{
		if(list[i]->costF < minD)
		{
			minD = list[i]->costF;
			minN = list[i];
		}
	}
	return minN;
}

bool PathPlanner::removeFromList(std::vector<Node *>& list, Node * n)
{
	for(int i = 0; i < list.size(); i++)
	{
		//printf("%p	%p\n",list[i],n);
		if(list[i] == n)
		{
			list.erase(list.begin()+i);
			return true;
		}
	}
	if(VERBOSITY & 1)printf("Error: vector to remove not found.\n");
	return false;

}

bool PathPlanner::isIn(std::vector<Node *>& list, Node * n)
{
	for(int i = 0; i < list.size(); i++)
	{
		if(list[i] == n)
		{
			return true;
		}
	}
	return false;
}
