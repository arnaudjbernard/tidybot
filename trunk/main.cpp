/*
* ECE 8853 - Autonomous Control of Robotic Systems
* Project - Tidyb
* Sethu Madhav Bhattiprolu && Arnaud BERNARD
* Spring 2010
*/


#include "main.hpp"

int main(int argc, char *argv[])
{
	using namespace PlayerCc;
	
	init();
	
	try
	{
		PlayerClient robot("localhost");
		LaserProxy lp(&robot, 0);
		Position2dProxy pp(&robot,0);
		CameraProxy cp(&robot,0);
	
		double turnrate = 0.0, speed = 0.0;
		pp.SetMotorEnable(true);
		pp.SetSpeed(speed, turnrate);
	
		robot.Read();
		if(!cp.GetImageSize())
		{
			printf("Webcam is not working.\n");
			return 1;
		}
		
		cam_width = cp.GetWidth();
		cam_height = cp.GetHeight();
		cam_depth = cp.GetDepth();
		uint8_t *imgBuffer = new uint8_t[cam_width * cam_height * cam_depth];
		cv::Mat imgClean(cam_height,cam_width, CV_8UC3, imgBuffer);
		
		
		for(;;)
		{
			robot.Read(); // 10Hz by default
			cp.GetImage(imgBuffer);
			
			locateCan(imgClean);

		
			printf("\nVector			rho		theta\n");
			printf("---------------------------------------------------\n");
			//Vect combinedVect = vectCombine(avoidObstacles(lp), wander(pp), goToBeacon(pp));
			//Vect newControl = move(combinedVect, pp);		
			Vect newControl = move(vectCombine(avoidObstacles(lp), wander(pp), goToBeacon(pp)), pp);
		
			if(!reachedBeacon)
			{
				printf("\n***Setting robot speed to [speed, turnrate] = [%lf	%lf]\n",newControl.rho, newControl.theta);
				pp.SetSpeed(newControl.rho/3, newControl.theta/2);
			}
			else
			{
				printf("\n***Beacon reached!\n");
				pp.SetSpeed(0.0, 0.0);
			}
		}
	}
	catch (PlayerCc::PlayerError e)
	{
		std::cerr << e << std::endl;
		return -1;
	}
	return 0; //unreachable
}


void locateCan(const cv::Mat &imgClean)
{
	cv::Mat hsv(cam_height,cam_width, CV_8UC3);
	cv::Mat img(cam_height,cam_width, CV_8UC3);
	cv::Mat maskRed = cv::Mat::zeros(cam_height,cam_width, CV_8UC1);
	cv::Mat maskBlue = cv::Mat::zeros(cam_height,cam_width, CV_8UC1);
	cv::Mat maskS = cv::Mat::zeros(cam_height,cam_width, CV_8UC1);
	cv::Mat maskV = cv::Mat::zeros(cam_height,cam_width, CV_8UC1);
	cv::Mat mask = cv::Mat::zeros(cam_height,cam_width, CV_8UC1);
	cv::Mat maskCoca = cv::Mat::zeros(cam_height,cam_width, CV_8UC1);
	cv::Mat maskSprite = cv::Mat::zeros(cam_height,cam_width, CV_8UC1);
	
	imgClean.copyTo(img);
	cv::imshow("img", img);
	
	cv::cvtColor(img, hsv, CV_BGR2HSV);
	cv::vector<cv::Mat> planes;
	cv::split(hsv, planes);
	
	mask.setTo(cv::Scalar(0));
	
	maskS = planes[1] <= 100;//if S<100, too grey
	mask.setTo(cv::Scalar(255), maskS);
	
	maskS = planes[2] <= 60;//if V<60, too dark
	mask.setTo(cv::Scalar(255), maskV);
	
	cv::inRange(planes[0], cv::Scalar(20/2), cv::Scalar(340/2), maskRed);//red H>330 || H<30
	
	cv::inRange(planes[0], cv::Scalar(90/2), cv::Scalar(270/2), maskBlue);//blue/green 90<H<270
	cv::threshold(maskBlue, maskBlue, 120, 255, cv::THRESH_BINARY_INV);
	
	mask.copyTo(maskCoca);
	mask.copyTo(maskSprite);
	maskCoca.setTo(cv::Scalar(255), maskRed);
	maskSprite.setTo(cv::Scalar(255), maskBlue);
	
	cv::erode( maskCoca, maskCoca,cv::Mat::ones( 5,5,CV_8UC1 ) );
	cv::erode( maskSprite, maskSprite,cv::Mat::ones( 5,5,CV_8UC1 ) );
	//cv::dilate(mask, mask, cv::Mat::ones( 5,5,CV_8UC1 ));
	//cv::imshow("mask", mask);
	//planes[2].setTo(cv::Scalar(0), mask);
	//cv::merge(planes, hsv);
	//cv::cvtColor(hsv, img, CV_HSV2BGR);
	//cv::imshow("hsv", img);
	
	cv::threshold(maskCoca, maskCoca, 120, 255, cv::THRESH_BINARY_INV);
	cv::threshold(maskSprite, maskSprite, 120, 255, cv::THRESH_BINARY_INV);
	cv::vector<cv::vector<cv::Point> > contoursCoca;
	cv::findContours(maskCoca, contoursCoca, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	cv::vector<cv::vector<cv::Point> > contoursSprite;
	cv::findContours(maskSprite, contoursSprite, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	imgClean.copyTo(img);
	cv::drawContours(img, contoursCoca, -1, cv::Scalar(0,55,255), 4);
	cv::drawContours(img, contoursSprite, -1, cv::Scalar(255,155,0), 4);
	cv::imshow("contours", img);
	
	cv::vector<cv::Point> bestCont;
	double bestArea = -1;
	bool bestCoca = true;
	for(int i = 0; i < contoursCoca.size(); i++)
	{
		double area = std::abs(cv::contourArea(cv::Mat(contoursCoca[i])));
		if(area > bestArea)
		{
			bestCont = contoursCoca[i];
			bestArea = area;
		}
	}
	for(int i = 0; i < contoursSprite.size(); i++)
	{
		double area = std::abs(cv::contourArea(cv::Mat(contoursSprite[i])));
		if(area > bestArea)
		{
			bestCont = contoursSprite[i];
			bestArea = area;
			bestCoca = false;
		}
	}
	
	//cv::moments(contours[i]);
	imgClean.copyTo(img);

	if(bestArea != -1)
	{
		cv::Scalar color = bestCoca ? cv::Scalar(0,0,255) : cv::Scalar(255,55,0);
		cv::RotatedRect ellipseBox = cv::fitEllipse(cv::Mat(bestCont));
		cv::ellipse(img, ellipseBox, color, 4, 8);
	}

	cv::imshow("ellipse", img);
	
	cv::waitKey(10);
}

void setBeacon(int x, int y)
{
	
}


void init()
{
	// Get beacons from text file ./beacons.txt created by python algorithm
	nbBeacons = 0;
	std::ifstream file("beacons.txt");
	if(file)
	{
			std::string line;
			while(std::getline(file, line))
			{
				beacons[nbBeacons] = atoi(line.c_str());
				nbBeacons++;
			}
	}
	nbBeacons /= 2;

	// get origin from text file ./origin.txt created by python algorithm
	origin[0] = 0;
	origin[1] = 0;
	mapSize[0] = 22;
	mapSize[1] = 22;
	beaconRange = 1000;
	std::ifstream file2("origin.txt");
	if(file2)
	{
				std::string line;
				std::getline(file2, line);
				origin[0] = atoi(line.c_str());
				std::getline(file2, line);
				origin[1] = atoi(line.c_str());
				std::getline(file2, line);
				mapSize[0] = atoi(line.c_str());
				std::getline(file2, line);
				mapSize[1] = atoi(line.c_str());
				std::getline(file2, line);
				beaconRange = atoi(line.c_str());
	}
	
	//initialize the random field 
	srand(SRAND_NB); // initialize C random number generator, SRAND_NB will determine the field topology
	wanderField = new double[mapSize[0]*mapSize[1]*2]; // Error if the robot get out of the map, pointer out of bound
	for(int i = 0; i < mapSize[0]; i++)
	{
		for(int j = 0; j < mapSize[1]; j++)
		{
			wanderField[ (i * mapSize[1] + j) * 2 ] =((double)rand())/((double)RAND_MAX);
			wanderField[ (i * mapSize[1] + j) * 2 + 1 ] =((double)rand())/((double)RAND_MAX);
		}
	}

	//PID
	error = 0;
	integral = 0;
}

Vect avoidObstacles(LaserProxy &lp)
{
	Vect result;

	for (unsigned int i=0; i < lp.GetCount(); i+=20)
	{
		if(lp.GetRange(i) < lp.GetMaxRange())
		{
			Vect temp;
			double angleRad = lp.GetMinAngle() + lp.GetScanRes()*i;
			temp.rho = (1/(lp.GetRange(i)-0.09) - 1/(10.0-0.1))/10.0;
			temp.theta = angleRad + PI;
			result = result + temp;
		}
	}

	printf("**avoidObstacles	%lf	%lf\n",result.rho, rtod(result.theta));
	return result;
}

Vect wander(Position2dProxy &pp)
{
	Vect result;
	int xpos = floor(pp.GetXPos());
	int ypos = floor(pp.GetYPos());
	result.rho = 1.0 - 2 * wanderField[ (xpos * mapSize[1] + ypos) * 2 ];
	result.theta = (1.0 - 2 * wanderField[ (xpos * mapSize[1] + ypos) * 2 + 1]) * PI - pp.GetYaw();
	printf("**wander		%lf	%lf\n",result.rho, rtod(result.theta));
	return result;
}

Vect goToBeacon(Position2dProxy &pp)
{
	Vect result;
	reachedBeacon = false;
	for( int i = 0; i < nbBeacons; i++ )
	{
		double dx = beacons[ i * 2 ] - pp.GetXPos();
		double dy = beacons[ i * 2 + 1 ] - pp.GetYPos();
		double dist = sqrt(pow(dx,2)+pow(dy,2));
		
		if(dist <= beaconRange) // Take only the closest to avoid Buridan's ass?
		{
			Vect temp;
			temp.rho = 1.0;
			temp.theta = 2*atan(dy/(dx+sqrt(pow(dx,2)+pow(dy,2)))) - pp.GetYaw();
			result = result + temp;
			if(dist < 1.0)
			{
				reachedBeacon = true;
				error = 0; //reset PID
				integral = 0;
			}
		}
	}
	printf("**goToBeacon		%lf	%lf\n",result.rho, rtod(result.theta));
	return result;
}

Vect vectCombine(Vect avoidObstaclesV, Vect wanderV, Vect goToBeaconV)
{
	Vect result, straight(1.0, 0.0);
	result = avoidObstaclesV * 1.0 + wanderV * 0.2 + goToBeaconV * 0.3 + straight * 1.0;
	printf("**vectCombine		%lf	%lf\n",result.rho, rtod(result.theta));
	return result;
}

Vect move(Vect combinedVect, Position2dProxy &pp)
{

////PID	

	//we assume delta_t constant between each function call 
	double alpha = 1.00 - 0.02;
	double KP = 1.0;
	double KI = 0.04;
	double KD = 1.0;

	printf("*PID: %lf	%lf\n",KP * error, KI * integral);
	double error_old = error;
	error = combinedVect.theta - 0; // the vector is in relative coordinates so the actual direction of the robot is theta = 0
	integral = alpha * integral + error;
	double derivative = error - error_old;
	printf("*PID: %lf	%lf	%lf\n",KP * error, KI * integral, KD * derivative);
	
	double command = KP * error + KI * integral + KD * derivative;

	
	Vect result;
	result.rho = combinedVect.rho;
	result.theta = command; //turnrate
	result.rho = cos(result.theta) > 0 ? result.rho * cos(result.theta) : 0;
	printf("**move			%lf	%lf\n",result.rho, rtod(result.theta));
	return result;
}
