/*
* ECE 8853 - Autonomous Control of Robotic Systems
* Project - Tidybot
* Sethu Madhav Bhattiprolu && Arnaud BERNARD
* Spring 2010
*/

#include "main.hpp"

int main(int argc, char *argv[])
{	
	init();
	
	try
	{
		PlayerClient robot("localhost");
		LaserProxy lp(&robot, 0);
		Position2dProxy pp(&robot,0);
		CameraProxy cp(&robot,0);
		LaserProxy sp(&robot,1);
		LocalizeProxy LocalP(&robot, 0);
		Position2dProxy pMCLp(&robot,1);
	
		pp.SetMotorEnable(true);
		pp.SetSpeed(0, 0);
		
		//Set the initial Position for the MCL
		double initialPose[] = {0, 0, 0};
		double covariance[]  = {0.2, 0.2, 0.27};
		LocalP.SetPose(initialPose,covariance);
	
		robot.Read();
		if(!cp.GetImageSize())
		{
			if(VERBOSITY & 1)printf("Webcam is not working.\n");
			return 1;
		}
		
		cam_width = cp.GetWidth();
		cam_height = cp.GetHeight();
		cam_depth = cp.GetDepth();
		Vect newControl;
				
		for(;;)
		{
			robot.Read(); // 10Hz by default

			//computePosition(lp, pp);

			switch(mode)
			{
			case 1://wander
				if(VERBOSITY & 8)printf("Searching for a can.\n");
				newControl = searchCan(lp, cp, pMCLp);
				break;
			case 2://follow path
				if(VERBOSITY & 8)printf("Going to storage place.\n");
				newControl = followPath(lp, pMCLp);
				break;
			case 3://grab can
				if(VERBOSITY & 8)printf("Grabbing the can.\n");
				newControl = grabCan(sp);
				break;
			case 4://put down can
				if(VERBOSITY & 8)printf("Putting down the can.\n");
				newControl = putDownCan();
				break;
			default:
				if(VERBOSITY & 1)printf("Unknown mode, exiting.\n");
				return -2;
			}
			
			if(VERBOSITY & 4)printf("\nSetting robot speed to [speed, turnrate] = [%lf	%lf]\n",newControl.rho, newControl.theta);
			pp.SetSpeed(newControl.rho, newControl.theta);
		}
	}
	catch (PlayerCc::PlayerError e)
	{
		std::cerr << e << std::endl;
		return -1;
	}
	return 0; //unreachable
}

void computePosition(LaserProxy &lp, Position2dProxy &pp)
{
	//double aX = 0, aY = 0, aYaw = 0;
	//Sethu's code?
	//pp.SetOdometry(aX, aY, aYaw);
}

Vect searchCan(LaserProxy &lp, CameraProxy &cp, Position2dProxy &pp)
{
	uint8_t *imgBuffer = new uint8_t[cam_width * cam_height * cam_depth];
	cv::Mat imgClean(cam_height,cam_width, CV_8UC3, imgBuffer);
	
	cp.GetImage(imgBuffer);
	player_pose2d position = locateCan(imgClean);
	
	if(VERBOSITY & 8)printf("\nVector			rho		theta\n");
	if(VERBOSITY & 8)printf("---------------------------------------------------\n");
	
	Vect newControl = move(vectCombine(avoidObstacles(lp), wander(pp), goToBeacon(position)), pp);

	if(!reachedBeacon)
	{
		if(VERBOSITY & 4)printf("\nSetting robot speed to [speed, turnrate] = [%lf	%lf]\n",newControl.rho, newControl.theta);
		return Vect(newControl.rho/3, newControl.theta/2);
	}
	else
	{
		if(VERBOSITY & 4)printf("\nBeacon reached!\n");
		return Vect(0.0, 0.0);
	}
}

Vect followPath(LaserProxy &lp, Position2dProxy &pp)
{
	//TODO
	return Vect(0,0);
}

Vect grabCan(LaserProxy &sp)
{
	player_pose2d position = locateCan(sp);
	if(position.pa < 0)
	{
		if(VERBOSITY & 2)printf("Cannot find the can with sonar, restart looking for it.\n");
		mode = 1;
		return Vect(0,0);
	}
	//Sethu's grabbing code (position)
	return Vect(0,0);
}

Vect putDownCan()
{
	//Sethu's dropping code
	return Vect(0,0);
}

player_pose2d locateCan(const cv::Mat &imgClean)
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
	//cv::imshow("img", img);
	
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
	//cv::imshow("contours", img);
	
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

	player_pose2d canPosition;
	canPosition.px = -1;
	canPosition.py = -1;
	canPosition.pa = -1;
	if(bestArea != -1)
	{
		cv::Scalar color = bestCoca ? cv::Scalar(0,0,255) : cv::Scalar(255,55,0);
		cv::RotatedRect ellipseBox = cv::fitEllipse(cv::Mat(bestCont));
		cv::ellipse(img, ellipseBox, color, 4, 8);
		if(VERBOSITY & 8)printf("Found a %s can.\n",bestCoca ? "Coke" : "Sprite");
		canPosition.pa = 1;
		canPosition.py = ellipseBox.center.y/cam_height;
		canPosition.px = (ellipseBox.center.x - cam_width/2)/(cam_width/2);
		
		if(ellipseBox.center.y + ellipseBox.size.height/2 > cam_height - TR_SWITCH_MODE_3)
		{
			mode = 3;
			canType = bestCoca ? 0 : 1;
			if(VERBOSITY & 2)printf("Reached the %s can, try to grab it.\n",bestCoca ? "Coke" : "Sprite");
		}
	}
	
	cv::imshow("ellipse", img);
	
	cv::waitKey(10);
	
	return canPosition;
}


player_pose2d locateCan(LaserProxy &sp)
{
	player_pose2d result;
	result.pa = -1;
	double min = sp.GetMaxRange() ;
	unsigned int minI = sp.GetCount() + 1;
	for (unsigned int i = 0; i < sp.GetCount(); i++)
	{
		//printf("%d:%f	",i, sp.GetRange(i));
		if(sp.GetRange(i) < min)
		{
			min = sp.GetRange(i);
			minI = i;
		}
	}
	if(min < sp.GetMaxRange())
	{
		double angleRad = sp.GetMinAngle() + sp.GetScanRes()*minI;// doesn't work?
		//printf("can ang	%lf	%lf %lf %lf %lf %d\n",angleRad, min, sp.GetMinAngle(), sp.GetScanRes(),sp.GetMaxRange(),minI);
		result.pa = 1;
		result.px = std::sin(angleRad)*min;
		result.py = std::cos(angleRad)*min;
	}
	if(VERBOSITY & 8)printf("Sonar sees can at	%lf	%lf\n",result.px, result.py);
	return result;
}

void init()
{
	// Get beacons from text file ./beacons.txt created by python algorithm
	/*nbBeacons = 0;
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
	nbBeacons /= 2;*/

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

	//PIDlp.GetRange(i)
	error = 0;
	integral = 0;
}

Vect avoidObstacles(LaserProxy &lp)
{
	Vect result;
	//TODO avoid very close cans

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

	if(VERBOSITY & 8)printf("avoidObstacles	%lf	%lf\n",result.rho, rtod(result.theta));
	return result;
}

Vect wander(Position2dProxy &pp)
{
	Vect result;
	int xpos = floor(pp.GetXPos());
	int ypos = floor(pp.GetYPos());
	printf("\npose:%d	%d\n\n",xpos,ypos);
	result.rho = 1.0 - 2 * wanderField[ (xpos * mapSize[1] + ypos) * 2 ];
	result.theta = (1.0 - 2 * wanderField[ (xpos * mapSize[1] + ypos) * 2 + 1]) * PI - pp.GetYaw();
	if(VERBOSITY & 8)printf("wander		%lf	%lf\n",result.rho, rtod(result.theta));
	return result;
}

Vect goToBeacon(player_pose2d position)
{
	Vect result(1.0, 0);
	
	if(position.pa > 0)
	{
		result.rho = 1.0;
		result.theta = -position.px * PI/2;
	}

	if(VERBOSITY & 8)printf("goToBeacon	%lf	%lf\n",result.rho, rtod(result.theta));
	return result;
}

Vect vectCombine(Vect avoidObstaclesV, Vect wanderV, Vect goToBeaconV)
{
	Vect result, straight(1.0, 0.0);
	if(goToBeaconV.theta > -0.01 && goToBeaconV.theta < 0.01)
	{
		result = avoidObstaclesV * 1.0 + wanderV * 0.2 + straight * 1.0;
	}
	else
	{
		result = avoidObstaclesV * 0*1.0 + wanderV *0* 0.2 + goToBeaconV * 1.0;
	if(VERBOSITY & 8)printf("vectCombine	%lf	%lf\n",result.rho, rtod(result.theta));
	}
	if(VERBOSITY & 8)printf("vectCombine	%lf	%lf\n",result.rho, rtod(result.theta));
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

	if(VERBOSITY & 16)printf("PID: %lf	%lf\n",KP * error, KI * integral);
	double error_old = error;
	error = combinedVect.theta - 0; // the vector is in relative coordinates so the actual direction of the robot is theta = 0
	integral = alpha * integral + error;
	double derivative = error - error_old;
	if(VERBOSITY & 16)printf("PID: %lf	%lf	%lf\n",KP * error, KI * integral, KD * derivative);
	
	//double command = KP * error + KI * integral + KD * derivative;
	double command = error;
	
	Vect result;
	result.rho = combinedVect.rho;
	result.theta = command; //turnrate
	result.rho = cos(result.theta) > 0 ? result.rho * cos(result.theta) : 0;
	if(VERBOSITY & 8)printf("move		%lf	%lf\n",result.rho, rtod(result.theta));
	return result;
}
