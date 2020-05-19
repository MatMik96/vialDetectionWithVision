#ifndef functions_hpp
#define functions_hpp


#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>



class functionsClass
{
public:
	//Declaration of getPosition function
	void getPosition(cv::Mat, cv::Mat, int*);

	//Declaration of doHomography function
	cv::Mat doHomography(cv::Mat);

	cv::Mat getRealCoords(cv::Mat, float*);


	//Features we may wanna take into account
	struct contourFeatures {
		double area;
		double circularity;
		int contourIndex;
		int perimeter;
		float elongation;
	};



};
#endif functions_hpp