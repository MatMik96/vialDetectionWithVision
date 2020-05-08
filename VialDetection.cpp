
// VialDetection.cpp : This file contains the 'main' function. Program execution begins and ends there.

//Includes for the project <librealsense2> and <opencv> are external libs, meaning property sheet is needed
#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>

//Alot of the constants needed in the code is defined here
#define PI 3.1415926
#define horisontalNumberOfPixels 848
#define verticalNumberOfPixels 480
#define distanceToSurface 30

//Features we may wanna take into account
struct contourFeatures {
    double area;
    double circularity;
    int contourIndex;
    int perimeter;
    float elongation;
    bool hasHole;
};


//Declaration of getPosition function
void getPosition(cv::Mat, std::vector<contourFeatures>, std::vector<std::vector<cv::Point>>, int* coord);

//Declaration of doHomography function
cv::Mat doHomography(cv::Mat);

cv::Mat getRealCoords(cv::Mat, float* xyCoord);


//The try function is called a "exception handler" if you wanna read more on it feel free
//The exception handler function will try to run the code and if it cannot it will CATCH the error and print it
//in the end of the code
int main() try
{

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
   
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Start streaming with default recommended configuration
    pipe.start();

    //Name for the depth window
    const auto window_name = " Live depth Image";
    
   
    //Creates a placeholder for the depth images from the rs
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    


    //We run a while loop to keep updating the depthImage
    while (cv::waitKey(1) < 0)
    {
        // Wait for next set of frames from the camera including a synchonization
        rs2::frameset data = pipe.wait_for_frames();

        //retrieve the first depthframe and putting in the colours
        rs2::frame depth = data.get_depth_frame().apply_filter(color_map);

        //This is made to read the depth of the centroid
        rs2::depth_frame centroidDepth = data.get_depth_frame();

        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        cv::Mat depthImage(cv::Size(w, h), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

        
        //Creating Matrix to store inrange image in
        cv::Mat detectImage = cv::Mat(depthImage.size(), CV_8UC3);
        cv::Mat liveImage = depthImage;

        // inrange to seperate colours BGR
      //As the vial will always be the closest point we only need to check for the red colour between 254-255
        inRange(depthImage, cv::Scalar(0, 0, 254), cv::Scalar(0, 15, 255), detectImage);

       //  cv::imshow("vial", detectImage);
         // Creating vector<vector<point>> to hold all contours
        std::vector<std::vector<cv::Point>> contours;
        //creating vector<vec4i> to hold the hierarchy of all contours
        //it can hold the parent, first child, first previous contour and last contour
        std::vector<cv::Vec4i> contourHierarchy;

        //Creating a structuring element with ellipse form
        cv::Mat elem = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));

        //Performing morphology on the image, makes it possible to classify the vial contour
        morphologyEx(detectImage, detectImage, cv::MORPH_CLOSE, elem);
       // morphologyEx(detectImage, detectImage, cv::MORPH_OPEN, elem);


       //   cv::imshow("mor", detectImage);

          //finding the BLOBs of the image. We are only interrested in the outer counters, and we will store all of them
        findContours(detectImage, contours, contourHierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        // Used to store the features of the objects
        std::vector<contourFeatures> featVec;


        //Goes through contours one pixel at a time
        for (int i = 0; i < contours.size(); i++)
        {
            // The reason [i][3] == -1 is so we only find the outer layer of the contours
            if (contourHierarchy[i][3] == -1)
            {
                //creating a instance of contoursFeatures and storing all features in featVec
                contourFeatures A;
                A.contourIndex = i;
                A.area = contourArea(contours[i]);
                A.perimeter = arcLength(contours[i], true);
                A.circularity = (4 * 3.14 * A.area) / pow(A.perimeter, 2);
                //Printing the area and circularity as those a the two parameters best defining the vial
               //  std::cout << "Area " << A.area << "\n";
                // std::cout << "Circularity " << A.circularity << "\n";

                //Storing features in featVec
                featVec.push_back(A);
            }

        }


        //int array to hold x and y coordinate, without 0,0 as intial values the project will eventually crash(ask me way if nessasary)
        int coordinates[2] = { 0, 0 };

        //Calling getPosition with 4 arguments
        getPosition(depthImage, featVec, contours, coordinates);

        //creating to values with x and y pixel value for centroid of vial 
        int xValue = coordinates[0];
        int yValue = coordinates[1];

        // getting the depth of the specific coordinate found from the centroid
        float depthForCentroid = centroidDepth.get_distance(xValue, yValue);

        cv::Mat homographyImg;

       // if (depthImage.empty() == false) {

            //Calling function to perform homography
            homographyImg = doHomography(depthImage);
            std::cout << " med " << std::endl;
       // }
     //   else { homographyImg = depthImage;
        std::cout << " med " << std::endl;
      //  }
      //  std::cout << " med " << std::endl;
       // cv::imshow("homo2", homographyImg);

        float xyCoord[2] = { 0, 0 };

      //  if (homographyImg.empty() == false) {
            //get x and y real world coordinates for vial from homography image
            getRealCoords(homographyImg, xyCoord);
       // }
        //meters -> cm
        depthForCentroid = depthForCentroid * 100;

       //real world x coordinate
        float realWorldx = xyCoord[0];

        //real world y coordinate
        float realWorldy = xyCoord[1];

        
       
        if (depthForCentroid > 0 && realWorldx > 0 && realWorldy > 0) {
          // std::cout << "x position -- y position -- z position " << "\n";
          // std::cout << realWorldx / 10 << " cm     " << realWorldy / 10 << " cm     " << depthForCentroid << " cm" << std::endl;
        }
        else {
              std::cout << "\n" << std::endl;
        }
        
       

        // Live depthImage
        imshow("live", liveImage);
        if (homographyImg.empty() == false) {
            imshow(window_name, homographyImg);
        }
        // contour found after inrange and morphology
        //   imshow("Detect", detectImage);



    }
    //everything went well(Y)
    return EXIT_SUCCESS;
}






//This is the handler to the try block.. It will run automaticaly if there is an error
catch (const rs2::error& error)
{
    //Displays the error
    std::cerr << "RealSense error calling " << error.get_failed_function() << "(" << error.get_failed_args() << "):\n    " << error.what() << "\n";
    return EXIT_FAILURE;
}






//getPosition function.. notice that last parameters is a pointer
void getPosition(cv::Mat depthImage, std::vector<contourFeatures> featVec, std::vector<std::vector<cv::Point>> contours, int* coord) {
    

    //Creating a matrics to print the centroid
    cv::Mat centroidImage = cv::Mat(depthImage.rows, depthImage.cols, CV_8UC3);

    //Creating a new matrix  to isolate BLOBs in.. background write
    cv::Mat contourImage = cv::Mat(depthImage.rows, depthImage.cols, CV_8UC3, cv::Scalar(255, 255, 255));

    //Loops through the featVec
    for (int i = 0; i < featVec.size(); i++)
    {
        //check if a contour fullfills all parameters, if so we have 'hopefully' found the vial
        if (featVec[i].area < 800 && featVec[i].area > 200 && featVec[i].circularity > 0.50) {
            //draw contour on contoursimage in black
            drawContours(contourImage, contours, featVec[i].contourIndex, cv::Scalar(0), -1);
         //   cv::imshow("vial", contourImage);

           // std::cout << "Circularity : " << featVec[i].circularity << "\n" << "Area : " << featVec[i].area << "\n" << "perimeter : " << featVec[i].perimeter << std::endl;

            // find moments of the binary image
            cv::Moments m = cv::moments(contours[i], true);

            //Only thing i'm not sure about yet..
            cv::Point position(m.m10 / m.m00, m.m01 / m.m00);

            //Store position of centroid.. remember coord is a pointer meaning that the values of coordinates will be overwritten
            //A function gets destroyed by the destructer when returning but since we point to global variables they will not be destroyed
            coord[0] =  position.x;
            coord[1] = position.y;
           
            //show vial counter
          // imshow("contour", contourImage);
        };
    }

}




//Function to find homography and find xy in world coordinate system
cv::Mat doHomography(cv::Mat depthImage) {


    // Output image
    cv::Mat hImage;
    cv::Mat copyDepth = depthImage;

    // making new matrics to detect box BLOB
    cv::Mat boxBLOBImage = cv::Mat(depthImage.size(), CV_8UC3);

    //Thresholding BGR
    inRange(depthImage, cv::Scalar(0, 0, 250), cv::Scalar(0, 30, 255), boxBLOBImage);

    // cv::imshow("box2", boxBLOBImage);

     //Creating a structuring element with ellipse form
    cv::Mat elemForBox = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(13, 13));
    cv::Mat elemForBox2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));


    //Performing morphology on the image, makes it possible to classify the vial contour
    morphologyEx(boxBLOBImage, boxBLOBImage, cv::MORPH_OPEN, elemForBox);
    morphologyEx(boxBLOBImage, boxBLOBImage, cv::MORPH_CLOSE, elemForBox2);

   //   cv::imshow("box2", boxBLOBImage);
      

    std::vector<std::vector<cv::Point>> contoursForBox;

    //creating vector<vec4i> to hold the hierarchy of all contours
    //it can hold the parent, first child, first previous contour and last contour
    std::vector<cv::Vec4i> contourHierarchyForBox;

    //We store all contours
    findContours(boxBLOBImage, contoursForBox, contourHierarchyForBox, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    // Used to store the features of the objects
    std::vector<contourFeatures> featVecForBox;


    //Goes through contours one pixel at a time
    for (int i = 0; i < contoursForBox.size(); i++)
    {
        // The reason [i][3] == -1 is so we only find the outer layer of the contours
        if (contourHierarchyForBox[i][3] == -1)
        {
            //creating a instance of contoursFeatures and storing all features in featVec
            contourFeatures box;
            box.contourIndex = i;
            box.area = contourArea(contoursForBox[i]);
            box.perimeter = arcLength(contoursForBox[i], true);
            box.circularity = (4 * 3.14 * box.area) / pow(box.perimeter, 2);
            box.hasHole = (contourHierarchyForBox[i][2] == -1) ? false : true;

            cv::RotatedRect rect = minAreaRect(contoursForBox[i]);
            box.elongation = cv::max(rect.size.width / rect.size.height, rect.size.height / rect.size.width);
           // Printing the area and circularity as those a the two parameters best defining the vial
           // std::cout << "Area " << box.area << "\n";
            //std::cout << "Area " << box.circularity << "\n";
            //std::cout << "elongation " << box.elongation << "\n";
            //std::cout << "has hole " << box.hasHole << "\n";

            //Storing features in featVec
            featVecForBox.push_back(box);
        }

    }


    //Creates black image same size as all others
    cv::Mat contourImageForBox = cv::Mat(depthImage.size(), CV_8U, cv::Scalar(0));


    //We create a vector<point> to hold points of interest
    std::vector<cv::Point> corners;

    //Loops through the featVec
    for (int i = 0; i < featVecForBox.size(); i++)
    {
        //check if a contour fullfills all parameters, if so we have 'hopefully' found the vial
        if (featVecForBox[i].area > 1000 && featVecForBox[i].area < 2200 && featVecForBox[i].elongation > 1.4) {


            //draw contour on contoursimage in pink
            drawContours(contourImageForBox, contoursForBox, featVecForBox[i].contourIndex, cv::Scalar(255), -1);



            cv::Moments m = cv::moments(contoursForBox[i], true);

            //Only thing i'm not sure about yet..
            cv::Point position(m.m10 / m.m00, m.m01 / m.m00);

           
            corners.push_back(cv::Point(position));

            //Checking the features for the specific BLOB
          //  std::cout << "Area " << featVecForBox[i].area << "\n";
          //  std::cout << "elongation " << featVecForBox[i].elongation << "\n";
           // std::cout << "Circularity " << featVecForBox[i].circularity << "\n";
           // std::cout << "perimeter " << featVecForBox[i].perimeter << "\n";
           // cv::imshow("contourBox1", contourImageForBox);
            //Creating a structuring element with ellipse form
           
            //  cv::imshow("contourBox", contourImageForBox);

        }
    }

  //  std::cout << "cornerCoord : " << corners << std::endl;



    


     //We convert from BI to BGR
    cv::cvtColor(contourImageForBox, contourImageForBox, cv::COLOR_GRAY2BGR);


    //We check if vector holds 4 points 
    if (corners.size() == 4) {

        //Loops through points
        for (int r = 0; r < corners.size(); r++) {

            //draws circles around found points
            cv::circle(contourImageForBox, corners.at(r), 5, cv::Scalar(50, 205, 50), -1, 8);

        }
    }


    //Show image with detected corners
   // cv::imshow("newBoxContour", contourImageForBox);



    // Four corners of the plane in of the real world

    std::vector<cv::Point2f> pts_src;

    pts_src.push_back(cv::Point2f(0, 0));

    pts_src.push_back(cv::Point2f(430, 0));

    pts_src.push_back(cv::Point2f(430, 235));

    pts_src.push_back(cv::Point2f(0, 235));


    //Creating vector<point> to store points in order
    std::vector<cv::Point2f> pts_dstInOrder;

    if (corners.size() == 4) {

        cv::Point allPoints[4] = { corners[0], corners[1], corners[2], corners[3] };

        //mean of x
        float meanXForAllPoints = (allPoints[0].x + allPoints[1].x + allPoints[2].x + allPoints[3].x) / 4;

        //mean of y
        float meanYForAllPoints = (allPoints[0].y + allPoints[1].y + allPoints[2].y + allPoints[3].y) / 4;

        cv::Point meanXLeftPoints[2];

        //Initial values
        meanXLeftPoints[0].x = 1;
        meanXLeftPoints[1].x = 1;
        meanXLeftPoints[0].y = 1;
        meanXLeftPoints[1].y = 1;

        //creating int to use as increment
        int x = 0;

        cv::Point meanXRightPoints[2];

        //initial values
        meanXRightPoints[0].x = 1;
        meanXRightPoints[1].x = 1;
        meanXRightPoints[0].y = 1;
        meanXRightPoints[1].y = 1;

        //creating int to use as increment
        int x2 = 0;

        //Forloop to store left points
        for (int i = 0; i < corners.size(); i++) {
            if (allPoints[i].x < meanXForAllPoints) {

                meanXLeftPoints[x] = allPoints[i];

                ++x;
            }
        }

        //Finding new mean value for left points
        float meanYLeftPoints = (meanXLeftPoints[0].y + meanXLeftPoints[1].y) / 2;


        //Forloop to store right points
        for (int i = 0; i < corners.size(); i++) {
            if (allPoints[i].x > meanXForAllPoints) {

                meanXRightPoints[x2] = allPoints[i];

                ++x2;
            }
        }

        //Finding new mean value for right points
        float meanYRightPoints = (meanXRightPoints[0].y + meanXRightPoints[1].y) / 2;




        //Finding top left point
        for (int i = 0; i < 2; i++) {
            if (meanXLeftPoints[i].y < meanYLeftPoints) {

                pts_dstInOrder.push_back(cv::Point(meanXLeftPoints[i]));
            }
        }


        //   std::cout << "Points int order : " << pts_dstInOrder << std::endl;



        //Finding top right point
        for (int i = 0; i < 2; i++) {
            if (meanXRightPoints[i].y < meanYRightPoints) {

                pts_dstInOrder.push_back(cv::Point(meanXRightPoints[i]));
            }
        }


        //Finding buttom right point
        for (int i = 0; i < 2; i++) {
            if (meanXRightPoints[i].y > meanYRightPoints) {

                pts_dstInOrder.push_back(cv::Point(meanXRightPoints[i]));
            }
        }


        //finding buttom left point
        for (int i = 0; i < 2; i++) {
            if (meanXLeftPoints[i].y > meanYLeftPoints) {

                pts_dstInOrder.push_back(cv::Point(meanXLeftPoints[i]));
            }
        }

        std::cout << "Points int order : " << pts_dstInOrder << std::endl;



        for (int i = 0; i < pts_dstInOrder.size(); i++) {
            if (pts_dstInOrder[i].x == 1) {

                std::cout << "hej " << std::endl;
                std::cout << "size : " << depthImage.size << std::endl;
                return copyDepth;
            }
        }

        

          //calculate Homography
        cv::Mat hMatrix = findHomography(pts_dstInOrder, pts_src);

        // Warp source image to destination based on homography
        warpPerspective(depthImage, hImage, hMatrix, depthImage.size());

        cv::Rect myROI(0,0,430,235);

        cv::Mat croppedImage = hImage(myROI);
        //  std::cout << "Homography : " << hMatrix << std::endl;
       // cv::imshow("homo", croppedImage);

       
        if (hImage.empty() == false) {
            return croppedImage;
        }
    }

    std::cout << "hej2 " << std::endl;
    std::cout << "size2 : " << copyDepth.size << std::endl;
    return copyDepth;

}




cv::Mat getRealCoords(cv::Mat homographyImg, float* xyCoord) {

    cv::Mat newHomographyImg = homographyImg;

    inRange(homographyImg, cv::Scalar(0, 0, 254), cv::Scalar(0, 15, 255), newHomographyImg);

    

    //  cv::imshow("vial", newHomographyImg);

      // Creating vector<vector<point>> to hold all contours
    std::vector<std::vector<cv::Point>> contours;
    //creating vector<vec4i> to hold the hierarchy of all contours
    //it can hold the parent, first child, first previous contour and last contour
    std::vector<cv::Vec4i> contourHierarchy;

    //Creating a structuring element with ellipse form
    cv::Mat elem = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));

    //Performing morphology on the image, makes it possible to classify the vial contour
    morphologyEx(newHomographyImg, newHomographyImg, cv::MORPH_CLOSE, elem);
     morphologyEx(newHomographyImg, newHomographyImg, cv::MORPH_OPEN, elem);


     //  cv::imshow("mor", newHomographyImg);

       //finding the BLOBs of the image. We are only interrested in the outer counters, and we will store all of them
    findContours(newHomographyImg, contours, contourHierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    // Used to store the features of the objects
    std::vector<contourFeatures> featVec;


    //Goes through contours one pixel at a time
    for (int i = 0; i < contours.size(); i++)
    {
        // The reason [i][3] == -1 is so we only find the outer layer of the contours
        if (contourHierarchy[i][3] == -1)
        {
            //creating a instance of contoursFeatures and storing all features in featVec
            contourFeatures A;
            A.contourIndex = i;
            A.area = contourArea(contours[i]);
            A.perimeter = arcLength(contours[i], true);
            A.circularity = (4 * 3.14 * A.area) / pow(A.perimeter, 2);
            //Printing the area and circularity as those a the two parameters best defining the vial
           //  std::cout << "Area " << A.area << "\n";
            // std::cout << "Circularity " << A.circularity << "\n";

            //Storing features in featVec
            featVec.push_back(A);
        }

    }


    //Creating a new matrix  to isolate BLOBs in.. background write
    cv::Mat vialImage = cv::Mat(homographyImg.rows, homographyImg.cols, CV_8UC3, cv::Scalar(255, 255, 255));

    //Loops through the featVec
    for (int i = 0; i < featVec.size(); i++)
    {
        //check if a contour fullfills all parameters, if so we have 'hopefully' found the vial
        if (featVec[i].area < 500 && featVec[i].area > 100 && featVec[i].circularity > 0.75) {
            //draw contour on contoursimage in black
            drawContours(vialImage, contours, featVec[i].contourIndex, cv::Scalar(147, 20, 255), -1);
             //  cv::imshow("vial", vialImage);

            //   std::cout << "Circularity : " << featVec[i].circularity << "\n" << "Area : " << featVec[i].area << "\n" << "perimeter : " << featVec[i].perimeter << std::endl;

               // find moments of the binary image
            cv::Moments m = cv::moments(contours[i], true);

            //Only thing i'm not sure about yet..
            cv::Point position(m.m10 / m.m00, m.m01 / m.m00);

            //Store position of centroid.. remember coord is a pointer meaning that the values of coordinates will be overwritten
            //A function gets destroyed by the destructer when returning but since we point to global variables they will not be destroyed
            xyCoord[0] = position.x;
            xyCoord[1] = position.y;

            //show vial counter
          // imshow("contour", contourImage);
        };
    }



    return newHomographyImg;


}