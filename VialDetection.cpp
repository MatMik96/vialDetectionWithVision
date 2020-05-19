
// VialDetection.cpp : This file contains the 'main' function. Program execution begins and ends there.

//Includes for the project <librealsense2> and <opencv> are external libs, meaning property sheet is needed
#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include "functions.hpp"

//Creating instance of the class functions
functionsClass func;

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
        cv::Mat depthImage(cv::Size(w, h), CV_8UC3, (void*)depth.get_data());

        
        //Creating Matrix to store inrange image in
        cv::Mat detectImage = cv::Mat(depthImage.size(), CV_8UC3);

        //Creating copy of depthImage to display in the end
        cv::Mat liveImage = depthImage;


        //int array to hold x and y coordinate, without 0,0 as intial values the project will eventually crash(ask me way if nessasary)
        int coordinates[2] = { 0, 0 };

        //Calling getPosition To find the center of the BLOB of the vial
        func.getPosition(depthImage, detectImage, coordinates);

        int xValue;
        int yValue;
        float depthForCentroid = 0;

        if (coordinates[0] != 0) {
            //creating to values with x and y pixel value for centroid of vial 
            xValue = coordinates[0];
            yValue = coordinates[1];

            // getting the depth of the specific coordinate found from the centroid.. Now we have found the Z-coordinate!
            depthForCentroid = centroidDepth.get_distance(xValue, yValue);
        }
        else;
       
        

        //creating Mat to hold homography image
        cv::Mat homographyImg = cv::Mat(depthImage.size(), CV_8UC3);

            //Calling function to perform homography
           homographyImg = func.doHomography(depthImage);

        //Creating array of floats to hold coordinates for the vial on the homography image
        float xyCoord[2] = { 0, 0 };

        //Checking if homography image is empty, as it is sometimes not possible to do the transformation and in that case an empty Mat would be store in homographyImg
        if (homographyImg.empty() == false) {

            //If it was not possible to find 4 corners in doHomography function the return image will be another size than the original, therefor we also check the size
            if (homographyImg.size() != depthImage.size()) {

                //get x and y real world coordinates for vial from homography image
                func.getRealCoords(homographyImg, xyCoord);
            }
        }
        //meters -> cm for Z
        depthForCentroid = depthForCentroid * 100;

       //real world x coordinate
        float realWorldx = xyCoord[0];

        //real world y coordinate
        float realWorldy = xyCoord[1];

      

       //CHeck of the calculations went wrong
        if (depthForCentroid > 0 && realWorldx > 0 && realWorldy > 0) {

            //Prints out positions
           std::cout << "x position -- y position -- z position " << "\n";
           std::cout << realWorldx / 10 << " cm     " << realWorldy / 10 << " cm     " << depthForCentroid << " cm " << "\n\n";

          
        }

        //else skip a line an try again
        else  std::cout << "\n" << std::endl;
        
        
       

        // Live depthImage
       // imshow("live", liveImage);
        if (homographyImg.empty() == false) {
        
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