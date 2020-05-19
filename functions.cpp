#include "functions.hpp"


void functionsClass::getPosition(cv::Mat depthImage, cv::Mat detectImage, int* coord) {
    // inrange to seperate colours BGR
    //As the vial will always be the closest point we only need to check for the red colour between 254-255
    inRange(depthImage, cv::Scalar(0, 0, 254), cv::Scalar(0, 15, 255), detectImage);

    // cv::imshow("vial", detectImage);
     // Creating vector<vector<point>> to hold all contours
    std::vector<std::vector<cv::Point>> contours;
    //creating vector<vec4i> to hold the hierarchy of all contours
    //it can hold the parent, first child, first previous contour and last contour
    std::vector<cv::Vec4i> contourHierarchy;

    //Creating a structuring element with ellipse form
    cv::Mat elem = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));

    //Performing morphology on the image, makes it possible to classify the vial contour
    morphologyEx(detectImage, detectImage, cv::MORPH_OPEN, elem);
    // morphologyEx(detectImage, detectImage, cv::MORPH_OPEN, elem);


     // cv::imshow("mor", detectImage);

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


    //Creating a matrics to print the centroid
    cv::Mat centroidImage = cv::Mat(depthImage.rows, depthImage.cols, CV_8UC3);

    //Creating a new matrix  to isolate BLOBs in.. background write
    cv::Mat contourImage = cv::Mat(depthImage.rows, depthImage.cols, CV_8UC3, cv::Scalar(0, 0, 0));

    //Loops through the featVec
    for (int i = 0; i < featVec.size(); i++)
    {
        //check if a contour fullfills all parameters, if so we have 'hopefully' found the vial
        if (featVec[i].area < 400 && featVec[i].area > 200 && featVec[i].circularity > 0.75 && contours[i].size() < 55 && contours[i].size() > 47) {


            //draw contour on contoursimage in black
            drawContours(contourImage, contours, featVec[i].contourIndex, cv::Scalar(0, 255, 128), -1);

            //  cv::imshow("vial", contourImage);

            //  std::cout << "Circularity : " << featVec[i].circularity << "\n" << "Area : " << featVec[i].area << "\n" << "perimeter : " << featVec[i].perimeter << std::endl;

              // find moments of the binary image
            cv::Moments m = cv::moments(contours[i], true);

            //Only thing i'm not sure about yet..
            cv::Point position(m.m10 / m.m00, m.m01 / m.m00);

            //Store position of centroid.. remember coord is a pointer meaning that the values of coordinates will be overwritten
            //A function gets destroyed by the destructer when returning but since we point to global variables they will not be destroyed
            coord[0] = position.x;
            coord[1] = position.y;

            //show vial counter
          // imshow("contour", contourImage);
        };
    }

}


cv::Mat functionsClass::doHomography(cv::Mat depthImage) {


    // Output image
    cv::Mat hImage;
    cv::Mat EmptyImg;


    // making new matrics to detect box BLOB
    cv::Mat boxBLOBImage = cv::Mat(depthImage.size(), CV_8UC3);

    //Thresholding BGR
    inRange(depthImage, cv::Scalar(0, 0, 250), cv::Scalar(0, 25, 255), boxBLOBImage);

    // cv::imshow("box1", boxBLOBImage);

      //Creating a structuring element with ellipse form
    cv::Mat elemForBox = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11));
    cv::Mat elemForBox2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));


    //Performing morphology on the image, makes it possible to classify the vial contour
    morphologyEx(boxBLOBImage, boxBLOBImage, cv::MORPH_OPEN, elemForBox);
    morphologyEx(boxBLOBImage, boxBLOBImage, cv::MORPH_CLOSE, elemForBox2);

    //  cv::imshow("box2", boxBLOBImage);


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
        if (featVecForBox[i].area > 500 && featVecForBox[i].area < 1100 && featVecForBox[i].elongation > 1 && featVecForBox[i].elongation < 2.3 && featVecForBox[i].circularity > 0.55 && featVecForBox[i].circularity < 0.85) {


            //draw contour on contours image
            drawContours(contourImageForBox, contoursForBox, featVecForBox[i].contourIndex, cv::Scalar(255), -1);



            cv::Moments m = cv::moments(contoursForBox[i], true);

            //Only thing i'm not sure about yet..
            cv::Point position(m.m10 / m.m00, m.m01 / m.m00);


            corners.push_back(cv::Point(position));

            //Checking the features for the specific BLOB
           // std::cout << "Area " << featVecForBox[i].area << "\n";
            //std::cout << "elongation " << featVecForBox[i].elongation << "\n";
            //std::cout << "Circularity " << featVecForBox[i].circularity << "\n";
           // std::cout << "perimeter " << featVecForBox[i].perimeter << "\n";
            //cv::imshow("contourBox1", contourImageForBox);


          //   cv::imshow("contourBox", contourImageForBox);

        }
    }


      //We convert from BI to BGR
    cv::cvtColor(contourImageForBox, contourImageForBox, cv::COLOR_GRAY2BGR);



    //We check if vector holds 4 points 
    if (corners.size() == 4) {

        //Loops through points
        for (int r = 0; r < corners.size(); r++) {

              //draws circles around found points
            cv::circle(contourImageForBox, corners.at(r), 5, cv::Scalar(50, 205, 50), -1, 8);

        }


        cv::imshow("newBoxContour", contourImageForBox);

        // Four corners of the plane in of the real world

        std::vector<cv::Point2f> pts_src;

        pts_src.push_back(cv::Point2f(0, 0));

        pts_src.push_back(cv::Point2f(430, 0));

        pts_src.push_back(cv::Point2f(430, 235));

        pts_src.push_back(cv::Point2f(0, 235));




        //Creating vector<point> to store points in order
        std::vector<cv::Point2f> pts_dstInOrder;

        cv::Point allPoints[4] = { corners[0], corners[1], corners[2], corners[3] };

        //mean of x
        float meanXForAllPoints = (allPoints[0].x + allPoints[1].x + allPoints[2].x + allPoints[3].x) / 4;

        //mean of y
        float meanYForAllPoints = (allPoints[0].y + allPoints[1].y + allPoints[2].y + allPoints[3].y) / 4;




        //Forloop to store left points
        for (int i = 0; i < corners.size(); i++) {
            if (allPoints[i].x < meanXForAllPoints && allPoints[i].y < meanYForAllPoints) {

                pts_dstInOrder.push_back(cv::Point(allPoints[i]));
            }

        }

        

        //Forloop to store left points
        for (int i = 0; i < corners.size(); i++) {
            if (allPoints[i].x > meanXForAllPoints && allPoints[i].y < meanYForAllPoints) {

                pts_dstInOrder.push_back(cv::Point(allPoints[i]));
            }

        }

        

        //Forloop to store left points
        for (int i = 0; i < corners.size(); i++) {
            if (allPoints[i].x > meanXForAllPoints && allPoints[i].y > meanYForAllPoints) {

                pts_dstInOrder.push_back(cv::Point(allPoints[i]));
            }

        }

        
        //Forloop to store left points
        for (int i = 0; i < corners.size(); i++) {
            if (allPoints[i].x < meanXForAllPoints && allPoints[i].y > meanYForAllPoints) {

                pts_dstInOrder.push_back(cv::Point(allPoints[i]));
            }

        }



        if (pts_dstInOrder.size() == 4) {

            //calculate Homography
            cv::Mat hMatrix = findHomography(pts_dstInOrder, pts_src);

            // Warp source image to destination based on homography
            warpPerspective(depthImage, hImage, hMatrix, depthImage.size());

            cv::Rect myROI(0, 0, 430, 235);

            cv::Mat croppedImage = hImage(myROI);
            //  std::cout << "Homography : " << hMatrix << std::endl;

            double x = hMatrix.at<double>(cv::Point(2, 0));
            double y = hMatrix.at<double>(cv::Point(2, 1));

            // cv::imshow("homo", croppedImage);

            if (x < -500 || y > 0) {
               
                return EmptyImg;

            }

            else
            {
                
                return croppedImage;
            }
        }
    }

    return depthImage;
}


cv::Mat functionsClass::getRealCoords(cv::Mat homographyImg, float* xyCoord) {

    cv::Mat newHomographyImg = homographyImg;

    inRange(homographyImg, cv::Scalar(0, 0, 254), cv::Scalar(0, 8, 255), newHomographyImg);

    //  cv::imshow("vial2", newHomographyImg);

      // Creating vector<vector<point>> to hold all contours
    std::vector<std::vector<cv::Point>> contours;

    //creating vector<vec4i> to hold the hierarchy of all contours
    //it can hold the parent, first child, first previous contour and last contour
    std::vector<cv::Vec4i> contourHierarchy;

    //Creating a structuring element with ellipse form
    cv::Mat elem = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));

    //Performing morphology on the image, makes it possible to classify the vial contour
   // morphologyEx(newHomographyImg, newHomographyImg, cv::MORPH_CLOSE, elem);
    morphologyEx(newHomographyImg, newHomographyImg, cv::MORPH_OPEN, elem);

    // morphologyEx(newHomographyImg, newHomographyImg, cv::MORPH_CLOSE, elem);

    cv::imshow("mor", newHomographyImg);

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

    cv::Mat test;
    homographyImg.copyTo(test);


    //Loops through the featVec
    for (int i = 0; i < featVec.size(); i++)
    {
        //check if a contour fullfills all parameters, if so we have 'hopefully' found the vial
        if (featVec[i].area < 400 && featVec[i].area > 200 && featVec[i].circularity > 0.70) {
            //draw contour on contoursimage in black
            drawContours(vialImage, contours, featVec[i].contourIndex, cv::Scalar(147, 20, 255), -1);


            //  std::cout << "Circularity : " << featVec[i].circularity << "\n" << "Area : " << featVec[i].area << "\n" << "perimeter : " << featVec[i].perimeter << std::endl;

            cv::Rect bb = cv::boundingRect(contours[i]);

            cv::rectangle(vialImage, bb, cv::Scalar(0, 0, 0));
            // std::cout << "size : " << vialImage.size() << "\n";


            int xMin = 5;
            int yMin = 5;
            int widthmax = 430 - 5;
            int heightmax = 235 - 5;

            bool vial = false;


            if (bb.x <= xMin || bb.y <= yMin || (bb.x + bb.width) >= widthmax || (bb.y + bb.height) >= heightmax)
            {
                vial = false;
            }

            else  vial = true;





            cv::imshow("vial", vialImage);

            if (vial == true) {
                // find moments of the binary image
                cv::Moments m = cv::moments(contours[i], true);

                //Only thing i'm not sure about yet..
                cv::Point position(m.m10 / m.m00, m.m01 / m.m00);

                cv::circle(test, position, 5, cv::Scalar(123, 45, 67), -1);
                cv::imshow("test", test);
                //Store position of centroid.. remember coord is a pointer meaning that the values of coordinates will be overwritten
                //A function gets destroyed by the destructer when returning but since we point to global variables they will not be destroyed
                xyCoord[0] = position.x;
                xyCoord[1] = position.y;
            }

        }

    }

    return newHomographyImg;
}