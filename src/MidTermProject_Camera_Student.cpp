/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    /*Data logging code used for performance measurements*/
    //std::ofstream outdata;
    //std::cout<<"Choose Detector Type from SHITOMASI/HARRIS/FAST/BRISK/ORB/AKAZE/SIFT: ";
    //string detectorType;
    //std::cin>>detectorType;

    //string descriptorType;// = "BRIEF"; // BRIEF, ORB, FREAK, AKAZE, SIFT
    //std::cout<<"Choose Descriptor Type from BRIEF, ORB, FREAK, AKAZE, SIFT: ";
    //std::cin>>descriptorType;

    //outdata.open("DET_TYPE:"+detectorType+"DES_TYPE:"+descriptorType+"_execution_time.txt");
    //outdata<<"Image_No\t"<<"keypoint_size\t"<<"neigh_size_min\t"<<"neigh_size_max\t"<<"neigh_size_mean\t"<<"neigh_size_std\n";
    //outdata<<"Image_No\t"<<"det_time\t"<<"des_time\n";
    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize
        // push image into data frame buffer
        if(dataBuffer.size()<dataBufferSize)
        {/*Insert at the beginning until buffer is full*/
        	DataFrame frame;
        	frame.cameraImg = imgGray;
        	dataBuffer.insert(dataBuffer.begin(),frame);

        }
        else
        {/*Shift all the buffers , remove the last buffer and insert the new image in the first buffer*/
			std::rotate(dataBuffer.rbegin(), dataBuffer.rbegin() + 1, dataBuffer.rend());
			dataBuffer[0].cameraImg = imgGray;

        }

        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        string detectorType = "SHITOMASI";


        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
        double t = (double)cv::getTickCount();
        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, (dataBuffer.end() - 1)->cameraImg, false);
        }
        else if(detectorType.compare("HARRIS") == 0)
        {
        	detKeypointsHarris(keypoints, (dataBuffer.end() - 1)->cameraImg, false);
        }
        else
        {
        	detKeypointsModern(keypoints, (dataBuffer.end() - 1)->cameraImg, detectorType, false);
        }
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

        /*FOllowing comment code was used for logging time */
        //outdata<<imgNumber.str()<<"\t"<<1000 * t / 1.0 << " ms" <<"\t";

        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
        	vector<cv::KeyPoint> keypoints_temp;
        	for(unsigned int idx=0;idx<keypoints.size();idx++)
        	{
        		if(vehicleRect.contains(keypoints[idx].pt)==true)
        		{
        			keypoints_temp.push_back(keypoints[idx]);

        		}
        	}
        	keypoints = keypoints_temp;

        }
        // FOllowing commented code was used for logging neighbourhood statistics
        /*std::vector<float> kpts_neigh_size;
        for(auto it = keypoints.begin();it<keypoints.end();it++)
        {
        	kpts_neigh_size.push_back(it->size);
        }
        double sum = std::accumulate(kpts_neigh_size.begin(), kpts_neigh_size.end(), 0.0);
        double mean = sum / kpts_neigh_size.size();

        std::vector<double> diff(kpts_neigh_size.size());
        std::transform(kpts_neigh_size.begin(), kpts_neigh_size.end(), diff.begin(),
                       std::bind2nd(std::minus<double>(), mean));
        double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
        double stdev = std::sqrt(sq_sum / kpts_neigh_size.size());

        float min_size = *min_element(kpts_neigh_size.begin(), kpts_neigh_size.end());
        float max_size =*max_element(kpts_neigh_size.begin(), kpts_neigh_size.end());


        outdata<< imgNumber.str() <<"\t"<<keypoints.size()<<"\t"<<min_size<<"\t"<<max_size<<"\t"<<mean<<"\t"<<stdev<<"\n";
        std::cout<<"key points size: "<<keypoints.size()<< std::endl;*/


        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;

        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        string descriptorType = "BRIEF"; // BRIEF, ORB, FREAK, AKAZE, SIFT
        t = (double)cv::getTickCount();
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        /*Following code is for logging*/
        //outdata<<t<< " ms"<<endl;
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */
            vector<cv::DMatch> matches;
            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
            string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

            // Following code is dynamically choosing the type
            /*if((descriptorType.compare("SIFT")==0)||(descriptorType.compare("SURF")==0))
            {
            	descriptorType_HOG_BIN = "DES_HOG";
            }
            else
            {
            	descriptorType_HOG_BIN = "DES_BINARY";
            }*/

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            bVis = true;
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
              //  cv::rectangle(matchImg, vehicleRect, cv::Scalar(0, 255, 0));

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
            /*Following code was used for keypoint size measurements*/
           // outdata<<imgNumber.str()<<"\t"<<(dataBuffer.end() - 1)->kptMatches.size()<<"\n";

        }

    } // eof loop over all images

    return 0;
}
