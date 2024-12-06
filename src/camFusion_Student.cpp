
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // find the euclidean distance of keypoint matches in the prev frame to the curr frame
    std::vector<float> distKeypointMatches;
    for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it)
    {
        float dist;
        cv::KeyPoint keypointPrevFrame, keypointCurrFrame;
        keypointPrevFrame = kptsPrev[it->queryIdx];
        keypointCurrFrame = kptsCurr[it->trainIdx];
        if (boundingBox.roi.contains(keypointCurrFrame.pt))
        {
            cv::Point2f diff = keypointCurrFrame.pt - keypointPrevFrame.pt; 
            float eucDist = cv::sqrt(diff.x * diff.x + diff.y * diff.y);
            distKeypointMatches.push_back(eucDist); 
        }
    }
    // compute the distance mean
    float distMean = std::accumulate(distance.begin(), distance.end(), 0.0) / distance.size();
    // remove outlier keypoint matches from the bounding box
    for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it)
    {
        cv::KeyPoint keypointPrevFrame, keypointCurrFrame;
        keypointPrevFrame = kptsPrev[it->queryIdx];
        keypointCurrFrame = kptsCurr[it->trainIdx];
        if(boundingBox.roi.contains(keypointCurrFrame.pt))
        {
            cv::Point2f diff = keypointCurrFrame.pt - keypointPrevFrame.pt; 
            float eucDist = cv::sqrt(diff.x * diff.x + diff.y * diff.y);
            // if distance is less than threshold
            if(eucDist < distMean * 0.75)
            {
                // add the matches and keypoints into Box data
                boundingBox.keypoints.push_back(keypointCurrFrame);
                boundingBox.kptMatches.push_back(*it);
            }
        }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // ...
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    std::multimap<int, int> bbTempMatchesMap;
    // Loop over all the keypoint match pairs
    for (auto matchIt = matches.begin(); matchIt != matches.end(); ++matchIt)
    {
        int prevImgBoxID = -1;
        int currImgBoxID = -1;

        // Loop through all bounding boxes in previous image and find the box ID of the 'query' keypoint
        for (auto bbit = prevFrame.boundingBoxes.begin(); bbit != prevFrame.boundingBoxes.end(); ++bbit)
        {
            cv::KeyPoint keyPt;
            keyPt = prevFrame.keypoints[matchIt->queryIdx];
            if (bbit->roi.contains(keyPt.pt))
            {
                bbit->keypoints.push_back(keyPt);
                prevImgBoxID = bbit->boxID;
                break;
            }
        }

        // Loop through all bounding boxes in current image and find the box ID of the 'train' keypoint
        for (auto bbit = currFrame.boundingBoxes.begin(); bbit != currFrame.boundingBoxes.end(); ++bbit)
        {
            cv::KeyPoint keyPt;
            keyPt = currFrame.keypoints[matchIt->trainIdx];
            if (bbit->roi.contains(keyPt.pt))
            {
                bbit->keypoints.push_back(keyPt);
                currImgBoxID = bbit->boxID;
                break;
            }
        }

        // Save the box ID pairs into bbTempMatchesMap, only consider if keypoint match pairs appear in both current and previous data frame
        if ((prevImgBoxID != -1) && (currImgBoxID != -1))
        {
            bbTempMatchesMap.insert(std::make_pair(prevImgBoxID, currImgBoxID));
        }

    }

    // find the unique key from the bbTempMatchesMap
    std::set<int> uniqueKeys;
    int lastKey = INT_MIN; 

    for (auto bbit = bbTempMatchesMap.begin(); bbit != bbTempMatchesMap.end(); ++bbit)
    {
        if (bbit->first != lastKey)
        {
            uniqueKeys.insert(bbit->first);
            lastKey = bbit->first;
        }
    }

    // Find the unique key from the bbTempMatchesMap
    // And create a map to count the occurrences of each key-value pair
    std::set<int> uniqueKeys;
    int lastKey = INT_MIN; 
    std::map<std::pair<int, int>, int> mapOccurences;
    for (auto bbit = bbTempMatchesMap.begin(); bbit != bbTempMatchesMap.end(); ++bbit)
    {
        // Check the unique key
        if (bbit->first != lastKey)
        {
            uniqueKeys.insert(bbit->first);
            lastKey = bbit->first;
        }
        // Make pair
        std::pair<int, int> keyValPair = std::make_pair(bbit->first, bbit->second);
        // Check if the pair already exists in count_map
        if (mapOccurences.find(keyValPair) == mapOccurences.end())
        {
            // If not, then add into the map
            mapOccurences.insert(std::make_pair(keyValPair, 1));
        }
        else
        {
            // If found, then increase the count
            mapOccurences[keyValPair]++;
        }
    }

    // Iterate through each unique bounding box IDs in the previous image, to find the one with highest occurences
    for (auto ukit = uniqueKeys.begin(); ukit != uniqueKeys.end(); ++ukit)
    {
        int bbIdx1 = -1; // bounding box index
        int bbIdx2 = -1;
        int maxKeypointCount = INT_MIN;
    
        for (auto mapit = mapOccurences.begin(); mapit != mapOccurences.end(); ++mapit)
        {
            int currKeypointCount = mapit->second;

            if (mapit->first.first == *ukit)
            {
                if (currKeypointCount >= maxKeypointCount)
                {
                    maxKeypointCount = currKeypointCount;
                    bbIdx1 = mapit->first.first;
                    bbIdx2 = mapit->first.second;
                }
            }
        }

        if ((bbIdx1 != -1) && (bbIdx2 != -1))
        {
            bbBestMatches.insert(std::make_pair(bbIdx1, bbIdx2));
        }
    }
}
