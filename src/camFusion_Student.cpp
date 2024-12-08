
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;

extern vector<double> ttcLidar;
extern vector<double> ttcLidarLPF;
extern vector<double> ttcCamera;
extern vector<double> ttcCameraLPF;

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



static float percentile(const std::vector<float> &dataIn, float p)
{
    std::vector<float> data = dataIn;
    std::sort(data.begin(), data.end());

    int N = data.size();
    float n = (N - 1) * p + 1;

    // If n is an integer, then percentile is a data point
    if (n == floor(n))
    {
        return data[n - 1];
    }
    else
    {
        int k = floor(n);
        float d = n - k;
        return data[k - 1] + d * (data[k] - data[k - 1]);
    }
}

// This function is to remove the outlier based on interquartile range
static std::vector<float> removeOutliers(const std::vector<float> &data)
{
    std::vector<float> sorted_data = data;
    std::sort(sorted_data.begin(), sorted_data.end());

    float q1 = percentile(sorted_data, 0.35f);
    float q3 = percentile(sorted_data, 0.65f);
    float iqr = q3 - q1;

    std::vector<float> filtered_data;
    for (float x : data)
    {
        if (x >= q1 - 1.5 * iqr && x <= q3 + 1.5 * iqr)
        {
            filtered_data.push_back(x);
        }
    }

    return filtered_data;
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // find the euclidean distance of keypoint matches in the prev frame to the curr frame
    std::vector<float> distKeypointMatches;
    for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it)
    {
        cv::KeyPoint keypointPrevFrame, keypointCurrFrame;
        keypointPrevFrame = kptsPrev[it->queryIdx];
        keypointCurrFrame = kptsCurr[it->trainIdx];
        if (boundingBox.roi.contains(keypointCurrFrame.pt))
        {
            cv::Point2f diff = keypointCurrFrame.pt - keypointPrevFrame.pt; 
            float eucDist = cv::sqrt(diff.x * diff.x + diff.y * diff.y);
            distKeypointMatches.push_back(eucDist); 
            boundingBox.kptMatches.push_back(*it);
        }
    }
    // Compute q1, q3 and interquartile range
    float q1 = percentile(distKeypointMatches, 0.35f);
    float q3 = percentile(distKeypointMatches, 0.65f);
    float iqr = q3 - q1;

    // Iterate through the matched keypoint pairs in the current bounding box and remove the ones which are outliers from the bounding box
    auto bbit = boundingBox.kptMatches.begin();
    while (bbit != boundingBox.kptMatches.end())
    {
        cv::KeyPoint keypointPrevFrame, keypointCurrFrame;
        keypointPrevFrame = kptsPrev[bbit->queryIdx];
        keypointCurrFrame = kptsCurr[bbit->trainIdx];
        cv::Point2f diff = keypointCurrFrame.pt - keypointPrevFrame.pt; 
        float eucDist = cv::sqrt(diff.x * diff.x + diff.y * diff.y);
        if ((eucDist < (q1 - 1.5 * iqr)) || (eucDist > (q3 + 1.5 * iqr)))
        {
            // erase() returns the next element
            bbit = boundingBox.kptMatches.erase(bbit);
        }
        else
        {
            ++bbit;
        }
    }
    cout << "Debug: q1: " << q1 << " - q3: " << q3 << " - iqr: " << iqr << endl; 
    cout << "Debug: kptMatches size: " << boundingBox.kptMatches.size() << endl;     
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{   
    // compute distance ratio between all matches keypoints
    vector<float> vDistRatios; 
    for (auto kpit1 = kptMatches.begin(); kpit1 != kptMatches.end() - 1; ++kpit1)
    {
        cv::KeyPoint keypointOuterCurr = kptsCurr.at(kpit1->trainIdx);
        cv::KeyPoint keypointOuterPrev = kptsPrev.at(kpit1->queryIdx);

        for (auto kpit2 = kptMatches.begin() + 1; kpit2 != kptMatches.end(); ++kpit2)
        {
            double minDist = 100.0; // min. required distance in pixels

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint keypointInnerCurr = kptsCurr.at(kpit2->trainIdx);
            cv::KeyPoint keypointInnerPrev = kptsPrev.at(kpit2->queryIdx);

            // compute distances and distance ratios
            cv::Point2f diff = keypointOuterCurr.pt - keypointInnerCurr.pt;
            float distCurr = cv::sqrt(diff.x * diff.x + diff.y * diff.y);
            diff = keypointOuterPrev.pt - keypointInnerPrev.pt;
            float distPrev = cv::sqrt(diff.x * diff.x + diff.y * diff.y);

            if (distPrev > std::numeric_limits<float>::epsilon() && distCurr >= minDist)
            {
                float distRatio = distCurr / distPrev;
                vDistRatios.push_back(distRatio);
            }
        }
    }
    // check if vDistRatios size is 0
    if (vDistRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // compute median dist. ratio to remove outlier influence
    std::sort(vDistRatios.begin(), vDistRatios.end());
    int medIndex = floor(vDistRatios.size() / 2.0);
    float medDistRatio = vDistRatios.size() % 2 == 0 ? (vDistRatios[medIndex - 1] + vDistRatios[medIndex]) / 2.0 : vDistRatios[medIndex]; 

    // compute camera-based TTC from distance ratios
    float meanDistRatio = std::accumulate(vDistRatios.begin(), vDistRatios.end(), 0.0) / vDistRatios.size();
    float ttcCamCurr = -1 / frameRate / (1 - meanDistRatio);
    float betaCamCoef = 0.15; 
    static double ttcCamPrev = 0; 
    static bool lpfInit = true; 
    if (lpfInit)
    {
        TTC = ttcCamCurr;
        lpfInit = false;
    }
    else 
    {
        TTC = (1 - betaCamCoef) * ttcCamPrev + betaCamCoef * ttcCamCurr; 
    }
    cout << "Debug: meanDistRatio: " << meanDistRatio << " - TTC Cam: " << TTC << endl; 
    cout << "Debug: ttcCamPrev: " << ttcCamPrev << " - ttcCamCurr: " << ttcCamCurr << endl; 
    ttcCamPrev = ttcCamCurr;
    ttcCamera.push_back(ttcCamCurr);
    ttcCameraLPF.push_back(TTC);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    std::vector<float> lidarPointXPrev;
    std::vector<float> lidarPointXCurr;
    std::vector<float> filteredLidarPointXPrev;
    std::vector<float> filteredLidarPointXCurr;
    static double ttcLidarPrev = 0; 

    // copy lidar point --> X
    for (auto ldit = lidarPointsPrev.begin(); ldit != lidarPointsPrev.end(); ++ldit)
    {
        lidarPointXPrev.push_back(ldit->x);
    }
    for (auto ldit = lidarPointsCurr.begin(); ldit != lidarPointsCurr.end(); ++ldit)
    {
        lidarPointXCurr.push_back(ldit->x);
    }
    // remove the outliers
    filteredLidarPointXPrev = removeOutliers(lidarPointXPrev);
    filteredLidarPointXCurr = removeOutliers(lidarPointXCurr);    
    // find the closest lidar point
    float d0 = INT_MAX; 
    float d1 = INT_MAX; 
    for (auto ldit = filteredLidarPointXPrev.begin(); ldit < filteredLidarPointXPrev.end(); ldit++)
    {
        d0 = (d0 > *ldit)? (*ldit) : d0;
    }
    for (auto ldit = filteredLidarPointXCurr.begin(); ldit < filteredLidarPointXCurr.end(); ldit++)
    {
        d1 = (d1 > *ldit)? (*ldit) : d1;
    }
    // compute TTC
    double ttcLidarCurr = d1 / frameRate / (d0 - d1);
    double betaCoef = 0.25;
    static bool lpfInit = true; 
    if (lpfInit)
    {
        TTC = ttcLidarCurr;
        lpfInit = false;
    }
    else 
    {
        TTC = (1 - betaCoef) * ttcLidarPrev + betaCoef * ttcLidarCurr;
    }

    cout << "Debug: d1: " << d1 << " - d0: " << d0 << " - TTC Lidar: " << TTC << endl; 
    cout << "Debug: ttcLidarPrev: " << ttcLidarPrev << " - ttcLidarCurr: " << ttcLidarCurr << endl; 
    ttcLidarPrev = ttcLidarCurr;
    ttcLidar.push_back(ttcLidarCurr);
    ttcLidarLPF.push_back(TTC);
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
    cout << "Debug: bounding box bestMatches size: " << bbBestMatches.size() << endl; 
}
