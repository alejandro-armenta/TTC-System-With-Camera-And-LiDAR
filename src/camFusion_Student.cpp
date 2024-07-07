
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

            //NOTE: No necesita el xmax!
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
        putText(topviewImg, str1, cv::Point2f(left, bottom+50), cv::FONT_ITALIC, 1, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left, bottom+125), cv::FONT_ITALIC, 1, currColor);  
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
    //Here I need to add DMatches to boundingBox!

    //Iterate through prev and curr keyPoints, get euclidean distance, remove keypoints whose euclidean distance is too big!

    double threshold = 10.0;

    for(auto match : kptMatches)
    {
        cv::KeyPoint firstKpCurr = kptsCurr.at(match.trainIdx);
        cv::KeyPoint firstKpPrev = kptsPrev.at(match.queryIdx);

        double dist = cv::norm(firstKpCurr.pt - firstKpPrev.pt);

        if(dist < threshold && boundingBox.roi.contains(firstKpCurr.pt))
        {
            cout << "Euclidean distance witin the box: " << dist << endl;
            boundingBox.kptMatches.push_back(match);
        }
    }
    
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...

    double minDist = 100.0; // min. required distance
    // compute distance ratios between all matched keypoints
    //So here i already have all the dist ratios! not repeated!
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer keypoint loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint firstKpCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint firstKpPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = it1 + 1; it2 != kptMatches.end(); ++it2)
        { // inner keypoint loop


            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint secondKpCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint secondKpPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(secondKpCurr.pt - firstKpCurr.pt);
            double distPrev = cv::norm(secondKpPrev.pt - firstKpPrev.pt);

            cv::line(*visImg, firstKpCurr.pt, secondKpCurr.pt, cv::Scalar(1,0,0), 2);
            //cout << "distances: " << distCurr << ", " << distPrev << endl;

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                //No detecta cambios! 
                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    cout << "distRatios Size: " << distRatios.size() << endl;

    for(auto distRatio : distRatios)
    {
        //cout << distRatio << endl;
    }
    // compute camera-based TTC from distance ratios


    // TODO: STUDENT TASK (replacement for meanDistRatio)
    std::sort(distRatios.begin(), distRatios.end());

    double medianDistRatio;

    int median = floor(distRatios.size() / 2);
    if(distRatios.size() % 2 == 0)
    {
        double median_0 = distRatios[median - 1];
        double median_1 = distRatios[median];
        
        medianDistRatio = (median_0 + median_1) / 2.0;
    }
    else
    {
        medianDistRatio = distRatios[median];
    }
    
    cout << "medianDistRatio: "<< medianDistRatio << endl;
    //ttc lidar = 12.9722s
    
    double dT = 1 / frameRate;
    //TTC = -dT / (1 - meanDistRatio); // ttc = 13.0042s
    if(medianDistRatio != 1.0)
    {
        TTC = -dT / (1 - medianDistRatio); // ttc = 12.4576s
    }
    else
    {
        TTC = NAN;
    }
    

    cout << "Camera based TTC: " << TTC << endl;
    string windowName = "dist Ratios";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, *visImg);

}

std::vector<LidarPoint> SegmentObject(std::vector<LidarPoint> & pointCloud, int maxIterations, float distanceThreshold)
{
    std::unordered_set<int> inliersResult;

	srand(time(NULL));

	// For max iterations 

	for(int it = 0; it < maxIterations; ++it)
	{
		std::unordered_set<int> plane;

		// Randomly sample subset and fit line

		while(plane.size() < 3)
		{
			plane.insert(rand() % (pointCloud.size()));
		}

		auto iter = plane.begin();

		LidarPoint p0 = pointCloud[*iter];

		++iter;
		
		LidarPoint p1 = pointCloud[*iter];

		++iter;

		LidarPoint p2 = pointCloud[*iter];


		//NOTE: This is the plane.

        
		float a1 = p1.x - p0.x;
		float a2 = p1.y - p0.y;
		float a3 = p1.z - p0.z;

		float b1 = p2.x - p0.x;
		float b2 = p2.y - p0.y;
		float b3 = p2.z - p0.z;


		float i = a2*b3 - a3*b2;
		float j = a3*b1 - a1*b3;
		float k = a1*b2 - a2*b1;
        

		float A = i;
		float B = j;
		float C = k;
		float D = -(i*p0.x + j*p0.y + k*p0.z);

		for(int j = 0; j < pointCloud.size(); ++j)
		{
			//auto lineIt = line.find(i);

            //if point is within plane go to next!
			if(plane.count(j) > 0)
			{
				continue;
			}

            //Measure distance between every point and plane!
			LidarPoint p = pointCloud[j];
 
			float d = fabs(A*p.x + B*p.y + C * p.z + D) / sqrt(A*A + B*B + C*C);

			// If distance is smaller than threshold count it as inlier

			if(d < distanceThreshold)
			{
				plane.insert(j);
			}
		}

		if(plane.size() > inliersResult.size())
		{
			inliersResult = plane;
		}
	}

    std::vector<LidarPoint> objectPoints;

    for(int index = 0; index < pointCloud.size(); ++index)
    {
        if (inliersResult.find(index) == inliersResult.end())
        {
            objectPoints.push_back(pointCloud[index]);
        }
    }

    return objectPoints;
}

bool CompareLidarPointsX(LidarPoint p0, LidarPoint p1)
{
    //The first one goes first if it is less than the second!
    return (p0.x < p1.x);
}

double getMedianX(std::vector<LidarPoint> & lidarPointCloud)
{
    double medianX;

    std::sort(lidarPointCloud.begin(), lidarPointCloud.end(), CompareLidarPointsX);

    int median = floor(lidarPointCloud.size() / 2);
    if(lidarPointCloud.size() % 2 == 0)
    {
        double median_0 = lidarPointCloud[median - 1].x;
        double median_1 = lidarPointCloud[median].x;
        
        medianX = (median_0 + median_1) / 2.0;
    }
    else
    {
        medianX = lidarPointCloud[median].x;
    }

    return medianX;
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    
    //I will use the median x for all lidar points and compute ttc based on those!

    //x value index
    std::vector<LidarPoint> lidarPPrev = lidarPointsPrev;
    std::vector<LidarPoint> lidarPCurr = lidarPointsCurr;

    double d0 = getMedianX(lidarPPrev);
    double d1 = getMedianX(lidarPCurr);

    double deltaTime = 1.0 / frameRate;
    
    //Esto esta asumiendo que esta cambiando la posicion de manera constante!
    //Cuando esta cambiando de manera cuadrática!
    TTC = d1 * deltaTime / (d0 - d1);
}


std::vector<BoundingBox>::iterator getBBoxForKeypoint(std::vector<BoundingBox> &boundingBoxes, cv::KeyPoint& keyPoint)
{

    vector<vector<BoundingBox>::iterator> enclosingBoxes;
    cv::Point pt = keyPoint.pt;

    for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            //cv::Rect smallerBox;
            //smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            //smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            //smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            //smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (it2->roi.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes

        
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            return enclosingBoxes[0];
        }
        else
        {
            return boundingBoxes.end();
        }
        
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    std::multimap<int, int> matchBBoxesMMap;

    for(cv::DMatch & match : matches)
    {
        //NOTE: QueryIdx prevImage descriptor, train Idx curr image!
        auto prevKp = prevFrame.keypoints[match.queryIdx]; 
        auto currKp = currFrame.keypoints[match.trainIdx]; 
        
        std::vector<BoundingBox>::iterator prevBBox = getBBoxForKeypoint(prevFrame.boundingBoxes, prevKp);
        std::vector<BoundingBox>::iterator  currBBox = getBBoxForKeypoint(currFrame.boundingBoxes, currKp);

        //Si existen los dos BBoxes!
        if(prevBBox != prevFrame.boundingBoxes.end() && currBBox != currFrame.boundingBoxes.end())
        {
            matchBBoxesMMap.insert(std::pair<int, int>(prevBBox->boxID, currBBox->boxID));
        }
    }

        //prev,    curr
        //firstID, secondID, frequency!
    std::map<int, std::map<int, int>> idFreq;
    for(auto match : matchBBoxesMMap)
    {
        //std::string id = to_string() + to_string(match.second);
        //cout << "Match pair: " << id << endl;
        if(idFreq.find(match.first) != idFreq.end())
        {
            //add 1;
            if(idFreq[match.first].find(match.second) != idFreq[match.first].end())
            {
                idFreq[match.first][match.second] += 1;
            }
            else
            {
                idFreq[match.first][match.second] = 1;
            }
        }
        else
        {
            idFreq[match.first][match.second] = 1;
        }
    }

    //NOTE: Look up the most frequent pairs!
    //crea un mapa de idFirst con frequencia!
    
    
    
    for(auto freq : idFreq)
    {
        cout << "First ID: " << freq.first << endl;
        int biggestFreq = 0;
        for(auto secondIDFrequencies : freq.second)
        {
            cout << "\tSecond ID: " << secondIDFrequencies.first << " Frequencies: " << secondIDFrequencies.second << endl;

            if(biggestFreq < secondIDFrequencies.second)
            {
                bbBestMatches[freq.first] = secondIDFrequencies.first;
                biggestFreq = secondIDFrequencies.second;
            }
        }
    }

    cout << "BEST MATCHES" << endl;
    
    for(auto bestMatch : bbBestMatches)
    {
        cout << bestMatch.first << ", " << bestMatch.second << endl;
    }

}
