# Task MP.1

Your first task is to set up the loading procedure for the images, which is currently not optimal. In the student version of the code, we push all images into a vector inside a for-loop and with every new image, the data structure grows. Now imagine you want to process a large image sequence with several thousand images and Lidar point clouds over night - in the current implementation this would push the memory of your computer to its limit and eventually slow down the entire program. So in order to prevent this, we only want to hold a certain number of images in memory so that when a new one arrives, the oldest one is deleted from one end of the vector and the new one is added to the other end. The following figure illustrates the principle.


![RingBuffer](https://user-images.githubusercontent.com/51704629/66124679-02184e00-e620-11e9-8213-111e291e51a2.png)

### Modified file: MidTermProject_Camera_Student.cpp
```c++
// ...modified start: MP.1 Data Buffer Optimization
// dataBufferSize: no. of images which are held in memory (ring buffer) at the same time
int dataBufferSize = 3; // default, original code: int dataBufferSize = 2;
// ...modified end: MP.1 Data Buffer Optimization

  ...
  
// ...add start: MP.1 Data Buffer Optimization
if (dataBuffer.size() + 1 > dataBufferSize)
{
dataBuffer.erase(dataBuffer.begin());
cout << "REPLACE IMAGE IN BUFFER done" << endl;
}
// ...add end: MP.1 Data Buffer Optimization
```


# TASK MP.2

Your second task is to focus on keypoint detection. In the student version of the code you will find an existing implementation of the Shi-Tomasi detector. Please implement a selection of alternative detectors, which are HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT. The following figure shows keypoints detected with the SIFT method.


![image](https://user-images.githubusercontent.com/51704629/66124819-620ef480-e620-11e9-97a3-4be7e5b0820d.png)


### Modified file: matching2D_Student.cpp

```c++
// ...add start: MP.2 Keypoint Detection
// detectorType = HARRIS
void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    double t = (double)cv::getTickCount();

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // Look for prominent corners and instantiate keypoints

    double maxOverlap = 0.0; // max. permissible overlap between two features in %, used during non-maxima suppression
    for (size_t j = 0; j < dst_norm.rows; j++)
    {
        for (size_t i = 0; i < dst_norm.cols; i++)
        {
            int response = (int)dst_norm.at<float>(j, i);
            if (response > minResponse)
            { // only store points above a threshold

                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(i, j);
                newKeyPoint.size = 2 * apertureSize;
                newKeyPoint.response = response;

                // perform non-maximum suppression (NMS) in local neighbourhood around new key point
                bool bOverlap = false;
                for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {
                    double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                    if (kptOverlap > maxOverlap)
                    {
                        bOverlap = true;
                        if (newKeyPoint.response > (*it).response)
                        {                      // if overlap is >t AND response is higher for new kpt
                            *it = newKeyPoint; // replace old key point with new one
                            break;             // quit loop over keypoints
                        }
                    }
                }
                if (!bOverlap)
                {                                     // only add new key point if no overlap has been found in previous NMS
                    keypoints.push_back(newKeyPoint); // store new keypoint in dynamic list
                }
            }
        } // eof loop over cols
    }     // eof loop over rows

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Harris corner detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Harris Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

// detectorType = FAST, BRISK, ORB, AKAZE, SIFT
void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{
    double t = (double)cv::getTickCount();
    cv::Ptr<cv::Feature2D> detector;

    if(detectorType.compare("FAST") == 0)
    {
        detector = cv::FastFeatureDetector::create();
        detector->detect(img, keypoints);
    }
    else if(detectorType.compare("BRISK") == 0)
    {
        detector = cv::BRISK::create();
        detector->detect(img, keypoints);
    }
    else if(detectorType.compare("ORB") == 0)
    {
        detector = cv::ORB::create();
        detector->detect(img, keypoints);
    }
    else if(detectorType.compare("AKAZE") == 0)
    {
        detector = cv::AKAZE::create();
        detector->detect(img, keypoints);
    }
    else if(detectorType.compare("SIFT") == 0)
    {
        detector = cv::xfeatures2d::SIFT::create();
        detector->detect(img, keypoints);
    }
    else
    {
        throw invalid_argument(detectorType + " is not a valid detectorType. Try FAST, BRISK, ORB, AKAZE, SIFT.");
    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << detectorType << " detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = detectorType + " Keypoint Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}
// ...add end: MP.2 Keypoint Detection
```

### Modified file: MidTermProject_Camera_Student.cpp


```c++
// ...add start: MP.2 Keypoint Detection
// detectorType = HARRIS
else if (detectorType.compare("HARRIS") == 0)
{
	detKeypointsHarris(keypoints, imgGray, false);
}
// Mordern detector types, detectorType = FAST, BRISK, ORB, AKAZE, SIFT
else if (detectorType.compare("FAST") == 0 ||
		detectorType.compare("BRISK") == 0 ||
		detectorType.compare("ORB") == 0 ||
		detectorType.compare("AKAZE") == 0 ||
		detectorType.compare("SIFT") == 0)
{
	detKeypointsModern(keypoints, imgGray, detectorType, false);
}
else
{
	throw invalid_argument(detectorType + " is not a valid detectorType. Try SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT,");
}
// ...add end: MP.2 Keypoints Detection
```


# TASK MP.3

In a later part of the mid-term project, you will be evaluating the various detectors and descriptors with regard to a set of performance metrics. As we are focussing on a collision detection system in this course, keypoints on the preceding vehicle are of special interest to us. Therefore, in order to enable a more targeted evaluation, we want to discard feature points that are not located on the preceding vehicle.

Your third task therefore is to remove all keypoints outside of a bounding box around the preceding vehicle. Box parameters you should use are : cx = 535, cy = 180, w = 180, h = 150.

Please note that this cropping of feature points should only be used for evaluation purposes in the mid-term project. For later sections of this course, especially in the final project, please disable keypoint cropping using this method. Also, there will be keypoints within the box that are e.g. on the road surface or on other vehicles. Please keep an eye on their number in relation to keypoints on the actual vehicle and discuss this later in the evaluation part of the mid-term project.


```c++
// ...add start: MP.3 Keypoint Removal
vector<cv::KeyPoint>::iterator keypoint;
vector<cv::KeyPoint> keypoints_roi;
// ...add end: MP.3 Keypoint Removal

if (bFocusOnVehicle)
{
	// ...add start: MP.3 Keypoint Removal
	for(keypoint = keypoints.begin(); keypoint != keypoints.end(); ++keypoint)
	{
		if (vehicleRect.contains(keypoint->pt))
		{  
			cv::KeyPoint newKeyPoint;
			newKeyPoint.pt = cv::Point2f(keypoint->pt);
			newKeyPoint.size = 1;
			keypoints_roi.push_back(newKeyPoint);
		}
	}

	keypoints =  keypoints_roi;
	cout << "IN ROI n= " << keypoints.size()<<" keypoints"<<endl;
	// ...add end: MP.3 Keypoint Removal
}
```


# TASK MP.4

Your fourth task is to implement a variety of keypoint descriptors to the already implemented BRISK method and make them selectable using the string 'descriptorType'. The methods you must integrate are BRIEF, ORB, FREAK, AKAZE and SIFT.


# TASK MP.5

Your fifth task will focus on the matching part. The current implementation uses Brute Force matching combined with Nearest-Neighbor selection. You must now add FLANN as an alternative to brute-force as well as the K-Nearest-Neighbor approach.


# TASK MP.6

As your sixth task, you will then implement the descriptor distance ratio test as a filtering method to remove bad keypoint matches.

### Modified file: MidTermProject_Camera_Student.cpp

```c++
//.. add start: MP.4 Keypoint Descriptors
string descriptorType;
if (descriptorType.compare("SIFT") == 0) 
{
descriptorType == "DES_HOG";
}
else
{
descriptorType == "DES_BINARY";
}                    
//.. add end: MP.4 Keypoint Descriptors  
```


### Modified file: matching2D_Student.cpp, MP.4: function descKeypoints / MP.5, MP.6: function matchDescriptors

```c++
// ...add start: MP.4 Keypoint Descriptors
else if(descriptorType.compare("BRIEF") == 0)
{
extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
}
else if(descriptorType.compare("ORB") == 0)
{
extractor = cv::ORB::create();
}
else if(descriptorType.compare("AKAZE") == 0)
{
extractor = cv::AKAZE::create();
}
else if(descriptorType.compare("FREAK") == 0)
{
extractor = cv::xfeatures2d::FREAK::create();
}
else if(descriptorType.compare("SIFT") == 0)
{
extractor = cv::xfeatures2d::SIFT::create();
}
else
{
throw invalid_argument( "The input method is not supported. Try BRIEF, BRISK, ORB, AKAZE, FREAK, SIFT." );
}
// ...add end: MP.4 Keypoint Descriptors
```

```c++
// ...add start: MP.5 Descriptor Matching
if (matcherType.compare("MAT_BF") == 0)
{
int normType = cv::NORM_L2;

if(descriptorType.compare("DES_BINARY") == 0)
{
    normType = cv::NORM_HAMMING;        
}        
matcher = cv::BFMatcher::create(normType, crossCheck);
cout << "BF matching cross-check=" << crossCheck;
}    
else if (matcherType.compare("MAT_FLANN") == 0)
{
// OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
if (descSource.type() != CV_32F)
{ 
    descSource.convertTo(descSource, CV_32F);
    descRef.convertTo(descRef, CV_32F);
}

matcher = cv::FlannBasedMatcher::create();              
}
// ...add end: MP.5 Descriptor Matching

// perform matching task
if (selectorType.compare("SEL_NN") == 0)
{ // nearest neighbor (best match)

matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
}
else if (selectorType.compare("SEL_KNN") == 0)
{ // k nearest neighbors (k=2)

// ...add start: MP.6 Descriptor Distance Ratio
vector<vector<cv::DMatch>> knn_matches;
matcher->knnMatch(descSource, descRef, knn_matches, 2);

double minDescDistRatio = 0.8;
for(auto it = knn_matches.begin(); it != knn_matches.end(); ++it)
{
    if( ((*it)[0].distance) < ((*it)[1].distance * minDescDistRatio) )
    {
	matches.push_back((*it)[0]);
    }                
}
cout << "# keypoints removed = " << knn_matches.size() - matches.size() << endl;
// ...add end: MP.6 Descriptor Distance Ratio
}
```

