/* Plane Estimators
 * 
 * Implemented by Levente Hajder
 * hajder@inf.elte.hu
 * 01-07-2021
 */

#ifndef PLANE_ESTIMATION
#define PLANE_ESTIMATION

#include <opencv2/opencv.hpp>


#include <vector>


using namespace std;
using namespace cv;



//Special structure for method PlanePointRANSACDifferences

typedef struct RANSACDiffs{
    int inliersNum;  //number of inliers
    vector<float> distances; //point-plane distances
    vector<bool> isInliers; //vector of inlier/outlier segmenttion results; true: inlier false: outlier
} RANSACDiffs;


float* EstimatePlaneImplicit(vector<Point3f>);
float* EstimatePlaneRANSAC(vector<Point3f>,float,int);
RANSACDiffs PlanePointRANSACDifferences(vector<Point3f> pts, float* plane, float threshold);




#endif
