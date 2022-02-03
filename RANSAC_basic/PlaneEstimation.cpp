/* Plane Estimators
 * 
 * Implemented by Levente Hajder
 * hajder@inf.elte.hu
 * 01-07-2021
 */

#include "PlaneEstimation.h"

/* Plane Estimation
 * Goal: Fit plane to the given spatial points
 * Plane is given in implicit form: Ax + By + Cz + D=0 
 * 
 * Input: 
 * vector<Point3f> pts: input points for plane fitting
 * 
 * Output:
 * float* an array with four elements
 * return[0]: A
 * return[1]: B
 * return[2]: C
 * return[3]: D
 * 
 * 
 */

float* EstimatePlaneImplicit(vector<Point3f> pts){
    int num=pts.size();
    
    Mat Cfs(num,4,CV_32F);
    
    
    //Get points
    for (int idx=0;idx<num;idx++){
        Point3d pt=pts.at(idx);
        Cfs.at<float>(idx,0)=pt.x;
        Cfs.at<float>(idx,1)=pt.y;
        Cfs.at<float>(idx,2)=pt.z;
        Cfs.at<float>(idx,3)=1.0f;
    }
    
    Mat mtx=Cfs.t()*Cfs;
    Mat evals, evecs;


    eigen(mtx, evals, evecs);
    
    
    
    //normalize plane normal;
    
    float A=evecs.at<float>(3,0);
    float B=evecs.at<float>(3,1);
    float C=evecs.at<float>(3,2);
    float D=evecs.at<float>(3,3);
    
    float norm=sqrt(A*A+B*B+C*C); //Plane parameters are normalized
    
    float* ret=new float[4];
        
    ret[0]=A/norm;
    ret[1]=B/norm;
    ret[2]=C/norm;
    ret[3]=D/norm;

    return ret;

    }

    
/* Plane Estimation
 * Goal: Fit plane to the given spatial points. The fitting is robust, RANSAC method is applied
 * Plane is given in implicit form: Ax + By + Cz + D=0 
 * 
 * Input: 
 * vector<Point3f> pts: input points for plane fitting
 * threshold: threshold for point-plane distabce
 * iterateNum: number of iteration within RANSAC
 * 
 * 
 * Output:
 * float* an array with four elements
 * return[0]: A
 * return[1]: B
 * return[2]: C
 * return[3]: D
 * 
 */
    

float* EstimatePlaneRANSAC(vector<Point3f> pts,float threshold,int iterateNum){
        int num=pts.size();
        
        int bestSampleInlierNum=0;
        float bestPlane[4];
        
    for(int iter=0;iter<iterateNum;iter++){
        
        
            float rand1=(float)(rand())/RAND_MAX;
            float rand2=(float)(rand())/RAND_MAX;
            float rand3=(float)(rand())/RAND_MAX;
                
                
            //Generate three different(!) random numbers:
            int index1=(int)(rand1*num);
            int index2=(int)(rand2*num);
            while (index2==index1) {rand2=(float)(rand())/RAND_MAX; index2=(int)(rand2*num);}
            int index3=(int)(rand3*num);
            while ((index3==index1)||(index3==index2)) {rand3=(float)(rand())/RAND_MAX; index3=(int)(rand3*num);}
            

            
            Point3f pt1=pts.at(index1);
            Point3f pt2=pts.at(index2);
            Point3f pt3=pts.at(index3);
            
            //In each RANSAC cycle, a minimal sample with 3 points are formed
            
            vector<Point3f> minimalSample;
            
            minimalSample.push_back(pt1);
            minimalSample.push_back(pt2);
            minimalSample.push_back(pt3);
            
            float* samplePlane=EstimatePlaneImplicit(minimalSample);
            
//            printf("Plane params: %f %f %f %f \n",samplePlane[0],samplePlane[1],samplePlane[2],samplePlane[3]);
            
            //Compute consensus set
            
            RANSACDiffs sampleResult=PlanePointRANSACDifferences(pts, samplePlane, threshold);
            
//            printf("NumInliers: %d \n",sampleResult.inliersNum);
            
            //Check the new test is larger than the best one.
            
            if (sampleResult.inliersNum>bestSampleInlierNum){
                bestSampleInlierNum=sampleResult.inliersNum;
                bestPlane[0]=samplePlane[0];
                bestPlane[1]=samplePlane[1];
                bestPlane[2]=samplePlane[2];
                bestPlane[3]=samplePlane[3];
            }//end if
            
            delete[] samplePlane;
            
    }//end for iter
    
    //Finally, the plane is refitted from thew best consensus set
    RANSACDiffs bestResult=PlanePointRANSACDifferences(pts, bestPlane, threshold);
    
    
    vector<Point3f> inlierPts;
    
    
    for (int idx=0;idx<num;idx++){
        if (bestResult.isInliers.at(idx)){
            inlierPts.push_back(pts.at(idx));
        }
    }
    
    float* finalPlane=EstimatePlaneImplicit(inlierPts);

    return finalPlane;

    }



    
/* Plane-point differences
 * Goal: This method calculates the plane point differences, and determines if a point is an outlier.
 * Plane is given in implicit form: Ax + By + Cz + D=0 
 * Plane parameters are normalized: A^2 + B^2 + C^2 =1
 * 
 * Input: 
 * vector<Point3f> pts: input points
 * float plane: plane parameters; plane[0]:A, plane[1]:B, plane[2]:C, plane[3]:D
 * float threshold: the threshold for inlier/outlier separation
 * 
 * Output:
 * RANSACDiffs 
 * see header file for details
 * 
 * 
 */
    
    
RANSACDiffs PlanePointRANSACDifferences(vector<Point3f> pts, float* plane, float threshold){
        int num=pts.size();
        
        float A=plane[0];
        float B=plane[1];
        float C=plane[2];
        float D=plane[3];
        

        RANSACDiffs ret;
        
        vector<bool> isInliers;
        vector<float> distances;
        
        int inlierCounter=0;
        for (int idx=0;idx<num;idx++){
            Point3f pt=pts.at(idx);
            float diff=fabs(A*pt.x+B*pt.y+C*pt.z+D);
            distances.push_back(diff);
            if (diff<threshold){
                isInliers.push_back(true);
                inlierCounter++;
            }
            else{
                isInliers.push_back(false);
            }
            
        }//end for idx;
        
        ret.distances=distances;
        ret.isInliers=isInliers;
        ret.inliersNum=inlierCounter;
        
        return ret;
    }
