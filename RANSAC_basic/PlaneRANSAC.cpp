#include "MatrixReaderWriter.h"
#include "PlaneEstimation.h"
#include "PLYWriter.h"
#include <opencv2/opencv.hpp>
#include <vector>


using namespace cv;


#define THERSHOLD 0.2  //RANSAC threshold (if Velodyne scans are processed, the unit is meter)

#define RANSAC_ITER  200    //RANSAC iteration

#define FILTER_LOWEST_DISTANCE 0.3 //threshold for pre-filtering

int main(int argc, char** argv){
    
    if (argc!=3){
        printf("Usage:\n PlanRansac input.xyz output.ply\n");
        exit(EXIT_FAILURE);
    }
    
    MatrixReaderWriter mrw(argv[1]);
    
    int num=mrw.rowNum;
    
    cout<< "Rows:" << num <<endl;
    cout<< "Cols:" << mrw.columnNum << endl;



    //Read data from text file
    
    vector<Point3f> points;
    
    for (int idx=0;idx<num;idx++){
       double x=mrw.data[3*idx];
       double y=mrw.data[3*idx+1];
       double z=mrw.data[3*idx+2];
       
       float distFromOrigo=sqrt(x*x+y*y+z*z);


//First filter: minimal work distance for a LiDAR limited.        
       
       if (distFromOrigo>FILTER_LOWEST_DISTANCE){
           Point3f newPt;
           newPt.x=x;
           newPt.y=y;
           newPt.z=z;
           points.push_back(newPt);
           }
       
    }
    
    
    //Number of points:

    int num1=points.size();
    cout << "After filtering-Rows:" << num1 << endl;
    
    
    //Estimate plane parameters without robustification
    
    float* plane=EstimatePlaneImplicit(points);
    printf("Plane fitting results for the whole data:\nA:%f B:%f C:%f D:%f\n",plane[0],plane[1],plane[2],plane[3]);
    
    delete[] plane;
    
    
    //RANSAC-based robust estimation
    
    float* planeParams=EstimatePlaneRANSAC(points,THERSHOLD,RANSAC_ITER);

    
    printf("Plane params RANSAC (i):\n A:%f B:%f C:%f D:%f \n",planeParams[0],planeParams[1],planeParams[2],planeParams[3]);
    
    //Compute differences of the fitted plane in order to separate inliers from outliers
    
    RANSACDiffs differences=PlanePointRANSACDifferences(points,planeParams,THERSHOLD);

    delete[] planeParams;
    
    vector<int> g1;
    vector<Point3f> plane1;
    for (int idx = 0;idx < num1;idx++) {
        Point3f pts = points.at(idx);
        if (differences.isInliers.at(idx)) {
            g1.push_back(1);
        }
        else {
            g1.push_back(0);
            plane1.push_back(pts);
        }
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Number of points:
    int num2=plane1.size();
    cout << "Rows:" << num2 << endl;



    //Estimate second plane parameters without robustification

    float* plane11 = EstimatePlaneImplicit(plane1);
    printf("Plane fitting results for second time:\nA:%f B:%f C:%f D:%f\n", plane11[0], plane11[1], plane11[2], plane11[3]);

    delete[] plane11;


    //RANSAC-based robust estimation

    float* planeParams11 = EstimatePlaneRANSAC(plane1, THERSHOLD, RANSAC_ITER);


    printf("Plane params RANSAC (ii):\n A:%f B:%f C:%f D:%f \n", planeParams11[0], planeParams11[1], planeParams11[2], planeParams11[3]);

    //Compute differences of the fitted plane in order to separate inliers from outliers

    RANSACDiffs differences11 = PlanePointRANSACDifferences(plane1, planeParams11, THERSHOLD);

    delete[] planeParams11;
    vector<int> g2;
    vector<Point3f> plane2;
    for (int idx = 0;idx < num2;idx++) {
        Point3f pts = plane1.at(idx);
        if (differences11.isInliers.at(idx)) {
            g2.push_back(1);
        }
        else {
            g2.push_back(0);
            plane2.push_back(pts);
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Number of points:
    int num3 = plane2.size();
    cout << "Rows:" << num3 << endl;



    //Estimate plane parameters without robustification

    float* plane22 = EstimatePlaneImplicit(plane2);
    printf("Plane fitting results for the third time:\nA:%f B:%f C:%f D:%f\n", plane22[0], plane22[1], plane22[2], plane22[3]);

    delete[] plane22;


    //RANSAC-based robust estimation

    float* planeParams22 = EstimatePlaneRANSAC(plane2, THERSHOLD, RANSAC_ITER);


    printf("Plane params RANSAC (iii):\n A:%f B:%f C:%f D:%f \n", planeParams22[0], planeParams22[1], planeParams22[2], planeParams22[3]);

    //Compute differences of the fitted plane in order to separate inliers from outliers

    RANSACDiffs differences22 = PlanePointRANSACDifferences(plane2, planeParams22, THERSHOLD);

    delete[] planeParams22;
    vector<int> g3;
    vector<Point3f> plane3;
        for (int idx = 0;idx < num3;idx++) {
            Point3f pts = plane2.at(idx);

         if (differences22.isInliers.at(idx)) {
             g3.push_back(1);
             
            }
         else {
                g3.push_back(0);
                plane3.push_back(pts);
          }

         }

    //Colouring of the three detected planes using seq RANSAC by red, green and pink respectively and left over outliers by white

    vector<Point3i> plane1cRANSAC;
    int n1=0, n2=0, n3=0;
    for (int idx = 0;idx < num1;idx++) {
        Point3i planecol;

        if (g1[n1]==1) {
            planecol.x = 255;
            planecol.y = 0;
            planecol.z = 0;
            n1++;
        }
        else{
            if (g2[n2]==1) {
                planecol.x = 0;
                planecol.y = 255;
                planecol.z = 0;
                n2++;
            }
            else{

                if (g3[n3]==1) {
                    planecol.x = 255;
                    planecol.y = 192;
                    planecol.z = 203;
                    n3++;
                }
                else
                {
                    planecol.x = 255;
                    planecol.y = 255;
                    planecol.z = 255;
                    n3++;
                }
                n2++;
            }
            n1++;
        }


        plane1cRANSAC.push_back(planecol);


    }


//Write results into a PLY file. 
//It can be isualized by open-source 3D application Meshlab (www.meshlab.org)

    WritePLY(argv[2], points , plane1cRANSAC);
        
}
