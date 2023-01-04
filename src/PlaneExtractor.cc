#include<stdint.h>
#include<stdio.h>
#include<cmath>
#include<opencv2/opencv.hpp>
#include<numeric>
#include<random>
#include<vector>
#include<utility>
#include<ctime>
#include "KeyFrame.h"
#include "MapPoint.h"
#include"PlaneExtractor.h"
#include <opencv2/core/eigen.hpp>
#include<boost/filesystem.hpp>

float MIN_LINE_ANGLE =0.1745; // 20 degrees
int MIN_LINE_SIZE =10;
float COPLANARITY_THRES =0.05;
float INTER_KEYFRAME_DIST_THRESH = 0.06;// 6cm 
float INTER_KEYFRAME_ANGLE_THRESH = 0.9781;//12 Degrees 

PlaneExtractor::PlaneExtractor()
{
}
void PlaneExtractor::ComputePlanes(ORB_SLAM2::KeyFrame* kf )
{
     kf->SetNotEraseSemiDense();
     if( kf->isBad() || !kf->semidense_flag_ || !kf->interKF_depth_flag_) {
            kf->SetEraseSemiDense();
            return;
        }

        GetPlanesfromLines(kf);
        
        InterKFPlaneMatch(kf);

        kf->SetEraseSemiDense();

}
void PlaneExtractor::GetPlanesfromLines(ORB_SLAM2::KeyFrame* kf)
{
    for(size_t i=0; i< kf->mLines3D.rows;i++)
    {
        cv::Point3d diri =getLineDirection(kf->mLines3D.row(i));
        if (isZero(diri))
            {
                continue;
            }
       for(size_t j=i+1; j< kf->mLines3D.rows;j++)
       {
            cv::Point3d dirj = getLineDirection(kf->mLines3D.row(j));
            if (isZero(dirj)){
                continue;
            }
            if(abs(diri.dot(dirj)) > cos(MIN_LINE_ANGLE))
            {
            //GetPlanesFromParallelLines(i, j, kf);
            continue;
            }
            else
            {
                if (checkFeasibility(i,j,kf))
                {
                    cv::Point3d planeNormal =diri.cross(dirj);
                    float norm= sqrt(planeNormal.x*planeNormal.x + planeNormal.y*planeNormal.y + planeNormal.z*planeNormal.z);
                    planeNormal=planeNormal/norm;
                    cv::Mat line1= kf->mLines3D.row(i);
                    cv::Mat line2= kf->mLines3D.row(j);
                    cv::Point3d psi(line1.at<float>(0),line1.at<float>(1),line1.at<float>(2));
                    cv::Point3d pei(line1.at<float>(3),line1.at<float>(4),line1.at<float>(5));
                    cv::Point3d psj(line2.at<float>(0),line2.at<float>(1),line2.at<float>(2));
                    cv::Point3d pej(line2.at<float>(3),line2.at<float>(4),line2.at<float>(5));
                    float d1 = planeNormal.dot(psi);
                    float d2 = planeNormal.dot(pei);
                    float d3 = planeNormal.dot(psj);
                    float d4 = planeNormal.dot(pej);
                    float dmax = std::max({d1,d2,d3,d4});
                    float dmin = std::min({d1,d2,d3,d4});
                    if(abs(dmax-dmin) > COPLANARITY_THRES)
                       {
                           continue;
                       }
                    float D = -(d1+d2+d3+d4)/4;
                    cv::Vec4f plane(planeNormal.x,planeNormal.y,planeNormal.z,D);
                    if (plane[3]<0)
                    {
                        plane= -plane;
                        planeNormal=-planeNormal;
                    }
                    if (checkRedundant(plane,kf))
                        {
                            continue;
                        }
                    //cv::Mat wplane = WorldFramePlane(plane,kf);// World Frame
                    //cv::Vec4f wvplane(wplane.at<float>(0),wplane.at<float>(1),wplane.at<float>(2),wplane.at<float>(3));
                    kf->mPlanes.push_back(plane);
                    kf->mPlaneNormals.push_back(planeNormal);
                    kf->mPlaneLines.push_back(make_pair(i,j));
                    kf->mValidPlane.push_back(false);
                    //mAllPlanes.push_back(plane);
                    //mPlaneValidity.push_back(0);
                    //mAllPlaneLines.push_back(getPlaneLines(i,j,kf));
                    //break;

                }

            }
       }

    }
}

bool PlaneExtractor::isZero(cv::Point3d pt)
{
    if(pt.x==0 && pt.y==0 && pt.z==0)
        return true;
    else
        return false;
}

cv::Point3d PlaneExtractor::getLineDirection(cv::Mat lines)
{
    cv::Point3d ps(lines.at<float>(0),lines.at<float>(1),lines.at<float>(2));
    cv::Point3d pe(lines.at<float>(3),lines.at<float>(4),lines.at<float>(5));
    cv::Point3d dir;
    dir=pe-ps;
    float norm= sqrt(dir.x*dir.x + dir.y*dir.y + dir.z*dir.z);
    dir=dir/norm;
    return dir;
}

bool PlaneExtractor::checkFeasibility(int i,int j, ORB_SLAM2::KeyFrame* kf)
{
    cv::Point2d psi(kf->mLinesSeg.at<float>(i,0),kf->mLinesSeg.at<float>(i,1));
    cv::Point2d pei(kf->mLinesSeg.at<float>(i,2),kf->mLinesSeg.at<float>(i,3));
    cv::Point2d psj(kf->mLinesSeg.at<float>(j,0),kf->mLinesSeg.at<float>(j,1));
    cv::Point2d pej(kf->mLinesSeg.at<float>(j,2),kf->mLinesSeg.at<float>(j,3));

    cv::Point2d pi_mid((psi.x+pei.x)/2,(psi.y+pei.y)/2);
    cv::Point2d pj_mid((psj.x+pej.x)/2,(psj.y+pej.y)/2);
    float li = sqrt(pow(psi.x-pei.x,2)+pow(psi.y-pei.y,2));
    float lj = sqrt(pow(psj.x-pej.x,2)+pow(psj.y-pej.y,2));
    float dist_center = 2*sqrt(pow(pi_mid.x - pj_mid.x,2)+pow(pi_mid.y - pj_mid.y,2));

    if(dist_center < (li+lj))
    {

        cv::Mat line1= kf->mLines3D.row(i);
        cv::Mat line2= kf->mLines3D.row(j);
        cv::Point3d psi3d(line1.at<float>(0),line1.at<float>(1),line1.at<float>(2));
        cv::Point3d pei3d(line1.at<float>(3),line1.at<float>(4),line1.at<float>(5));
        cv::Point3d psj3d(line2.at<float>(0),line2.at<float>(1),line2.at<float>(2));
        cv::Point3d pej3d(line2.at<float>(3),line2.at<float>(4),line2.at<float>(5));    

        cv::Point3d pi_mid3d((psi3d.x+pei3d.x)/2,(psi3d.y+pei3d.y)/2,(psi3d.z+pei3d.z)/2);
        cv::Point3d pj_mid3d((psj3d.x+pej3d.x)/2,(psj3d.y+pej3d.y)/2,(psj3d.z+pej3d.z)/2); 

        float li3d = sqrt(pow(psi3d.x-pei3d.x,2)+pow(psi3d.y-pei3d.y,2)+pow(psi3d.z-pei3d.z,2));
        float lj3d = sqrt(pow(psj3d.x-pej3d.x,2)+pow(psj3d.y-pej3d.y,2)+pow(psj3d.z-pej3d.z,2));
        float dist_center3d = 2*sqrt(pow(pi_mid3d.x - pj_mid3d.x,2)+pow(pi_mid3d.y - pj_mid3d.y,2)+pow(pi_mid3d.z - pj_mid3d.z,2));

        if(dist_center3d < (li3d+lj3d))
        {
            return true;
        }


    }
    return false;


}

bool PlaneExtractor::checkRedundant(cv::Vec4f plane,ORB_SLAM2::KeyFrame *kf)
{
    //std::cout<<"mAllPlanes.size() "<<mAllPlanes.size()<<std::endl;
    for(size_t i=0; i < kf->mPlanes.size();i++)
    {

        cv::Vec4f prev_plane = kf->mPlanes[i];
        float d = plane[3] - prev_plane[3];
        float angle = plane[0]*prev_plane[0] + plane[1]*prev_plane[1] + plane[2]*prev_plane[2];
        if(d > 0.2 || d < -0.2)
           continue;
        if(angle < 0.866 || angle > -0.866)
            continue;
        return true;
    }
    return false;

}

cv::Point3d PlaneExtractor::getIntersectionPoint(cv::Mat line1, cv::Mat line2)
{
    cv::Point3d diri = getLineDirection(line1);
    cv::Point3d dirj = getLineDirection(line2);
    cv::Point3d psi(line1.at<float>(0),line1.at<float>(1),line1.at<float>(2));
    //cv::Point3d pei(line1.at<double>(3),line1.at<double(4),line1.at<double>(5));
    cv::Point3d psj(line2.at<float>(0),line2.at<float>(1),line2.at<float>(2));
    //cv::Point3d pej(line2.at<double>(3),line2.at<double(4),line2.at<double>(5));
    cv::Point3d normal = diri.cross(dirj);
    cv::Point3d temp1 = diri.cross(normal);
    float t = (temp1.dot(psi)-temp1.dot(psj))/temp1.dot(dirj);
    cv::Point3d intersection = psj+(dirj*t);
}
void PlaneExtractor::saveAllPlanes(std::string mStrDateTime)
{
    boost::filesystem::path results_dir("results_line_segments/" + mStrDateTime);
    boost::filesystem::path results_path = boost::filesystem::current_path() / results_dir;

    if( ! boost::filesystem::exists(results_path) && ! boost::filesystem::create_directories(results_path) ){
        std::cerr << "Failed to create directory: " << results_dir << std::endl;
        return;
    }

    // save points on planes and form plane model
    std::string strFileName2Fold("results_line_segments/" + mStrDateTime + "/planeModel.obj");
    std::ofstream fileOut2Fold(strFileName2Fold.c_str(), std::ios::out);
    if(!fileOut2Fold){
        std::cerr << "Failed to open stream to file: " << strFileName2Fold << std::endl;
        return;
    }
    std::string strFileName2Fold1("results_line_segments/" + mStrDateTime + "/planeValidity.txt");
    std::ofstream fileOut2Fold1(strFileName2Fold1.c_str(), std::ios::out);
    if(!fileOut2Fold1){
        std::cerr << "Failed to open stream to file: " << strFileName2Fold1 << std::endl;
        return;
    }


    for (int k = 0; k < mAllPlanes.size(); k++){
	fileOut2Fold1 << "Plane " <<k+1 << " " << mPlaneValidity[k]<< std::endl;
        if(mPlaneValidity[k]<3)
        {continue;}
        float step=0.01;
        std::pair<cv::Mat,cv::Mat> lines= mAllPlaneLines[k];
        cv::Vec4f plane = mAllPlanes[k];
        cv::Mat line1 = lines.first;
        cv::Mat line2 = lines.second;
        cv::Mat ps1 = line1(cv::Range::all(),cv::Range(0,3));
        cv::Mat pe1 = line1(cv::Range::all(),cv::Range(3,6));
        cv::Mat ps2 = line2(cv::Range::all(),cv::Range(0,3));
        cv::Mat pe2 = line2(cv::Range::all(),cv::Range(3,6));
        
        cv::Mat dir1=(pe1-ps1);
        dir1=dir1/cv::norm(dir1);

        cv::Mat dir2=(pe2-ps2);
        dir2=dir2/cv::norm(dir2);

        cv::Mat normal= (cv::Mat_<float>(1, 3)<< plane[0], plane[1], plane[2]);
        cv::Mat t1 = normal.cross(dir1);
        float iend = (pe1.at<float>(0)-ps1.at<float>(0)) /dir1.at<float>(0);

        for(float i=0;i<iend;i=i+step)
        {
            for(float j= -0.25;j<0.25;j=j+step)
            {
                cv::Mat pcl;
                pcl = ps1 + i*dir1 + j*t1;
                fileOut2Fold << "v " <<pcl.at<float>(0) << " " << pcl.at<float>(1) << " " << pcl.at<float>(2) << std::endl;
            }
        }
        cv::Mat t2 = normal.cross(dir2);
        iend = (pe2.at<float>(0)-ps2.at<float>(0)) /dir2.at<float>(0);

        for(float i=0;i<iend;i=i+step)
        {
            for(float j= -0.25;j<0.25;j=j+step)
            {
                cv::Mat pcl;
                pcl = ps2 + i*dir2 + j*t2;
                fileOut2Fold << "v " <<pcl.at<float>(0) << " " << pcl.at<float>(1) << " " << pcl.at<float>(2) << std::endl;
            }
        }
}

    fileOut2Fold.flush();
    fileOut2Fold.close();
    fileOut2Fold1.flush();
    fileOut2Fold1.close();
    std::cout << "saved plane point cloud" << std::endl;
}

std::pair<cv::Mat,cv::Mat> PlaneExtractor::getPlaneLines(int i, int j, ORB_SLAM2::KeyFrame *kf)
{
    cv::Mat line1=kf->mLines3D.row(i);
    cv::Mat line2=kf->mLines3D.row(j);
    std::pair<cv::Mat,cv::Mat> planeLines = std::make_pair(line1,line2);

    return planeLines;
}
void PlaneExtractor::Reset()
{
    mAllPlanes.clear();
    mAllPlaneLines.clear();
}
void PlaneExtractor::SaveAllPlaneLineSegments(std::string mStrDateTime) {

    boost::filesystem::path results_dir("results_line_segments/" + mStrDateTime);
    boost::filesystem::path results_path = boost::filesystem::current_path() / results_dir;

    if( ! boost::filesystem::exists(results_path) && ! boost::filesystem::create_directories(results_path) ){
        std::cerr << "Failed to create directory: " << results_dir << std::endl;
        return;
    }

    // save line segments
    std::string strFileName2Fold("results_line_segments/" + mStrDateTime + "/Plane_line_segments.obj");
    std::ofstream fileOut2Fold(strFileName2Fold.c_str(), std::ios::out);
    if(!fileOut2Fold){
        std::cerr << "Failed to open stream to file: " << strFileName2Fold << std::endl;
        return;
    }

  
    std::map<int,int> lines2points2Fold;

    for (int k = 0; k < mAllPlanes.size(); k++){

        std::pair<cv::Mat,cv::Mat> lines= mAllPlaneLines[k];
        cv::Vec4f plane = mAllPlanes[k];
        cv::Mat line1 = lines.first;
        cv::Mat line2 = lines.second;
        cv::Mat ps1 = line1(cv::Range::all(),cv::Range(0,3));
        cv::Mat pe1 = line1(cv::Range::all(),cv::Range(3,6));
        cv::Mat ps2 = line2(cv::Range::all(),cv::Range(0,3));
        cv::Mat pe2 = line2(cv::Range::all(),cv::Range(3,6));

        fileOut2Fold << "v " << ps1.at<float>(0) << " " << ps1.at<float>(1) << " " << ps1.at<float>(2) << std::endl;
        fileOut2Fold << "v " << pe1.at<float>(0) << " " << pe1.at<float>(1) << " " << pe1.at<float>(2) << std::endl;
        fileOut2Fold << "v " << ps2.at<float>(0) << " " << ps2.at<float>(1) << " " << ps2.at<float>(2) << std::endl;
        fileOut2Fold << "v " << pe2.at<float>(0) << " " << pe2.at<float>(1) << " " << pe2.at<float>(2) << std::endl;


    }
    int l=0;
    for(int k = 0; k < mAllPlanes.size(); k++)
    {
        fileOut2Fold << "l " << l+1<< " " << l+2 << std::endl;
        fileOut2Fold << "l " << l+3<< " " << l+4 << std::endl;
        l=l+4;
    }

    fileOut2Fold.flush();
    fileOut2Fold.close();
    std::cout << "Saved Plane line segment cloud" << std::endl;

}

void PlaneExtractor::InterKFPlaneMatch(ORB_SLAM2::KeyFrame *kf)
{   //std::cout<<"All planes size "<<mAllPlanes.size()<<std::endl;
    if(mAllPlanes.empty())
    {
     for (int i=0; i<kf->mPlanes.size(); i++)
     {
         mAllPlanes.push_back(kf->mPlanes[i]);
         mPlaneValidity.push_back(0);
         int p = kf->mPlaneLines[i].first;
         int q = kf->mPlaneLines[i].second;
         mAllPlaneLines.push_back(getPlaneLines(p,q,kf));

     }   
    }
    else
    {
    for (int i=0; i<kf->mPlanes.size(); i++)
    {   cv::Vec4f plane = kf->mPlanes[i];
        cv::Mat MatchPlane = (cv::Mat_<float>(4,1)<<plane[0],plane[1],plane[2],plane[3]);
        int p = kf->mPlaneLines[i].first;
        int q = kf->mPlaneLines[i].second;
        cv::Mat line1= kf->mLines3D.row(p);
        cv::Mat line2= kf->mLines3D.row(q);
        cv::Point3d psi3d(line1.at<float>(0),line1.at<float>(1),line1.at<float>(2));
        cv::Point3d pei3d(line1.at<float>(3),line1.at<float>(4),line1.at<float>(5));
        cv::Point3d psj3d(line2.at<float>(0),line2.at<float>(1),line2.at<float>(2));
        cv::Point3d pej3d(line2.at<float>(3),line2.at<float>(4),line2.at<float>(5)); 

        float disTh = INTER_KEYFRAME_DIST_THRESH;
        float angTh = INTER_KEYFRAME_ANGLE_THRESH;
        
        int maxSearchDepth = 100; //100 planes deep search 
        int matchFlag=0;

        for (int j = mAllPlanes.size()-1;j>=0 && maxSearchDepth >0;j--)
        {
            maxSearchDepth--;
            cv::Vec4f temp_plane = mAllPlanes[j];
            cv::Mat cPlane = (cv::Mat_<float>(4,1)<<temp_plane[0],temp_plane[1],temp_plane[2],temp_plane[3]);

            if(cPlane.at<float>(3,0)<0)
            {
                cPlane=-cPlane;
            }
            float angle = MatchPlane.at<float>(0)*cPlane.at<float>(0) + MatchPlane.at<float>(1)*cPlane.at<float>(1) + MatchPlane.at<float>(2)*cPlane.at<float>(2);
            if (angle >angTh)
            {
                float dist = MatchPlane.at<float>(3) - cPlane.at<float>(3);
                float d1 = (cPlane.at<float>(0) * psi3d.x) + (cPlane.at<float>(1) * psi3d.y) + (cPlane.at<float>(2) * psi3d.z) + cPlane.at<float>(3,0);
                float d2 = (cPlane.at<float>(0) * pei3d.x) + (cPlane.at<float>(1) * pei3d.y) + (cPlane.at<float>(2) * pei3d.z) + cPlane.at<float>(3,0);
                float d3 = (cPlane.at<float>(0) * psj3d.x) + (cPlane.at<float>(1) * psj3d.y) + (cPlane.at<float>(2) * psj3d.z) + cPlane.at<float>(3,0);
                float d4 = (cPlane.at<float>(0) * pej3d.x) + (cPlane.at<float>(1) * pej3d.y) + (cPlane.at<float>(2) * pej3d.z) + cPlane.at<float>(3,0);
                dist = (d1+d2+d3+d4)/4;
                if(dist < disTh && dist > -disTh)
                {
		            if(mPlaneValidity[j]<3)
			        {
	                    mAllPlanes[j]=kf->mPlanes[i];
	                    mPlaneValidity[j]++;
	                    mAllPlaneLines[j]=getPlaneLines(p,q,kf);
                        if(mPlaneValidity[j]==2)
                        {
                            kf->mValidPlane[i]=true;
                        }
		            }
                    else{  
                    kf->mValidPlane[i]=true;
                    }       
                    disTh = abs(dist);
                    angTh = angle;
                    matchFlag=1;
                }

            }
        }
        if(matchFlag==0)
        {
            mAllPlanes.push_back(kf->mPlanes[i]);
            mPlaneValidity.push_back(0);
            mAllPlaneLines.push_back(getPlaneLines(p,q,kf));
        }

 
    }
    }
}
cv::Mat PlaneExtractor::CameraFramePlane(cv::Vec4f plane,ORB_SLAM2::KeyFrame *kf)
{
    cv::Mat Tcw = kf->GetPose();
    cv::Mat camPlane = (cv::Mat_<float>(4,1)<<plane[0],plane[1],plane[2],plane[3]);
    
    return Tcw*camPlane;
}
cv::Mat PlaneExtractor::WorldFramePlane(cv::Vec4f plane,ORB_SLAM2::KeyFrame *kf)
{
    cv::Mat Twc = kf->GetPoseInverse();
    cv::Mat worldPlane = (cv::Mat_<float>(4,1)<<plane[0],plane[1],plane[2],plane[3]);
    
    return Twc*worldPlane;
}
cv::Mat PlaneExtractor::CameraFrameLine(cv::Mat line,ORB_SLAM2::KeyFrame *kf)
{
    cv::Mat Tcw = kf->GetPose();
    cv::Mat Pws = (cv::Mat_<float>(4, 1) << line.at<float>(0), line.at<float>(1), line.at<float>(2), 1); // point in world frame.
    cv::Mat Pcs = Tcw * Pws;
    cv::Mat Pwe = (cv::Mat_<float>(4, 1) << line.at<float>(3), line.at<float>(4), line.at<float>(5), 1); // point in world frame.
    cv::Mat Pce = Tcw * Pwe;    
    cv::Mat tline= (cv::Mat_<float>(1,6) << Pcs.at<float>(0), Pcs.at<float>(1), Pcs.at<float>(2), Pce.at<float>(0),Pce.at<float>(1),Pce.at<float>(2));
    
    return tline;
}
cv::Mat PlaneExtractor::WorldFrameLine(cv::Mat line,ORB_SLAM2::KeyFrame *kf)
{
    cv::Mat Twc = kf->GetPoseInverse();
    cv::Mat Pcs = (cv::Mat_<float>(4, 1) << line.at<float>(0), line.at<float>(1), line.at<float>(2), 1); // point in world frame.
    cv::Mat Pws = Twc * Pcs;
    cv::Mat Pce = (cv::Mat_<float>(4, 1) << line.at<float>(3), line.at<float>(4), line.at<float>(5), 1); // point in world frame.
    cv::Mat Pwe = Twc * Pce;    
    cv::Mat tline= (cv::Mat_<float>(1,6) << Pws.at<float>(0), Pws.at<float>(1), Pws.at<float>(2), Pwe.at<float>(0),Pwe.at<float>(1),Pwe.at<float>(2));
    
    return tline;
}
void PlaneExtractor::GetPlanesFromParallelLines(int i, int j, ORB_SLAM2::KeyFrame *kf)
{
    cv::Point2d psi(kf->mLinesSeg.at<float>(i,0),kf->mLinesSeg.at<float>(i,1));
    cv::Point2d pei(kf->mLinesSeg.at<float>(i,2),kf->mLinesSeg.at<float>(i,3));
    cv::Point2d psj(kf->mLinesSeg.at<float>(j,0),kf->mLinesSeg.at<float>(j,1));
    cv::Point2d pej(kf->mLinesSeg.at<float>(j,2),kf->mLinesSeg.at<float>(j,3));

    cv::Point2d pi_mid((psi.x+pei.x)/2,(psi.y+pei.y)/2);
    cv::Point2d pj_mid((psj.x+pej.x)/2,(psj.y+pej.y)/2);
    float li = sqrt(pow(psi.x-pei.x,2)+pow(psi.y-pei.y,2));
    float lj = sqrt(pow(psj.x-pej.x,2)+pow(psj.y-pej.y,2));
    float dist_center = 4*sqrt(pow(pi_mid.x - pj_mid.x,2)+pow(pi_mid.y - pj_mid.y,2));

    if(dist_center > (li+lj))
    {
        return;
    }

    cv::Mat line1= kf->mLines3D.row(i);
    cv::Mat line2= kf->mLines3D.row(j);
    cv::Point3d psi3d(line1.at<float>(0),line1.at<float>(1),line1.at<float>(2));
    cv::Point3d pei3d(line1.at<float>(3),line1.at<float>(4),line1.at<float>(5));
    cv::Point3d psj3d(line2.at<float>(0),line2.at<float>(1),line2.at<float>(2));
    cv::Point3d pej3d(line2.at<float>(3),line2.at<float>(4),line2.at<float>(5));  

    cv::Point3d pi_mid3d((psi3d.x+pei3d.x)/2,(psi3d.y+pei3d.y)/2,(psi3d.z+pei3d.z)/2);
    cv::Point3d pj_mid3d((psj3d.x+pej3d.x)/2,(psj3d.y+pej3d.y)/2,(psj3d.z+pej3d.z)/2); 

    float li3d = sqrt(pow(psi3d.x-pei3d.x,2)+pow(psi3d.y-pei3d.y,2)+pow(psi3d.z-pei3d.z,2));
    float lj3d = sqrt(pow(psj3d.x-pej3d.x,2)+pow(psj3d.y-pej3d.y,2)+pow(psj3d.z-pej3d.z,2));
    float dist_center3d = 2*sqrt(pow(pi_mid3d.x - pj_mid3d.x,2)+pow(pi_mid3d.y - pj_mid3d.y,2)+pow(pi_mid3d.z - pj_mid3d.z,2));

        if(dist_center3d < 0.1)
        {
            return;
        }
    dist_center3d=4*dist_center3d;
    if(dist_center3d > (li3d+lj3d))
    {
        return;
    }

    cv::Point3d diri = getLineDirection(kf->mLines3D.row(i));
    cv::Point3d dirj = pj_mid3d - psi3d;
    float norm= sqrt(dirj.x*dirj.x + dirj.y*dirj.y + dirj.z*dirj.z);
    dirj=dirj/norm;
    cv::Point3d planeNormal =diri.cross(dirj);
    norm= sqrt(planeNormal.x*planeNormal.x + planeNormal.y*planeNormal.y + planeNormal.z*planeNormal.z);
    planeNormal=planeNormal/norm;
    float d1 = planeNormal.dot(psi3d);
    float d2 = planeNormal.dot(pei3d);  
    float d3 = planeNormal.dot(psj3d);
    float d4 = planeNormal.dot(pej3d);
    float dmax = std::max({d1,d2,d3,d4});
    float dmin = std::min({d1,d2,d3,d4});
    if(abs(dmax-dmin) > COPLANARITY_THRES)
    { 
        return;
    }
    float D = -(d1+d2+d3+d4)/4;
    cv::Vec4f plane(planeNormal.x,planeNormal.y,planeNormal.z,D);
    if (plane[3]<0)
    {
        plane= -plane;
        planeNormal=-planeNormal;
    }
    if (checkRedundant(plane,kf))
    {
        return;
    }
        //cv::Mat wplane = WorldFramePlane(plane,kf);// World Frame
        //cv::Vec4f wvplane(wplane.at<float>(0),wplane.at<float>(1),wplane.at<float>(2),wplane.at<float>(3));
        kf->mPlanes.push_back(plane);
        kf->mPlaneNormals.push_back(planeNormal);
        kf->mPlaneLines.push_back(make_pair(i,j));
        kf->mValidPlane.push_back(false);
        //mAllPlanes.push_back(plane);
        //mPlaneValidity.push_back(0);
        //mAllPlaneLines.push_back(getPlaneLines(i,j,kf));
        //break;    



                    
}
