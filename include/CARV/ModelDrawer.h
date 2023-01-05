//
// Created by shida on 06/12/16.
//

#ifndef __MODELDRAWER_H
#define __MODELDRAWER_H

#include<pangolin/pangolin.h>

#include<mutex>

#include <map>
#include <list>
#include <deque>
#include <vector>
#include "CARV/Matrix.h"
#include "Modeler.h"
#include "CARV/TextureFrame.h"

class Modeler;

namespace ORB_SLAM2{

    class KeyFrame;
}


class ModelDrawer{
    public:
        ModelDrawer();

        void DrawModel(bool bRGB);
        void DrawModelPoints();
        void DrawTriangles(pangolin::OpenGlMatrix &Twc);
        void DrawFrame(bool bRGB);
        //cv::Mat DrawLines();

        void UpdateModel();
        void SetUpdatedModel(const vector<dlovi::Matrix> & modelPoints, const list<dlovi::Matrix> & modelTris);

        void MarkUpdateDone();
        bool UpdateRequested();
        bool UpdateDone();

        vector<dlovi::Matrix> & GetPoints();
        list<dlovi::Matrix> & GetTris();

        void SetModeler(Modeler* pModeler);
        int writeobj(string strfilename);
        Modeler* mpModeler;
        std::map<dlovi::Matrix,list<vector<float>>> Vt;
        std::mutex mMutexVt;
    private:



        bool mbModelUpdateRequested;
        bool mbModelUpdateDone;

        std::pair<vector<dlovi::Matrix>, list<dlovi::Matrix>> mModel;
        std::pair<vector<dlovi::Matrix>, list<dlovi::Matrix>> mUpdatedModel;

};
  

    


#endif //__MODELDRAWER_H
