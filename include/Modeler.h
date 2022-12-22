
#ifndef __MODELER_H
#define __MODELER_H

#include <mutex>

#include <list>
#include <vector>
#include "CARV/Matrix.h"
#include "CARV/SFMTranscriptInterface_ORBSLAM.h"
#include "CARV/SFMTranscriptInterface_Delaunay.h"
#include "CARV/ModelDrawer.h"
#include "CARV/TextureFrame.h"

class Cluster;
class TextureFrame;
class ModelDrawer;

namespace ORB_SLAM2 {
    class KeyFrame;
    class Map;
    class MapPoint;
    class Tracking;
    class LoopClosing;
    class LocalMapping;
}


class Model {
public:
    Model(const vector<dlovi::Matrix> & modelPoints, const list<dlovi::Matrix> & modelTris);
    vector<dlovi::Matrix> & GetPoints();
    list<dlovi::Matrix> & GetTris();

    void SetNotErase();
    void SetErase();
    void Release();

private:
    std::mutex mMutexErase;
    bool mbNotErase;
    bool mbToBeErased;
    std::pair<std::vector<dlovi::Matrix>, std::list<dlovi::Matrix>> mData;

};


// interface class for surface reconstruction using CARV system
class Modeler {
public:
    
    Modeler(ORB_SLAM2::Map* pMap);
    Modeler(ModelDrawer* pModelDrawer);

    //TODO: Loop Closing
    void SetLoopCloser(ORB_SLAM2::LoopClosing* pLoopCloser);
    void SetLocalMapper(ORB_SLAM2::LocalMapping* pLocalMapper);
    void SetTracker(ORB_SLAM2::Tracking* pTracker);

    void AddKeyFrameEntry(ORB_SLAM2::KeyFrame* pKF);
    void AddLineSegmentKeyFrameEntry(ORB_SLAM2::KeyFrame* pKF);

    bool CheckNewTranscriptEntry();

    void RunRemainder();
    void RunOnce();
    void UpdateModel();
    void UpdateModelDrawer();

    void WriteModel(std::string filename);
    void AddPlaneKeyFrameEntry(ORB_SLAM2::KeyFrame *pKF);
    void AddTexture(ORB_SLAM2::KeyFrame* pKF);
    std::vector<pair<cv::Mat,TextureFrame>> GetTextures(int n);
    void RequestReset();
    void AddFrameImage(const long unsigned int &frameID,const cv::Mat &im);
    void Run();
    void SetFinish();
    bool CheckFinish();
    //TODO:: 
    void RequestFinish();
    bool isFinished();


public:
    //CARV interface
    SFMTranscriptInterface_ORBSLAM mTranscriptInterface; // An interface to a transcript / log of the map's work.
    //CARV runner instance
    dlovi::FreespaceDelaunayAlgorithm mObjAlgorithm;
    SFMTranscriptInterface_Delaunay mAlgInterface; // An encapsulation of the interface between the transcript and the surface inferring algorithm.

    ORB_SLAM2::Map* mpMap;
    ModelDrawer* mpModelDrawer; 
    std::mutex mMutexReset;
    void ResetIfRequested();
    bool mbResetRequested;

    // This avoid that two transcript entries are created simultaneously in separate threads
    std::mutex mMutexTranscript;

    //number of lines in transcript last time checked
    int mnLastNumLines;

    bool mbFirstKeyFrame;
    //queue for the keyframes used to texture the model, keyframe mnFrameId
    std::deque<TextureFrame> mdTextureQueue;
    size_t mnMaxTextureQueueSize;
    std::mutex mMutexTexture;

    //queue for the frames recieved, pair<mnID,image>
    std::map<long unsigned int, cv::Mat> mmFrameQueue;
    size_t mnMaxFrameQueueSize;
    std::mutex mMutexFrame;

    //queue for the texture frames used to detect lines
    std::deque<ORB_SLAM2::KeyFrame*> mdToLinesQueue;
    size_t mnMaxToLinesQueueSize;
    std::mutex mMutexToLines;

    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    ORB_SLAM2::Tracking* mpTracker;
    ORB_SLAM2::LocalMapping* mpLocalMapper;
    ORB_SLAM2::LoopClosing* mpLoopCloser;


};



#endif //__MODELVIEWER_H
