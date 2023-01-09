
#include <mutex>
#include "Modeler.h"
#include "Map.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include "LineDetector.h"
#include <boost/filesystem.hpp>

// Model Class
Model::Model(const vector<dlovi::Matrix> &modelPoints, const list<dlovi::Matrix> &modelTris)
{
    mbNotErase = false;
    mbToBeErased = false;
    mData.first = modelPoints;
    mData.second = modelTris;
}

vector<dlovi::Matrix> & Model::GetPoints()
{
    return mData.first;
}

list<dlovi::Matrix> & Model::GetTris()
{
    return mData.second;
}

void Model::SetNotErase()
{
    unique_lock<mutex> lock(mMutexErase);
    mbNotErase = true;
}

void Model::SetErase()
{
	{
    	unique_lock<mutex> lock(mMutexErase);
    	mbNotErase = false;
	}

    if (mbToBeErased){
        Release();
    }
}

void Model::Release()
{
	{
    	unique_lock<mutex> lock(mMutexErase);
    	if (mbNotErase){
        	mbToBeErased = true;
        	return;
    	}
	}
    // suicide, need to be careful
    // delete this;

    // deallocate memory
    vector<dlovi::Matrix>().swap(mData.first);
    list<dlovi::Matrix>().swap(mData.second);
}


// Modeler Class
Modeler::Modeler(ORB_SLAM2::Map* pMap)
{
    mnLastNumLines = 2;
    mbFirstKeyFrame = true;

    mpMap = pMap;
    mAlgInterface.setAlgorithmRef(&mObjAlgorithm);
    mAlgInterface.setTranscriptRef(mTranscriptInterface.getTranscriptToProcessRef());
    mAlgInterface.rewind();
}
Modeler::Modeler(ModelDrawer* pModelDrawer): mpModelDrawer(pModelDrawer),mnLastNumLines(2),mbResetRequested(false),mbFirstKeyFrame(true),mnMaxTextureQueueSize(10),
        mnMaxFrameQueueSize(5000),mnMaxToLinesQueueSize(500),mbFinishRequested(false), mbFinished(true)
{
    mAlgInterface.setAlgorithmRef(&mObjAlgorithm);
    mAlgInterface.setTranscriptRef(mTranscriptInterface.getTranscriptToProcessRef());
    mAlgInterface.rewind();
}
void Modeler::WriteModel(std::string filename)
{
    mAlgInterface.writeCurrentModelToFile(filename);
    //mpModelDrawer->writeobj(filename);

}

void Modeler::AddKeyFrameEntry(ORB_SLAM2::KeyFrame *pKF)
{
    if (mbFirstKeyFrame) {
        unique_lock<mutex> lock(mMutexTranscript);
        mTranscriptInterface.addFirstKeyFrameInsertionEntry(pKF);
        mbFirstKeyFrame = false;
    } else {
        unique_lock<mutex> lock(mMutexTranscript);
        mTranscriptInterface.addKeyFrameInsertionEntry(pKF);
    }
    

}

void Modeler::AddLineSegmentKeyFrameEntry(ORB_SLAM2::KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexTranscript);
    //mTranscriptInterface.addLineSegmentKeyFrameInsertionEntry(pKF);
    mTranscriptInterface.addLineSegmentKeyFrameInsertionEntry(pKF);
    AddTexture(pKF);

    pKF->mTranscriptFlag=true;
}
void Modeler::AddPlaneKeyFrameEntry(ORB_SLAM2::KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexTranscript);
    mTranscriptInterface.addPlaneKeyFrameInsertionEntry(pKF);
    //mTranscriptInterface.addSemiDenseKeyFrameInsertionEntry(pKF);
    pKF->mTranscriptFlag=true;
    //AddTexture(pKF);

}

void Modeler::RunRemainder()
{
    std::cout << "running transcript" << std::endl;

    mAlgInterface.runRemainder();
}

void Modeler::RunOnce()
{
    std::cout << "running transcript once" << std::endl;

    mAlgInterface.setComputingModel(false);
    mAlgInterface.runRemainder();
    mAlgInterface.setComputingModel(true);
    mAlgInterface.computeCurrentModel();
}
void Modeler::Run()
    {
        mbFinished =false;

        while(1) {

            if (CheckNewTranscriptEntry()) {

                RunRemainder();

                UpdateModelDrawer();
                boost::filesystem::path dir("ObjectFiles");
                boost::filesystem::path objfile = boost::filesystem::current_path()/dir;
                if(!boost::filesystem::exists(objfile) && !boost::filesystem::create_directories(objfile))
                {
                    std::cerr<<"Failed to create directory:" <<objfile<<std::endl;
                    continue;
                }
                std::string strFileName("ObjectFiles/model.obj");
                std::string strFinalFilename("ObjectFiles/finalmodel.obj");
                WriteModel(strFileName);
                boost::filesystem::copy_file(strFileName,strFinalFilename,boost::filesystem::copy_option::overwrite_if_exists);               
            }
            ResetIfRequested();

            if(CheckFinish())
                break;

            usleep(1000);
        }

        SetFinish();

    }
void Modeler::SetFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }
bool Modeler::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }


void Modeler::UpdateModel()
{
    std::cout << "updating model" << std::endl; 

    std::pair<std::vector<dlovi::Matrix>, std::list<dlovi::Matrix> > objModel = mAlgInterface.getCurrentModel();
    Model* pNewModel = new Model(objModel.first, objModel.second);

    mpMap->UpdateModel(pNewModel);
}

bool Modeler::CheckNewTranscriptEntry()
{
    unique_lock<mutex> lock(mMutexTranscript);
    int numLines = mTranscriptInterface.getTranscriptRef()->numLines();
    if (numLines > mnLastNumLines) {
        mnLastNumLines = numLines;
        mTranscriptInterface.UpdateTranscriptToProcess();
        std::cout << "checking transcript: " << numLines << std::endl;
        return true;
    } else {
        return false;
    }
}

void Modeler::AddFrameImage(const long unsigned int &frameID, const cv::Mat &im)
    {
        unique_lock<mutex> lock(mMutexFrame);

        // make a copy of image and save as RGB
        cv::Mat imc;
        im.copyTo(imc);
        if(imc.channels() < 3)
            cvtColor(imc,imc,CV_GRAY2RGB);

        if (mmFrameQueue.size() >= mnMaxFrameQueueSize) {
            mmFrameQueue.erase(mmFrameQueue.begin());
        }
        if (mmFrameQueue.count(frameID) > 0){
            std::cerr << "ERROR: trying to add an existing frame" << std::endl;
            return;
        }
        mmFrameQueue.insert(make_pair(frameID,imc));
    }

std::vector<pair<cv::Mat,TextureFrame>> Modeler::GetTextures(int n)
    {
        unique_lock<mutex> lock(mMutexTexture);
        unique_lock<mutex> lock2(mMutexFrame);
        int nLastKF = mdTextureQueue.size() - 1;
        std::vector<pair<cv::Mat,TextureFrame>> imAndTexFrame;
        // n most recent KFs
        for (int i = 0; i<n && i <= nLastKF; i++){
            TextureFrame texFrame = mdTextureQueue[std::max(0,nLastKF-i)];
            imAndTexFrame.push_back(make_pair(mmFrameQueue[texFrame.mFrameID],texFrame));
        }

        return imAndTexFrame;
    }

void Modeler::AddTexture(ORB_SLAM2::KeyFrame* pKF)
    {
        unique_lock<mutex> lock(mMutexTexture);

        TextureFrame texFrame(pKF);
        if (mdTextureQueue.size() >= mnMaxTextureQueueSize) {
            mdTextureQueue.pop_front();
        }
        mdTextureQueue.push_back(texFrame);
    }

void Modeler::UpdateModelDrawer() {
        if(mpModelDrawer->UpdateRequested() && ! mpModelDrawer->UpdateDone()) {
            std::pair<std::vector<dlovi::Matrix>, std::list<dlovi::Matrix> > objModel = mAlgInterface.getCurrentModel();
            mpModelDrawer->SetUpdatedModel(objModel.first, objModel.second);
            mpModelDrawer->MarkUpdateDone();
        }
    }

 void Modeler::RequestReset()
    {
        {
            unique_lock<mutex> lock(mMutexReset);
            mbResetRequested = true;
        }

        while(1)
        {
            {
                unique_lock<mutex> lock2(mMutexReset);
                if(!mbResetRequested)
                    break;
            }
            usleep(100);
        }
    }
    void Modeler::SetLocalMapper(ORB_SLAM2::LocalMapping* pLocalMapper)
{
    mpLocalMapper = pLocalMapper;
}

    void Modeler::SetLoopCloser(ORB_SLAM2::LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

    void Modeler::SetTracker(ORB_SLAM2::Tracking* pTracker)
{
    mpTracker = pTracker;
}

void Modeler::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Modeler::isFinished()
{
unique_lock<mutex> lock(mMutexFinish);
return mbFinished;
}

void Modeler::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        {
            unique_lock<mutex> lock2(mMutexTranscript);
            mTranscriptInterface.addResetEntry();
        }
        {
            unique_lock<mutex> lock2(mMutexTexture);
            mdTextureQueue.clear();
        }
        {
            unique_lock<mutex> lock2(mMutexFrame);
            mmFrameQueue.clear();
        }
        {
            //TODO: ADD Lines and Planes
        }
        mbFirstKeyFrame =true;
        mbResetRequested =false;
    }
}