//
// Created by root on 19-4-24.
//

#include "tracker.h"
#include <exception>
#include <iostream>
#include <math.h>

Track::Track(int _trackId, VectorXf _centerPrediction)
{
    this->trackId = _trackId;
    this->centerPrediction = _centerPrediction;

//    this->centerKFPtr = new KalmanFilter();
}

//Track::~Track()
//{
//    if (this->centerKFPtr != nullptr)
//    {
//        delete this->centerKFPtr;
//        this->centerKFPtr = nullptr;
//    }
//}

Tracker::Tracker(int _distThresh, int _maxFrameToSkip, int _maxTrackLength, int _trackIdCount)
{
    this->distThresh = _distThresh;
    this->maxFrameToSkip = _maxFrameToSkip;
    this->maxTrackLength = _maxTrackLength;
    this->trackIdCount = _trackIdCount;
}

void Tracker::Update(vector<TrackObj>& obj)
{
    //如果未找到轨迹(tracks)矢量，则创建轨迹(tracks)
    if (tracks.size() == 0)
    {
        for (int i = 0; i < obj.size(); i++)
        {
            Track track(this->trackIdCount, obj[i].center);
            this->trackIdCount += 1;
            this->tracks.push_back(track);
        }
    }

    //使用centerPrediction与obj的center之间的平方距离之和计算成本
    int N = (int)this->tracks.size();
    int M = (int)obj.size();
    MatrixXf cost = MatrixXf::Zero(N, M);
    for (int i = 0; i < this->tracks.size(); i++)
    {
        for (int j = 0; j < obj.size(); j++)
        {
            try
            {
                VectorXf diff = this->tracks[i].centerPrediction - obj[i].center;
                cost(i, j) = sqrt(diff(0) * diff(0) + diff(1) * diff(1));
            }
            catch (std::exception& e)
            {
                std::cerr << e.what() << std::endl;
            }
        }
    }

    //求平方误差的平均值
    cost = 0.5 * cost;
    //使用匈牙利算法将正确检测到的测量值分配给预测 tracks
    vector<int> assignment;
    vector< vector<double> > costMatrix(N);
    for (int i = 0; i < N; i++) {
        costMatrix[i].resize(M);
    }
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < M; j++)
        {
            costMatrix[i][j] = cost(i, j);
        }
    }
    double costNum = this->HungAlgo.Solve(costMatrix, assignment);
    int row_ind = (int)costMatrix.size();

    //识别没有分配的追踪路径（如果有）
    vector<int> unAssignedTracks;
    for (int i = 0; i < assignment.size(); i++)
    {
        if (assignment[i] != -1)
        {
            //检查成本距离阈值。如果成本很高，则取消分配（删除）轨道
            if (cost(i, assignment[i]) > this->distThresh)
            {
                assignment[i] = -1;
                unAssignedTracks.push_back(i);
            }
        }
        else
        {
            this->tracks[i].skippedFrames += 1;
        }
    }

    //如果长时间未检测到曲目，将其删除
    vector<int> delTracks;
    for (int i = 0; i < this->tracks.size(); i++)
    {
        if (this->tracks[i].skippedFrames > this->maxFrameToSkip)
        {
            delTracks.push_back(i);
        }
    }
    if (delTracks.size() > 0)
    {
        for (int i = 0; i < delTracks.size(); i++)
        {
            if (delTracks[i] < this->tracks.size())
            {
                //删除元素

                ////////////////////////////////
                //注意：测试的时候如果发现此处this->tracks.size() 一直等于 assignment
                ////////////////////////////////

                int p = 0;
                vector<int>::iterator itAssignment = assignment.begin();
                for (vector<Track>::iterator itTracks = this->tracks.begin(); itTracks != this->tracks.end();)
                {
                    if (i == delTracks[i])
                    {
                        itTracks = this->tracks.erase(itTracks);
                        itAssignment = assignment.erase(itAssignment);
                    }
                    else
                    {
                        itTracks++;
                        itAssignment++;
                    }
                    p++;
                }
            }
            else
            {
                cerr << "id is greater than length of tracks" << endl;
            }
        }
    }

    //现在寻找未分配的检测
    vector<int> unAssignedDetects;
    for (int i = 0; i < obj.size(); i++)
    {
        vector<int>::iterator ret = find(assignment.begin(), assignment.end(), i);
        if (ret == assignment.end())
        {
            unAssignedDetects.push_back(i);
        }
    }

    //start new tracks
    if (unAssignedDetects.size() != 0)
    {
        for (int i = 0; i < unAssignedDetects.size(); i++)
        {
            Track track(this->trackIdCount, obj[i].center);
            this->trackIdCount += 1;
            this->tracks.push_back(track);
        }
    }

    //更新kalmanfilter状态、最后结果和跟踪
    for (int i = 0; i < assignment.size(); i++)
    {

        this->tracks[i].centerKF.predict();
        if (assignment[i] != -1)
        {
            this->tracks[i].skippedFrames = 0;
            int num = assignment[i];
            this->tracks[i].centerKF.correct(obj[num].center);
            this->tracks[i].centerPrediction = this->tracks[i].centerKF.X;
        }
        else
        {
            VectorXf nullCenter = VectorXf::Zero(2);
            this->tracks[i].centerPrediction = nullCenter;
        }

        if (this->tracks[i].centerPredictionTrackList.size() > this->maxTrackLength)
        {
            for (int j = 0; j < (this->tracks[i].centerPredictionTrackList.size() - this->maxTrackLength);)
            {
                //删除vector的第一个元素
                this->tracks[i].centerPredictionTrackList.erase(this->tracks[i].centerPredictionTrackList.begin());
            }
        }

        this->tracks[i].centerPredictionTrackList.push_back(this->tracks[i].centerPrediction);
    }

}