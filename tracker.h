//
// Created by root on 19-4-24.
//

#ifndef TRACKER_TRACKER_H
#define TRACKER_TRACKER_H



#include "kalmanFilter.h"
#include "hungarian.h"
#include <vector>

using namespace std;


class TrackObj
{
public:
    VectorXf center;
};

class Track
{
public:
    int trackId = 0;
    int skippedFrames = 0;
    VectorXf centerPrediction; // 预测的中心点
    vector<VectorXf> centerPredictionTrackList;
//    KalmanFilter* centerKFPtr = nullptr;
    KalmanFilter centerKF;

    Track(int _trackId, VectorXf _centerPrediction);
//    ~Track();
};

class Tracker
{
public:
    int distThresh; //距离阈值：距离阈值。当超过阈值时， 将删除曲目并创建新曲目
    int maxFrameToSkip; //最大帧到跳过：允许跳过的最大帧数 未检测到的跟踪对象
    int maxTrackLength; //最大跟踪长度：跟踪路径历史长度
    int trackIdCount; //每个跟踪对象的标识
    vector<Track> tracks;
    HungarianAlgorithm HungAlgo;

    Tracker(int _distThresh, int _maxFrameToSkip, int _maxTrackLength, int _trackIdCount);
    void Update(vector<TrackObj>& obj);
};

#endif //TRACKER_TRACKER_H
