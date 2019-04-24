#include "tracker.h"
#include <iostream>

using namespace std;

int main()
{
    Tracker tracker(160, 30, 6, 100);
    VectorXf tempCenter(2);
    for (int i = 0; i < 23; i++)
    {
        tempCenter << i * 11, i * 11;

        TrackObj trackObj;
        trackObj.center = tempCenter;
        vector<TrackObj> obj;
        obj.push_back(trackObj);
        tracker.Update(obj);
        if (tracker.tracks.size() > 0)
        {
            for (int j = 0; j < tracker.tracks.size(); j++)
            {
                if (tracker.tracks[j].centerPredictionTrackList.size() > 1)
                {
                    for (int k = 0; k < (tracker.tracks[j].centerPredictionTrackList.size() - 1); k++)
                    {
                        float x1 = tracker.tracks[j].centerPredictionTrackList[k](0);
                        float y1 = tracker.tracks[j].centerPredictionTrackList[k](1);
                        float x2 = tracker.tracks[j].centerPredictionTrackList[k+1](0);
                        float y2 = tracker.tracks[j].centerPredictionTrackList[k+1](1);

                        cout << x1 << " " << y1 << " " << x2 << " " << y2 << endl;
                    }
                }
            }
        }
        else
        {
            cout<<"tracks.size < 0"<< endl;
        }
    }

//    system("PAUSE");
    return 0;
}