#include "Frame.h"

using namespace std;

Frame::Frame(): frameNo(-1), numPoints(0), numCtrlPoints(0) {}

Frame::Frame(const int _no): frameNo(_no), numPoints(0), numCtrlPoints(0) {}

int Frame::getCtrlPoint(const int _id, Eigen::Vector3d& pt)
{
    if (pointsCtrl.find(_id) != pointsCtrl.end()) {
        pt = pointsCtrl[_id];
        return 0;
    }
    else
        return -1;
}

int Frame::getPoint(const int _id, Eigen::Vector2d& pt)
{
    if (points2D.find(_id) != points2D.end()) {
        pt = points2D[_id];
        return 0;
    }
    else
        return -1;
}

int Frame::addCtrlPoint(const int _id, const double _x, const double _y, const double _z)
{
    if (pointsCtrl.find(_id) == pointsCtrl.end()) {
        numCtrlPoints++;
        pointsCtrl.emplace(_id, Eigen::Vector3d(_x, _y, _z));
        return 0;
    }
    else
        return -1;
}

int Frame::addPoint(const int _id, const double _x, const double _y)
{
    if (points2D.find(_id) == points2D.end()) {
        numPoints++;
        points2D.emplace(_id, Eigen::Vector2d(_x, _y));
    }
    else
        return -1;

    if (mapPointToShowedFrame.find(_id) == mapPointToShowedFrame.end()) {
        vector<Frame*> v;
        v.push_back(this);
        mapPointToShowedFrame.emplace(_id, v);
    }
    else {
        mapPointToShowedFrame[_id].push_back(this);
    }

    return 0;
}

int Frame::getOccurList(const Frame* frameA, const Frame* frameB, vector<int>& vecCo)
{
    auto pointList = frameB->points2D;
    for (const auto& it : frameA->points2D) {
        int idx = it.first;
        if (pointList.find(idx) != pointList.end())
            vecCo.push_back(idx);
    }
    return 0;
}

int Frame::getPointOccur(const int _id, vector<Frame*>& vecFrame)
{
    if (mapPointToShowedFrame.find(_id) != mapPointToShowedFrame.end()) {
        vecFrame = mapPointToShowedFrame[_id];
        return 0;
    }
    else
        return -1;
}

int Frame::printPointList()
{
    cout << "Frame No. " << frameNo << endl;
    cout << "Number of 2D points: " << numPoints << endl;
    cout << "ID x y" << endl;
    for (const auto& it : points2D) {
        int idx = it.first;
        Eigen::Vector2d pt;
        if (getPoint(idx, pt) == 0)
            cout << setw(9) << idx << " " << pt.transpose() << endl;
        else
            cout << setw(9) << idx << " ERROR" << endl;
    }

    cout << "Number of 3D control points: " << numCtrlPoints << endl;
    cout << "ID X Y Z" << endl;
    for (const auto& it : pointsCtrl) {
        int idx = it.first;
        Eigen::Vector3d pt;
        if (getCtrlPoint(idx, pt) == 0)
            cout << setw(9) << idx << " " << pt.transpose() << endl;
        else
            cout << setw(9) << idx << " ERROR" << endl;
    }

    return 0;
}

int Frame::printFullOccurList()
{
    map<int, vector<int>> list;
    int numPoints[4] = {0, 0, 0, 0}; // numPoints[0]为总点数，numPoints[1-3]分别为出现1,2,3次的点数
    for (const auto& it : mapPointToShowedFrame) {
        numPoints[0]++;
        int times = it.second.size();
        if ((times >= 1) && (times <= 3))
            numPoints[times]++;

        vector<int> v;
        for (auto ptFrame : it.second)
            v.push_back(ptFrame->frameNo);
        list.emplace(it.first, v);
    }

    vector<pair<int, vector<int> > > sortedList;
    sortMapByValueLength(list, sortedList);

    cout << "Number of points: " <<  numPoints[0] << endl;
    for (int i = 1; i < 4; i++)
        cout << "Number of points which show in " << i << " frames: " << numPoints[i] << endl;

    cout << "Point list:" <<  endl;
    cout << "ID Frame(s)" << endl;
    for (const auto& it : sortedList) {
        cout << setw(9) << it.first << " ";
        for (auto frameNo : it.second)
            cout << frameNo << " ";
        cout << endl;
    }

    return 0;
}

int Frame::cmp(const pair<int, vector<int> >& x, const pair<int, vector<int> >& y)
{
    return x.second.size() > y.second.size();
}

void Frame::sortMapByValueLength(const map<int, vector<int> >& tMap, vector<pair<int, vector<int> > >& tVector)
{
    for (const auto& it : tMap)
        tVector.emplace_back(it);

    sort(tVector.begin(), tVector.end(), cmp);
}

Frame::~Frame() = default;



