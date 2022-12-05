#ifndef OUTLIERDETECTION_FRAME_H
#define OUTLIERDETECTION_FRAME_H

#include <iostream>
#include <iomanip>
#include <vector>
#include <map>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

class Frame
{
private:
    unordered_map<int, Eigen::Vector2d, hash<int>, equal_to<int>, Eigen::aligned_allocator<pair<int, Eigen::Vector2d>>> points2D;
    unordered_map<int, Eigen::Vector3d, hash<int>, equal_to<int>, Eigen::aligned_allocator<pair<int, Eigen::Vector3d>>> pointsCtrl;

public:
    int frameNo, numPoints, numCtrlPoints; //frameNo 帧编号, numPoints 点数, numCtrlPoints 控制点数

    Frame();
    explicit Frame(int _no);

    int getPoint(int _id, Eigen::Vector2d& pt);
    int getCtrlPoint(int _id, Eigen::Vector3d& pt);
    int addPoint(int _id, double _x, double _y);
    int addCtrlPoint(int _id, double _x, double _y, double _z);
    int printPointList();

    ~Frame();

public:
    static unordered_map<int, vector<Frame*>> mapPointToShowedFrame;
    static int getPointOccur(int _id, vector<Frame*>& vecFrame);// 某个编号的点在哪几个编号的图像上出现了
    static int getOccurList(const Frame* frameA, const Frame* frameB, vector<int>& vecCo); // 计算帧与帧间的同名点列表
    static int printFullOccurList(); // 打印当前所有帧的共视点
    static int cmp(const pair<int, vector<int> >& x, const pair<int, vector<int> >& y);
    static void sortMapByValueLength(const map<int, vector<int> >& tMap, vector<pair<int, vector<int> > >& tVector);

};



#endif //OUTLIERDETECTION_FRAME_H
