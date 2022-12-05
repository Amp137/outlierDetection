#include "OrientationSolver.h"

using namespace std;

OrientationSolver::OrientationSolver(Frame *_f1, Frame *_f2, const double _f, const double _sigma0)
{
    frame1 = _f1;
    frame2 = _f2;
    f = _f;
    sigma0 = _sigma0;

    Frame::getOccurList(frame1, frame2, co_occurList); // 生成两帧共视点列表

    // 为基线分量 bu 赋初值
    Eigen::Vector2d meanPt(0, 0);
    numPt = 0;
    for (auto idx : co_occurList) {
        Eigen::Vector2d pt1, pt2;
        frame1->getPoint(idx, pt1);
        frame2->getPoint(idx, pt2);
        meanPt += pt2 - pt1;
        numPt++;
    }
    meanPt /= numPt;
    initBu = meanPt[0];
    // cout << "Initial bu: " << initBu << endl;
}

void OrientationSolver::relativeOrientation(const int _rt)
{
    maxIterRelative = _rt; // 最大迭代次数
    errorList.clear();

    // 为平差变量赋初值
    dPhi = dOmega = dKappa = vByu = wByu = 0;

    // 定义误差方程参数
    Eigen::Matrix<double, Eigen::Dynamic, 5> A;
    Eigen::Matrix<double, 5, 1> x;
    Eigen::Matrix<double, Eigen::Dynamic, 1> L;
    Eigen::Matrix<double, Eigen::Dynamic, 1> V;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P;


}
