#ifndef OUTLIERDETECTION_ORIENTATIONSOLVER_H
#define OUTLIERDETECTION_ORIENTATIONSOLVER_H

#include <iostream>
#include <iomanip>
#include <vector>
#include <map>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Frame.h"

using namespace std;

class OrientationSolver
{
private:
    Frame* frame1;
    Frame* frame2; // 双像模型的左图、右图，像点单位为mm，控制点单位为m，已经统一到右手系
    double f; // 焦距 100.5 mm
    double sigma0; // 中误差 2.8 um = 2.8e-3 mm
    vector<int> co_occurList;
    int numPt;

    int maxIterRelative = 0; // 相对定向最大迭代次数
    double dPhi = 0, dOmega = 0, dKappa = 0, vByu = 0, wByu = 0; // 相对定向五参数
    double bu = 0, bv = 0, bw = 0; // 基线分量
    map<int, double> errorList; // 保存平差中各同名像点的残差，用于判断粗差

    int maxIterAbsolute = 0; // 绝对定向最大迭代次数
    double lambda = 0, phi = 0, omega = 0, kappa = 0, Xs = 0, Ys = 0, Zs = 0; // 绝对定向七参数

    static void sortMapByValueAbs(const map<int, double>& tMap, vector<pair<int, double> >& tVector);
    static int cmpAbs(const pair<int, double>& x, const pair<int, double>& y);

public:
    OrientationSolver(Frame* _f1, Frame* _f2, double _f, double _sigma0);
    void relativeOrientation(int _maxIter, int _showDetailEachIter); // 连续像对相对定向
    void printErrorList(bool _sortByValue, bool _onlyOutliers); // 打印各像点相对定向后的残差，用于判断粗差
    bool isOutlier(int _id); // 如果点_id是粗差点，返回true
    //void modelPoints();
    //void absoluteOrientation();
    //void calculateWorldPoints();

    ~OrientationSolver() = default;

};

Eigen::Matrix3d computeRFromPWK(double phi, double omega, double kappa);

#endif //OUTLIERDETECTION_ORIENTATIONSOLVER_H
