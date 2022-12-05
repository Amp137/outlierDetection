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
    double initBu;

    int maxIterRelative; // 相对定向最大迭代次数
    double dPhi, dOmega, dKappa, vByu, wByu; // 相对定向五参数
    double bu, bv, bw; // 基线分量
    unordered_map<int, double> errorList; // 保存平差中各同名像点的残差，用于判断粗差

    int maxIterAbsolute; // 绝对定向最大迭代次数
    double lambda, phi, omega, kappa, Xs, Ys, Zs; // 绝对定向七参数
    double N1, N2;

public:
    OrientationSolver(Frame* _f1, Frame* _f2, double _f, double _sigma0);
    void relativeOrientation(int _rt);
    void modelPoints();
    void absoluteOrientation();
    void calculateWorldPoints();
    ~OrientationSolver() = default;

};

Eigen::Matrix3d computeRFromPWK(double phi, double omega, double kappa);

#endif //OUTLIERDETECTION_ORIENTATIONSOLVER_H
