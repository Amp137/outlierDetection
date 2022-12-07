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
        meanPt += pt1 - pt2;
        numPt++;
    }
    meanPt /= numPt;
    bu = meanPt[0];
    // cout << "Initial bu: " << bu << endl;
}

// 连续像对相对定向
void OrientationSolver::relativeOrientation(const int _maxIter, const int _showDetailEachIter)
{
    cout << "Relative orientation of frame " << frame1->frameNo << " and frame " << frame2->frameNo << endl;
    maxIterRelative = _maxIter; // 最大迭代次数
    errorList.clear();
    double cost = 0, lastCost = 0; // 保存每次迭代的残差L2-norm，判断迭代是否正常

    // 为平差量赋初值
    dPhi = dOmega = dKappa = vByu = wByu = 0;

    // 定义误差方程参数 V = Ax - L, P
    typedef Eigen::Matrix<double, 5, 5> Matrix5d;
    typedef Eigen::Matrix<double, 5, 1> Vector5d;
    Eigen::MatrixXd A(numPt, 5);
    Vector5d x;
    Eigen::MatrixXd L(numPt, 1);
    Eigen::MatrixXd V(numPt, 1);
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(numPt, numPt);

    for (int iter = 0; iter < maxIterRelative; iter++) {
        // 根据上一轮的平差变量，更新相关参数
        bv = bu * vByu; // 基线分量 bv
        bw = bu * wByu; // 基线分量 bw，从片1投影中心到片2投影中心的向量，也即从片2变换到片1（像空间辅助坐标系）的平移向量

        Eigen::Matrix3d R = computeRFromPWK(dPhi, dOmega, dKappa); // 从片2变换到片1（像空间辅助坐标系）的旋转矩阵

        // 将共视点列表中的各点加入误差方程
        for (int i = 0; i < numPt; i++) {
            Eigen::Vector2d p1, p2;
            int ptIdx = co_occurList[i];
            frame1->getPoint(ptIdx, p1);
            frame2->getPoint(ptIdx, p2);

            Eigen::Vector3d P1(p1[0], p1[1], -f); // P1(u1, v1, w1)
            Eigen::Vector3d P2_(p2[0], p2[1], -f);
            Eigen::Vector3d P2 = R * P2_; // P2(u2, v2, w2)

            double N1 = (bu * P2[2] - bw * P2[0]) / (P1[0] * P2[2] - P1[2] * P2[0]);
            double N2 = (bu * P1[2] - bw * P1[0]) / (P1[0] * P2[2] - P1[2] * P2[0]);

            A(i, 0) =  bu;                                   // d vByu
            A(i, 1) = -bu * P2[1] / P2[2];                   // d wByu
            A(i, 2) = -P2[0] * P2[1] * N2 / P2[2];           // d dPhi
            A(i, 3) = -(P2[2] + P2[1] * P2[1] / P2[2]) * N2; // d dOmega
            A(i, 4) =  P2[0] * N2;                           // d dKappa

            L(i) = N1 * P1[1] - N2 * P2[1] - bv;
        }

        // Gauss-Newton法求解最小二乘
        Matrix5d H = A.transpose() * P * A; // Hessian
        Vector5d b = A.transpose() * P * L; // bias
        x = H.ldlt().solve(b); // Hx = b
        V = A * x - L;
        cost = (V.transpose() * P * V)(0);

        if (isnan(x[0])) {
            cout << "Result is NaN! Aborted." << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            cout << "Cost: " << cost << ", last cost: " << lastCost << endl;
            cout << "Update is not good! Aborted." << endl;
            break;
        }

        // 选权迭代
        double r = numPt - 5; // 多余观测数
        double sigma0hat = (V.transpose() * P * V)(0) / r; // 单位权方差的估值
        Eigen::MatrixXd QVVP(numPt, numPt);
        QVVP = Eigen::MatrixXd::Identity(numPt, numPt) - A * H.inverse() * A.transpose() * P;
        for (int i = 0; i < numPt; i++) {
            double Ti = V(i) * V(i) / (sigma0hat * QVVP(i, i) + 1e-10);
            if ((iter + 1) <= 3)
                if (sqrt(Ti) > 1)
                    P(i, i) = 1.0 / Ti;
                else
                    P(i, i) = 1.0;
            else
                if (sqrt(Ti) > 3.29)
                    P(i, i) = 1.0 / Ti;
                else
                    P(i, i) = 1.0;
        }

        // 更新平差变量
        vByu   += x[0];
        wByu   += x[1];
        dPhi   += x[2];
        dOmega += x[3];
        dKappa += x[4];
        lastCost = cost;

        // 显示迭代细节
        if ((_showDetailEachIter > 0) && (iter % _showDetailEachIter == 0)) {
            cout << setprecision(8) << "Iteration " << iter + 1 << " cost: " << cost << endl;
            cout << "Current bv, bw, dPhi, dOmega, dKappa:" << endl;
            cout << setprecision(8) <<
                 bu * vByu << " " <<
                 bu * wByu << " " <<
                 dPhi << " " <<
                 dOmega << " " <<
                 dKappa << endl;
        }

        // 更新量小于某一阈值，结束迭代
        if (x.norm() < 1e-12) {
            cout << "Converged (delta x < 1e-12)! Exit." << endl;
            for (int i = 0; i < numPt; i++)
                errorList.emplace(co_occurList[i], V(i));
            break;

        }

        if (iter + 1 == maxIterRelative) {
            cout << "Max iteration reached! Exit." << endl;
            for (int i = 0; i < numPt; i++)
                errorList.emplace(co_occurList[i], V(i));
        }
    }
}

void OrientationSolver::printErrorList(bool _sortByValue, bool _onlyOutliers)
{
    vector<pair<int, double> > sortedList;
    if (_sortByValue) {
        sortMapByValueAbs(errorList, sortedList);
    }
    else {
        for (auto it: errorList)
            sortedList.emplace_back(it);
    }

    if (_onlyOutliers) {
        cout << "Outliers after relative orientation:" << endl;
        cout << "\t\tID\t\tError" << endl;
        for (auto it : sortedList)
            if (abs(it.second) > 3 * sigma0)
                cout << setw(10) << setprecision(6) << it.first << " " << it.second << endl;
    }
    else {
        cout << "Point and error after relative orientation:" << endl;
        cout << "\t\tID\t\tError (outlier if > 4*0.0028)" << endl;
        for (auto it: sortedList)
            cout << setw(10) << setprecision(6) << it.first << " " << it.second << endl;
    }
}

bool OrientationSolver::isOutlier(int _id)
{
    if ((errorList.find(_id) != errorList.end()) && (abs(errorList[_id]) > 3 * sigma0))
        return true;
    else
        return false;
}

int OrientationSolver::cmpAbs(const pair<int, double>& x, const pair<int, double>& y)
{
    return abs(x.second) > abs(y.second);
}

void OrientationSolver::sortMapByValueAbs(const map<int, double>& tMap, vector<pair<int, double> >& tVector)
{
    for (const auto& it : tMap)
        tVector.emplace_back(it);

    sort(tVector.begin(), tVector.end(), OrientationSolver::cmpAbs);
}

Eigen::Matrix3d computeRFromPWK(double phi, double omega, double kappa)
{
    Eigen::Matrix3d R;
    R(0, 0) = cos(phi) * cos(kappa) - sin(phi) * sin(omega) * sin(kappa);
    R(0, 1) = -cos(phi) * sin(kappa) - sin(phi) * sin(omega) * cos(kappa);
    R(0, 2) = -sin(phi) * cos(omega);
    R(1, 0) = cos(omega) * sin(kappa);
    R(1, 1) = cos(omega) * cos(kappa);
    R(1, 2) = -sin(omega);
    R(2, 0) = sin(phi) * cos(kappa) + cos(phi) * sin(omega) * sin(kappa);
    R(2, 1) = -sin(phi) * sin(kappa) + cos(phi) * sin(omega) * cos(kappa);
    R(2, 2) = cos(phi) * cos(omega);
    return R;
}