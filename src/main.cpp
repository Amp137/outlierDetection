#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "Frame.h"
#include "OrientationSolver.h"

#define NUM_FRAME 3


using namespace std;

unordered_map<int, vector<Frame*>> Frame::mapPointToShowedFrame;

int main (int argc, char **argv)
{
    vector<Frame> vecFrame;
    vecFrame.resize(NUM_FRAME);

    ifstream in("data/PREPHIy10.DAT");
    if (!in) {
        cout << "找不到测量点文件!" << endl;
        return -1;
    }

    string s;
    getline(in, s);
    for (int i = 0; i < NUM_FRAME; i++) {
        getline(in, s);
        stringstream ss;
        ss << s;
        int frameNo;
        ss >> frameNo;
        vecFrame[i].frameNo = abs(frameNo);

        while (true) {
            ss.str("");
            getline(in, s);
            ss << s;
            int pointNo;
            double xCoord, yCoord;
            ss >> pointNo;
            if (pointNo < 0) break;
            ss >> xCoord; ss >> yCoord;
            vecFrame[i].addPoint(pointNo, xCoord, yCoord);
        }
    }

    in.close();

    ifstream in2("data/PRECKI.DAT");
    if (!in2) {
        cout << "找不到控制点文件!" << endl;
        return -1;
    }

    stringstream ss;
    while (true) {
        ss.str("");
        getline(in2, s);
        ss << s;
        int pointNo;
        double YCoord, XCoord, ZCoord;
        ss >> pointNo;
        if (pointNo < 0) break;
        ss >> YCoord; ss >> XCoord; ss >> ZCoord; // 因大地测量坐标为左手系，故交换X,Y坐标，使其与像片坐标系一致
        for (int i = 0; i < NUM_FRAME; i++) {
            Eigen::Vector2d pt;
            if (vecFrame[i].getPoint(pointNo, pt) == 0)
                vecFrame[i].addCtrlPoint(pointNo, XCoord, YCoord, ZCoord);
        }
    }

    in2.close();



    return 0;
}