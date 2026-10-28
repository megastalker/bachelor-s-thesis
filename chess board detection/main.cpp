
#include <iostream>
#include <filesystem>
#include <chrono>
//#include <xiApi.h>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
using namespace std;


vector<cv::Point2f> detection(cv::Mat input_image){
    cv::cvtColor(input_image,input_image,cv::COLOR_BGR2GRAY);
    auto lsd = cv::createLineSegmentDetector();
    vector<cv::Vec4f> lines;
    lsd->detect(input_image,lines);
    lsd->drawSegments(input_image,lines);
    //cv::resize(input_image,input_image,cv::Size(),0.5,0.4);
    for (const auto& l : lines) {
    cv::line(
        input_image,
        cv::Point2f(l[0], l[1]),
        cv::Point2f(l[2], l[3]),
        cv::Scalar(0, 0, 255),
        2
    );
}
    vector<double> angles(lines.size());
    vector<double> longg(lines.size());
    for(int i = 0;i<lines.size();i++){
        angles[i]=atan2(lines[i][3]-lines[i][1],lines[i][2]-lines[i][0]);
        if(angles[i]<0)angles[i]+=CV_PI;    
        longg[i]=hypot(lines[i][3]-lines[i][1],lines[i][2]-lines[i][0]);
    }




    cv::namedWindow("111", cv::WINDOW_NORMAL | cv::WINDOW_GUI_EXPANDED);
    cv::imshow("111",input_image);
    cv::waitKey(0);
    return {};
}

vector<int> line_kef(cv::Point p1,cv::Point p2){
    vector<int> v(3);
    v[0]=p1.y-p2.y;
    v[1]=p2.x-p1.x;
    v[2]=p1.x*p2.y-p2.x*p1.y;
    return v;
}

double distanceToLine(
    cv::Point p,
    const std::vector<int>& line
);
void RANSAC(){
    vector<cv::Point> points = {
        {1, 3},
        {2, 5},
        {3, 7},
        {4, 9},
        {5, 11},
        {6, 13},
        {7, 15},
        {8, 17},

        // Шум
        {2, 15},
        {5, 3},
        {7, 6},
        {9, 2}
    };

}
int main() {
 cv::Mat im;
 im = cv::imread(R"(E:\BP\bachelor's thesis\chess board detection\img\chessboard_1792850830789002000.jpg)");
 if(im.empty()){
    cout<<"cannot find image";
    return 1;
 }
 detection(im);

    return 0;
}
