
#include <iostream>
#include <filesystem>
#include <chrono>
//#include <xiApi.h>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <cstdlib>
#include <ctime>
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



class RANSAC{
    public:
    vector<cv::Point2f> points;
    double threshold;
    //vector<cv::
    //vector<vector<double>> lin_k;
    RANSAC(const vector<cv::Point2f> &points, double threshold = 5): points(points), threshold(threshold){
        //kef.resize(3);
        if(threshold < 0)cout<<"error trashold < 0";
    };  

    double distanceToLine(const cv::Point2f &p, const std::vector<double>& line){
        if(line[0]==0 && line[1]==0)return threshold+1;
        return abs(line[0]*p.x+line[1]*p.y+line[2])/sqrt(line[0]*line[0]+line[1]*line[1]);
    }

    void new_points(const vector<cv::Point2f> &points){
        this->points=points;
    }

    int countPointInline(const vector<double> &line){
        int kol=0;
        for(const auto &p:points){
            if(threshold - distanceToLine(p,line)>=0){
            kol++;
            }
        }
        return kol;
    }

    vector<double> line_kef(cv::Point2f p1,cv::Point2f p2){
        vector<double> kef = {
            p1.y - p2.y,
            p2.x - p1.x,
            p1.x * p2.y - p2.x * p1.y
        };
        return kef;
    }
    vector<double> fit_line(int it){
        if (points.size() < 2) throw std::invalid_argument("RANSAC requires at least 2 points");
        vector<double> BestLine(3);
        vector<double> line(3);
        int BestValue;
        int p1 = rand()%points.size();
        int p2 = rand()%points.size();
        for(;p1==p2;) p2 = rand()%points.size();
        BestLine = line_kef(points[p1],points[p2]);
        BestValue = countPointInline(BestLine);
        for(int i = 0;i<it-1;i++){
            p1 = rand()%points.size();
            p2 = rand()%points.size();
            for(;p1==p2;) p2 = rand()%points.size();
            line = line_kef(points[p1],points[p2]);
            int val = countPointInline(line);
            if(val>BestValue){
                BestLine = line;
                BestValue = val;
            }
        }
        return BestLine;
    }


};
int main() {
        vector<cv::Point2f> points = {
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
 srand(time(nullptr)); 
 cv::Mat im;
 im = cv::imread(R"(E:\BP\bachelor's thesis\chess board detection\img\chessboard_1792850830789002000.jpg)");
 if(im.empty()){
    cout<<"cannot find image";
    return 1;
 }
RANSAC r(points,1);
vector<double> v;
v = r.fit_line(5);
for(auto x:v){
    cout<<" "<<x;
}
 //detection(im);

    return 0;
}
    // vector<cv::Point> points = {
    //     {1, 3},
    //     {2, 5},
    //     {3, 7},
    //     {4, 9},
    //     {5, 11},
    //     {6, 13},
    //     {7, 15},
    //     {8, 17},

    //     // Шум
    //     {2, 15},
    //     {5, 3},
    //     {7, 6},
    //     {9, 2}
    // };