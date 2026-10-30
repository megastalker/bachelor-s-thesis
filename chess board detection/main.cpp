
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
int imageWidth = 2464;
int imageHeight = 2056;
cv::Mat m;
class RANSAC_for_chessboard{
    vector<cv::Vec4f> lines;
    vector<cv::Point2f> intersection;
    double threshold;
    double min_line_long = 40;
    double radius = 15;
    public:
    struct cluster
    {
        cv::Point2f center;
        int count=0;
    };
    vector<cluster> clusters;
    int min_count=15;
    RANSAC_for_chessboard(vector<cv::Vec4f> &lines, double threshold = 5): lines(lines),threshold(threshold){
        if(threshold < 0)cout<<"error trashold < 0";
    }

    vector<double> line_kef(const cv::Vec4f& p) {
        vector<double> kef = {
            p[1] - p[3],                 // a = y1 - y2
            p[2] - p[0],                 // b = x2 - x1
            p[0] * p[3] - p[2] * p[1]  // c = x1*y2 - x2*y1
        };
        return kef;
    }

    bool intersect_line(cv::Vec4f &l1,cv::Vec4f &l2){
        auto k1 = line_kef(l1);
        auto k2 = line_kef(l2);
        double D = k1[0]*k2[1]-k2[0]*k1[1]; // determinant
        if(D == 0)return false;
        double x = (k1[1]*k2[2]-k1[2]*k2[1])/D;
        double y = (k1[2]*k2[0]-k2[2]*k1[0])/D;
        if(x<0 || y<0 || x>=imageWidth || y>=imageHeight)return false;
        intersection.push_back({float(x),float(y)});
        return true;
    }


    void find_intersection(){
        intersection.clear();
        vector<double> angle(lines.size());
        vector<double> length(lines.size());

        for(int i = 0;i < lines.size();i++){
            //length[i]=hypot(lines[i][2] - lines[i][0],lines[i][3] - lines[i][1]);
            length[i]= (lines[i][2] - lines[i][0])*(lines[i][2] - lines[i][0])+(lines[i][3] - lines[i][1])*(lines[i][3] - lines[i][1]);
            if(length[i]<min_line_long*min_line_long)continue;
            angle[i]=atan2(lines[i][3] - lines[i][1],lines[i][2] - lines[i][0]);
            if(angle[i]<0)angle[i]+=CV_PI;
        }
        for(int i = 0;i < lines.size();i++){
            for(int j = i+1;j < lines.size();j++){
            if (length[i] < min_line_long*min_line_long || length[j]<min_line_long*min_line_long) continue;
            double diff = abs(angle[i]-angle[j]);
            diff = min(diff,CV_PI-diff);
            if(diff<CV_PI/6)continue;
                intersect_line(lines[i],lines[j]);
            }
        }
        cout<<intersection.size();
        cluster_intersection();
    }


    
    void cluster_intersection(){
        clusters.clear();
        for(const auto &p:intersection){
            if(clusters.empty()){
                clusters.push_back({p,1});
                continue;
            }
            double mindist = -1;
            int pos;
            for(int i = 0;i<clusters.size();i++){
                double dist = (p.x - clusters[i].center.x)*(p.x - clusters[i].center.x)+(p.y-clusters[i].center.y)*(p.y-clusters[i].center.y);
                if(dist<radius*radius){
                    if( mindist == -1 || dist < mindist){
                        mindist=dist;
                        pos = i;
                    }
                }
            }
            if(mindist == -1)clusters.push_back({p,1});
            else{
                clusters[pos].center.x = clusters[pos].center.x+(p.x-clusters[pos].center.x)/(clusters[pos].count+1);
                clusters[pos].center.y = clusters[pos].center.y+(p.y-clusters[pos].center.y)/(clusters[pos].count+1);
                clusters[pos].count++;
            }
            
        }
        cout<<" "<<clusters.size();

        for(const auto &c:clusters){
            if(c.count>=min_count)
            cv::circle(m,c.center,15,cv::Scalar(255,255,0),-1);
        }
    }



};



vector<cv::Point2f> detection(cv::Mat input_image){
    imageWidth = input_image.cols;
    imageHeight = input_image.rows;
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
    RANSAC_for_chessboard r(lines);
    m = input_image;
    r.find_intersection();
    input_image=m;
    
    cv::namedWindow("111", cv::WINDOW_NORMAL | cv::WINDOW_GUI_EXPANDED);
    cv::imshow("111",input_image);
    cv::waitKey(0);
    return {};
}





class RANSAC_for_line{
    public:
    vector<cv::Point2f> points;
    double threshold;
    //vector<cv::
    //vector<vector<double>> lin_k;
    RANSAC_for_line(const vector<cv::Point2f> &points, double threshold = 5): points(points), threshold(threshold){
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
 srand(time(nullptr)); 
 cv::Mat im;
 im = cv::imread(R"(E:\BP\bachelor's thesis\chess board detection\img\chessboard_1792850830789002000.jpg)");
 if(im.empty()){
    cout<<"cannot find image";
    return 1;
 }

 detection(im);

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