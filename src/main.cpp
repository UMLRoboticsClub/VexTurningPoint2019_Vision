//#define USE_WEBCAM
#define DEBUG
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <algorithm>

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::stringstream;
using namespace std::chrono;
using namespace cv;

enum Color { BLUE, RED, GREEN };

Color team = BLUE;

const int cam_index = 0;
const char *test_image = "../img/field.png";

/* params to tune */

//ratio of dist to green/
const double minRatio = 0.3;
const double maxRatio = 0.65;
//canny params
const int edgeThresh = 100;
const int maxEdgeThresh = 200;
//polygon approximation
const double polyEpsilon = 14;

/* HSV color ranges for flags */

//blue
const int bSensitivity = 20;
Scalar bMin(120 - bSensitivity, 70, 70);
Scalar bMax(120 + bSensitivity, 255, 255);

//green
const int gSensitivity = 20;
Scalar gMin(60 - gSensitivity, 20, 100);
Scalar gMax(60 + gSensitivity, 255, 255);

//red
const int rSensitivity = 20;
Scalar rMin1(0, 100, 100);
Scalar rMax1(rSensitivity/2, 255, 255);
Scalar rMin2(180-rSensitivity/2, 100, 100);
Scalar rMax2(180, 255, 255);

namespace B {
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    vector<vector<Point>> polygons;
    vector<double> pAreas;
    vector<Rect> boundRect;
    vector<Point> centers;

    Mat orig;
    Mat thresh;
    Mat canny_output;
};

namespace G {
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    vector<Rect> boundRect;
    vector<Point> centers;

    Mat orig;
    Mat thresh;
    Mat canny_output;
};

void prepFrame(Mat &frame);
void prepFrame2(Mat &frame, Color color);
void processFrame(const Mat &frame);
void findTargets(vector<Point> &targets);

void processB(const Mat &_frame, const Mat &_orig);
void drawB();
void processG(const Mat &_frame, const Mat &_orig);
void drawG();
void drawAll(const Mat &_orig, const vector<Point> &targets);

int main(int argc, char **argv){
    const auto &getTime = []{
        return duration<double>(high_resolution_clock::now().time_since_epoch()).count();
    };

    if(argc >= 2){
        if(strcmp(argv[1], "r") == 0){
            team = RED;
        }
    }


#ifdef USE_WEBCAM
    VideoCapture cap;
    if(!cap.open(cam_index)){
        cout << "cannot open video device\n";
        exit(1);
    }
    Mat frame;
#else
    Mat frame = imread(test_image);
#endif
#ifdef USE_WEBCAM
    cout << "\nHit 'q' to exit...\n";
    while(waitKey(1) != 'q'){
        cap >> frame;
#endif
        const double before = getTime();
        processFrame(frame);
        const double delta = getTime() - before;
        cout << "Time elapsed: " << delta * 1000.f << " ms" << endl;

#ifdef USE_WEBCAM
    }
#else
    waitKey(0);
#endif
}

void prepFrame(Mat &frame){
    //noise smoothing
    GaussianBlur(frame, frame, Size(5, 5), 3.0, 3.0);
    //HSV conversion
    cvtColor(frame, frame, CV_BGR2HSV);
}

void prepFrame2(Mat &frame, Color color){
    //color thresholding
    Mat rangeRes = Mat::zeros(frame.size(), CV_8UC1);

    switch(color){
        case BLUE:
            inRange(frame, bMin, bMax, frame);
            break;
        case RED:
            inRange(frame, rMin1, rMax1, frame);
            inRange(frame, rMin2, rMax2, frame);
            break;
        case GREEN:
            inRange(frame, gMin, gMax, frame);
            break;
    }

    //Improving the result
    erode(frame, frame, Mat(), Point(-1, -1), 2);
    dilate(frame, frame, Mat(), Point(-1, -1), 2);
}


void processFrame(const Mat &frame){
    Mat processed = frame.clone();
    prepFrame(processed);

    processB(processed, frame);
    processG(processed, frame);

    vector<Point> targets;
    findTargets(targets);

#ifdef DEBUG
    //drawB();
    //drawG();
    drawAll(frame, targets);
#endif
}

void findTargets(vector<Point> &targets){
    auto dist = [](const Point& pt1, const Point& pt2) -> float {
        float deltaX = pt1.x - pt2.x;
        float deltaY = pt1.y - pt2.y;
        return (deltaX * deltaX) + (deltaY * deltaY);
    };

    //cout << "distance from " << b_objects[0].center << " to " << g_objects[0].center << " is " << 
    //dist(b_objects[0].center, g_objects[0].center) << endl;

    struct Pair {
        Pair(int a, int b, float dist):
            a(a), b(b), dist(dist){}
        int a, b;
        float dist;
    };

    vector<Pair> closest_pairs;
    closest_pairs.reserve(B::polygons.size());

    for(unsigned i = 0; i < B::polygons.size(); ++i){
        float smallestVal = 1E10;
        int smallestObjIndex = 0;
        for(unsigned j = 0; j < G::contours.size(); ++j){
            float val = dist(B::centers[i], G::centers[j]);
            if(val < smallestVal){
                smallestVal = val;
                smallestObjIndex = j;
            }
        }
        closest_pairs.emplace_back(i, smallestObjIndex, smallestVal);
    }

    std::sort(closest_pairs.begin(), closest_pairs.end(),
            [](Pair &a, Pair &b) -> bool {
            return a.dist < b.dist;
            });

#ifdef DEBUG
    Mat canvas = Mat::zeros(B::canny_output.size(), CV_8UC3);
#endif
    for(unsigned i = 0; i < closest_pairs.size(); ++i){
        //if(closest_pairs[i].dist > minDist && closest_pairs[i].dist < maxDist){

        const double minRatio = 0.3;
        const double maxRatio = 0.65;
        double ratio = closest_pairs[i].dist / B::pAreas[closest_pairs[i].a];
        //cout << "ratio:" << ratio << endl;

        if(ratio > minRatio && ratio < maxRatio){
            targets.push_back(B::centers[closest_pairs[i].a]);

#ifdef DEBUG
            circle(canvas, B::centers[closest_pairs[i].a], 3, Scalar(32,255,255));
            line(canvas, B::centers[closest_pairs[i].a], G::centers[closest_pairs[i].b], Scalar(255,255,255));
#endif
        }

#ifdef DEBUG
        putText(canvas, string("dist:") + std::to_string(closest_pairs[i].dist), B::centers[closest_pairs[i].a] + Point(0,-10), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
        putText(canvas, string("area:") + std::to_string(B::pAreas[closest_pairs[i].a]), B::centers[closest_pairs[i].a] + Point(0,10), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
#endif
    }

#ifdef DEBUG
    imshow("targets", canvas);
#endif
}

void drawAll(const Mat &_orig, const vector<Point> &targets){
    Mat orig = _orig.clone();

    //using namespace B;
    /***************** B *****************/
    for(unsigned i = 0; i < B::polygons.size(); ++i){
        drawContours(orig, B::polygons, i, Scalar(32,255,255), 1, 8, B::hierarchy, 0, Point());
        //circle(orig, B::centers[i], 3, Scalar(32,255,255));
    }

    /***************** G *****************/

    for(unsigned i = 0; i < G::contours.size(); ++i){
        drawContours(orig, G::contours, i, Scalar(32,255,255), 1, 8, G::hierarchy, 0, Point());
        //circle(orig, G::centers[i], 3, Scalar(32,255,255));
    }

    /***************** TARGETS *****************/
    for(unsigned i = 0; i < targets.size(); ++i){
        circle(orig, targets[i], 8, Scalar(0,0,0), -1);
        circle(orig, targets[i], 3, Scalar(255,255,255), -1);
        putText(orig, "TARGET", targets[i], FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
    }

    imshow("overlay", orig);
}

void processB(const Mat &_frame, const Mat &_orig){
    using namespace B;

    contours.clear();
    hierarchy.clear();
    polygons.clear();
    boundRect.clear();
    centers.clear();
    //

    orig = _orig.clone();
    thresh = _frame.clone();

    prepFrame2(thresh, team);

    //canny edge detection
    Canny(thresh, canny_output, edgeThresh, maxEdgeThresh, 3);

    //find contours
    findContours(canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    //CV_RETR_EXTERNAL = get just external contours (no nesting)

    polygons.resize(contours.size());
    pAreas.resize(contours.size());
    for(unsigned i = 0; i < contours.size(); ++i){
        //get polygons from contours
        approxPolyDP(contours[i], polygons[i], polyEpsilon, true);
    }

    //remove polygons that aren't squares
    polygons.erase(std::remove_if(polygons.begin(), polygons.end(),
                [](const auto &poly){ return poly.size() != 4; }), polygons.end());

    boundRect.reserve(polygons.size());
    centers.reserve(polygons.size());

    for(unsigned i = 0; i < polygons.size(); ++i){
        //get bounding rectangle for each polygon
        boundRect[i] = boundingRect(polygons[i]);
        //find center of bounding rectangle
        centers[i] = Point(boundRect[i].x + boundRect[i].width/2, boundRect[i].y + boundRect[i].height/2);
        //find area of polygon
        pAreas[i] = contourArea(polygons[i]);
    }
}

void drawB(){
    using namespace B;

    Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);

    for(unsigned i = 0; i < contours.size(); ++i){
        drawContours(drawing, contours, i, Scalar(32,32,32), 2, 8, hierarchy, 0, Point());
    }

    for(unsigned i = 0; i < polygons.size(); ++i){
        drawContours(drawing, polygons, i, Scalar(32,255,255), 1, 8, hierarchy, 0, Point());

        //circle(drawing, polygons[i][0], 3, Scalar(32,255,255));
        //circle(drawing, polygons[i][1], 3, Scalar(32,255,255));
        //circle(drawing, polygons[i][2], 3, Scalar(32,255,255));
        //circle(drawing, polygons[i][3], 3, Scalar(32,255,255));

        drawContours(orig, polygons, i, Scalar(32,255,255), 1, 8, hierarchy, 0, Point());
        circle(drawing, centers[i], 3, Scalar(32,255,255));
        circle(orig, centers[i], 3, Scalar(32,255,255));
    }

    //imshow("canny", canny_output);
    imshow("bthreshold", thresh);
    imshow("bdrawing", drawing);
    imshow("borig", orig);
}

void processG(const Mat &_frame, const Mat &_orig){
    using namespace G;

    contours.clear();
    hierarchy.clear();
    boundRect.clear();
    centers.clear();
    //

    orig = _orig.clone();
    thresh = _frame.clone();

    prepFrame2(thresh, GREEN);

    //canny edge detection
    Canny(thresh, canny_output, edgeThresh, maxEdgeThresh, 3);

    //find contours
    findContours(canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    //CV_RETR_EXTERNAL = get just external contours (no nesting)

    boundRect.reserve(contours.size());
    centers.reserve(contours.size());

    for(unsigned i = 0; i < contours.size(); ++i){
        //get bounding rectangle for each contour 
        boundRect[i] = boundingRect(contours[i]);
        //find center of bounding rectangle
        centers[i] = Point(boundRect[i].x + boundRect[i].width/2, boundRect[i].y + boundRect[i].height/2);
    }
}

void drawG(){
    using namespace G;

    Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);

    for(unsigned i = 0; i < contours.size(); ++i){
        //draw stuff
        drawContours(drawing, contours, i, Scalar(32,32,32), 2, 8, hierarchy, 0, Point());
        drawContours(drawing, contours, i, Scalar(32,255,255), 1, 8, hierarchy, 0, Point());
        drawContours(orig,    contours, i, Scalar(32,255,255), 1, 8, hierarchy, 0, Point());

        circle(drawing, centers[i], 3, Scalar(32,255,255));
        circle(orig,    centers[i], 3, Scalar(32,255,255));
    }

    //imshow("canny", canny_output);
    imshow("gthreshold", thresh);
    imshow("gdrawing", drawing);
    imshow("gorig", orig);
}
