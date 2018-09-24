#define USE_WEBCAM
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

const int cam_index = 0;
const char *test_image = "field.png";

//params to tune
const int edgeThresh = 100;
const int maxEdgeThresh = 200;
const double polyEpsilon = 14;

//HSV color ranges for flags

//blue
const int bSensitivity = 20;
Scalar bMin(120 - bSensitivity, 70, 100);
Scalar bMax(120 + bSensitivity, 255, 255);

//green
const int gSensitivity = 20;
Scalar gMin(60 - gSensitivity, 20, 100);
Scalar gMax(60 + gSensitivity, 255, 255);

//red
//Scalar rMin(, 100, 100);
//Scalar rMax(, 100, 100);

namespace B {
    vector<vector<Point>> contours;
    vector<double> contourAreas;
    vector<Vec4i> hierarchy;
    vector<vector<Point>> polygons;
    vector<Rect> boundRect;
    vector<Point> centers;

    Mat orig;
    Mat thresh;
    Mat canny_output;
};

namespace G {
    vector<vector<Point>> contours;
    vector<double> contourAreas;
    vector<Vec4i> hierarchy;
    vector<Rect> boundRect;
    vector<Point> centers;

    Mat orig;
    Mat thresh;
    Mat canny_output;
};

void prepFrame(Mat &frame);
void prepFrame2(Mat &frame, const Scalar &minColorRange, const Scalar &maxColorRange);
void processFrame(const Mat &frame);
void findTargets(vector<Point> &targets);

void processB(const Mat &_frame, const Mat &_orig);
void drawB();
void processG(const Mat &_frame, const Mat &_orig);
void drawG();
void drawAll(const Mat &_orig, const vector<Point> &targets);

int main(){
    const auto &getTime = []{
        return duration<double>(high_resolution_clock::now().time_since_epoch()).count();
    };

    VideoCapture cap;
    if (!cap.open(cam_index)){
        cout << "cannot open video device\n";
        exit(1);
    }

    Mat frame;

#ifndef USE_WEBCAM
    frame = imread(test_image);
#else
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

void prepFrame2(Mat &frame, const Scalar &minColorRange, const Scalar &maxColorRange){
    //color thresholding
    Mat rangeRes = Mat::zeros(frame.size(), CV_8UC1);
    inRange(frame, minColorRange, maxColorRange, frame);

    //Improving the result
    erode(frame, frame, Mat(), Point(-1, -1), 2);
    dilate(frame, frame, Mat(), Point(-1, -1), 2);
}


void processFrame(const Mat &frame){
    Mat processed = frame.clone();
    prepFrame(processed);

    processB(processed, frame);
    //drawB();
    processG(processed, frame);
    //drawG();

    vector<Point> targets;
    findTargets(targets);
    
    drawAll(frame, targets);
}

void findTargets(vector<Point> &targets){
    struct Obj {
        Obj(const Point &center, double area, int index):
            center(center), area(area), index(index){}
        Point center;
        double area; 
        int index;
    };

    auto dist = [](const Point& pt1, const Point& pt2) -> float {
        float deltaX = pt1.x - pt2.x;
        float deltaY = pt1.y - pt2.y;
        return (deltaX * deltaX) + (deltaY * deltaY);
    };

    vector<Obj> b_objects;
    b_objects.reserve(B::polygons.size());

    for(unsigned i = 0; i < B::polygons.size(); ++i){
        b_objects.emplace_back(B::centers[i], B::contourAreas[i], i);
    }

    vector<Obj> g_objects;
    g_objects.reserve(G::contours.size());

    for(unsigned i = 0; i < G::contours.size(); ++i){
        g_objects.emplace_back(G::centers[i], G::contourAreas[i], i);
    }

    //cout << "distance from " << b_objects[0].center << " to " << g_objects[0].center << " is " << 
    //dist(b_objects[0].center, g_objects[0].center) << endl;

    struct Pair {
        Pair(Obj a, Obj b, float dist):
            a(a), b(b), dist(dist){}
        Obj a, b;
        float dist;
    };

    vector<Pair> closest_pairs;
    closest_pairs.reserve(b_objects.size());

    for(unsigned i = 0; i < b_objects.size(); ++i){
        float smallestVal = 1E10;
        int smallestObjIndex = 0;
        for(unsigned j = 0; j < g_objects.size(); ++j){
            float val = dist(b_objects[i].center, g_objects[j].center);
            if(val < smallestVal){
                smallestVal = val;
                smallestObjIndex = j;
            }
        }
        closest_pairs.emplace_back(b_objects[i], g_objects[smallestObjIndex], smallestVal);
        g_objects.erase(g_objects.begin() + smallestObjIndex);
    }

    std::sort(closest_pairs.begin(), closest_pairs.end(),
            [](Pair &a, Pair &b) -> bool {
            return a.dist < b.dist;
            });

    Mat canvas = Mat::zeros(B::canny_output.size(), CV_8UC3);
    for(unsigned i = 0; i < closest_pairs.size(); ++i){
        //cout << "dist:" << closest_pairs[i].dist << endl;
        line(canvas, closest_pairs[i].a.center, closest_pairs[i].b.center, Scalar(255,255,255));

        int aIndex = closest_pairs[i].a.index;
        int bIndex = closest_pairs[i].b.index;

        double ratio = G::contourAreas[bIndex] / B::contourAreas[aIndex];
        //supposed to be 0.31966, but looks like most are ~0.433
        const double idealRatio = 0.43;
        const double precision = 0.1;

            putText(canvas, string("r=") + std::to_string(ratio), closest_pairs[i].a.center + Point(0,-20), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
        if(ratio < idealRatio + precision && ratio > idealRatio - precision){
            circle(canvas, closest_pairs[i].a.center, 3, Scalar(32,255,255));
            targets.push_back(B::centers[aIndex]);
        }
    }

    imshow("targets", canvas);
}

void drawAll(const Mat &_orig, const vector<Point> &targets){
    Mat orig = _orig.clone();

    //using namespace B;
    /***************** B *****************/
    for(unsigned i = 0; i < B::polygons.size(); ++i){
        drawContours(orig, B::polygons, i, Scalar(32,255,255), 1, 8, B::hierarchy, 0, Point());
        //circle(orig, B::centers[i], 3, Scalar(32,255,255));
    }

    //for(unsigned i = 0; i < B::contourAreas.size(); ++i){
    //    string str("A=");
    //    str += std::to_string(B::contourAreas[i]);
    //    putText(orig, str, B::centers[i] + Point(0, 20), FONT_HERSHEY_PLAIN, 1, Scalar(255,200,200));
    //}

    /***************** G *****************/

    for(unsigned i = 0; i < G::contours.size(); ++i){
        drawContours(orig, G::contours, i, Scalar(32,255,255), 1, 8, G::hierarchy, 0, Point());
        //circle(orig, G::centers[i], 3, Scalar(32,255,255));
    }

    //for(unsigned i = 0; i < G::contourAreas.size(); ++i){
    //    string str("A=");
    //    str += std::to_string(G::contourAreas[i]);
    //    putText(orig, str, G::centers[i], FONT_HERSHEY_PLAIN, 1, Scalar(200,255,200));
    //}

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
    contourAreas.clear();
    hierarchy.clear();
    polygons.clear();
    boundRect.clear();
    centers.clear();
    //

    orig = _orig.clone();
    thresh = _frame.clone();

    prepFrame2(thresh, bMin, bMax);

    //canny edge detection
    Canny(thresh, canny_output, edgeThresh, maxEdgeThresh, 3);

    //find contours
    findContours(canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    //CV_RETR_EXTERNAL = get just external contours (no nesting)

    polygons.resize(contours.size());
    for(unsigned i = 0; i < contours.size(); ++i){
        //get polygons from contours
        approxPolyDP(contours[i], polygons[i], polyEpsilon, true);
    }

    //remove polygons that aren't squares
    polygons.erase(std::remove_if(polygons.begin(), polygons.end(),
                [](const auto &poly){ return poly.size() != 4; }), polygons.end());

    boundRect.reserve(polygons.size());
    centers.reserve(polygons.size());
    contourAreas.resize(polygons.size());

    for(unsigned i = 0; i < polygons.size(); ++i){
        //get bounding rectangle for each polygon
        boundRect[i] = boundingRect(polygons[i]);
        //find center of bounding rectangle
        centers[i] = Point(boundRect[i].x + boundRect[i].width/2, boundRect[i].y + boundRect[i].height/2);

        //get contour areas
        contourAreas[i] = contourArea(polygons[i]);
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

        drawContours(orig,    polygons, i, Scalar(32,255,255), 1, 8, hierarchy, 0, Point());

        circle(drawing, centers[i], 3, Scalar(32,255,255));
        circle(orig,    centers[i], 3, Scalar(32,255,255));
    }

    for(unsigned i = 0; i < contourAreas.size(); ++i){
        string str("A=");
        str += std::to_string(contourAreas[i]);
        putText(orig, str, centers[i], FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
    }

    //imshow("canny", canny_output);
    imshow("bthreshold", thresh);
    imshow("bdrawing", drawing);
    imshow("borig", orig);
}

void processG(const Mat &_frame, const Mat &_orig){
    using namespace G;

    contours.clear();
    contourAreas.clear();
    hierarchy.clear();
    boundRect.clear();
    centers.clear();
    //

    orig = _orig.clone();
    thresh = _frame.clone();

    prepFrame2(thresh, gMin, gMax);

    //canny edge detection
    Canny(thresh, canny_output, edgeThresh, maxEdgeThresh, 3);

    //find contours
    findContours(canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    //CV_RETR_EXTERNAL = get just external contours (no nesting)

    boundRect.reserve(contours.size());
    centers.reserve(contours.size());

    contourAreas.resize(contours.size());
    for(unsigned i = 0; i < contours.size(); ++i){
        //get contour areas
        contourAreas[i] = contourArea(contours[i]); 
        //get bounding rectangle for each contour 
        boundRect[i] = boundingRect(contours[i]);
        //find center of bounding rectangle
        centers[i] = Point(boundRect[i].x + boundRect[i].width/2, boundRect[i].y + boundRect[i].height/2);
    }

    ////remove contours with tiny area
    //const int minArea = 50;
    //int index = 0;
    //contourAreas.erase(std::remove_if(contourAreas.begin(), contourAreas.end(),
    //            [&](const double area) -> bool {
    //            ++index;
    //            if(area < minArea){
    //            contours.erase(contours.begin() + index);
    //            boundRect.erase(boundRect.begin() + index);
    //            centers.erase(centers.begin() + index);
    //            return true; 
    //            }
    //            return false;
    //            }), contourAreas.end());
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

    for(unsigned i = 0; i < contourAreas.size(); ++i){
        string str("A=");
        str += std::to_string(contourAreas[i]);
        putText(orig, str, centers[i], FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
    }

    //imshow("canny", canny_output);
    imshow("gthreshold", thresh);
    imshow("gdrawing", drawing);
    imshow("gorig", orig);
}
