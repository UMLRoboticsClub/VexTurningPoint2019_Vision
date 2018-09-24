//#define USE_WEBCAM
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>

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

void processB(const Mat &_frame, const Mat &_orig);
void drawB();
void processG(const Mat &_frame, const Mat &_orig);
void drawG();

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
    drawB();

    processG(processed, frame);
    drawG();
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
    contourAreas.resize(contours.size());
    for(unsigned i = 0; i < contours.size(); ++i){
        //get contour areas
        contourAreas[i] = contourArea(contours[i]); 

        //get polygons from contours
        approxPolyDP(contours[i], polygons[i], polyEpsilon, true);
    }

    //remove polygons that aren't squares
    polygons.erase(std::remove_if(polygons.begin(), polygons.end(),
                [](auto &poly){ return poly.size() != 4; }), polygons.end());

    boundRect.reserve(polygons.size());
    centers.reserve(polygons.size());

    for(unsigned i = 0; i < polygons.size(); ++i){
        //get bounding rectangle for each polygon
        boundRect[i] = boundingRect(polygons[i]);
        //find center of bounding rectangle
        centers[i] = Point(boundRect[i].x + boundRect[i].width/2, boundRect[i].y + boundRect[i].height/2);
    }
}

void drawB(){
    using namespace B;

    Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);

    for(unsigned i = 0; i < contours.size(); ++i){
        drawContours(drawing, contours, i, Scalar(32,32,32),   2, 8, hierarchy, 0, Point());
    }

    for(unsigned i = 0; i < polygons.size(); ++i){
        drawContours(drawing, polygons, i, Scalar(32,255,255), 1, 8, hierarchy, 0, Point());
        drawContours(orig,    polygons, i, Scalar(32,255,255), 1, 8, hierarchy, 0, Point());

        circle(drawing, centers[i], 3, Scalar(32,255,255));
        circle(orig,    centers[i], 3, Scalar(32,255,255));
    }

    for(unsigned i = 0; i < contours.size(); ++i){
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

    for(unsigned i = 0; i < contours.size(); ++i){
        string str("A=");
        str += std::to_string(contourAreas[i]);
        putText(orig, str, centers[i], FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
    }

    //imshow("canny", canny_output);
    imshow("gthreshold", thresh);
    imshow("gdrawing", drawing);
    imshow("gorig", orig);
}
