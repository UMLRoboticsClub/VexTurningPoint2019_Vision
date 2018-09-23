#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>
#include <vector>
#include <chrono>

using std::cout;
using std::endl;
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

enum class TargetColor { RED, BLUE, GREEN };

//Color to be tracked
//#define MIN_H_BLUE 200
//#define MAX_H_BLUE 300
//HSV
//red
//Scalar minColorRange(345, 100, 100);
//Scalar maxColorRange(30, 100, 100);

//blue
Scalar minColorRange(100, 70, 70);
Scalar maxColorRange(150, 255, 255);

//green
//Scalar minColorRange(50, 140, 170);
//Scalar maxColorRange(70, 255, 255);

//inRange(frmHsv, Scalar(MIN_H_BLUE / 2, 100, 80), Scalar(MAX_H_BLUE / 2, 255, 255), rangeRes);

//#define USE_WEBCAM

void processFrame(Mat &frame);
void prepFrame(Mat &frame);

const auto &getTime = []{
    return duration<double>(high_resolution_clock::now().time_since_epoch()).count();
};

int main(){
    VideoCapture cap;
    if (!cap.open(cam_index)){
        cout << "cannot open video device\n";
        return EXIT_FAILURE;
    }

    //cout << "\nHit 'q' to exit...\n";

    //Mat frame = imread("testimage.png");
    Mat frame;
#ifndef USE_WEBCAM
    frame = imread(test_image);
#else
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
    //Color Thresholding
    Mat rangeRes = Mat::zeros(frame.size(), CV_8UC1);
    inRange(frame, minColorRange, maxColorRange, frame);

    //Improving the result
    erode(frame, frame, Mat(), Point(-1, -1), 2);
    dilate(frame, frame, Mat(), Point(-1, -1), 2);
}

void processFrame(Mat &frame){
    Mat orig = frame.clone();

    //noise smoothing
    GaussianBlur(frame, frame, Size(5, 5), 3.0, 3.0);
    //HSV conversion
    cvtColor(frame, frame, CV_BGR2HSV);

    prepFrame(frame);


    imshow("Threshold", frame);

    Mat canny_output;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    //canny edge detection
    Canny(frame, canny_output, edgeThresh, maxEdgeThresh, 3);

    //find contours
    findContours(canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    //CV_RETR_EXTERNAL = get just external contours (no nesting)

    //get polygons from contours
    vector<vector<Point>> polygons(contours.size());
    for(unsigned i = 0; i < contours.size(); ++i){
        approxPolyDP(contours[i], polygons[i], polyEpsilon, true);
    }

    //remove polygons that aren't squares
    polygons.erase(std::remove_if(polygons.begin(), polygons.end(),
                [](auto &poly){ return poly.size() != 4; }), polygons.end());

    vector<Rect> boundRect;(polygons.size());
    boundRect.reserve(polygons.size());

    vector<Point> centers;
    centers.reserve(polygons.size());

    Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);

    for(unsigned i = 0; i < polygons.size(); ++i){
        //get bounding rectangle for each polygon
        boundRect[i] = boundingRect(polygons[i]);
        //find center of bounding rectangle
        centers[i] = Point(boundRect[i].x + boundRect[i].width/2, boundRect[i].y + boundRect[i].height/2);

        //draw stuff
        drawContours(drawing, contours, i, Scalar(32,32,32), 2, 8, hierarchy, 0, Point());
        drawContours(drawing, polygons, i, Scalar(32,255,255), 1, 8, hierarchy, 0, Point());
        drawContours(orig,    polygons, i, Scalar(32,255,255), 1, 8, hierarchy, 0, Point());

        circle(drawing, centers[i], 3, Scalar(32,255,255));
        circle(orig,    centers[i], 3, Scalar(32,255,255));
    }

    //imshow("canny", canny_output);
    imshow("contours", drawing);
    imshow("orig", orig);
}
