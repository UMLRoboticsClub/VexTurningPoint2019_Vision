#define USE_WEBCAM
//#define DEBUG
//#define DEBUG_OUTPUT
#define DRAW_OVERLAY
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <algorithm>
#include <thread>
#include <typeinfo>
#include <cmath>

#include "crc.h"

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::thread;
using namespace cv;

enum Color { BLUE, RED, GREEN };

struct Pair {
    Pair(int a, int b, int dist):
        a(a), b(b), dist(dist){}
    int a, b;
    int dist;
};

/* settings */
const int cam_index = 0;
static string test_image = "../img/field2.jpg";

/* params to tune */

//ratio of dist to green/
const float minRatio = 0.3;
const float maxRatio = 0.65;
//canny params
const int edgeThresh = 100;
const int maxEdgeThresh = 200;
//polygon approximation
const double polyEpsilon = 14;

/* HSV color ranges for flags */

//blue
const int bSensitivity = 20;
const static Scalar bMin(120 - bSensitivity, 70, 70);
const static Scalar bMax(120 + bSensitivity, 255, 255);

//green
const int gSensitivity = 28;
const static Scalar gMin(60 - gSensitivity, 25, 100);
const static Scalar gMax(60 + gSensitivity, 255, 255);

//red
//this has 2 ranges because red 
//wraps around the HSV spectrum
const int rSensitivity = 20;
const static Scalar rMin1(0, 70, 70);
const static Scalar rMax1(rSensitivity/2, 255, 255);
const static Scalar rMin2(180-rSensitivity/2, 70, 70);
const static Scalar rMax2(180, 255, 255);

namespace F {
    static vector<vector<Point>> contours;
    static vector<Vec4i> hierarchy;
    static vector<vector<Point>> polygons;
    static vector<double> pAreas;
    static vector<Point> centers;

    static Mat thresh;
    static Mat canny_output;
};

namespace G {
    static vector<vector<Point>> contours;
    static vector<Vec4i> hierarchy;
    static vector<Point> centers;

    static Mat thresh;
    static Mat canny_output;
};

namespace R {
    static Mat mask1, mask2;
}

//globals
Color team = BLUE;
static vector<Pair> closest_pairs;
static vector<Point> targets;
static Size frameSize;

//extract all info from frame
void processFrame(Mat &_frame);
//blur and convert colors
void prepFrame(Mat &frame);
//filter color and refine mask
void filterColor(Mat &frame, Color color);
//finally determine where the flags are
void findTargets(vector<Point> &targets);

//process flag colored blobs
void processF(const Mat &_frame);
//process green colored blobs
void processG(const Mat &_frame);
//draw thresholds and bounding boxes for everything
void drawDbg(const Mat &_orig);
//draw original image overlayed with found targets
void drawOverlay(const Mat &_orig, const vector<Point> &targets);

//send target data to v5
void sendTargets();

int main(int argc, char **argv){
    const auto &getTime = []{
        using namespace std::chrono;
        return duration<double>(high_resolution_clock::now().time_since_epoch()).count();
    };

    switch(argc){
        case 3:
            test_image = argv[2];
            [[fallthrough]];
        case 2:
            if(strcmp(argv[1], "r") == 0){
                team = RED;
            }
            break;
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
    frameSize = frame.size();
#endif
#ifdef USE_WEBCAM
    cap >> frame;
    frameSize = frame.size();


    //for c930e, these need to be set after first frame is received
    cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M','J','P','G'));
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
    //cap.set(CV_CAP_PROP_FPS, 30); //not really needed
    //hold only one frame in buffer for less latency (default is 5)
    cap.set(CV_CAP_PROP_BUFFERSIZE, 1);

    cout << "\nHit 'q' to exit...\n";
    while(waitKey(1) != 'q'){
        cap >> frame;
#endif
        const double before = getTime();
        processFrame(frame);
        const double delta = getTime() - before;
        cout << "Time elapsed: " << delta * 1000.f << " ms" << endl;

        sendTargets();

        cout << endl;

#ifdef USE_WEBCAM
    }
#else
    waitKey(0);
#endif
}

void sendTargets(){
    auto digits = [](int32_t num) -> size_t {
        bool neg = false;
        if(num < 0){
            num *= -1; //make it pos
            neg = true;
        }
        int len = 1;

        //this looks really dumb, but it's pretty efficient and simple
        //todo improvement: binary search the length
        if     (num < 10)          len =  1; 
        else if(num < 100)         len =  2; 
        else if(num < 1000)        len =  3; 
        else if(num < 10000)       len =  4; 
        else if(num < 100000)      len =  5; 
        else if(num < 1000000)     len =  6; 
        else if(num < 10000000)    len =  7; 
        else if(num < 100000000)   len =  8; 
        else if(num < 1000000000)  len =  9; 
        else len = 10; 

        if(neg){ return ++len; }
        return len;
    };

    if(targets.size() == 0){ return; }

    const int dataBufSize = 256;
    //absolute maximum should be 216 digits
    char databuf[dataBufSize];

    int len = 0;
    for(unsigned i = 0; i < targets.size(); ++i){
        len += snprintf(databuf + len, dataBufSize - len, "%d %d ", targets[i].x, targets[i].y); //x[space]y[space]
    }

    uint32_t crc = crc32buf(databuf, len - 1);

    const int sizeSize = digits(targets.size()) + 1;
    const int crcSize = digits(crc) + 1;
    const char *header = "zz ";
    const int headerSize = strlen(header);
    const int bufSize = len + sizeSize + crcSize + headerSize;
    char *buf = new char[bufSize];
    
    snprintf(buf, headerSize + 1, "%s", header);
    //add length to beginning
    snprintf(buf + headerSize, sizeSize + headerSize, "%d ", (int)targets.size());
    //copy over databuf
    memcpy(buf + sizeSize + headerSize, databuf, len);
    //add crc to end
    snprintf(buf + len + sizeSize + headerSize, crcSize, "%u", crc);
    buf[bufSize - 1] = '\n';

#ifdef DEBUG_OUTPUT
    //all this tricky stuff
    // databuf:[\|\|\databuf\|\|\]
    //     buf:[_____\|\|\|databuf|\|\_____]
    //     buf:[len__\|\|\|databuf|\|\_____]
    //     buf:[len__\|\|\|databuf|\|\_crc_]

    for(unsigned i = 0; i < targets.size(); ++i){
        cout << targets[i];
    }
    cout << endl;

    cout << "crc:" << crc << endl;
    cout << "crcsize:" << digits(crc) << endl;
    cout << "\nlen:" << len << endl;
    cout << "databuf:";
    for(int i = 0; i < len; ++i){
        cout << '[' << databuf[i] << ']';
    }
    cout << endl;

    cout << "\nbuf:";
    for(int i = 0; i < bufSize; ++i){
        cout << '[' << buf[i] << ']';
    }
    cout << endl;
#endif

    for(int i = 0; i < bufSize; ++i){
        putchar(buf[i]);
    }
#ifdef DEBUG_OUTPUT
    cout << endl << endl;
#endif

    delete[] buf;
}

void prepFrame(Mat &frame){
    //noise smoothing
    GaussianBlur(frame, frame, Size(5, 5), 3.0, 3.0);
    //HSV conversion
    cvtColor(frame, frame, COLOR_BGR2HSV);
}

void filterColor(Mat &frame, Color color){
    //color thresholding
    switch(color){
        case BLUE:
            inRange(frame, bMin, bMax, frame);
            break;
        case RED:
            {
                using namespace R;
                inRange(frame, rMin1, rMax1, mask1);
                inRange(frame, rMin2, rMax2, mask2);
                frame = mask1|mask2;
            }
            break;
        case GREEN:
            inRange(frame, gMin, gMax, frame);
            break;
    }

    //Improving the result
    erode(frame, frame, Mat(), Point(-1, -1), 2);
    dilate(frame, frame, Mat(), Point(-1, -1), 2);
}

void processFrame(Mat &frame){
#if defined(DEBUG) || defined(DRAW_OVERLAY)
    Mat _frame = frame.clone();
#endif
    prepFrame(frame);

    //processF(frame);
    //processG(frame);

    thread t_b([&](){ processF(frame); });
    thread t_g([&](){ processG(frame); });

    targets.clear();
    targets.reserve(F::polygons.size());

    t_b.join();
    t_g.join();

    findTargets(targets);
    cout << "found " << targets.size() << " targets" << endl;

#ifdef DEBUG
    drawDbg(_frame);
#endif
#ifdef DRAW_OVERLAY
    drawOverlay(_frame, targets);
#endif
}

void findTargets(vector<Point> &targets){
    auto dist = [](const Point& pt1, const Point& pt2) -> int {
        const int deltaX = pt1.x - pt2.x;
        const int deltaY = pt1.y - pt2.y;
        return (deltaX * deltaX) + (deltaY * deltaY);
    };

    closest_pairs.clear();
    closest_pairs.reserve(F::polygons.size());

    for(unsigned i = 0; i < F::polygons.size(); ++i){
        int smallestVal = INT_MAX;
        int smallestObjIndex = 0;
        for(unsigned j = 0; j < G::contours.size(); ++j){
            int val = dist(F::centers[i], G::centers[j]);
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
    Mat canvas = Mat::zeros(frameSize, CV_8UC3);
#endif
    for(unsigned i = 0; i < closest_pairs.size(); ++i){
        const float ratio = closest_pairs[i].dist / F::pAreas[closest_pairs[i].a];
        //cout << "ratio:" << ratio << endl;

        if(ratio > minRatio && ratio < maxRatio){
            targets.push_back(F::centers[closest_pairs[i].a]);

#ifdef DEBUG
            circle(canvas, F::centers[closest_pairs[i].a], 3, Scalar(32,255,255));
            line(canvas, F::centers[closest_pairs[i].a], G::centers[closest_pairs[i].b], Scalar(255,255,255));
#endif
        }

#ifdef DEBUG
        putText(canvas, string("dist:") + std::to_string(closest_pairs[i].dist), F::centers[closest_pairs[i].a] + Point(0,-10), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
        putText(canvas, string("area:") + std::to_string(F::pAreas[closest_pairs[i].a]), F::centers[closest_pairs[i].a] + Point(0,10), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
#endif
    }

#ifdef DEBUG
    imshow("targets", canvas);
#endif
}

void drawDbg(const Mat &_orig){
    Mat orig = _orig.clone();

    /***************** B *****************/
    for(unsigned i = 0; i < F::polygons.size(); ++i){
        drawContours(orig, F::polygons, i, Scalar(32,255,255), 1, 8, F::hierarchy, 0, Point());
        //circle(orig, F::centers[i], 3, Scalar(32,255,255));
    }

    Mat drawing = Mat::zeros(frameSize, CV_8UC3);

    for(unsigned i = 0; i < F::contours.size(); ++i){
        drawContours(drawing, F::contours, i, Scalar(32,32,32), 2, 8, F::hierarchy, 0, Point());
    }

    for(unsigned i = 0; i < F::polygons.size(); ++i){
        drawContours(drawing, F::polygons, i, Scalar(32,255,255), 1, 8, F::hierarchy, 0, Point());

        //circle(drawing, polygons[i][0], 3, Scalar(32,255,255));
        //circle(drawing, polygons[i][1], 3, Scalar(32,255,255));
        //circle(drawing, polygons[i][2], 3, Scalar(32,255,255));
        //circle(drawing, polygons[i][3], 3, Scalar(32,255,255));

        drawContours(drawing, F::polygons, i, Scalar(32,255,255), 1, 8, F::hierarchy, 0, Point());
        circle(drawing, F::centers[i], 3, Scalar(32,255,255));
    }

    //imshow("canny", canny_output);
    imshow("bthreshold", F::thresh);
    imshow("bdrawing", drawing);

    /***************** G *****************/

    for(unsigned i = 0; i < G::contours.size(); ++i){
        drawContours(orig, G::contours, i, Scalar(32,255,255), 1, 8, G::hierarchy, 0, Point());
        //circle(orig, G::centers[i], 3, Scalar(32,255,255));
    }

    drawing = Mat::zeros(frameSize, CV_8UC3);

    for(unsigned i = 0; i < G::contours.size(); ++i){
        //draw stuff
        drawContours(drawing, G::contours, i, Scalar(32,32,32), 2, 8, G::hierarchy, 0, Point());
        drawContours(drawing, G::contours, i, Scalar(32,255,255), 1, 8, G::hierarchy, 0, Point());

        circle(drawing, G::centers[i], 3, Scalar(32,255,255));
    }

    //imshow("canny", canny_output);
    imshow("gthreshold", G::thresh);
    imshow("gdrawing", drawing);
}

void drawOverlay(const Mat &_orig, const vector<Point> &targets){
    Mat orig = _orig.clone();
    for(unsigned i = 0; i < F::polygons.size(); ++i){
        drawContours(orig, F::polygons, i, Scalar(32,255,255), 1, 8, F::hierarchy, 0, Point());
    }
    for(unsigned i = 0; i < G::contours.size(); ++i){
        drawContours(orig, G::contours, i, Scalar(32,255,255), 1, 8, G::hierarchy, 0, Point());
    }
    for(unsigned i = 0; i < targets.size(); ++i){
        circle(orig, targets[i], 8, Scalar(0,0,0), -1);
        circle(orig, targets[i], 3, Scalar(255,255,255), -1);
        putText(orig, "TARGET", targets[i] + Point(5,-5), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
    }

    imshow("overlay", orig);
}

void processF(const Mat &_frame){
    using namespace F;

    contours.clear();
    hierarchy.clear();
    polygons.clear();
    centers.clear();

    _frame.copyTo(thresh);

    filterColor(thresh, team);

    //canny edge detection
    Canny(thresh, canny_output, edgeThresh, maxEdgeThresh, 3);

    //find contours
    //CV_RETR_EXTERNAL = get just external contours (no nesting)
    findContours(canny_output, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

    if(contours.size() == 0) return;

    polygons.resize(contours.size());
    for(unsigned i = 0; i < contours.size(); ++i){
        //get polygons from contours
        approxPolyDP(contours[i], polygons[i], polyEpsilon, true);
    }

    //remove polygons that aren't squares
    polygons.erase(std::remove_if(polygons.begin(), polygons.end(),
                [](const auto &poly){ return poly.size() != 4; }), polygons.end());

    if(polygons.size() == 0) return;

    centers.reserve(polygons.size());
    pAreas.resize(polygons.size());

    Rect boundRect;
    for(unsigned i = 0; i < polygons.size(); ++i){
        //get bounding rectangle for each polygon
        boundRect = boundingRect(polygons[i]);
        //find center of bounding rectangle
        centers.emplace_back(boundRect.x + boundRect.width/2, boundRect.y + boundRect.height/2);
        //find area of polygon
        pAreas[i] = contourArea(polygons[i]);
    }
}

void processG(const Mat &_frame){
    using namespace G;

    contours.clear();
    hierarchy.clear();
    centers.clear();

    _frame.copyTo(thresh);

    filterColor(thresh, GREEN);

    //canny edge detection
    Canny(thresh, canny_output, edgeThresh, maxEdgeThresh, 3);

    //find contours
    //CV_RETR_EXTERNAL = get just external contours (no nesting)
    findContours(canny_output, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

    if(contours.size() == 0) return;

    centers.reserve(contours.size());

    Rect boundRect;
    for(unsigned i = 0; i < contours.size(); ++i){
        //get bounding rectangle for each contour 
        boundRect = boundingRect(contours[i]);
        //find center of bounding rectangle
        centers.emplace_back(boundRect.x + boundRect.width/2, boundRect.y + boundRect.height/2);
    }
}
