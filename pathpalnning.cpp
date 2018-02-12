
/* code to apply A-star algorithm on a defined arena "image available in the folder"  */


#include <fstream>
#include <stdio.h>
#include <bits/stdc++.h>
#include <iostream>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <cstring>
#include <vector>
#include <list>
#include <sys/time.h>
#include <cmath>
#include <string.h>
////////////////////////////////////version 1
using namespace cv;
using namespace std;

#ifndef __APPLE__
#define EXPOSURE_CONTROL // only works in Linux
#endif

#ifdef EXPOSURE_CONTROL
#include <libv4l2.h>

#include <linux/videodev2.h>
#include <fcntl.h>
#include <errno.h>
#endif
#include "opencv2/opencv.hpp"

#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"
#include "AprilTags/TagDetection.h"

#include <unistd.h>
extern int optind;
extern char *optarg;

#include "Serial.h"
//const char* windowName = "apriltags_demo";


const string usage = "\n"
  "Usage:\n"
  "  apriltags_demo [OPTION...] [IMG1 [IMG2...]]\n"
  "\n"
  "Options:\n"
  "  -h  -?          Show help options\n"
  "  -a              Arduino (send tag ids over serial port)\n"
  "  -d              Disable graphics\n"
  "  -t              Timing of tag extraction\n"
  "  -C <bbxhh>      Tag family (default 36h11)\n"
  "  -D <id>         Video device ID (if multiple cameras present)\n"
  "  -F <fx>         Focal length in pixels\n"
  "  -W <width>      Image width (default 640, availability depends on camera)\n"
  "  -H <height>     Image height (default 480, aivailability depends on camera)\n"
  "  -S <size>       Tag size (square black frame) in meters\n"
  "  -E <exposure>   Manually set camera exposure (default auto; range 0-10000)\n"
  "  -G <gain>       Manually set camera gain (default auto; range 0-255)\n"
  "  -B <brightness> Manually set the camera brightness (default 128; range 0-255)\n"
  "\n";

const string intro = "\n"
    "April tags test code\n"
    "(C) 2012-2014 Massachusetts Institute of Technology\n"
    "Michael Kaess\n"
    "\n";
    
  double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

#include <cmath>

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;
double yaw1;
/**
 * Normalize angle to be within the interval [-pi,pi].
 */
inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}

void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    yaw1 = yaw;
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}
/////////////////////////

float Cx,Cy;
//int ctr ;//////////////////////////////if no apriltag is detected ctr = 1;

////////////////////////////FUNCTIONS OF APRILTAG///////////

class Demo {

  AprilTags::TagDetector* m_tagDetector;
  AprilTags::TagCodes m_tagCodes;

  bool m_draw; // draw image and April tag detections?
  bool m_arduino; // send tag detections to serial port?
  bool m_timing; // print timing information for each tag extraction call

  int m_width; // image size in pixels
  int m_height;
  double m_tagSize; // April tag side length in meters of square black frame
  double m_fx; // camera focal length in pixels
  double m_fy;
  double m_px; // camera principal point
  double m_py;

  int m_deviceId; // camera id (in case of multiple cameras)

  list<string> m_imgNames;

  cv::VideoCapture m_cap;

  int m_exposure;
  int m_gain;
  int m_brightness;

  Serial m_serial;

public:

  // default constructor
  Demo() :
    // default settings, most can be modified through command line options (see below)
    m_tagDetector(NULL),
    m_tagCodes(AprilTags::tagCodes36h11),

    m_draw(true),
    m_arduino(false),
    m_timing(false),

	m_width(350),
    m_height(350),
    m_tagSize(0.105),
    
    m_fx(400),
    m_fy(400),
    m_px(m_width/2),
    m_py(m_height/2),

    m_exposure(20),
    m_gain(100),
    m_brightness(150),

    m_deviceId(1)
  {}
  
   void parseOptions(int argc, char* argv[]) {
    int c;
    while ((c = getopt(argc, argv, ":h?adtC:F:H:S:W:E:G:B:D:")) != -1) {
      // Each option character has to be in the string in getopt();
      // the first colon changes the error character from '?' to ':';
      // a colon after an option means that there is an extra
      // parameter to this option; 'W' is a reserved character
      switch (c) {
      case 'h':
      case '?':
        cout << intro;
        cout << usage;
        exit(0);
        break;
      case 'a':
        m_arduino = true;
        break;
      case 'd':
        m_draw = false;
        break;
      case 't':
        m_timing = true;
        break;
      case 'C':
        setTagCodes(optarg);
        break;
      case 'F':
        m_fx = atof(optarg);
        m_fy = m_fx;
        break;
      case 'H':
        m_height = atoi(optarg);
        m_py = m_height/2;
         break;
      case 'S':
        m_tagSize = atof(optarg);
        break;
      case 'W':
        m_width = atoi(optarg);
        m_px = m_width/2;
        break;
      case 'E':
#ifndef EXPOSURE_CONTROL
        cout << "Error: Exposure option (-E) not available" << endl;
        exit(1);
#endif
        m_exposure = atoi(optarg);
        break;
      case 'G':
#ifndef EXPOSURE_CONTROL
        cout << "Error: Gain option (-G) not available" << endl;
        exit(1);
#endif
        m_gain = atoi(optarg);
        break;
      case 'B':
#ifndef EXPOSURE_CONTROL
        cout << "Error: Brightness option (-B) not available" << endl;
        exit(1);
#endif
        m_brightness = atoi(optarg);
        break;
      case 'D':
        m_deviceId = atoi(optarg);
        break;
      case ':': // unknown option, from getopt
        cout << intro;
        cout << usage;
        exit(1);
        break;
      }
       if (argc > optind) {
      for (int i=0; i<argc-optind; i++) {
        m_imgNames.push_back(argv[optind+i]);
      }
    }
}
}
    


   void setTagCodes(string s) {
    if (s=="16h5") {
      m_tagCodes = AprilTags::tagCodes16h5;
    } else if (s=="25h7") {
      m_tagCodes = AprilTags::tagCodes25h7;
    } else if (s=="25h9") {
      m_tagCodes = AprilTags::tagCodes25h9;
    } else if (s=="36h9") {
      m_tagCodes = AprilTags::tagCodes36h9;
    } else if (s=="36h11") {
      m_tagCodes = AprilTags::tagCodes36h11;
    } else {
      cout << "Invalid tag family specified" << endl;
      exit(1);
    }
  }


   void print_detection(AprilTags::TagDetection& detection) const {
    std::pair< float, float > cxy = detection.cxy;
    Cx = cxy.first;
    Cy= cxy.second;
    float stepSize = 48;
    int x = (Cx/stepSize);
    int y = (Cy/stepSize);
    cout << "Cx = " << Cx
	<< " Cy = " << Cy << endl;
	cout << "bot is in the (" << x << "," << y << ")" <<endl;

    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                             translation, rotation);

    Eigen::Matrix3d F;
    F <<
      1, 0,  0,
      0,  -1,  0,
      0,  0,  1;
    Eigen::Matrix3d fixed_rot = F*rotation;
    double yaw, pitch, roll;
    wRo_to_euler(fixed_rot, yaw, pitch, roll);

    cout << "  distance=" << translation.norm()
        // << "m, x=" << translation(0)
         << ", y=" << translation(1)
         << ", z=" << translation(2)
         << ", yaw=" << yaw
        // << ", pitch=" << pitch
        // << ", roll=" << roll
         << endl;

  }
  
    void setup() {
    m_tagDetector = new AprilTags::TagDetector(m_tagCodes);

    // prepare window for drawing the camera images
    if (m_draw) {
     // cv::namedWindow(windowName, CV_WINDOW_NORMAL);
    }
  }
  
  void processImage(cv::Mat& image, cv::Mat& image_gray) {
 
    //ctr =0;
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    double t0;
    if (m_timing) {
      t0 = tic();
    }
    vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);
    if (m_timing) {
      double dt = tic()-t0;
      cout << "Extracting tags took " << dt << " seconds." << endl;
    }

    // print out each detection
   // cout << detections.size() << " tags detected:" << endl;
 //  ctr = detections.size();
    for (int i=0; i<detections.size(); i++) {
      print_detection(detections[i]);
    }

    // show the current image including any detections
    if (m_draw) {
      for (int i=0; i<detections.size(); i++) {
        // also highlight in the image
        detections[i].draw(image);
      }
     // imshow(windowName, image); // OpenCV call
    }

    // optionally send tag information to serial port (e.g. to Arduino)
    if (m_arduino) {
      if (detections.size() > 0) {
        // only the first detected tag is sent out for now
        Eigen::Vector3d translation;
        Eigen::Matrix3d rotation;
        detections[0].getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                                     translation, rotation);
        m_serial.print(detections[0].id);
        m_serial.print(",");
        m_serial.print(translation(0));
        m_serial.print(",");
        m_serial.print(translation(1));
        m_serial.print(",");
        m_serial.print(translation(2));
        m_serial.print("\n");
      } else {
        // no tag detected: tag ID = -1
        m_serial.print("-1,0.0,0.0,0.0\n");
      }
    }
  }
 };

//////////////////////////////////////////////////////////////

///////////////////////////// A-STAR ALGORITHM/////////////////////////////////
int shortestPath[2][50];
int sp[2][50];
float cent[2][7][7];

#define ROW 7
#define COL 7


typedef pair<int, int> Pair;
typedef pair<double, pair<int, int> > pPair;

struct cell
{
    int parent_i, parent_j;
    double f, g, h;
};

bool isValid(int row, int col)
{

    return (row >= 0) && (row < ROW) &&
           (col >= 0) && (col < COL);
}

bool isUnBlocked(int grid[][COL], int row, int col)
{
    if (grid[row][col] == 1)
        return (true);
    else
        return (false);
}

bool isDestination(int row, int col, Pair dest)
{
    if (row == dest.first && col == dest.second)
        return (true);
    else
        return (false);
}

double calculateHValue(int row, int col, Pair dest)
{
  
    return ((double)sqrt ((row-dest.first)*(row-dest.first)
                          + (col-dest.second)*(col-dest.second)));
}

void tracePath(cell cellDetails[][COL], Pair dest)
{
    printf ("\nThe Path is ");
    int row = dest.first;
    int col = dest.second;

    stack<Pair> Path;

    while (!(cellDetails[row][col].parent_i == row
             && cellDetails[row][col].parent_j == col ))
    {
        Path.push (make_pair (row, col));
        int temp_row = cellDetails[row][col].parent_i;
        int temp_col = cellDetails[row][col].parent_j;
        row = temp_row;
        col = temp_col;
    }

    Path.push (make_pair (row, col));
   int k=0;
    while (!Path.empty())
    {
        pair<int,int> p = Path.top();
        Path.pop();
        shortestPath[0][k] = p.first;
        shortestPath[1][k] = p.second;
        printf("-> (%d,%d) ",p.first,p.second);
        k++;
    }

    return;
}

void aStarSearch(int grid[][COL], Pair src, Pair dest)
{
    if (isValid (src.first, src.second) == false)
    {
        printf ("Source is invalid\n");
        return;
    }

    // If the destination is out of range
    if (isValid (dest.first, dest.second) == false)
    {
        printf ("Destination is invalid\n");
        return;
    }

    // Either the source or the destination is blocked
    if (isUnBlocked(grid, src.first, src.second) == false ||
            isUnBlocked(grid, dest.first, dest.second) == false)
    {
        printf ("Source or the destination is blocked\n");
        return;
    }

    // If the destination cell is the same as source cell
    if (isDestination(src.first, src.second, dest) == true)
    {
        printf ("We are already at the destination\n");
        return;
    }

    bool closedList[ROW][COL];
    memset(closedList, false, sizeof (closedList));

    cell cellDetails[ROW][COL];

    int i, j;

    for (i=0; i<ROW; i++)
    {
        for (j=0; j<COL; j++)
        {
            cellDetails[i][j].f = FLT_MAX;
            cellDetails[i][j].g = FLT_MAX;
            cellDetails[i][j].h = FLT_MAX;
            cellDetails[i][j].parent_i = -1;
            cellDetails[i][j].parent_j = -1;
        }
    }

    // Initialising the parameters of the starting node
    i = src.first, j = src.second;
    cellDetails[i][j].f = 0.0;
    cellDetails[i][j].g = 0.0;
    cellDetails[i][j].h = 0.0;
    cellDetails[i][j].parent_i = i;
    cellDetails[i][j].parent_j = j;


    set<pPair> openList;


    openList.insert(make_pair (0.0, make_pair (i, j)));

    bool foundDest = false;

    while (!openList.empty())
    {
        pPair p = *openList.begin();

        openList.erase(openList.begin());
        i = p.second.first;
        j = p.second.second;
        closedList[i][j] = true;
     
        double gNew, hNew, fNew;

        //----------- 1st Successor (North) ------------

        // Only process this cell if this is a valid one
        if (isValid(i-1, j) == true)
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i-1, j, dest) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i-1][j].parent_i = i;
                cellDetails[i-1][j].parent_j = j;
                printf ("The destination cell is found\n");
                tracePath (cellDetails, dest);
                foundDest = true;
                return;
            }
            else if (closedList[i-1][j] == false &&
                     isUnBlocked(grid, i-1, j) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue (i-1, j, dest);
                fNew = gNew + hNew;
                
                if (cellDetails[i-1][j].f == FLT_MAX ||
                        cellDetails[i-1][j].f > fNew)
                {
                    openList.insert( make_pair(fNew,make_pair(i-1, j)));

                    // Update the details of this cell
                    cellDetails[i-1][j].f = fNew;
                    cellDetails[i-1][j].g = gNew;
                    cellDetails[i-1][j].h = hNew;
                    cellDetails[i-1][j].parent_i = i;
                    cellDetails[i-1][j].parent_j = j;
                }
            }
        }

        //----------- 2nd Successor (South) ------------

        // Only process this cell if this is a valid one
        if (isValid(i+1, j) == true)
        {
           
            if (isDestination(i+1, j, dest) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i+1][j].parent_i = i;
                cellDetails[i+1][j].parent_j = j;
                printf("The destination cell is found\n");
                tracePath(cellDetails, dest);
                foundDest = true;
                return;
            }
    
            else if (closedList[i+1][j] == false &&
                     isUnBlocked(grid, i+1, j) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i+1, j, dest);
                fNew = gNew + hNew;

                if (cellDetails[i+1][j].f == FLT_MAX ||
                        cellDetails[i+1][j].f > fNew)
                {
                    openList.insert( make_pair (fNew, make_pair (i+1, j)));
                    // Update the details of this cell
                    cellDetails[i+1][j].f = fNew;
                    cellDetails[i+1][j].g = gNew;
                    cellDetails[i+1][j].h = hNew;
                    cellDetails[i+1][j].parent_i = i;
                    cellDetails[i+1][j].parent_j = j;
                }
            }
        }

        //----------- 3rd Successor (East) ------------

        // Only process this cell if this is a valid one
        if (isValid (i, j+1) == true)
        {
          
            if (isDestination(i, j+1, dest) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i][j+1].parent_i = i;
                cellDetails[i][j+1].parent_j = j;
                printf("The destination cell is found\n");
                tracePath(cellDetails, dest);
                foundDest = true;
                return;
            }


            else if (closedList[i][j+1] == false &&
                     isUnBlocked (grid, i, j+1) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue (i, j+1, dest);
                fNew = gNew + hNew;

                if (cellDetails[i][j+1].f == FLT_MAX ||
                        cellDetails[i][j+1].f > fNew)
                {
                    openList.insert( make_pair(fNew,
                                        make_pair (i, j+1)));


                    cellDetails[i][j+1].f = fNew;
                    cellDetails[i][j+1].g = gNew;
                    cellDetails[i][j+1].h = hNew;
                    cellDetails[i][j+1].parent_i = i;
                    cellDetails[i][j+1].parent_j = j;
                }
            }
        }

        //----------- 4th Successor (West) ------------

        // Only process this cell if this is a valid one
        if (isValid(i, j-1) == true)
        {
 
            if (isDestination(i, j-1, dest) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i][j-1].parent_i = i;
                cellDetails[i][j-1].parent_j = j;
                printf("The destination cell is found\n");
                tracePath(cellDetails, dest);
                foundDest = true;
                return;
            }

            else if (closedList[i][j-1] == false &&
                     isUnBlocked(grid, i, j-1) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i, j-1, dest);
                fNew = gNew + hNew;

                if (cellDetails[i][j-1].f == FLT_MAX ||
                        cellDetails[i][j-1].f > fNew)
                {
                    openList.insert( make_pair (fNew,
                                          make_pair (i, j-1)));

                    // Update the details of this cell
                    cellDetails[i][j-1].f = fNew;
                    cellDetails[i][j-1].g = gNew;
                    cellDetails[i][j-1].h = hNew;
                    cellDetails[i][j-1].parent_i = i;
                    cellDetails[i][j-1].parent_j = j;
                }
            }
        }
     }
   
    if (foundDest == false){
        printf("Failed to find the Destination Cell\n");

    return;
}
}

//////////////////////////////

int main( int argc, char** argv )
{
    //////////////////////////////////////////////////////////
    
    for(int i =0; i <7;i++)
    {
		for(int j=0;j<7;j++)
		{
			cent[0][i][j] = (48*i)+24;
			cent[1][i][j] = (48*j)+24;
		}
	}
    
    
    ////////////////////////////////////////////////////////
    VideoCapture cap(1); 

    if ( !cap.isOpened() )
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }
    

	double largest_area=0;
	int largest_contour_index=0;

	Rect bounding_rect;
	Mat img;
	cap.read(img);

/////////////////////////////

	Mat roi1 =img( Rect(0,0,320,240));

	Mat imgHSV;

	cvtColor(roi1, imgHSV, COLOR_BGR2HSV);
 
	Mat imgThresholded;
 	int iLowH = 113;
	int iHighH = 162;

	int iLowS = 59; 
	int iHighS = 180;

	int iLowV = 67;
	int iHighV = 255;

	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
   
	//morphological opening (removes small objects from the foreground)
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

	//morphological closing (removes small holes from the foreground)
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  
	vector<Vec4i> hierarchy1;//hierarchy of contours
	vector< vector<Point> > contours; // Vector for storing selected contour
       
 
	findContours(imgThresholded , contours, hierarchy1,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE); // Find the contours in the image
        
	for( int i = 0; i< contours.size(); i++ ) // iterate through each contour.
	{
		double a=contourArea( contours[i],false);  //  Find the area of contour
       
		if(a>largest_area){
		largest_area=a;
		largest_contour_index=i;                //Store the index of largest contour
       
		}
 
	}
	bounding_rect=boundingRect(contours[largest_contour_index]);
	rectangle(img,bounding_rect,Scalar(0,250,0),1,8,0); 
     
	///////Centroid detection of 1st vertex
		 /// Get the moments
  vector<Moments> mu1(contours.size() );
  
     mu1[largest_contour_index] = moments( contours[largest_contour_index], false );

  ///  Get the mass centers:
  vector<Point2f> mc1( contours.size() );
  
   mc1[largest_contour_index] = Point2f( mu1[largest_contour_index].m10/mu1[largest_contour_index].m00, mu1[largest_contour_index].m01/mu1[largest_contour_index].m00 );
   circle( img, mc1[largest_contour_index], 4, Scalar (0,0,255), -1, 8, 0 );


	/////////////////////////////////////////
	largest_area=0;
	largest_contour_index=0;
	Rect bounding_rect2;

	Mat roi2 =img( Rect(320,240,320,240));

	Mat imgHSV2;

	cvtColor(roi2, imgHSV2, COLOR_BGR2HSV);
 
	Mat imgThresholded2;
  
	int iLowH2 = 113;
	int iHighH2 =162;

	int iLowS2= 59; 
	int iHighS2 =183;

	int iLowV2 = 67;
	int iHighV2 = 255;

	inRange(imgHSV2, Scalar(iLowH2, iLowS2, iLowV2), Scalar(iHighH2, iHighS2, iHighV2), imgThresholded2); //Threshold the image
   
	//morphological opening (removes small objects from the foreground)
	erode(imgThresholded2, imgThresholded2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate( imgThresholded2, imgThresholded2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

	//morphological closing (removes small holes from the foreground)
	dilate( imgThresholded2, imgThresholded2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	erode(imgThresholded2, imgThresholded2, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
       
	vector< vector<Point> > contours1;
	findContours(imgThresholded2, contours1, hierarchy1,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE); // Find the contours in the image
        
	for( int i = 0; i< contours1.size(); i++ ) // iterate through each contour.
	{
		double a=contourArea( contours1[i],false);  //  Find the area of contour
       
       	if(a>largest_area){
		largest_area=a;
		largest_contour_index=i;                //Store the index of largest contour
       
		}
 
	}
	bounding_rect2=boundingRect(contours1[largest_contour_index])+Point(320,240);
	rectangle(img,bounding_rect2,Scalar(0,250,0),1,8,0); 
     
	///////Centroid detection of 2nd vertex
	
	
	 /// Get the moments
  vector<Moments> mu(contours1.size() );
  
     mu[largest_contour_index] = moments( contours1[largest_contour_index], false );

  ///  Get the mass centers:
  vector<Point2f> mc( contours1.size() );
  
   mc[largest_contour_index] = Point2f( mu[largest_contour_index].m10/mu[largest_contour_index].m00 +320, mu[largest_contour_index].m01/mu[largest_contour_index].m00 +240);
   circle( img, mc[largest_contour_index], 4, Scalar (0,0,255), -1, 8, 0 );



	int height = mu[largest_contour_index].m01/mu[largest_contour_index].m00 +240 - mu1[largest_contour_index].m01/mu1[largest_contour_index].m00;
		int width = (mu[largest_contour_index].m10/mu[largest_contour_index].m00 +320) - mu1[largest_contour_index].m10/mu1[largest_contour_index].m00;
		int x = mu1[largest_contour_index].m10/mu1[largest_contour_index].m00;
		int y = mu1[largest_contour_index].m01/mu1[largest_contour_index].m00;
		int smallest;
		if(height<width){
			smallest = height;
		}
		else{
			smallest = width;
		}
		float factor = (smallest%7);
		cout << "factor is " << factor <<endl;
		float newX = x+factor;
		float newY = y+factor;
		float newHeight = smallest - (factor);
		float stepSize = (336/7);
		Mat roi = img (Rect(newX,newY,newHeight,newHeight));
		///////////////////////
		int array[7][7];
		for(int i=0;i<7;i++){
			
			for(int j=0;j<7;j++){
				array[i][j]=1;
				int count = 0;
				float h = j*stepSize;
				
				for(float k = h; k-h < stepSize;k+=1){
					float g = i*stepSize;
					
					for(float l = g;l-g<stepSize;l+=12){
						Mat HSV;
						Mat RGB=roi(Rect(k,l,1,1));
						cvtColor(RGB, HSV,CV_BGR2HSV);
						Vec3b hsv=HSV.at<Vec3b>(0,0);
						int H= hsv.val[0];
						int S= hsv.val[1];
						int V=hsv.val[2];
						if(H>0 && H<12 && S>133&& S<255  ){
							count++;
						}
					}
				}
				if(count > 20){
					array[i][j]=0;
				}
			}
		}
		//////////////////////////
		int m;
for( m = 0; m < 50; m++)
{
	shortestPath[0][m] = -1;
	shortestPath[1][m] = -1;
	sp[0][m] = -1;
	sp[1][m] = -1;
}	
	// Source is the left-most bottom-most corner
    Pair src = make_pair(6,3);

    // Destination is the left-most top-most corner
    Pair dest = make_pair(1,1);
	aStarSearch(array, src, dest);
		
		//////////////////////
		
		 Demo demo;
		 
		 demo.parseOptions(argc, argv);
		///////////////////////
		
		
		
		sp[0][0] = shortestPath[0][0];
		sp[1][0] = shortestPath[1][0];
		int i=1;int k = 1, flag = 1;
		while(shortestPath[0][i]!=-1){
			flag = 1;
			if(shortestPath[0][i-1]==shortestPath[0][i] && shortestPath[0][i]==shortestPath[0][i+1] )
			{
				i++;
				flag =0;
				
			}
			
			if(shortestPath[1][i-1]==shortestPath[1][i] && shortestPath[1][i]==shortestPath[1][i+1])
			{
				i++;
				flag = 0;
			}	
			if(flag == 1){
				sp[0][k] = 	shortestPath[0][i];
				sp[1][k] =  shortestPath[1][i];
				k++;
				i++;
			}
			
			
		}
		///////////////////////////////////////////////////////
		int z = 0;
		ofstream myfile;//to be in write mode
		
		int flag1=0;
		
	int theta = 0,theta2 = 0;
	double theta1;
	
	int first=0;
	
	while(true)
	{
		
		Mat frame;
		cap.read(frame);
		cv::Mat image_gray;


		///////ROI
		roi = frame (Rect(newX,newY,newHeight,newHeight));
		cout << roi.size().width << "x" << roi.size().height << endl;
		float nstep = 336/(7*3);
		int stepSize = nstep;
		////////////////////////////
		demo.setup();
		demo.processImage(roi, image_gray);/////////do not change to frame otherwise centroid of bot will change
		/////////////////////////
		
		float stepb = (336/7);
		for (float i = 0; i<=(newHeight+nstep); i += stepSize)
		line(roi, Point(0, i), Point(newHeight+nstep, i), Scalar(0, 0, 0));

		for (float i = 0; i<=(newHeight+nstep); i += stepSize)
		line(roi, Point(i, 0), Point(i, newHeight+nstep), Scalar(0, 0, 0));
		////////////////////////////////
		
		for (float i = 0; i<=(newHeight+nstep); i +=stepb)
		line(roi, Point(0, i), Point(newHeight+nstep, i), Scalar(0, 0, 255));

		for (float i = 0; i<=(newHeight+nstep); i += stepb)
		line(roi, Point(i, 0), Point(i, newHeight+nstep), Scalar(0, 0, 255));
		
		//////////////////////////////////////
		namedWindow("ROi", CV_WINDOW_NORMAL);
		imshow("ROi", roi);
		///////////////////////////////////////
		  
		theta1 = (((180*1.00)/3.14)*yaw1);  //////////////////bot theta
		
		theta = int(theta1);
	
//cout<<theta1<<"   "<<theta<<endl;
	///////////////////////////////////////////////////
   
		int dis;
		//char snum[6];
		int sec=0;
		for(int y=0;sp[0][y] != -1; y++)
		{
			float centcellx =  cent[0][sp[0][y]][sp[1][y]];
			float centcelly =  cent[1][sp[0][y]][sp[1][y]];
			int t = z;
			if(((Cx - centcellx) < 10) && (( Cx - centcellx) >-10) && ((Cy - centcelly) < 10) && ((Cy - centcelly) > -10)){
				z=y; 
			    
				if(z==0){
					
				theta2=theta;
				
			     }
			     
			     else if(t != z){
					 //////////x constant
					 if(sp[0][z-1] == sp[0][z]){
						 if(sp[1][z] - sp[1][z-1] > 0)//////////// y2 - y1 is positive
						 {
							 
							if(sp[0][z+1] - sp[0][z] > 0) /////////////////x3 -x2 is positive
							{	
								theta2 += 90; ///////left
							}
							else
							{
								theta2-=90;
						
							}
						 }
					else
					{
		
						if(sp[0][z+1] - sp[0][z] > 0)/////////////
						{
							theta2 -= 90;
						}
				
						else
						{
							theta2 += 90;
						}
				
					}
				}
					////////////////////y constant
		
			if(sp[1][z-1] == sp[1][z])
			{
				
			
			if(sp[0][z] - sp[0][z-1] > 0)///////////////////x2 -x1 is positive
			{
				
				
				if(sp[1][z+1] - sp[1][z] > 0) ////////////y3 - y2 is positive
				{
					theta2 -= 90;
				}
				else
				{
						theta2 += 90;
				}
			
			}
			
			else
			{

				
				
				if(sp[1][z+1] - sp[1][z] > 0) //////y3-y2
				{
					theta2 += 90;
				}
				else
				{
					theta2 -= 90;
				}
			
			}
		}
			
	}			 
						 
				
				flag1=1;
				//cout << "hello i am here" <<  y  <<  "	" << (cent[0][sp[0][y]][sp[1][y]]) - Cx << "	" << (cent[1][sp[0][y]][sp[0][y]]) - Cy <<endl;
  
		     }
			
			
		}
		
		
		int theta3 = theta-theta2;          
		
		if(theta3 < 0)
		{
			 theta3 +=360;
		}
			
		
		if(sp[0][z+1] == -1)
		{
			myfile.open ("/dev/rfcomm0");
				myfile <<0;
				myfile<<",";
				myfile <<0;
				myfile<<",";
				//sleep(1);
				myfile.close();
		}		
			
		else
		{
				
		if(abs(Cx - cent[0][sp[0][z+1]][sp[1][z+1]]) < abs(Cy - cent[1][sp[0][z+1]][sp[1][z+1]]) && flag1 == 1)
		{
			dis = Cy-cent[1][sp[0][z+1]][sp[1][z+1]];
			
		        
				myfile.open ("/dev/rfcomm0");
				myfile <<theta3;
				myfile<<",";
				myfile <<dis;
				myfile<<",";
				//sleep(1);
				myfile.close();
				

			
					
		
		}
		
		if(abs(Cy - cent[1][sp[0][z+1]][sp[1][z+1]] )< abs(Cx - cent[0][sp[0][z+1]][sp[1][z+1]] ) && flag1 == 1)
		{
			
			dis = Cx-cent[0][sp[0][z+1]][sp[1][z+1]];
			
			
				myfile.open ("/dev/rfcomm0");
				myfile <<theta3;
				myfile<<",";
				myfile <<dis;
				myfile<<",";
				//sleep(1);
				myfile.close();
					
		}
		
		
		}
		
	cout<<"this is theta3  ->"<<theta3<<endl;
   
	///////////////////////////////////////////////
		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break; 
        }
        
	}//while ends
	/////////////////
	
	///////////////Array Printing
	for(int i=0;i<7;i++){
		for(int j=0;j<7;j++){
			cout << array[i][j] <<"	";
		}
		cout << endl;
	}
	
		for(int i=0;shortestPath[0][i]!=-1;i++){
			
				cout <<"(" << shortestPath[0][i] <<","<< shortestPath[1][i] << ") =>";
			
			}
			cout << endl;
		
		
		
		for(int i=0;sp[0][i]!=-1;i++){
			
				cout <<"(" << sp[0][i] <<","<< sp[1][i] << ") =>";
			
			}
			cout << endl;
	/*
	 
	 	for(int i=0;i<7;i++){
			for(int j=0;j<7;j++){
				cout <<  "(" <<  cent[0][i][j] << "," << cent[1][i][j] << ")	" ;
			}
			cout << endl;
		}
				
	*/
	
	
	////////////////////////////
    
    return(0);

}
