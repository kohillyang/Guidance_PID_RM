
#include <opencv2/objdetect/objdetect.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include <detect_car/flight_msg.h>

using namespace std;
using namespace cv;
const int type = 11;
geometry_msgs::Vector3 data;
int low_h=105, low_s=100, low_v=50; // h:0 to 10 for red circle 105 to 115 for blue
int high_h=115, high_s=255, high_v=200;
int low_h2=0; // h:0 to 10 for red circle 105 to 115 for blue
int high_h2=10;
VideoCapture cap = VideoCapture("/home/leory/shit_ws/src/detect_car/codes/Video.avi");
double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); 
double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); 
Size frameSize(static_cast<int>(dWidth), static_cast<int>(dHeight));
VideoWriter oVideoWriter ("circle.avi",CV_FOURCC('P','I','M','1') , 30, frameSize, true); 


///////distance estimate using enclosing rect for PNP 
void solveDistance (Mat frame, Rect rect){
	Point2f bl = (rect.br() - Point(rect.width,0));
	Point2f tr = (rect.tl() + Point(rect.width,0));
	std::vector<cv::Point3d> model_points;
	std::vector<cv::Point2d> image_points;  
          
    model_points.push_back(cv::Point3d(-0.25f, -0.25f, 0.4f)); 
    model_points.push_back(cv::Point3d(0.25f, -0.25f, 0.4f));  
	model_points.push_back(cv::Point3d(0.25f, 0.25f, 0.4f));
	model_points.push_back(cv::Point3d(-0.25f, 0.25f, 0.4f));  
    
   // cout << (Point2d) pts[0] << endl;       
    image_points.push_back( bl);  
	image_points.push_back( rect.br() ); 

	image_points.push_back( tr ); 
	image_points.push_back(rect.tl() ); 
/*	double focal_length = frame.cols; // Approximate focal length.
    Point2d center2 = cv::Point2d(frame.cols/2,frame.rows/2);
   cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << focal_length, 0, center2.x, 0 , focal_length, center2.y, 0, 0, 1);
   cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion*/
   	//cv::Mat camera_matrix =  (cv::Mat_<double>(3,3) << 609, 0, 319, 0 , 609, 226, 0, 0, 1);
   	cv::Mat camera_matrix =  (cv::Mat_<double>(3,3) << 600, 0, 319, 0 , 600, 240, 0, 0, 1);
   	cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion
   	//cv::Mat dist_coeffs = (cv::Mat_<double>(5,1)<<0.0014, 0.00076,0.00117,0.00117,-0.036) ; // Assuming no lens distortion
    //cout << "Camera Matrix " << endl << camera_matrix << endl ;
    Mat rotation_vector; // Rotation in axis-angle form
	Mat translation_vector;

	// Solve for pose
	solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);
    //vector<Point3d> end_point3D;
    //vector<Point2d> end_point2D;
    //end_point3D.push_back(Point3d(0.1f,0.1f,0.0f));
     
    //projectPoints(end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, end_point2D);
  

  	double tx = translation_vector.at<double>(0, 0);
	double ty = -1 * translation_vector.at<double>(1, 0);
	     //////////////////////////using proportion to solve PNP.
	//double prop = 0.5/rect.width;
	//cout << prop << endl;
	//Point2f center_dist = Point2f(rect.br() + Point(-rect.width/2,rect.height/2)- Point(320,240));
	//double tx = center_dist.x * prop;
	//double ty = center_dist.y * prop;
	double tz = translation_vector.at<double>(2, 0);
	data.x = tx;
	data.y = ty;
	data.z = tz;
	double dist = std::sqrt(tx*tx + ty*ty + tz*tz);
	cout << "x:" << tx <<",  " << "y: " << ty <<",  " << "z:" << tz << endl;
    for(int i=0; i < image_points.size(); i++)
    {
        circle(frame, image_points[i], 3, Scalar(0,0,255), -1);
        //cout << image_points[i] << endl;
    }




  //  cv::line(frame,image_points[0], end_point2D[0], cv::Scalar(255,0,0), 2);
     
    //cout << "Rotation Vector " << endl << rotation_vector << endl;
   // cout << "Translation Vector" << endl << translation_vector << endl;
     
    //cout <<  end_point2D << endl;
}

///////////////////////////////////
/////////////////set trackbar
void find_circle(Mat frame){
	Mat HSV;
	cvtColor(frame, HSV, CV_BGR2HSV);
	imshow("HSV",HSV);
	Mat frame_threshold;
	Mat frame_threshold2;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//-- Trackbars to set thresholds for RGB values
	inRange(HSV,Scalar(low_h,low_s,low_v), Scalar(high_h,high_s,high_v),frame_threshold);
	inRange(HSV,Scalar(low_h2,low_s,low_v), Scalar(high_h2,high_s,high_v),frame_threshold2);
	frame_threshold = frame_threshold|frame_threshold2;
	findContours( frame_threshold, contours, hierarchy, CV_RETR_EXTERNAL , CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	//Mat drawing = Mat::zeros( frame_threshold.size(), CV_8UC3 );
	vector<std::vector<Point> > contours_new;
	Rect tmp;
	vector<Rect> new_Rect;
	for(int i=0;i < contours.size(); i++)
	{		
		Point2f center;
		float radius = 0;
		minEnclosingCircle(Mat(contours[i]), center, radius);
		double area0 = contourArea(contours[i]);
		double area1 = 3.1415926 *  radius * radius;
		tmp = boundingRect( Mat(contours[i]) );
		if(area0 / area1>0.85){
			contours_new.push_back(contours[i]);
			//center  - Point2f(320, 240)
			cout << center.x  - 320 << ", " << 240 - center.y  << endl;
			new_Rect.push_back(tmp);
			
		}
	}
	for( int i = 0; i< contours_new.size(); i++ )
	{
		drawContours( frame, contours_new, i, Scalar(0,255,255), 2, 8, hierarchy, 0, Point() );
		//drawContours( frame, boundRect, i, Scalar(0,255,255), 2, 8, hierarchy, 0, Point() );
		solveDistance (frame, new_Rect[i]);
	}
}
	
int main(int argc, char **argv)
{
	//int thresh = 30;
	ros::init(argc, argv, "circle_detector");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<detect_car::flight_msg>("/circle_pose", 100);
    ros::Rate loop_rate(100);
    detect_car::flight_msg msg;

	Mat frame;
	//while(1){
	while(cap.isOpened()&& ros::ok()){
	// Mat grayframe;
		//Mat frame = imread("imgs/leroy.jpg", CV_LOAD_IMAGE_COLOR); 
		cap.read(frame);
		if(! frame.data )                            
		{
			cout <<  "Could not open or find the image" << std::endl ;
			oVideoWriter.release();
			return -1;
		}	
		

		find_circle(frame);
		msg.type = type;
    	msg.data = data;
		pub.publish(msg);
		//cout << "Sdsad" << endl;
		cv::line(frame, cv::Point2f(320, 0), cv::Point2f(320, 480), cv::Scalar(255,255,0,0) );
		cv::line(frame, cv::Point2f(0, 240), cv::Point2f(640, 240), cv::Scalar(255,255,0,0) );
  /// Show in a window
/*		#ifdef __DEBUG
		#endif*/
		//namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
		//cout << "asda " <<endl;
		imshow( "Contours", frame );
		oVideoWriter.write(frame);
		loop_rate.sleep();
        ros::spinOnce();
		waitKey(1);

	//}
			
	}
return 0;
}


