#include <opencv2/objdetect/objdetect.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <stdio.h>
#include <detect_car/flight_msg.h>
using namespace std;
using namespace cv;


/////////detect green circle! H:70-100!!  (light green: 45 - 55) (dark green: 75 - 85)
int low_h=75, low_s=50, low_v=50;//20,100,50
int high_h=85, high_s=255, high_v=255;//35,255, 255
VideoCapture cap = VideoCapture(0);
double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); 
double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); 
Size frameSize(static_cast<int>(dWidth), static_cast<int>(dHeight));
VideoWriter oVideoWriter ("circle.avi",CV_FOURCC('P','I','M','1') , 30, frameSize, true); 
///////distance estimate using enclosing rect for PNP 
void solveDistance (Mat frame, RotatedRect rect){
	Point2f pts[4];
	rect.points(pts);
	std::vector<cv::Point3d> model_points;
	std::vector<cv::Point2d> image_points;
    model_points.push_back(cv::Point3d(0.0f, 0.0f, 0.5f));            
    model_points.push_back(cv::Point3d(0.5f *cos(rect.angle), 0.5f * sin(rect.angle), 0.5f));        
    model_points.push_back(cv::Point3d(0.5f *cos(180-rect.angle), 0.5f * sin(180-rect.angle), 0.5f));        
    model_points.push_back(cv::Point3d(0.5f * sin(rect.angle), -0.5f *cos(rect.angle), 0.5f));  
    if(rect.angle <= 45 &&  rect.angle >= -45){
   		image_points.push_back( pts[0] );  
  	 // cout << (Point2d) pts[0] << endl;
    	image_points.push_back( pts[3] );    
    	image_points.push_back( pts[2] );     
    	image_points.push_back( pts[1] );
    } 
    if(rect.angle < -45 &&  rect.angle >= -135){
   		image_points.push_back( pts[2] );  
  	 // cout << (Point2d) pts[0] << endl;
    	image_points.push_back( pts[3] );    
    	image_points.push_back( pts[1] );     
    	image_points.push_back( pts[0] );
    }  
    if(rect.angle <= 135 &&  rect.angle > 45){
   		image_points.push_back( pts[1] );  
  	 // cout << (Point2d) pts[0] << endl;
    	image_points.push_back( pts[0] );    
    	image_points.push_back( pts[3] );     
    	image_points.push_back( pts[2] );
    }  
    if(rect.angle <= 225 &&  rect.angle > 135){
   		image_points.push_back( pts[2] );  
  	 // cout << (Point2d) pts[0] << endl;
    	image_points.push_back( pts[3] );    
    	image_points.push_back( pts[0] );     
    	image_points.push_back( pts[1] );
    }    


	double focal_length = 609; // Approximate focal length.
    //Point2d center2 = cv::Point2d(frame.cols/2,frame.rows/2);
   // cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << focal_length, 0, center2.x, 0 , focal_length, center2.y, 0, 0, 1);
   
   	cv::Mat camera_matrix =  (cv::Mat_<double>(3,3) << 609, 0, 319, 0 , 609, 226, 0, 0, 1);
   	cv::Mat dist_coeffs = (cv::Mat_<double>(5,1)<<0.0014, 0.00076,0.00117,0.00117,-0.036) ;
//  cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion
   	 // Assuming no lens distortion
    //cout << "Camera Matrix " << endl << camera_matrix << endl ;
    Mat rotation_vector; // Rotation in axis-angle form
	Mat translation_vector;

// Solve for pose
	solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);
    //vector<Point3d> end_point3D;
    //vector<Point2d> end_point2D;
    //end_point3D.push_back(Point3d(0.1f,0.1f,0.0f));
     
   // projectPoints(end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, end_point2D);
    double tx = translation_vector.at<double>(0, 0);
	double ty = -1 * translation_vector.at<double>(1, 0);
	double tz = translation_vector.at<double>(2, 0);
	cout << "x:" << tx <<",  " << "y: " << ty  << endl;
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
bool confirm(RotatedRect rect, Mat frame){
		float angle = rect.angle;
		Size rect_size = rect.size;

		Mat M, rotated, roi, roi_hsv;
	    // get angle and size from the bounding box

	if (rect.angle < -45.) {
		angle += 90.0;
		swap(rect_size.width, rect_size.height);
	}
			//	cout << area1 << endl;
		//cout << area0 / area1 << endl;

	// get the rotation matrix
	M = getRotationMatrix2D(rect.center, angle, 1.0);
	// perform the affine transformation
	warpAffine(frame, rotated, M, frame.size(), INTER_CUBIC);
	// crop the resulting image
	getRectSubPix(rotated, rect_size, rect.center, roi);
	double s = sum(roi)[0];
	Mat total_bright = Mat::ones( roi.size(), CV_8UC3 );

	cvtColor(total_bright, roi_hsv, CV_BGR2HSV); 
	inRange(roi_hsv,Scalar(0,0,0), Scalar(255,255,255),roi_hsv);
	double s2 = sum(roi_hsv)[0];


	//cout << s/s2 << endl;
	//imshow("roi",roi);
	//imshow("roi2",roi_hsv);
		//rectangle(frame,pts[0],pts[2],Scalar(0,255,255), 2, 8, 0);
			//Mat roi = combine[Range()]
	if(s/s2 < 0.28){//0.32!
		cout <<s/s2 << endl;
		return true;
	}
	return false;
}

void find_circle(Mat frame){
	Mat HSV;
	cvtColor(frame, HSV, CV_BGR2HSV);
	imshow("HSV",HSV);
	Mat frame_threshold;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//-- Trackbars to set thresholds for RGB values
	inRange(HSV,Scalar(low_h,low_s,low_v), Scalar(high_h,high_s,high_v),frame_threshold);
	imshow("threshold",frame_threshold);
	findContours( frame_threshold, contours, hierarchy, CV_RETR_EXTERNAL , CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	//Mat drawing = Mat::zeros( frame_threshold.size(), CV_8UC3 );
	vector<std::vector<Point> > contours_new;
	vector<RotatedRect> boundRect( contours.size() );
	vector<RotatedRect> new_Rect;
	for(int i=0;i < contours.size(); i++)
	{		
		Point2f center;
		float radius = 0;
		minEnclosingCircle(Mat(contours[i]), center, radius);
		double area0 = contourArea(contours[i]);
		double area1 = 3.1415926 *  radius * radius;
		boundRect[i] = minAreaRect( Mat(contours[i]) );
		if(area0 / area1>0.9){	//area determined
			if(confirm(boundRect[i],frame)){//see if all green
				contours_new.push_back(contours[i]);
				cout << center  - Point2f(320, 240) << endl;
				new_Rect.push_back(boundRect[i]);
			}
		}
	}
	for( int i = 0; i< contours_new.size(); i++ )
	{
		drawContours( frame, contours_new, i, Scalar(0,255,255), 2, 8, hierarchy, 0, Point() );
		//drawContours( frame, boundRect, i, Scalar(0,255,255), 2, 8, hierarchy, 0, Point() );
		solveDistance (frame, new_Rect[i]);
	}
}
	
int main()
{
	//int thresh = 30;
	
	Mat frame;
	//while(1){
	while(cap.isOpened()){
	// Mat grayframe;
		//Mat frame = imread("leroy.jpg", CV_LOAD_IMAGE_COLOR); 
		cap.read(frame);
		if(! frame.data )                            
		{
			cout <<  "Could not open or find the image" << std::endl ;
			oVideoWriter.release();
			return -1;
		}
		find_circle(frame);

  /// Show in a window
/*		#ifdef __DEBUG
		#endif*/
		//namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
		//cout << "asda " <<endl;
		imshow( "Contours", frame );
		oVideoWriter.write(frame);
		waitKey(1);
	//}
			
}
return 0;
}


