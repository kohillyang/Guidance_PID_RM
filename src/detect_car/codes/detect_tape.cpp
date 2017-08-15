#include <opencv2/objdetect/objdetect.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <stdio.h>
using namespace std;
using namespace cv;
int low_h=20, low_s=100, low_v=50;//20,100,50
int high_h=35, high_s=255, high_v=255;//35,255, 255
VideoCapture cap = VideoCapture("/home/leory/shit_ws/src/detect_car/imgs/MyVideo.avi");
double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); 
double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); 
Size frameSize(static_cast<int>(dWidth), static_cast<int>(dHeight));
VideoWriter oVideoWriter ("MyVideo2.avi",CV_FOURCC('P','I','M','1') , 30, frameSize, true); 

///param needs rain check 

// void solveDistance (Mat frame, RotatedRect rect){
//  Point2f pts[4];
//  rect.points(pts);
//  std::vector<cv::Point3d> model_points;
//  std::vector<cv::Point2d> image_points;
//     model_points.push_back(cv::Point3d(0.0f, 0.0f, 0.0f));            
//     model_points.push_back(cv::Point3d(0.0f, 0.3f, 0.0f));        
//     model_points.push_back(cv::Point3d(0.3f, 0.3f, 0.0f));        
//     model_points.push_back(cv::Point3d(0.3f, 0.0f, 0.0f));  
//     image_points.push_back((Point2d) pts[0] );  
//    // cout << (Point2d) pts[0] << endl;
//     image_points.push_back( (Point2d)pts[1] );    
//     image_points.push_back( (Point2d)pts[2] );     
//     image_points.push_back( (Point2d)pts[3] );  


//  double focal_length = 609; // Approximate focal length.
//     Point2d center2 = cv::Point2d(frame.cols/2,frame.rows/2);
//    // cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << focal_length, 0, center2.x, 0 , focal_length, center2.y, 0, 0, 1);
//    // cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion
//      cv::Mat camera_matrix =  (cv::Mat_<double>(3,3) << 609, 0, 319, 0 , 609, 226, 0, 0, 1);
//      cv::Mat dist_coeffs = (cv::Mat_<double>(5,1)<<0.0014, 0.00076,0.00117,0.00117,-0.036) ; // Assuming no lens distortion
//     //cout << "Camera Matrix " << endl << camera_matrix << endl ;
//     Mat rotation_vector; // Rotation in axis-angle form
//  Mat translation_vector;

// // Solve for pose
//  solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);
//     //vector<Point3d> end_point3D;
//     //vector<Point2d> end_point2D;
//     //end_point3D.push_back(Point3d(0.1f,0.1f,0.0f));
     
//    // projectPoints(end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, end_point2D);
//     double tx = translation_vector.at<double>(0, 0);
//  double ty = translation_vector.at<double>(1, 0);
//  double tz = translation_vector.at<double>(2, 0);
//  double dist = std::sqrt(tx*tx + ty*ty + tz*tz);
//     //cout << dist << endl;
//     for(int i=0; i < image_points.size(); i++)
//     {
//         circle(frame, image_points[i], 3, Scalar(0,0,255), -1);
//         //cout << image_points[i] << endl;
//     }
     
//   //  cv::line(frame,image_points[0], end_point2D[0], cv::Scalar(255,0,0), 2);
     
//     //cout << "Rotation Vector " << endl << rotation_vector << endl;
//    // cout << "Translation Vector" << endl << translation_vector << endl;
     
//     //cout <<  end_point2D << endl;
// }
void solveDistance_rotated (Mat frame, RotatedRect rect){
  Point2f pts[4];
  rect.points(pts);
  std::vector<cv::Point3d> model_points;
  std::vector<cv::Point2d> image_points;
    model_points.push_back(cv::Point3d(0.0f, 0.0f, 0.0f));            
    model_points.push_back(cv::Point3d(0.3f *cos(rect.angle), 0.3f * sin(rect.angle), 0.5f));        
    model_points.push_back(cv::Point3d(0.3f *cos(180-rect.angle), 0.3f * sin(180-rect.angle), 0.3f));        
    model_points.push_back(cv::Point3d(0.3f * sin(rect.angle), -0.3f *cos(rect.angle), 0.3f));  
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
      image_points.push_back(pts[1] );     
      image_points.push_back( pts[0] );
    }  
    if(rect.angle <= 135 &&  rect.angle > 45){
      image_points.push_back( pts[1] );  
     // cout << (Point2d) pts[0] << endl;
      image_points.push_back( pts[0] );    
      image_points.push_back( pts[3] );     
      image_points.push_back(pts[2] );
    }  
    if(rect.angle <= 225 &&  rect.angle > 135){
      image_points.push_back( pts[2] );  
     // cout << (Point2d) pts[0] << endl;
      image_points.push_back( pts[3] );    
      image_points.push_back( pts[0] );     
      image_points.push_back(pts[1] );
    }    


  double focal_length = 609; // Approximate focal length.
    //Point2d center2 = cv::Point2d(frame.cols/2,frame.rows/2);
   // cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << focal_length, 0, center2.x, 0 , focal_length, center2.y, 0, 0, 1);
   
    cv::Mat camera_matrix =  (cv::Mat_<double>(3,3) << 609, 0, 319, 0 , 609, 226, 0, 0, 1);
    //cv::Mat dist_coeffs = (cv::Mat_<double>(5,1)<<0.0014, 0.00076,0.00117,0.00117,-0.036) ;
    cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion
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

 bool confirm_tape(RotatedRect rect, Mat frame){
    float angle = rect.angle;
    Size rect_size = rect.size;

    Mat M, rotated, roi, roi_hsv;
      // get angle and size from the bounding box

  if (rect.angle < -45.) {
    angle += 90.0;
    swap(rect_size.width, rect_size.height);
  }
      //  cout << area1 << endl;
    //cout << area0 / area1 << endl;

  // get the rotation matrix
  M = getRotationMatrix2D(rect.center, angle, 1.0);
  // perform the waffine transformation
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
  if(s/s2 < 0.28){//0.32:0.20!
   // cout <<s/s2 << endl;
    return true;
  }
  return false;
}


int main(){
  Mat HSV;
  Mat frame;
  Mat threshold;
  Mat threshold2;
  vector<Mat> hsv_plane;
  Mat combine;
  Mat combine2;
  Mat combine3;
  vector<vector<Point> > contours;
  //while(1){
  while(cap.isOpened()){
  // Mat grayframe;
  //Mat frame = imread("IMG25.png", CV_LOAD_IMAGE_COLOR); //IMG4,IMG5,IMG7,13,14
  cap.read(frame);
  if(! frame.data )                            
  {
    cout <<  "Could not open or find the image" << std::endl ;
      //oVideoWriter.release();
    return -1;
  }
    //frame = imread("img!!.jpg",CV_LOAD_IMAGE_COLOR);
    imshow("image", frame);
/*  inRange(frame,Scalar(0,0,0), Scalar(100,100,100),combine2);//60 
  imshow("thresh1",combine2);*/
  cvtColor(frame, HSV, CV_BGR2HSV);           
    imshow("HSV", HSV);
  inRange(HSV,Scalar(low_h,low_s,low_v), Scalar(high_h,high_s,high_v),threshold);
    //imshow("thresh", threshold);
  inRange(HSV,Scalar(0,25,0), Scalar(255,255,60),threshold2);//60

    //imshow("thresh2", threshold2);
    //inRange(combine,Scalar(0,0,0), Scalar(high_h,high_s,high_v),combine2);
    //add(threshold,threshold2,combine);
  Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));//(5,5)
  combine = threshold2|threshold;
  //imshow("combine2", combine);
  //GaussianBlur(combine,combine,Size( 3, 3 ),1,1); //no
  dilate(combine,combine,element,Point(-1,-1), 4);//dilate twice

  imshow("combine", combine);
/*    GaussianBlur( combine, combine2, Size( 3, 3 ), 1, 1 );  
    imshow("combine3", combine2);*/
  vector<Vec4i> hierarchy;
  vector<std::vector<Point> > contours_new;
  findContours( combine, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  vector<RotatedRect> boundRect( contours.size() );
    //vector<vector<Point> > contours_poly( contours.size() );
  vector<Point2f> center;
  for(int i=0;i < contours.size(); i++) 
  { 
      //approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
      //Rect r(contours[i].point1, contours[i].point3);

      //double s = sum(mean(v))[0]
    boundRect[i] = minAreaRect( Mat(contours[i]) );
    double area0 = contourArea(contours[i]);
    double area1 = boundRect[i].size.width * boundRect[i].size.height;
              //cout << area0/area1 << endl;
    //cout << area0 / area1 << endl;
    if(boundRect[i].size.width/boundRect[i].size.height > 0.6 
      && boundRect[i].size.width/boundRect[i].size.height < 1.3 && 
      area1 <= 13000 && area1 >= 5000 &&area0 / area1>0.74 && area0 / area1 < 0.95){
      //square, area limit(13000, 5000), (0.6,1.3), matching. (0.74:0.8, 0.95) 
      float angle = boundRect[i].angle;
      Size rect_size = boundRect[i].size;

      Mat M, rotated, roi, roi_hsv;
          // get angle and size from the bounding box
    //cout << s/s2 << endl;
    //imshow("roi",roi);
    //imshow("roi2",roi_hsv);
      //rectangle(frame,pts[0],pts[2],Scalar(0,255,255), 2, 8, 0);
        //Mat roi = combine[Range()]
    if(confirm_tape(boundRect[i],threshold2)){//0.32!
      contours_new.push_back(contours[i]);
      //cout << area1 << endl;
      //cout << area0 / area1 << endl;
      center.push_back(boundRect[i].center);
      ////////////////////solve pnp for distance
      solveDistance_rotated(frame, boundRect[i]);
      //cout << "find" <<endl;


      ////////////
      //waitKey(0);
    }
  }
}
    //Mat drawing = Mat::zeros( combine.size(), CV_8UC3 );
for( int i = 0; i< contours_new.size(); i++ ) 
{ 
      //approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
  //drawContours( frame, contours_new, i, Scalar(0,0,255), 2, 8, hierarchy, 0, Point(0, 0) );
  circle( frame, center[i], 4,  Scalar(0,0,255), 2, 8, 0 );
      //Point2f rect_points[4]; 
/*      contours_new[i].points(rect_points);
          for( int j = 0; j < 4; j++ )
              line( drawing, rect_points[j], rect_points[(j+1)%4], Scalar(0,0,255), 1, 8 ); */  
}
    //split(threshold2, hsv_plane);
imshow("img", frame);
oVideoWriter.write(frame);  
waitKey(1);

  }
return -1;
  
}

