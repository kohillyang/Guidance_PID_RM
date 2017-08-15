#include "opencv2/opencv.hpp"
#include "detect.h"
#include <detect_car/flight_msg.h>
using namespace std;
using namespace cv;


VideoCapture detect_init(int m_cap){
VideoCapture cap = VideoCapture(0); // VC(0)
   //VideoCapture cap = VideoCapture("/home/ubuntu/catkin_wstwo/src/detect_car/codes/MyVideo.avi"); // VC(0)
    //while(1){
    if(!cap.isOpened()){
        cout << "error!" << endl;
        return 0;
    // Mat grayframe;
        //Mat frame = imread("imgs/leroy.jpg", CV_LOAD_IMAGE_COLOR);
    }
    return cap;
}
void detect(VideoCapture cap, Data& data_circle, Data& data_car, Data& data_tape){
    Mat frame;
    cap.read(frame);
    if(! frame.data )
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return;
    }
    find(frame, data_circle,low_hsv_circleR, high_hsv_circleR);
    find_car(frame, data_car, low_hsv_pink, high_hsv_pink);
//	find_tape(frame,data_tape,low_hsv_tape, high_hsv_tape);
    //imshow("frame",frame);
    //waitKey(0);
}
void solveDistance (Mat frame, vector<Point2d> image_points, Data& data){

    vector<Point3d> model_points;
    model_points.push_back(cv::Point3d(-0.16f, -0.16f, 0.0f));
    model_points.push_back(cv::Point3d(0.16f, -0.16f, 0.0f));
    model_points.push_back(cv::Point3d(0.16f, 0.16f, 0.0f));
    model_points.push_back(cv::Point3d(-0.16f, 0.16f, 0.0f));

    //cv::Mat camera_matrix =  (cv::Mat_<double>(3,3) << 609, 0, 319, 0 , 609, 226, 0, 0, 1);
    cv::Mat camera_matrix =  (cv::Mat_<double>(3,3) << 600, 0, 319, 0 , 600, 240, 0, 0, 1);
    cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion
    //cv::Mat dist_coeffs = (cv::Mat_<double>(5,1)<<0.0014, 0.00076,0.00117,0.00117,-0.036) ; // Assuming no lens distortion
    //cout << "Camera Matrix " << endl << camera_matrix << endl ;
    Mat rotation_vector; // Rotation in axis-angle form
    Mat translation_vector;
    solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);

    double tx = translation_vector.at<double>(0, 0);
    double ty =  translation_vector.at<double>(1, 0);
         //////////////////////////using proportion to solve PNP.
    double tz = -1 * translation_vector.at<double>(2, 0);
    if(tx < 1.2 && tx > -1.2 && ty < 1 && ty > -1 && tz > 3 && tz < -3 ){
    	data.data.x = -tx;
    	data.data.y = -ty;
    	data.data.z = tz;
    	data.flag = true;
	}
    double dist = std::sqrt(tx*tx + ty*ty + tz*tz);
    //cout << "x:" << tx <<",  " << "y: " << ty <<",  " << "z:" << tz << endl;
    for(int i=0; i < image_points.size(); i++)
    {
        circle(frame, image_points[i], 3, Scalar(0,0,255), -1);
    }
}

void find(Mat frame, Data& data, int low_hsv[3], int high_hsv[3]){
    Mat HSV;//largest circle
    cvtColor(frame, HSV, CV_BGR2HSV);
   // imshow("HSV",HSV);
    Mat frame_threshold;
    Mat frame_threshold2;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    //-- Trackbars to set thresholds for RGB values
    inRange(HSV,Scalar(low_hsv[0],low_hsv[1],low_hsv[2]),
        Scalar(high_hsv[0],high_hsv[1],high_hsv[2]),frame_threshold);
    /*inRange(HSV,Scalar(low_hsv_circleR[0],low_hsv_circleR[1],low_hsv_circleR[2]),
        Scalar(high_hsv_circleR[0],high_hsv_circleR[1],high_hsv_circleR[2]),frame_threshold2);*/
    frame_threshold = frame_threshold;
    //imshow("threshold",frame_threshold);
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
        //cout <<area0 << endl;
        if(area0 / area1>0.80 && area1 > 4000){
            contours_new.push_back(contours[i]);
            new_Rect.push_back(tmp);

        }
    }
    for(int i = 0; i< contours_new.size(); i++ )
    {
        Rect rect = new_Rect[i];
        drawContours( frame, contours_new, i, Scalar(0,255,255), 2, 8, hierarchy, 0, Point() );
        //drawContours( frame, boundRect, i, Scalar(0,255,255), 2, 8, hierarchy, 0, Point() );
        //cout << "find" << endl;
        Point2f bl = (rect.br() - Point(rect.width,0));
        Point2f tr = (rect.tl() + Point(rect.width,0));

        vector<Point2d> image_points;
        image_points.push_back( bl);
        image_points.push_back( rect.br() );
        image_points.push_back( tr );
        image_points.push_back(rect.tl() );
        solveDistance (frame, image_points, data);
    }
}

void find_car(Mat frame, Data& data, int low_hsv[3], int high_hsv[3]){
    Mat HSV;
    cvtColor(frame, HSV, CV_BGR2HSV);
    //imshow("HSV",HSV);
    Mat frame_threshold;
    Mat frame_threshold2;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    //-- Trackbars to set thresholds for RGB values
    inRange(HSV,Scalar(low_hsv[0],low_hsv[1],low_hsv[2]),
        Scalar(high_hsv[0],high_hsv[1],high_hsv[2]),frame_threshold);
    frame_threshold = frame_threshold;
   // imshow("threshold",frame_threshold);
    findContours( frame_threshold, contours, hierarchy, CV_RETR_EXTERNAL , CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    //Mat drawing = Mat::zeros( frame_threshold.size(), CV_8UC3 );
    vector<std::vector<Point> > contours_new;
    Rect tmp;
    Rect maxTmp;
    Rect rect;
    double maxArea = 0;
    vector<Rect> new_Rect;
    for(int i=0;i < contours.size(); i++)
    {
        Point2f center;
        float radius = 0;
        minEnclosingCircle(Mat(contours[i]), center, radius);
        double area0 = contourArea(contours[i]);
        double area1 = 3.1415926 *  radius * radius;
        tmp = boundingRect( Mat(contours[i]) );
        //cout <<area0/area1 << endl;
        if(area0 / area1>0.55 && area1 > 1500){
            //cout <<area0/area1 << endl;
            contours_new.push_back(contours[i]);
            new_Rect.push_back(tmp);

        }
    }
    for(int i = 0; i< contours_new.size(); i++ )
    {
        rect = new_Rect[i];
        if(contourArea(contours_new[i])>maxArea){
            maxArea = contourArea(contours_new[i]);
            maxTmp = rect;
        }
        drawContours( frame, contours_new, i, Scalar(0,255,255), 2, 8, hierarchy, 0, Point() );
        //drawContours( frame, boundRect, i, Scalar(0,255,255), 2, 8, hierarchy, 0, Point() );
        //cout << "find" << endl;
    }
    if(maxArea != 0){
        rect = maxTmp;
        Point2f bl = (rect.br() - Point(rect.width,0));
        Point2f tr = (rect.tl() + Point(rect.width,0));

        vector<Point2d> image_points;
        image_points.push_back( bl);
        image_points.push_back( rect.br() );
        image_points.push_back( tr );
        image_points.push_back(rect.tl() );
        solveDistance (frame, image_points, data);
    }
}




void solveDistance_rotated (Mat frame, RotatedRect rect, Data& data){
  Point2f pts[4];
  rect.points(pts);
  std::vector<cv::Point3d> model_points;
  std::vector<cv::Point2d> image_points;
    model_points.push_back(cv::Point3d(0.0f, 0.0f, 0.0f));
    model_points.push_back(cv::Point3d(0.3f *cos(rect.angle), 0.3f * sin(rect.angle), 0.0f));
    model_points.push_back(cv::Point3d(0.3f *cos(180-rect.angle), 0.3f * sin(180-rect.angle), 0.0f));
    model_points.push_back(cv::Point3d(0.3f * sin(rect.angle), -0.3f *cos(rect.angle), 0.0f));
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
    //cv::Mat dist_coeffs = (cv::Mat_<double>(5,1)<<0.0014, 0.00076,0.00117,0.00117,-0.036) ;
    cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion
    Mat rotation_vector; // Rotation in axis-angle form
    Mat translation_vector;

// Solve for pose
    solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);
    double tx = translation_vector.at<double>(0, 0);
    double ty =  translation_vector.at<double>(1, 0);
    double tz = -1 * translation_vector.at<double>(2, 0);
    if(tz > -5 & tz < -2){
        data.data.x = -tx;
        data.data.y = -ty;
        data.data.z = tz;
        data.flag = true;
    }
//cout << "x:" << tx <<",  " << "y: " << ty  << endl;
    for(int i=0; i < image_points.size(); i++)
    {
        circle(frame, image_points[i], 3, Scalar(0,0,255), -1);
        //cout << image_points[i] << endl;
    }

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
  if(s/s2 < 0.27){//0.32:0.20!
   // cout <<s/s2 << endl;
    return true;
  }
  return false;
}

void find_tape(Mat frame, Data& data, int low_hsv[3], int high_hsv[3]){
  Mat HSV;
  Mat threshold;
  Mat threshold2;
  Mat combine;
  vector<vector<Point> > contours;
  if(! frame.data )
  {
    cout <<  "Could not open or find the image" << std::endl ;
      //oVideoWriter.release();
    return;
  }
  cvtColor(frame, HSV, CV_BGR2HSV);
  inRange(HSV,Scalar(low_hsv[0],low_hsv[1],low_hsv[2]), Scalar(high_hsv[0],high_hsv[1],high_hsv[2]),threshold);
    //imshow("thresh", threshold);
  inRange(HSV,Scalar(0,25,0), Scalar(255,255,60),threshold2);//60
  Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));//(5,5)
  combine = threshold2|threshold;
  dilate(combine,combine,element,Point(-1,-1), 4);//dilate twice
  vector<Vec4i> hierarchy;
  vector<std::vector<Point> > contours_new;
  findContours( combine, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
  vector<RotatedRect> boundRect( contours.size() );
    //vector<vector<Point> > contours_poly( contours.size() );
  vector<Point2f> center;
  for(int i=0; i < contours.size(); i++)
  {
    boundRect[i] = minAreaRect( Mat(contours[i]) );
    double area0 = contourArea(contours[i]);
    double area1 = boundRect[i].size.width * boundRect[i].size.height;
              //cout << area0/area1 << endl;
    //cout << area0 / area1 << endl;
    if(boundRect[i].size.width/boundRect[i].size.height > 0.6
      && boundRect[i].size.width/boundRect[i].size.height < 1.3 &&
      area1 <= 13000 && area1 >= 5000 &&area0 / area1>0.77 && area0 / area1 < 0.95){
      //square, area limit(13000, 5000), (0.6,1.3), matching. (0.74:0.8, 0.95)
      float angle = boundRect[i].angle;
      Size rect_size = boundRect[i].size;
    if(confirm_tape(boundRect[i], threshold2)){//0.32!
      contours_new.push_back(contours[i]);
      center.push_back(boundRect[i].center);
      ////////////////////solve pnp for distance
      solveDistance_rotated(frame, boundRect[i], data);// also solve distance
    }
  }
}
    //Mat drawing = Mat::zeros( combine.size(), CV_8UC3 );
    for( int i = 0; i< contours_new.size(); i++ )
    {
        circle( frame, center[i], 4,  Scalar(0,0,255), 2, 8, 0 );
    }
}




//for test
int main(int argc, char **argv){
    ros::init(argc, argv, "detector");
    ros::NodeHandle n;
    ros::Publisher circle_pub = n.advertise<detect_car::flight_msg>("/circle_pose", 100);
    ros::Publisher car_pub = n.advertise<detect_car::flight_msg>("/car_pose", 100);
    ros::Publisher tape_pub = n.advertise<detect_car::flight_msg>("/tape_pose", 100);
    ros::Rate loop_rate(100);
    detect_car::flight_msg circle_msg;
    detect_car::flight_msg car_msg;
    detect_car::flight_msg tape_msg;
    circle_msg.type = 11;
    car_msg.type = 11;
    tape_msg.type = 11;
    Data data_car = {};
    Data data_circle = {};
    Data data_tape = {};
    VideoCapture cap = detect_init(0);
    while(ros::ok()){
        detect(cap, data_circle, data_car, data_tape);
        if(data_car.flag){
            //cout << data_car.data << endl;
            car_msg.data = data_car.data;
            car_pub.publish(car_msg);
            data_car.flag = false;
        }
        if(data_circle.flag){
            //cout << data_circle.data << endl;
            circle_msg.data = data_circle.data;
            circle_pub.publish(circle_msg);
            data_circle.flag = false;
        }
        if(data_tape.flag){
            tape_msg.data = data_tape.data;
            tape_pub.publish(tape_msg);
            data_tape.flag = false;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
}

///////////////////////
