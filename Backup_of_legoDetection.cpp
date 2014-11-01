#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>
#include <stdio.h>
#include <vector>

// Used for sending the transformation matrix
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

using namespace cv;
using namespace std;

///////////////////////////////////////////////////////////
// Global variables
///////////////////////////////////////////////////////////
static const std::string OPENCV_WINDOW = "Image window";

// Red
int hue_min_red = 1;
int hue_max_red = 21;
int saturation_min_red = 0;
int saturation_max_red = 255;
int value_min_red = 160;
int value_max_red = 255;

// Yellow
int hue_min_yellow = 0;     // Yep, full hue range, and then subtracting red to get yellow
int hue_max_yellow = 255;
int saturation_min_yellow = 0;
int saturation_max_yellow = 255;
int value_min_yellow = 200;
int value_max_yellow = 255;

// Blue
//int hue_min_blue = 50;
//int hue_max_blue = 255;
//int saturation_min_blue = 0;
//int saturation_max_blue = 72;
//int value_min_blue = 0;
//int value_max_blue = 255;

// Blue - test with inverted images
int hue_min_blue = 0;
int hue_max_blue = 50;
int saturation_min_blue = 40;
int saturation_max_blue = 255;
int value_min_blue = 0;
int value_max_blue = 255;

// Morphology trackbars
int erode_iterations_red = 3;    // Tre erosioner for at sikre at alt gult er væk
int dilate_iterations_red = 3;   // Tre dilates efter eriosion for at sikre få samme area igen til de røde klodsser
int erode_iterations_yellow = 3;
int dilate_iterations_yellow = 3;
int erode_iterations_blue = 3;		
int dilate_iterations_blue = 3;

double publish_frequency = 2;
///////////////////////////////////////////////////////////
// Class...
///////////////////////////////////////////////////////////
class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	
	ros::Publisher p_pub;
	
	public:
	ImageConverter():it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
		image_pub_ = it_.advertise("/legoDetection/output_video", 1);
		cv::namedWindow(OPENCV_WINDOW);
	}

	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}
	
	///////////////////////////////////////////////////////////
	// My functions...
	///////////////////////////////////////////////////////////
	
	float RandomNumber(float Min, float Max)
	{
		return ((float(rand()) / float(RAND_MAX)) * (Max - Min)) + Min;
	}
	
	vector<Mat> GetRGB(Mat image)
	{
		vector<Mat> rgb_channel;
		split(image, rgb_channel);
		return rgb_channel;
	}
	
	void CreateTrackBarRed()
	{
		createTrackbar("Hue min", "Red segmentation", &hue_min_red, 255);
		createTrackbar("Hue max", "Red segmentation", &hue_max_red, 255);
		createTrackbar("Saturation min", "Red segmentation", &saturation_min_red, 255);
		createTrackbar("Saturation max", "Red segmentation", &saturation_max_red, 255);
		createTrackbar("Value min", "Red segmentation", &value_min_red, 255);
		createTrackbar("Value max", "Red segmentation", &value_max_red, 255);
	}
	
	void CreateTrackBarYellow()
	{
		createTrackbar("Hue min", "Yellow segmentation", &hue_min_yellow, 255);
		createTrackbar("Hue max", "Yellow segmentation", &hue_max_yellow, 255);
		createTrackbar("Saturation min", "Yellow segmentation", &saturation_min_yellow, 255);
		createTrackbar("Saturation max", "Yellow segmentation", &saturation_max_yellow, 255);
		createTrackbar("Value min", "Yellow segmentation", &value_min_yellow, 255);
		createTrackbar("Value max", "Yellow segmentation", &value_max_yellow, 255);
	}
	
	void CreateTrackBarBlue()
	{
		createTrackbar("Hue min", "Blue segmentation", &hue_min_blue, 255);
		createTrackbar("Hue max", "Blue segmentation", &hue_max_blue, 255);
		createTrackbar("Saturation min", "Blue segmentation", &saturation_min_blue, 255);
		createTrackbar("Saturation max", "Blue segmentation", &saturation_max_blue, 255);
		createTrackbar("Value min", "Blue segmentation", &value_min_blue, 255);
		createTrackbar("Value max", "Blue segmentation", &value_max_blue, 255);
	}
	
	Mat Opening(Mat image, int erode_iterations, int dilate_iterations)
	{
		Mat morph_img;
		erode(image, morph_img, Mat(), Point(-1,-1), erode_iterations, BORDER_CONSTANT, morphologyDefaultBorderValue());
		dilate(morph_img, morph_img, Mat(), Point(-1,-1), dilate_iterations, BORDER_CONSTANT, morphologyDefaultBorderValue());		
		return morph_img;
	}
	
	Mat Closing(Mat image, int erode_iterations, int dilate_iterations)
	{
		Mat morph_img;
		dilate(image, morph_img, Mat(), Point(-1,-1), dilate_iterations, BORDER_CONSTANT, morphologyDefaultBorderValue());
		erode(morph_img, morph_img, Mat(), Point(-1,-1), erode_iterations, BORDER_CONSTANT, morphologyDefaultBorderValue());
		return morph_img;
	}
	
	void findCenterAndAngle(vector<vector<Point> > &contours, vector<Point2d> &center, vector<double> &angle, bool degrees, double minArea, double maxArea, int lowerLine, int upperLine)
	{
		// minArea is setted for red to 1700. maxArea is not defined yet
		Point2d tempCenter;
		double tempAngle, area;
		Moments m;
		double uap20, uap02, uap11;
		for (uint i = 0; i < contours.size(); i++) // each img
		{
			// Now we ignore if two same colored LEGO bricks is touching each other
			// Because if this is true, the area is much bigger and we have setted the
			// maximum Area. 
			
			area = contourArea(contours[i]);
			
			if ((area < minArea) or (area > maxArea))
			{
				cout << "Breaking the loop" << endl;
				continue;
			}
			cout << "Area is: " << area << endl;
			
			//approxPolyDP(contours[i], contours[i], 20, true);
			m = moments(contours[i], false);

			tempCenter.x = floor ( ( m.m10 / m.m00 ) + 0.5 );
			tempCenter.y = floor ( ( m.m01 / m.m00 ) + 0.5 );

			uap20 = m.mu20/m.m00;
			uap02 = m.mu02/m.m00;
			uap11 = m.mu11/m.m00;

			tempAngle = 0.5*atan2( ( 2 * uap11 ) , ( uap20 - uap02 ) );

			if ( tempAngle < 0 )
			{
				tempAngle = ( tempAngle + 2*M_PI );
				tempAngle = fmod( tempAngle ,M_PI ); //2*M_PI
			}

			if ( degrees == true )
			{
				tempAngle = tempAngle * 180 / M_PI;
			}
			
			cout << "lowerLine is: " << lowerLine << endl;
			cout << "upperLine is: " << upperLine << endl;
			
			if ((tempCenter.y > lowerLine) and (tempCenter.y < upperLine))
			{
				center.push_back(tempCenter);
				angle.push_back(tempAngle);
			}
		}
	}
	

	///////////////////////////////////////////////////////////
	// The image call back function
	///////////////////////////////////////////////////////////

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		
		// Here I added the publisher so we can publish the lego pse
		p_pub = nh_.advertise<geometry_msgs::Pose>("lego_pose", 1);
		ros::Rate rate(publish_frequency);
		geometry_msgs::Pose pose;
		tf::Quaternion q;
		
		q.setRPY(RandomNumber(-1.57, 1.57), 0, 0);
		pose.orientation.x = q.getX();
		pose.orientation.y = q.getY();
		pose.orientation.z = q.getZ();
		pose.orientation.w = q.getW();
		
		pose.position.x = RandomNumber(-0.450, -0.645);
		pose.position.y = RandomNumber(0.315, 0.475);
		pose.position.z = 0.889;
		
		p_pub.publish(pose);
		//rate.sleep();
		
		// Store the image from the webcam into inputImage
		Mat inputImage;
		inputImage = cv_ptr->image;
		
		// Resize scale
		int resizeScale = 2;
		// Resize the image
		Size size(inputImage.cols/resizeScale,inputImage.rows/resizeScale);//the dst image size,e.g.100x100
		resize(inputImage,inputImage,size);//resize image
		
		// Create a ROI since the
		Mat img_cropped;
		int roi_x = 110;   
		int roi_y = 0;
		int roi_width = inputImage.cols-(2*roi_x);
		//int roi_height = inputImage.rows - (2*roi_y);
		int roi_height = inputImage.rows - 100;
		//cout << "Image has width: " << roi_width << "and height: " << roi_height << endl;
		
		inputImage(Rect(roi_x,roi_y,roi_width,roi_height)).copyTo(img_cropped);
		//imshow("Cropped image", img_cropped);
		
		//Convert the image into hsv
		Mat hsvImage;
		cvtColor(img_cropped,hsvImage,CV_BGR2HSV);
		
		// Initial the trackbars for tweaking...
		CreateTrackBarRed();
		CreateTrackBarYellow();
		CreateTrackBarBlue();
		
		//Convert to binary image using thresholding with colorsegmentation
		Mat img_red;
		inRange(hsvImage, 
				Scalar(hue_min_red, saturation_min_red, value_min_red), 
				Scalar(hue_max_red, saturation_max_red, value_max_red),
				img_red);
		Mat img_yellow;
		inRange(hsvImage, 
				Scalar(hue_min_yellow, saturation_min_yellow, value_min_yellow), 
				Scalar(hue_max_yellow, saturation_max_yellow, value_max_yellow),
				img_yellow);
		Mat img_blue;
		inRange(hsvImage, 
				Scalar(hue_min_blue, saturation_min_blue, value_min_blue), 
				Scalar(hue_max_blue, saturation_max_blue, value_max_blue),
				img_blue);

		// Do some morphology - red
		Mat morph_red;
		createTrackbar("Erode", "morph_red", &erode_iterations_red, 20);
		createTrackbar("Dilate", "morph_red", &dilate_iterations_red, 20);
		morph_red = Opening(img_red, erode_iterations_red, dilate_iterations_red);
		
		// Since it is hard to use only Hue to seperate between red and yellow,
		// the yellow will get full hue range, and then use the red to subtract from the 
		// yellow images. 
		
		// IMPORTANT! Yellow depends on red!
		//imshow("morph_red", morph_red);	
		//imshow("Red segmentation", img_red);
		
		// Jeg bruger img_yellow - morph_red, da morph_red ikke indeholder segmenter af gul
		// Hvis jeg brugte img_red i subtraktionen, ville jeg trække gule segmenter fra
		// og efterlade huller i de gule lego klodser. --> Svære og flere morphologier.
		img_yellow = img_yellow - morph_red;
	
		// Do some morphology - yellow
		Mat morph_yellow;
		createTrackbar("Erode", "morph_yellow", &erode_iterations_yellow, 20);
		createTrackbar("Dilate", "morph_yellow", &dilate_iterations_yellow, 20);
		morph_yellow = Opening(img_yellow, erode_iterations_yellow, dilate_iterations_yellow);
		//imshow("morph_yellow", morph_yellow);	
		//imshow("Yellow segmentation", img_yellow);
		
		// Get the negative image from the blue	
		Mat img_blue_neg;
		img_blue_neg = 255 - img_blue;
		
		// And subtract the yellow image from the blue, so the yellow bricks
		// do not get detected in the blue mask. 
		
		// IMPORTANT! Blue depends on yellow, which depends on red! W
		img_blue_neg = img_blue_neg - img_yellow;
				
		// Do some morphology - blue
		Mat morph_blue;
		createTrackbar("Erode", "morph_blue", &erode_iterations_blue, 20);
		createTrackbar("Dilate", "morph_blue", &dilate_iterations_blue, 20);
		morph_blue = Opening(img_blue_neg, erode_iterations_blue, dilate_iterations_blue);
		//imshow("morph_blue", morph_blue);	
		//imshow("Blue segmentation", img_blue_neg);
		
		// Test the final result
		imshow("morph_red", morph_red);	
		imshow("morph_yellow", morph_yellow);	
		imshow("morph_blue", morph_blue);		
		waitKey(3);
		
		// Here the code from Michael will be implemened.
		vector< vector <Point> > contours_red;
		vector< vector <Point> > contours_yellow;
		vector< vector <Point> > contours_blue;
		findContours(morph_red, contours_red, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
		findContours(morph_yellow, contours_yellow, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
		findContours(morph_blue, contours_blue, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
		
		vector<Point2d> center_red;
		vector<Point2d> center_yellow;
		vector<Point2d> center_blue;
		
		vector<double> angle_red;
		vector<double> angle_yellow;
		vector<double> angle_blue;
		
		// Note: If the brick is not found after the line, then check on the minArea and maxArea
		// Perhaps the limits is too tight and should be expanded a little. 
		Point leftLowerPoint, rightLowerPoint, leftUpperPoint, rightUpperPoint;
		leftLowerPoint.x = 0;
		leftLowerPoint.y = 100;
		rightLowerPoint.x = img_cropped.cols;
		rightLowerPoint.y = leftLowerPoint.y;
		
		leftUpperPoint.x = 0;
		leftUpperPoint.y = 350;
		rightUpperPoint.x = img_cropped.cols;
		rightUpperPoint.y = leftUpperPoint.y;
		
		// Below: This is only for single LEGO bricks.
		// If two LEGO bricks is touching each other, then the area is of course larger
		// This is at the moment not talking into account...
		
		findCenterAndAngle(contours_red, center_red, angle_red, true, 1500, 4000, leftLowerPoint.y, leftUpperPoint.y);
		findCenterAndAngle(contours_yellow, center_yellow, angle_yellow, true, 2500, 5000, leftLowerPoint.y, leftUpperPoint.y);
		findCenterAndAngle(contours_blue, center_blue, angle_blue, true, 800, 2000, leftLowerPoint.y, leftUpperPoint.y);
						
		for (int i = 0; i < center_red.size(); ++i)
		{
			cout << "Center red: " << i << " is " << center_red[i] << endl;
			cout << "Angle red:  " << i << " is " << angle_red[i] << endl;
			circle(img_cropped, center_red[i], 5, Scalar(0, 0, 255), -1, 8, 0);
			circle(img_cropped, center_red[i], 10, Scalar(0, 0, 0), 1, 8, 0);
		}
		
		for (int i = 0; i < center_yellow.size(); ++i)
		{
			cout << "Center yellow: " << i << " is " << center_yellow[i] << endl;
			cout << "Angle yellow:  " << i << " is " << angle_yellow[i] << endl;
			circle(img_cropped, center_yellow[i], 5, Scalar(0, 255, 255), -1, 8, 0);
			circle(img_cropped, center_yellow[i], 10, Scalar(0, 0, 0), 1, 8, 0);
		}
		
		for (int i = 0; i < center_blue.size(); ++i)
		{
			cout << "Center blue: " << i << " is " << center_blue[i] << endl;
			cout << "Angle blue:  " << i << " is " << angle_blue[i] << endl;
			circle(img_cropped, center_blue[i], 5, Scalar(255, 0, 0), -1, 8, 0);
			circle(img_cropped, center_blue[i], 10, Scalar(0, 0, 0), 1, 8, 0);
		}
		
		//
//		Point pt1, pt2;
//		pt1.x = 0;
//		pt1.y = 20;
//		pt2.x = img_cropped.cols;
//		pt2.y = 20;
		
		line(img_cropped, leftLowerPoint, rightLowerPoint, Scalar(0, 255, 0), 2, 8, 0);
		line(img_cropped, leftUpperPoint, rightUpperPoint, Scalar(0, 255, 0), 2, 8, 0);
		imshow("Cropped image", img_cropped);

		//drawContours(fun_test_img, contours, -1, CV_RGB(255, 255, 255), 2, 8);

    
		// Output modified video stream - No need to publish the image out on ROS
		// What we need to publish the transformation matrix, which include the
		// rotation and translation of each brick. 
		//image_pub_.publish(cv_ptr->toImageMsg());
		
		// Now I want to publish just the u and v. 
		
		int u = 200;
		int v = 300;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "legoDetection");
	ImageConverter ic;
	ros::spin();
	return 0;
}
