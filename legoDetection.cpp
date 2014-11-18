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

//Our self define msg file
#include <rsd_project/bricks_to_robot.h>

using namespace cv;
using namespace std;

#define xy_hys 10
#define maxMorph 20

// Maybee thise defines need to be redefined when interfacing to the MES-server, regarding some MES-standards.
#define RED_BRICK "Red"
#define YELLOW_BRICK "Yellow"
#define BLUE_BRICK "Blue"

///////////////////////////////////////////////////////////
// Global variables
///////////////////////////////////////////////////////////
static const std::string OPENCV_WINDOW = "Image window";

// Lower and Upper Limit
int top = 100;
int bund = 350;

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
int erode_iterations_red = 0;    // 1 erosioner for at sikre at alt gult er væk
int dilate_iterations_red = 3;   // Tre dilates efter eriosion for at sikre få samme area igen til de røde klodsser
int erode_iterations_yellow = 3;
int dilate_iterations_yellow = 3;
int erode_iterations_blue = 3;		
int dilate_iterations_blue = 3;

int erode_iterations = 0;
int dilate_iterations = 0;

double publish_frequency = 2;
vector<Point2d> alreadySend;

int redBricks = 0;
int yellowBricks = 0;
int blueBricks = 0;

bool bricksFlag = true;

///////////////////////////////////////////////////////////
// Class...
///////////////////////////////////////////////////////////
class ImageConverter
{
	ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

    /*
     Here the bricks_msg object is created, which use the user-defined msg file "bricks_to_robot.msg"
     There are some things to add, when creating a new msg file, and you want to use it in the code
        - Add file in "msg" folder with .msg at end of filename.
        - Enter the package.xml and add the following: (In this example the libraries is from geometry_msgs)
            <build_depend>geometry_msgs</build_depend> and
            <run_depend>geometry_msgs</run_depend>

        -find_package(catkin REQUIRED COMPONENTS
            geometry_msgs ....

        - Add in the CmakeList this:
            generate_messages(
                DEPENDENCIES
                std_msgs
                geometry_msgs

        - Add the msg file
            add_message_files(
                FILES
                bricks_to_robot.msg

   */

    rsd_project::bricks_to_robot bricks_msg;
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
        Mat element = (cv::Mat_<uchar>(3,3) <<      1,1,1,
                                                    1,1,1,
                                                    1,1,1);

        Mat element1 = (cv::Mat_<uchar>(3,3) <<     0,1,0,
                                                    1,1,1,
                                                    0,1,0);

        Mat element2 = (cv::Mat_<uchar>(3,3) <<     0,1,0,
                                                    0,1,0,
                                                    0,1,0);

        Mat element3 = (cv::Mat_<uchar>(3,3) <<     0,0,0,
                                                    1,1,1,
                                                    0,0,0);

        Mat element4 = (cv::Mat_<uchar>(3,3) <<     1,0,0,
                                                    0,1,0,
                                                    0,0,1);

        Mat element5 = (cv::Mat_<uchar>(3,3) <<     0,0,1,
                                                    0,1,0,
                                                    1,0,0);

        Mat element6 = (cv::Mat_<uchar>(3,3) <<     1,0,1,
                                                    0,1,0,
                                                    1,0,1);

        Mat element7 = (cv::Mat_<uchar>(2,2) <<     1,1,
                                                    1,1);

        //cout << "element is: " << "\n" <<element << endl;
		Mat morph_img;
        erode(image, morph_img, element, Point(-1,-1), erode_iterations, BORDER_CONSTANT, morphologyDefaultBorderValue());
        dilate(morph_img, morph_img, element, Point(-1,-1), dilate_iterations, BORDER_CONSTANT, morphologyDefaultBorderValue());
		return morph_img;
	}

	Mat Closing(Mat image, int erode_iterations, int dilate_iterations)
	{
		Mat morph_img;
		dilate(image, morph_img, Mat(), Point(-1,-1), dilate_iterations, BORDER_CONSTANT, morphologyDefaultBorderValue());
		erode(morph_img, morph_img, Mat(), Point(-1,-1), erode_iterations, BORDER_CONSTANT, morphologyDefaultBorderValue());
		return morph_img;
	}
	
    void findCenterAndAngle(vector<vector<Point> > &contours, vector<Point2d> &center, vector<Point2d> &showCenter, vector<double> &angle, vector<double> &showAngle, vector<double> &height, vector<double> &showHeight, bool degrees, int lowerLine, int upperLine, int minHeight, int maxHeight)
    {
        // minArea is setted for red to 1700. maxArea is not defined yet
        vector<RotatedRect> minRect( contours.size() );
        Point2d tempCenter;
        double tempAngle;
        double tempHeight;
        bool alreadySendBool;

        for (uint i = 0; i < contours.size(); i++) // each img
        {
            // Only look for contours with area larger than 500
            if (contourArea(contours[i]) < 500)
            {
                continue;
            }

            // Using the minAreaRect function to gain the width, height, center coordinate and area.
            minRect[i] = minAreaRect( Mat(contours[i]) );

            // Now we ignore if two same colored LEGO bricks is touching each other
            // Because if this is true, the area is much bigger and we have setted the
            // maximum Area.

            // Track LegoBricks compaired to the height and width of the system

            if(minRect[i].size.height > minRect[i].size.width)
            {
                tempHeight = minRect[i].size.height;
                //cout << "The heigh of the brick is: " << minRect[i].size.height << endl;
                //ratio = double(minRect[i].size.height)/double(minRect[i].size.width);
                //cout << "The ratio height/width is: " << ratio << endl;
            }

            else
            {
                tempHeight = minRect[i].size.width;
                //cout << "The heigh of the brick is: " << minRect[i].size.width << endl;
            }

            // Only look for ratio that is between minRatio and maxRatio
            if((tempHeight < minHeight) || tempHeight > maxHeight)
            {
                continue;
            }

            tempCenter = minRect[i].center;
            tempAngle = minRect[i].angle;

            if (tempAngle == -0)
            {
                tempAngle = 0;
            }

            if ( (floor(minRect[i].size.height + 0.5)) < (floor(minRect[i].size.width + 0.5)) and tempAngle != 0 )
            {
                tempAngle = tempAngle + M_PI*(180/M_PI);
            }
            else if ((floor(minRect[i].size.height + 0.5)) > (floor(minRect[i].size.width + 0.5)) or tempAngle != 0)
            {
                tempAngle = tempAngle + 0.5*M_PI*(180/M_PI);
            }

            if ( degrees == false )
            {
                tempAngle = tempAngle * (M_PI/180);
            }

            alreadySendBool = false;
            for (uint j = 0; j < alreadySend.size(); j++)
            {
                //cout << "TempCenter is: " << tempCenter << endl;
                if ((tempCenter.x <= alreadySend[j].x + xy_hys) && (tempCenter.x >= alreadySend[j].x - xy_hys) && (tempCenter.y <= alreadySend[j].y + xy_hys) && (tempCenter.y >= alreadySend[j].y - xy_hys) )
                {
                    //cout << "TempCenter is the same" << endl;
                    alreadySend[j].x = tempCenter.x;
                    alreadySend[j].y = tempCenter.y;

                    cout << "alreadySend contains: " << alreadySend << "with size: " << alreadySend.size() << endl;
                    if ((tempCenter.y < lowerLine) or (tempCenter.y > upperLine))
                    {
                        cout << "Now we are above or below the green lines with size: " << alreadySend.size() << endl;
                        alreadySend.erase(alreadySend.begin() + j);
                        cout << "alreadySend is less now and contains: " << alreadySend << endl;

                    }

                    alreadySendBool = true;
                    break;
                }
            }

            if (alreadySendBool == false)
            {
                if ((tempCenter.y > lowerLine) and (tempCenter.y < upperLine))
                {
                    center.push_back(tempCenter);
                    angle.push_back(tempAngle);
                    height.push_back(tempHeight);
                }
            }
            else
            {
                if ((tempCenter.y > lowerLine) and (tempCenter.y < upperLine))
                {
                    showCenter.push_back(tempCenter);
                    showAngle.push_back(tempAngle);
                    showHeight.push_back(tempHeight);
                }
            }
        }
    }
	
	string doubleToString( string info, double number )
	{
		stringstream ss;
		ss << info << number;
		return ss.str();
	}

	string centerToString( string info, Point center )
	{
		stringstream ss;
		ss << info << center;
		return ss.str();
	}
	
	double GetZValue(int u, int v)
	{
		double z = 0;
		
		return z;
	}

    double GetXY(int uv, double z, double fxfy, int imageColRow)
    {
        // Initialize variables. This could be either x or y output. Depends on the input arguments.
        // See how the function is called...

        double xy;
        // With the input argument, it is possible to use the pin hole model as followed
        // for x and y
        xy = (uv - imageColRow/2)*z/fxfy;

        // And return the xy
        return xy;
    }

//    double u = center_yellow[0].x;
//    double v = center_yellow[0].y;
//    double roll, pitch, yaw;
//    double fx = 1194.773485;
//    double fy = 1192.665666;

//    // Set roll, pitch and yaw
//    // roll - orientation around x axis
//    // The x,y,z coordinate is bounded with z pointing upward from the conveyorbelt
//    // and x is in the belt running forward direction.
//    roll = 0.0;

//    // Pitch is the inclination of the conveyor belt. Units is converted to radians
//    pitch = 17.03;
//    pitch = pitch*(M_PI/180);

//    // The yaw is the orientation around z-axis
//    yaw = angle_yellow[0];

//    // Set the orientation


//    q.setRPY(roll, pitch, yaw);

//    // Apply the pin hole model
//    double x;
//    double y;
//    double z = 0.030; // 30 cm meassured from camera to middle of conveyor belt

//    x = (u - img_cropped.cols/2)*z/fx;
//    y = (v - img_cropped.rows/2)*z/fy;
//    //cout << "x is: " << x << " and y is: " << y << endl;

//    pose.orientation.x = q.getX();
//    pose.orientation.y = q.getY();
//    pose.orientation.z = q.getZ();
//    pose.orientation.w = q.getW();

//    pose.position.x = x;
//    pose.position.y = y;
//    pose.position.z = z;

//    p_pub.publish(pose);



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
        //p_pub = nh_.advertise<geometry_msgs::Pose>("lego_pose", 1);
        p_pub = nh_.advertise<rsd_project::bricks_to_robot>("lego_pose", 1);
        ros::Rate rate(publish_frequency);
		geometry_msgs::Pose pose;
		tf::Quaternion q;

		// Store the image from the webcam into inputImage
		Mat inputImage;
		inputImage = cv_ptr->image;

        // Here the time is initialized...
        ros::Time tid;
        tid = ros::Time::now();

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
        //imshow("Red segmentation", img_red);

		Mat img_yellow;
		inRange(hsvImage, 
				Scalar(hue_min_yellow, saturation_min_yellow, value_min_yellow), 
				Scalar(hue_max_yellow, saturation_max_yellow, value_max_yellow),
				img_yellow);    
       //imshow("Yellow segmentation ernice", img_yellow);

		Mat img_blue;
		inRange(hsvImage, 
				Scalar(hue_min_blue, saturation_min_blue, value_min_blue), 
				Scalar(hue_max_blue, saturation_max_blue, value_max_blue),
                img_blue);

        // Show the inRange result for each image
        //imshow("Red segmentation", img_red);
        //imshow("Yellow segmentation",img_yellow);
        //imshow("Blue segmentation", img_blue);

        // A small HSV hack has been implemented to propper find blue bricks.
        // The HSV for the blue ones need to be inverted in the binary image, before adding to the red-yellow segmentated image.

        // Do the inversion of the blue image, since we did the HSV hack for the blue brick.
        Mat img_blue_neg;
        img_blue_neg = 255 - img_blue;

        // And the final image is:
        Mat img_seg;
        img_seg = img_yellow + img_blue_neg;

        // Here we can see, that all the bricks is founded in the img_seg.
        // Now we need to do some morph to remove small noise pixels etc...

        //imshow("Final image before morph", img_seg);

        // Note: Since the total segmentated image, img_seg is much nicer now there is really no noise pixel.
        // However the blue brick still suffer from small holes and therefore we close thoese holes by
        // use som Closing. Not Opening. Opening was more nicer to use, if we wanted to get rid of small noise pixels.
        Mat img_morph;
        createTrackbar("Erode", "Final image after morph", &erode_iterations, maxMorph);
        createTrackbar("Dilate", "Final image after morph", &dilate_iterations, maxMorph);
        img_morph = Closing(img_seg, erode_iterations, dilate_iterations);

        imshow("Final image after morph", img_morph);

        // Do segmentation
        vector< vector <Point> > contours;
        findContours(img_seg, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

//		findContours(morph_red, contours_red, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
//		findContours(morph_yellow, contours_yellow, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
//		findContours(morph_blue, contours_blue, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
		
        // Center
		vector<Point2d> center_red;
		vector<Point2d> center_yellow;
		vector<Point2d> center_blue;
        vector<Point2d> showCenter_red;
        vector<Point2d> showCenter_yellow;
        vector<Point2d> showCenter_blue;
		
        // Angle
		vector<double> angle_red;
		vector<double> angle_yellow;
		vector<double> angle_blue;
        vector<double> showAngle_red;
        vector<double> showAngle_yellow;
        vector<double> showAngle_blue;
		
        // Height
        vector<double> height_red;
        vector<double> height_yellow;
        vector<double> height_blue;
        vector<double> showHeight_red;
        vector<double> showHeight_yellow;
        vector<double> showHeight_blue;

		// Note: If the brick is not found after the line, then check on the minArea and maxArea
		// Perhaps the limits is too tight and should be expanded a little. 
		Point leftLowerPoint, rightLowerPoint, leftUpperPoint, rightUpperPoint;
		leftLowerPoint.x = 0;
		leftLowerPoint.y = top;
		rightLowerPoint.x = img_cropped.cols;
		rightLowerPoint.y = leftLowerPoint.y;
		
		leftUpperPoint.x = 0;
		leftUpperPoint.y = bund;
		rightUpperPoint.x = img_cropped.cols;
		rightUpperPoint.y = leftUpperPoint.y;
		
		// Below: This is only for single LEGO bricks.
		// If two LEGO bricks is touching each other, then the area is of course larger
		// This is at the moment not talking into account...
		
        findCenterAndAngle(contours, center_red,    showCenter_red,     angle_red,      showAngle_red,      height_red,     showHeight_red,     true, leftLowerPoint.y, leftUpperPoint.y, 70, 80);
        findCenterAndAngle(contours, center_yellow, showCenter_yellow,  angle_yellow,   showAngle_yellow,   height_yellow,  showHeight_yellow,  true, leftLowerPoint.y, leftUpperPoint.y, 100, 120);
        findCenterAndAngle(contours, center_blue,   showCenter_blue,    angle_blue,     showAngle_blue,     height_blue,    showHeight_blue,    true, leftLowerPoint.y, leftUpperPoint.y, 30, 45);

//        findCenterAndAngle(contours, center_red, angle_red, area_red, true, leftLowerPoint.y, leftUpperPoint.y, 1601, 3500);
//        findCenterAndAngle(contours, center_yellow, angle_yellow, area_yellow, true, leftLowerPoint.y, leftUpperPoint.y, 3501, 5000);
//        findCenterAndAngle(contours, center_blue, angle_blue, area_blue, true, leftLowerPoint.y, leftUpperPoint.y, 800, 1600);
						
		//findCenterAndAngle(contours_red, center_red, angle_red, true, 1500, 4000, leftLowerPoint.y, leftUpperPoint.y);
		//findCenterAndAngle(contours_yellow, center_yellow, angle_yellow, true, 2500, 5000, leftLowerPoint.y, leftUpperPoint.y);
		//findCenterAndAngle(contours_blue, center_blue, angle_blue, true, 800, 2000, leftLowerPoint.y, leftUpperPoint.y);
		
		//cout << "Here we got..." << endl;
        //previousRedBricks = currentRedBricks;
        //previousRedBricks = currentRedBricks;

        int tempSizeRed = 0;
        if(center_red.size() > 0)
        {
            tempSizeRed = center_red.size();
        }
        else
        {
            tempSizeRed = showCenter_red.size();
        }

        redBricks = 0;
        for (int i = 0; i < tempSizeRed; ++i)
		{
			//cout << "Center red: " << i << " is " << center_red[i] << endl;
			//cout << "Angle red:  " << i << " is " << angle_red[i] << endl;
            //circle(img_cropped, center_red[i], 5, Scalar(0, 0, 255), -1, 8, 0);
            //circle(img_cropped, center_red[i], 10, Scalar(0, 0, 0), 1, 8, 0);

            if(!showCenter_red.empty())
            {
                circle(img_cropped, showCenter_red[i], 5, Scalar(0, 0, 255), -1, 8, 0);
                circle(img_cropped, showCenter_red[i], 10, Scalar(0, 0, 0), 1, 8, 0);
                putText(img_cropped, centerToString("Center : ", showCenter_red[i]),    Point(showCenter_red[i].x, showCenter_red[i].y - 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, 8, false);
                putText(img_cropped, doubleToString("Angle  : ", showAngle_red[i]),     Point(showCenter_red[i].x, showCenter_red[i].y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, 8, false);
                putText(img_cropped, doubleToString("Height : ", showHeight_red[i]),    Point(showCenter_red[i].x, showCenter_red[i].y + 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, 8, false);
            }
            else
            {
                circle(img_cropped, center_red[i], 5, Scalar(0, 0, 255), -1, 8, 0);
                circle(img_cropped, center_red[i], 10, Scalar(0, 0, 0), 1, 8, 0);
                putText(img_cropped, centerToString("Center : ", center_red[i]), Point(center_red[i].x, center_red[i].y - 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, 8, false);
                putText(img_cropped, doubleToString("Angle  : ", angle_red[i]), Point(center_red[i].x, center_red[i].y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, 8, false);
                putText(img_cropped, doubleToString("Height : ", height_red[i]), Point(center_red[i].x, center_red[i].y + 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, 8, false);
            }

            redBricks++;
            //cout << "Center red coordinate is: " << center_red[i] << endl;
		
		}
		
		//cout << "-----------------------------------------------------------------" << endl;
		//cout << "\n" << endl;
		
        int tempSizeYellow = 0;
        if(center_yellow.size() > 0)
        {
            tempSizeYellow = center_yellow.size();
        }
        else
        {
            tempSizeYellow = showCenter_yellow.size();
        }

        yellowBricks = 0;
        for (int i = 0; i < tempSizeYellow; ++i)
		{
			//cout << "Center yellow: " << i << " is " << center_yellow[i] << endl;
			//cout << "Angle yellow:  " << i << " is " << angle_yellow[i] << endl;

            if(!showCenter_yellow.empty())
            {
                circle(img_cropped, showCenter_yellow[i], 5, Scalar(0, 255, 255), -1, 8, 0);
                circle(img_cropped, showCenter_yellow[i], 10, Scalar(0, 0, 0), 1, 8, 0);
                putText(img_cropped, centerToString("Center : ", showCenter_yellow[i]),    Point(showCenter_yellow[i].x, showCenter_yellow[i].y - 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, 8, false);
                putText(img_cropped, doubleToString("Angle  : ", showAngle_yellow[i]),     Point(showCenter_yellow[i].x, showCenter_yellow[i].y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, 8, false);
                putText(img_cropped, doubleToString("Height : ", showHeight_yellow[i]),    Point(showCenter_yellow[i].x, showCenter_yellow[i].y + 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, 8, false);
            }
            else
            {
                circle(img_cropped, center_yellow[i], 5, Scalar(0, 255, 255), -1, 8, 0);
                circle(img_cropped, center_yellow[i], 10, Scalar(0, 0, 0), 1, 8, 0);
                putText(img_cropped, centerToString("Center : ", center_yellow[i]), Point(center_yellow[i].x, center_yellow[i].y - 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, 8, false);
                putText(img_cropped, doubleToString("Angle  : ", angle_yellow[i]), Point(center_yellow[i].x, center_yellow[i].y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, 8, false);
                putText(img_cropped, doubleToString("Height : ", height_yellow[i]), Point(center_yellow[i].x, center_yellow[i].y + 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, 8, false);
            }

            yellowBricks++;
		}
		
		//cout << "-----------------------------------------------------------------" << endl;
		//cout << "\n" << endl;
		
        int tempSizeBlue = 0;
        if(center_blue.size() > 0)
        {
            tempSizeBlue = center_blue.size();
        }
        else
        {
            tempSizeBlue = showCenter_blue.size();
        }

        blueBricks = 0;
        for (int i = 0; i < tempSizeBlue; ++i)
		{
			//cout << "Center blue: " << i << " is " << center_blue[i] << endl;
			//cout << "Angle blue:  " << i << " is " << angle_blue[i] << endl;

            if(!showCenter_blue.empty())
            {
                circle(img_cropped, showCenter_blue[i], 5, Scalar(255, 0, 0), -1, 8, 0);
                circle(img_cropped, showCenter_blue[i], 10, Scalar(0, 0, 0), 1, 8, 0);
                putText(img_cropped, centerToString("Center : ", showCenter_blue[i]),    Point(showCenter_blue[i].x, showCenter_blue[i].y - 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, 8, false);
                putText(img_cropped, doubleToString("Angle  : ", showAngle_blue[i]),     Point(showCenter_blue[i].x, showCenter_blue[i].y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, 8, false);
                putText(img_cropped, doubleToString("Height : ", showHeight_blue[i]),    Point(showCenter_blue[i].x, showCenter_blue[i].y + 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, 8, false);

            }
            else
            {
                circle(img_cropped, center_blue[i], 5, Scalar(255, 0, 0), -1, 8, 0);
                circle(img_cropped, center_blue[i], 10, Scalar(0, 0, 0), 1, 8, 0);
                putText(img_cropped, centerToString("Center : ", center_blue[i]), Point(center_blue[i].x, center_blue[i].y - 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, 8, false);
                putText(img_cropped, doubleToString("Angle  : ", angle_blue[i]), Point(center_blue[i].x, center_blue[i].y), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, 8, false);
                putText(img_cropped, doubleToString("Height : ", height_blue[i]), Point(center_blue[i].x, center_blue[i].y + 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, 8, false);
            }

            blueBricks++;	
		}
		


        // Drawings stuff on the output RGB image
		line(img_cropped, leftLowerPoint, rightLowerPoint, Scalar(0, 255, 0), 2, 8, 0);
		line(img_cropped, leftUpperPoint, rightUpperPoint, Scalar(0, 255, 0), 2, 8, 0);
        Point center;
        center.x = img_cropped.cols/2;
        center.y = img_cropped.rows/2;
        circle(img_cropped, center, 5, Scalar(255, 255, 255), -1, 8, 0);

		imshow("Cropped image", img_cropped);
		
		createTrackbar("Lower Line", "Cropped image", &top, img_cropped.rows);
		createTrackbar("Upper Line", "Cropped image", &bund, img_cropped.rows);


        // Here each image has gone trough the following:
            // Yellow, Blue and Red mask has been investigated and seen how many center coordinates sets there is in each color mask image.
            //

        // If there is no brick in the cameras field of view, we just wait...
        //Scalar(RedBricks, YellowBricks, BlueBricks);

        if((redBricks == 0) && (yellowBricks == 0) && (blueBricks == 0) && (bricksFlag == true))
        {         
            cout << "Nothing has been changed in the brick situation..." << endl;
            bricksFlag = false;
        }

        // Else there must be some bricks in the camreas field of view, that changed the brick situation/state
        else
        {
            /*
             Perhaps do a do-while loop in here
             Like do <Find the transformation matrix for the u,v, and angle and publish the transformatin on a topic.
             Then update the brick state, so if there was 1 red LEGO brick in the image,
             --> previousRedBricks = currentRedBricks
             --> and then do this while (previousRedBricks =!currentRedBricks )
             */

            vector<double> elements; //
            vector<vector<double> > transformation_vector; //
            vector<geometry_msgs::Pose> poses;


            // Draw the centerpoint in the image
            double x,y,z,roll,pitch,yaw;
            z = 0.30;                     // Rough estimate of the depth, approx 30 cm. Should be remeassured in final implementation.
            roll = 0.1;                     // We need make a funktion that output the roll depending on the u value in the image.

            double fx = 1194.773485/resizeScale; // Info extracted from /camera/camera_info topic with ROS. Use rostopic echo /camera/camera_info
            double fy = 1192.665666/resizeScale; // Info extracted from /camera/camera_info topic with ROS. Use rostopic echo /camera/camera_info

            if (redBricks > 0)
            {
                //cout << "There is a red brick inside the field" << endl;
                for (int i = 0; i < center_red.size(); i++)
                {
                    x = GetXY(center_red[i].x, z, fx, img_cropped.cols);
                    y = GetXY(center_red[i].y, z, fy, img_cropped.rows);

                    // Multiply with 100 to get to cm. The white dot in the image is origo, (0,0)
                    pose.position.x = 100*x;
                    pose.position.y = 100*y;
                    pose.position.z = 100*z;
                    pose.orientation.x = roll;
                    pose.orientation.y = 17.03;   // Pitch is the inclination of the conveyor belt. This is static. Unit is in degree.
                    pose.orientation.z = angle_red[i]; // Yaw is the rotation of the bricks when lying on the conveyorbelt.
      //              pose.orientation.w = q.getW();   // What

                    // Dont know why we created a vector of poses, but we did somehow...
                    //poses.push_back(pose);

                    // Set the pose into the bricks_msg
                    bricks_msg.pose = pose;

                    // Set the time into bricks_msg
                    bricks_msg.header.stamp = tid;

                    // Set the time into bricks_msg
                    bricks_msg.header.frame_id = RED_BRICK;

                    // Pubish the yellow brick pose on a topic together with pose and time.
                    p_pub.publish(bricks_msg);

                    // Here we publish on the ROS topic, the pose.
                    // Then we keep track of what we have sent...
                    alreadySend.push_back(center_red[i]);

                    cout << "------------------ Next brick ------------------ \n" << bricks_msg << endl;
                }
            }

            if (yellowBricks > 0)
            {
                //cout << "There is a yellow brick inside the field" << endl;
                for (int i = 0; i < center_yellow.size(); i++)
                {
                    x = GetXY(center_yellow[i].x, z, fx, img_cropped.cols);
                    y = GetXY(center_yellow[i].y, z, fy, img_cropped.rows);

                    // Multiply with 100 to get to cm. The white dot in the image is origo, (0,0)
                    pose.position.x = 100*x;
                    pose.position.y = 100*y;
                    pose.position.z = 100*z;
                    pose.orientation.x = roll;
                    pose.orientation.y = 17.03;   // Pitch is the inclination of the conveyor belt. This is static. Unit is in degree.
                    pose.orientation.z = angle_yellow[i]; // Yaw is the rotation of the bricks when lying on the conveyorbelt.
      //              pose.orientation.w = q.getW();   // What

                    // Dont know why we created a vector of poses, but we did somehow...
                    //poses.push_back(pose);

                    // Set the pose into the bricks_msg
                    bricks_msg.pose = pose;

                    // Set the time into bricks_msg
                    bricks_msg.header.stamp = tid;

                    // Set the time into bricks_msg
                    bricks_msg.header.frame_id = YELLOW_BRICK;

                    // Pubish the yellow brick pose on a topic together with pose and time.
                    p_pub.publish(bricks_msg);


                    // Here we publish on the ROS topic, the pose.
                    // Then we keep track of what we have sent...
                    alreadySend.push_back(center_yellow[i]);

                    cout << "------------------ Next brick ------------------ \n" << bricks_msg << endl;
                }
            }

            if (blueBricks > 0)
            {
                //cout << "There is a blue brick inside the field" << endl;
                for (int i = 0; i < center_blue.size(); i++)
                {
                    x = GetXY(center_blue[i].x, z, fx, img_cropped.cols);
                    y = GetXY(center_blue[i].y, z, fy, img_cropped.rows);

                    // Multiply with 100 to get to cm. The white dot in the image is origo, (0,0)
                    pose.position.x = 100*x;
                    pose.position.y = 100*y;
                    pose.position.z = 100*z;
                    pose.orientation.x = roll;
                    pose.orientation.y = 17.03;   // Pitch is the inclination of the conveyor belt. This is static. Unit is in degree.
                    pose.orientation.z = angle_blue[i]; // Yaw is the rotation of the bricks when lying on the conveyorbelt.
      //            pose.orientation.w = q.getW();   // What

                    // Dont know why we created a vector of poses, but we did somehow...
                    //poses.push_back(pose);

                    // Set the pose into the bricks_msg
                    bricks_msg.pose = pose;

                    // Set the time into bricks_msg
                    bricks_msg.header.stamp = tid;

                    // Set the time into bricks_msg
                    bricks_msg.header.frame_id = BLUE_BRICK;

                    // Pubish the blue brick pose on a topic together with pose and time.
                    p_pub.publish(bricks_msg);

                    // Here we publish on the ROS topic, the pose.
                    // Then we keep track of what we have sent...
                    alreadySend.push_back(center_blue[i]);


                    cout << "------------------ Next brick ------------------ \n" << bricks_msg << endl;
                }
                //cout << "vector of poses size is: " << poses.size() << endl;
            }
        }



		// NOTE: Remember that if there is no color in the frame and we do 
		// something like: 
		
		/*
		//drawContours(fun_test_img, contours, -1, CV_RGB(255, 255, 255), 2, 8);
		
		// Now we just find one of the red bricks.
		// center_red[i] = [[u0,v0], [u1,v1], ... , [un,vn]] 
		double u = center_yellow[0].x;
		double v = center_yellow[0].y;
		double roll, pitch, yaw;

		
		// Set roll, pitch and yaw
		// roll - orientation around x axis
		// The x,y,z coordinate is bounded with z pointing upward from the conveyorbelt
		// and x is in the belt running forward direction.
		roll = 0.0;
		
		// Pitch is the inclination of the conveyor belt. Units is converted to radians
		pitch = 17.03;
		pitch = pitch*(M_PI/180);
		
		// The yaw is the orientation around z-axis
		yaw = angle_yellow[0];
		
		// Set the orientation
		

		q.setRPY(roll, pitch, yaw);
		
		// Apply the pin hole model
		double x;
		double y;
		double z = 0.030; // 30 cm meassured from camera to middle of conveyor belt
		
		x = (u - img_cropped.cols/2)*z/fx;
		y = (v - img_cropped.rows/2)*z/fy;
		//cout << "x is: " << x << " and y is: " << y << endl;

		pose.orientation.x = q.getX();
		pose.orientation.y = q.getY();
		pose.orientation.z = q.getZ();
		pose.orientation.w = q.getW();
			
		pose.position.x = x;
		pose.position.y = y;
		pose.position.z = z;
				
		p_pub.publish(pose);
		*/

        // Add this little wait for 1ms to be able to let OpenCV use the imshow function. IF this is avoided the imshow dont show
        // any images...
        //cout << "----------------------------------------------------------" << endl;
        //cout << "\n" << endl;
        waitKey(1);
    }

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "legoDetection");
	ImageConverter ic;
	ros::spin();
	return 0;
}
