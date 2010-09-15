#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <lv1_image/BlobResult.h>
#include <lv1_image/SetImageParam.h>

class ImageConverter {

public:
  int hue_max;
  int hue_min;
  int saturation_max;
  int saturation_min;
  int value_max;
  int value_min;
  int area_size_max;
  int area_size_min;

  ImageConverter(ros::NodeHandle &n) :
    n_(n), it_(n_)
  {
    image_pub_ = it_.advertise("image_topic_2",1);

    cvNamedWindow("Raw Image");
    cvNamedWindow("Hue Image");
    cvNamedWindow("Saturation Image");
    cvNamedWindow("Value Image");
    cvNamedWindow("Result Image");
    image_sub_ = it_.subscribe(
			       "image_topic", 1, &ImageConverter::imageCallback, this);

    // 閾値
    n_.param("hue/max", hue_max, 180);
    n_.param("hue/min", hue_min, 0);
    n_.param("saturation/max", saturation_max, 255);
    n_.param("saturation/min", saturation_min, 0);
    n_.param("value/max", value_max, 255);
    n_.param("value/min", value_min, 0);
    n_.param("area_size/max", area_size_max, 50000);
    n_.param("area_size/min", area_size_min, 1000);

    param_serv_ =n_.advertiseService("set_image_param", &ImageConverter::SetImageParam, this);
  }

  ~ImageConverter()
  {
    cvDestroyWindow("Raw Image");
    cvDestroyWindow("Hue Image");
    cvDestroyWindow("Saturation Image");
    cvDestroyWindow("Value Image");
    cvDestroyWindow("Result Image");
  }

        bool SetImageParam(lv1_image::SetImageParam::Request &req,
                           lv1_image::SetImageParam::Response &res)
        {
                ROS_INFO("setting parameters");
                hue_max = req.hue_max;
                hue_min = req.hue_min;
                saturation_max = req.saturation_max;
                saturation_min = req.saturation_min;
                value_max = req.value_max;
                value_min = req.value_min;
                area_size_max = req.area_size_max;
                area_size_max = req.area_size_min;
                res.result = true;
                return true;
        }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
  {

    IplImage *cv_image = NULL;
    try
      {
	cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
      }
    catch (sensor_msgs::CvBridgeException error)
      {
	ROS_ERROR("error");
      }

    // 閾値用パラメータ
    n_.getParam("/img_filter/hue/max", hue_max);
    n_.getParam("/img_filter/hue/min", hue_min);
    n_.getParam("/img_filter/saturation/max", saturation_max);
    n_.getParam("/img_filter/saturation/min", saturation_min);
    n_.getParam("/img_filter/value/max", value_max);
    n_.getParam("/img_filter/value/min", value_min);
    n_.getParam("/img_filter/area_size/min", area_size_min);
    n_.getParam("/img_filter/area_size/max", area_size_max);

    hsv_image = cvCreateImage(cvGetSize(cv_image), IPL_DEPTH_8U, 3);
    bgr_image = cvCreateImage(cvGetSize(cv_image), IPL_DEPTH_8U, 3);
    // HSVに分ける
    hue_image = cvCreateImage(cvGetSize(cv_image), IPL_DEPTH_8U, 1);
    s_image = cvCreateImage(cvGetSize(cv_image), IPL_DEPTH_8U, 1);
    v_image = cvCreateImage(cvGetSize(cv_image), IPL_DEPTH_8U, 1);
    // mask
    max_mask = cvCreateImage(cvGetSize(cv_image), IPL_DEPTH_8U, 1);
    min_mask = cvCreateImage(cvGetSize(cv_image), IPL_DEPTH_8U, 1);
    hue_mask = cvCreateImage(cvGetSize(cv_image), IPL_DEPTH_8U, 1);
    saturation_mask = cvCreateImage(cvGetSize(cv_image), IPL_DEPTH_8U, 1);
    value_mask = cvCreateImage(cvGetSize(cv_image), IPL_DEPTH_8U, 1);
    all_mask = cvCreateImage(cvGetSize(cv_image), IPL_DEPTH_8U, 1);
    // 表示用
    hsv_out_image = cvCreateImage(cvGetSize(cv_image), IPL_DEPTH_8U, 3);
    hue_out_image = cvCreateImage(cvGetSize(cv_image), IPL_DEPTH_8U, 1);
    s_out_image = cvCreateImage(cvGetSize(cv_image), IPL_DEPTH_8U, 1);
    v_out_image = cvCreateImage(cvGetSize(cv_image), IPL_DEPTH_8U, 1);

    // GBRをHSVにコンバート
    cvCvtColor(cv_image, hsv_image, CV_BGR2HSV);
    // Hue成分のみ抽出
    //cvSplit(hsv_image, hue_image, NULL, NULL, NULL);
    cvSplit(hsv_image, hue_image, s_image, v_image, NULL);

    // 閾値処理
    // 0 < (hue_max, hue_min) < 180
    if (hue_max > 180) hue_max = 180;
    if (hue_max <   0) hue_max = 0;
    if (hue_min > 180) hue_min = 180;
    if (hue_min <   0) hue_min = 0;

    if (hue_max > hue_min) {
//       cvThreshold(hue_image, hue_image, hue_max, 0, CV_THRESH_TOZERO_INV);
//       cvThreshold(hue_image, hue_image, hue_min, 0, CV_THRESH_TOZERO);
      cvThreshold(hue_image, max_mask, hue_max, 1, CV_THRESH_TOZERO_INV);
      cvThreshold(hue_image, min_mask, hue_min, 1, CV_THRESH_TOZERO);
    } else {
//       cvThreshold(hue_image, hue_image, hue_min, 0, CV_THRESH_TOZERO_INV);
//       cvThreshold(hue_image, hue_image, hue_max, 0, CV_THRESH_TOZERO);
      cvThreshold(hue_image, max_mask, hue_min, 1, CV_THRESH_TOZERO_INV);
      cvThreshold(hue_image, min_mask, hue_max, 1, CV_THRESH_TOZERO);
    }
    cvAnd(max_mask, min_mask, hue_mask, NULL);
    cvZero(hue_out_image);
    cvAndS(hue_image, cvScalarAll(255), hue_out_image, hue_mask);

//     cvZero(s_image);
//     cvZero(v_image);
//     cvThreshold(s_image, s_image, saturation_max, 0, CV_THRESH_TOZERO_INV);
//     cvThreshold(s_image, s_image, saturation_min, 0, CV_THRESH_TOZERO);
//     cvThreshold(v_image, v_image, value_max, 0, CV_THRESH_TOZERO_INV);
//     cvThreshold(v_image, v_image, value_min, 0, CV_THRESH_TOZERO);
    cvThreshold(s_image, max_mask, saturation_max, 1, CV_THRESH_TOZERO_INV);
    cvThreshold(s_image, min_mask, saturation_min, 1, CV_THRESH_TOZERO);
    cvAnd(max_mask, min_mask, saturation_mask, NULL);
    cvZero(s_out_image);
    cvAndS(s_image, cvScalarAll(255), s_out_image, saturation_mask);

    cvThreshold(v_image, max_mask, value_max, 1, CV_THRESH_TOZERO_INV);
    cvThreshold(v_image, min_mask, value_min, 1, CV_THRESH_TOZERO);
    cvAnd(max_mask, min_mask, value_mask, NULL);
    cvZero(v_out_image);
    cvAndS(v_image, cvScalarAll(255), v_out_image, value_mask);

    cvAnd(hue_mask, saturation_mask, all_mask, NULL);
    cvAnd(all_mask, value_mask, all_mask, NULL);
    //cvNot(all_mask, all_mask);

//     cvOrS(s_image, cvScalarAll(255), s_image, hue_image);
//     cvOrS(v_image, cvScalarAll(255), v_image, hue_image);

    // HSVをマージする
//     cvMerge(hue_image, s_image, v_image, NULL, hsv_image);
//     cvOrS(hsv_image, cvScalarAll(255), hsv_image, hue_image);
//     cvOrS(hsv_image, cvScalarAll(255), hsv_image, s_image);
//     cvOrS(hsv_image, cvScalarAll(255), hsv_image, v_image);
    cvZero(hsv_out_image);
    cvAndS(hsv_image, cvScalarAll(255), hsv_out_image, all_mask);
    //cvCvtColor(hsv_image, bgr_image, CV_HSV2BGR);
    cvCvtColor(hsv_out_image, bgr_image, CV_HSV2BGR);
    
    blobs = CBlobResult(all_mask, NULL, 0, false);
    blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_INSIDE, area_size_min, area_size_max);

    if (blobs.GetNumBlobs() != 0) {
      blob = blobs.GetBlob(0);
      p1.x = (int)blob.MinX();
      p1.y = (int)blob.MinY();
      p2.x = (int)blob.MaxX();
      p2.y = (int)blob.MaxY();

      cvRectangle(bgr_image, p1, p2, CV_RGB(255, 0, 0), 2, 8, 0);
    }

    cvShowImage("Raw Image", cv_image);
    cvShowImage("Hue Image", hue_out_image);
    cvShowImage("Saturation Image", s_out_image);
    cvShowImage("Value Image", v_out_image);
    cvShowImage("Result Image", bgr_image);
    cvWaitKey(3);

    try
      {
	//image_pub_.publish(bridge_.cvToImgMsg(cv_image, "bgr8"));
	image_pub_.publish(bridge_.cvToImgMsg(bgr_image, "bgr8"));
      }
    catch (sensor_msgs::CvBridgeException error)
      {
	ROS_ERROR("error");
      }
  };
        
        
protected:

  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::CvBridge bridge_;
  image_transport::Publisher image_pub_;
  ros::ServiceServer param_serv_;

  IplImage *hsv_image;
  IplImage *bgr_image;
  IplImage *hue_image;
  IplImage *s_image;
  IplImage *v_image;
  IplImage *hsv_out_image;
  IplImage *hue_out_image;
  IplImage *s_out_image;
  IplImage *v_out_image;
  IplImage *max_mask;
  IplImage *min_mask;
  IplImage *hue_mask;
  IplImage *saturation_mask;
  IplImage *value_mask;
  IplImage *all_mask;

  // ラベリング用
  CBlobResult blobs;
  CBlob blob;
  CvPoint p1, p2;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle n;
  ImageConverter ic(n);
  ros::spin();
  return 0;
}
