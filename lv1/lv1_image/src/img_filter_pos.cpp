///
/// Positionもハクようにしました。
///

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "lv1_image/BlobResult.h"
#include <string>
#include <std_msgs/Int32.h>
#include <algorithm>
#include <geometry_msgs/Vector3.h>

using namespace std;

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
      n_(n), it_(n_), area_(3), p1(3), p2(3), pos_(3), result_image(3)
  {
    //result_image = new IplImage *[3];
    //result_iamge.push_back()
    image_pub_ = it_.advertise("cv_image",1);
    image_pub1_ = it_.advertise("result_image_1",1);
    image_pub2_ = it_.advertise("result_image_2",1);
    image_pub3_ = it_.advertise("result_image_3",1);

    result_pub_ = n_.advertise<std_msgs::Int32>("result",1);
    position_pub1_ = n_.advertise<geometry_msgs::Vector3>("pos1",10);
    position_pub2_ = n_.advertise<geometry_msgs::Vector3>("pos2",10);
    position_pub3_ = n_.advertise<geometry_msgs::Vector3>("pos3",10);
    //         cvNamedWindow("Raw Image");
    //         cvNamedWindow("Image one");
    //         cvNamedWindow("Image two");
    //         cvNamedWindow("Image three");
        
    image_sub_ = it_.subscribe("image_topic", 1, &ImageConverter::imageCallback, this);

    n_.param("use_image", use_image_, true);


    // 閾値
    //         n_.param("hue/max", hue_max, 180);
    //         n_.param("hue/min", hue_min, 0);
    //         n_.param("saturation/max", saturation_max, 255);
    //         n_.param("saturation/min", saturation_min, 0);
    //         n_.param("value/max", value_max, 255);
    //         n_.param("value/min", value_min, 0);
    //         n_.param("area_size/max", area_size_max, 50000);
    //         n_.param("area_size/min", area_size_min, 1000);

  }
  void deleteImages() {
    if (hsv_image) {
      cvReleaseImage(&hsv_image);
    }
    if (hue_image) {
      cvReleaseImage(&hue_image);
    }
    if (s_image) {
      cvReleaseImage(&s_image);
    }
    if (v_image) {
      cvReleaseImage(&v_image);
    }
    if (max_mask) {
      cvReleaseImage(&max_mask);
    }
    if (min_mask) {
      cvReleaseImage(&min_mask);
    }
    if (hue_mask) {
      cvReleaseImage(&hue_mask);
    }
    if (saturation_mask) {
      cvReleaseImage(&saturation_mask);
    }
    if (value_mask) {
      cvReleaseImage(&value_mask);
    }
    if (all_mask) {
      cvReleaseImage(&all_mask);
    }
    if (hsv_out_image) {
      cvReleaseImage(&hsv_out_image);
    }
  }
  
  ~ImageConverter()
  {
    deleteImages();
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
  {
    static CvSize s_size = {0,0};
    IplImage *raw_image = NULL;

    try
    {
      raw_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException error)
    {
      ROS_ERROR("error");
    }

    CvSize cv_size = cvGetSize(raw_image);
    if ( s_size.width != cv_size.width ||
         s_size.height != cv_size.height)
    {
      //deleteImages();
      ROS_INFO("initialize images");
      // 本当はすでに古いのがあるときはりりーすしないといけない。
      cv_image = cvCreateImage(cv_size, IPL_DEPTH_8U, 3);
      result_image[0] = cvCreateImage(cv_size, IPL_DEPTH_8U, 3);
      result_image[1] = cvCreateImage(cv_size, IPL_DEPTH_8U, 3);
      result_image[2] = cvCreateImage(cv_size, IPL_DEPTH_8U, 3);

      hsv_image = cvCreateImage(cv_size, IPL_DEPTH_8U, 3);
      // HSVに分ける
      hue_image = cvCreateImage(cv_size, IPL_DEPTH_8U, 1);
      s_image = cvCreateImage(cv_size, IPL_DEPTH_8U, 1);
      v_image = cvCreateImage(cv_size, IPL_DEPTH_8U, 1);
      // mask
      max_mask = cvCreateImage(cv_size, IPL_DEPTH_8U, 1);
      min_mask = cvCreateImage(cv_size, IPL_DEPTH_8U, 1);
      hue_mask = cvCreateImage(cv_size, IPL_DEPTH_8U, 1);
      saturation_mask = cvCreateImage(cv_size, IPL_DEPTH_8U, 1);
      value_mask = cvCreateImage(cv_size, IPL_DEPTH_8U, 1);
      all_mask = cvCreateImage(cv_size, IPL_DEPTH_8U, 1);
      // 表示用
      hsv_out_image = cvCreateImage(cv_size, IPL_DEPTH_8U, 3);
      hue_out_image = cvCreateImage(cv_size, IPL_DEPTH_8U, 1);
      s_out_image = cvCreateImage(cv_size, IPL_DEPTH_8U, 1);
      v_out_image = cvCreateImage(cv_size, IPL_DEPTH_8U, 1);
        
      s_size =  cvGetSize(raw_image);
    }

    // 上下入れ替え
    cvFlip(raw_image, cv_image, -1);
    // GBRをHSVにコンバート
    cvCvtColor(cv_image, hsv_image, CV_BGR2HSV);
    // Hue成分のみ抽出
    //cvSplit(hsv_image, hue_image, NULL, NULL, NULL);
    cvSplit(hsv_image, hue_image, s_image, v_image, NULL);
    // 閾値用パラメータ
    for (int i = 0; i < 3; i++) {

      string str_head = "";
      string str_body;
      char ch[sizeof(int)];
      sprintf(ch, "%d", i+1);
      str_head = "/img_filter_lv1/" + string(ch) + "/";

      n_.getParam(str_head + "hue/max", hue_max);
      n_.getParam(str_head + "hue/min", hue_min);
      n_.getParam(str_head + "saturation/max", saturation_max);
      n_.getParam(str_head + "saturation/min", saturation_min);
      n_.getParam(str_head + "value/max", value_max);
      n_.getParam(str_head + "value/min", value_min);
      n_.getParam(str_head + "area_size/min", area_size_min);
      n_.getParam(str_head + "area_size/max", area_size_max);

      // 閾値処理
      // 0 < (hue_max, hue_min) < 180
      //             if (hue_max > 180) hue_max = 180;
      //             if (hue_max <   0) hue_max = 0;
      //             if (hue_min > 180) hue_min = 180;
      //             if (hue_min <   0) hue_min = 0;
      
      cvThreshold(hue_image, max_mask, hue_max, 255, CV_THRESH_BINARY_INV);
      cvThreshold(hue_image, min_mask, hue_min, 255, CV_THRESH_BINARY);
      if (hue_max > hue_min) {
        cvAnd(max_mask, min_mask, hue_mask, NULL);
      } else {
        cvOr(max_mask, min_mask, hue_mask, NULL);
      }
      cvZero(hue_out_image);
      //     cvAndS(hue_image, cvScalarAll(255), hue_out_image, hue_mask);
      cvAndS(hue_mask, cvScalarAll(255), hue_out_image, hue_mask);

  
      //     cvZero(s_image);
      //     cvZero(v_image);
      //     cvThreshold(s_image, s_image, saturation_max, 0, CV_THRESH_TOZERO_INV);
      //     cvThreshold(s_image, s_image, saturation_min, 0, CV_THRESH_TOZERO);
      //     cvThreshold(v_image, v_image, value_max, 0, CV_THRESH_TOZERO_INV);
      //     cvThreshold(v_image, v_image, value_min, 0, CV_THRESH_TOZERO);
      //     cvThreshold(s_image, max_mask, saturation_max, 1, CV_THRESH_TOZERO_INV);
      //     cvThreshold(s_image, min_mask, saturation_min, 1, CV_THRESH_TOZERO);
      cvThreshold(s_image, max_mask, saturation_max, 255, CV_THRESH_BINARY_INV);
      cvThreshold(s_image, min_mask, saturation_min, 255, CV_THRESH_BINARY);
      cvAnd(max_mask, min_mask, saturation_mask, NULL);
      cvZero(s_out_image);
      //     cvAndS(s_image, cvScalarAll(255), s_out_image, saturation_mask);
      cvAndS(saturation_mask, cvScalarAll(255), s_out_image, saturation_mask);

      //     cvThreshold(v_image, max_mask, value_max, 1, CV_THRESH_TOZERO_INV);
      //     cvThreshold(v_image, min_mask, value_min, 1, CV_THRESH_TOZERO);
      cvThreshold(v_image, max_mask, value_max, 255, CV_THRESH_BINARY_INV);
      cvThreshold(v_image, min_mask, value_min, 255, CV_THRESH_BINARY);
      cvAnd(max_mask, min_mask, value_mask, NULL);
      cvZero(v_out_image);
      //     cvAndS(v_image, cvScalarAll(255), v_out_image, value_mask);
      cvAndS(value_mask, cvScalarAll(255), v_out_image, value_mask);

      cvAnd(hue_mask, saturation_mask, all_mask, NULL);
      cvAnd(all_mask, value_mask, all_mask, NULL);
      //cvNot(all_mask, all_mask);
      cvDilate(all_mask, all_mask, NULL, 2);
      cvErode(all_mask, all_mask, NULL, 2);

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
      cvCvtColor(hsv_out_image, result_image[i], CV_HSV2BGR);
    
      blobs = CBlobResult(all_mask, NULL, 0, false);
      blobs.Filter(blobs, B_INCLUDE, CBlobGetArea(), B_INSIDE, area_size_min, area_size_max);

      if (blobs.GetNumBlobs() != 0) {
        blob = blobs.GetBlob(0);
        p1[i].x = (int)blob.MinX();
        p1[i].y = (int)blob.MinY();
        p2[i].x = (int)blob.MaxX();
        p2[i].y = (int)blob.MaxY();

        //ROS_INFO("%d, %d, %d, %d", p1.x, p1.y, p2.x, p2.y);
        if ( p1[i].x == 0 && p1[i].y == 0 &&
             p2[i].x == s_size.width+1 && p2[i].y == s_size.height+1)
        {
          if (blobs.GetNumBlobs() > 1) {
            blob = blobs.GetBlob(1);
            p1[i].x = (int)blob.MinX();
            p1[i].y = (int)blob.MinY();
            p2[i].x = (int)blob.MaxX();
            p2[i].y = (int)blob.MaxY();
            double rate = ((blob.MaxX() - blob.MinX()) /
                           (double)(blob.MaxY()- blob.MinY()));
            if ( rate < 2 && rate > 0.5)
            {
              area_[i] = blob.Area();
              pos_[i].x = ((blob.MaxX() + blob.MinX())*0.5) - (s_size.width*0.5);
              pos_[i].y = ((blob.MaxY() + blob.MinY())*0.5) - (s_size.height*0.5);
              cvRectangle(result_image[i], p1[i], p2[i], CV_RGB(255, 0, 0), 2, 8, 0);
            }
            else
            {
              area_[i] = 0;
            }
          }
          else
          {
            area_[i] = 0;
          }
        }
        else
        {
          double rate = ((blob.MaxX() - blob.MinX()) /
                         (double)(blob.MaxY()- blob.MinY()));
          if ( rate < 2 && rate > 0.5)
          {
            area_[i] = blob.Area();
            pos_[i].x = ((blob.MaxX() + blob.MinX())*0.5) - (s_size.width*0.5);
            pos_[i].y = ((blob.MaxY() + blob.MinY())*0.5) - (s_size.height*0.5);

            cvRectangle(result_image[i], p1[i], p2[i], CV_RGB(255, 0, 0), 2, 8, 0);
          }
          else{
            area_[i] = 0;
          }
                        
        }
      }
      else
      {
        area_[i] = 0;
      }
    }
        
    //         cvShowImage("Raw Image", cv_image);
    //         cvShowImage("Image one", result_image[0]);
    //         cvShowImage("Image two", result_image[1]);
    //         cvShowImage("Image three", result_image[2]);

    //double max_area = *std::max_element(area_.begin(), area_.end());
    double max_val = -1;
    int max_index = -1;
    for (int j = 0; j < 3; j++)
    {
      if (area_[j] > max_val) 
      {
        max_index = j;
        max_val = area_[j];
      }
    }
    
    if (area_[0] != 0) {position_pub1_.publish(pos_[0]);}
    if (area_[1] != 0) {position_pub2_.publish(pos_[1]);}
    if (area_[2] != 0) {position_pub3_.publish(pos_[2]);}
    
    if (max_val > 0)
    {
      std_msgs::Int32 result_msg;
      result_msg.data = max_index + 1;
      result_pub_.publish(result_msg);
      ROS_INFO("area=%f", max_val);
      cvRectangle(cv_image, p1[max_index], p2[max_index], CV_RGB(0, 255, 0), 2, 8, 0);
      cvRectangle(cv_image, p1[max_index], p2[max_index], CV_RGB(0, 0, 255), 2, 8, 0);
      cvRectangle(cv_image, p1[max_index], p2[max_index], CV_RGB(255, 0, 0), 2, 8, 0);
      
    }

    for (int j = 0; j < 3; j++)
    {
      area_[j] = 0;
    }
    try
    {
      if (use_image_)
      {
        image_pub_.publish(bridge_.cvToImgMsg(cv_image, "bgr8"));
        image_pub1_.publish(bridge_.cvToImgMsg(result_image[0], "bgr8"));
        image_pub2_.publish(bridge_.cvToImgMsg(result_image[1], "bgr8"));
        image_pub3_.publish(bridge_.cvToImgMsg(result_image[2], "bgr8"));
      }

    }
    catch (sensor_msgs::CvBridgeException error)
    {
      ROS_ERROR("error");
    }

  }

 protected:

  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::CvBridge bridge_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher image_pub1_;
  image_transport::Publisher image_pub2_;
  image_transport::Publisher image_pub3_;
  ros::Publisher position_pub1_;
  ros::Publisher position_pub2_;
  ros::Publisher position_pub3_;
  ros::Publisher result_pub_;
  std::vector<double> area_;
  std::vector<geometry_msgs::Vector3> pos_;

  bool use_image_;
  std::vector<IplImage *> result_image;
  IplImage *cv_image;
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
  std::vector<CvPoint> p1, p2;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle n;
  ImageConverter ic(n);
  ros::spin();
  return 0;
}
