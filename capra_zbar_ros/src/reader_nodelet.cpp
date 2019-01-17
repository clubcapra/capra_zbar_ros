#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
#include "nodelet/nodelet.h"
#include "std_msgs/String.h"
#include "capra_zbar_msgs/BarcodeBoxes.h"
#include "pluginlib/class_list_macros.h"
#include "zbar.h"

#include <string>
#include <vector>
#include <mutex>


namespace capra
{

struct Barcode
{
    std::string type;
    std::string data;
    std::vector <cv::Point> location;
};

class BarcodeZbarNodelet : public nodelet::Nodelet
{

public:
    BarcodeZbarNodelet() = default;
    void onInit() override;
private:
    // Subscriber
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::CameraSubscriber sub_camera_;
    int queue_size_;
    std::string subscriber_image_;

    // Publisher
    std::mutex connect_mutex_;
    image_transport::Publisher image_pub_;
    ros::Publisher barcode_pub_;
    zbar::ImageScanner scanner_;

    void connectCb();
    void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
                 const sensor_msgs::CameraInfoConstPtr& info_msg);
    void findBarcode(const cv::Mat& img, std::vector<Barcode>& detectedBarcodes);
};

void BarcodeZbarNodelet::onInit()
{
    ros::NodeHandle &nh         = getNodeHandle();
    ros::NodeHandle &private_nh = getPrivateNodeHandle();

    it_.reset(new image_transport::ImageTransport(nh));

    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    // Read parameters
    private_nh.param("subscriber_image", subscriber_image_, std::string("image_raw"));
    private_nh.param("queue_size", queue_size_, 5);

    ros::SubscriberStatusCallback connect_cb = std::bind(&BarcodeZbarNodelet::connectCb, this);
    image_transport::SubscriberStatusCallback img_connect_cb = std::bind(&BarcodeZbarNodelet::connectCb, this);
    // Make sure we don't enter connectCb() between advertising and assigning to pub_rect_
    std::lock_guard<std::mutex> lock(connect_mutex_);

    barcode_pub_ = nh.advertise<capra_zbar_msgs::BarcodeBoxes>("barcodes", 1, connect_cb, connect_cb);
    image_pub_ = it_->advertise("image", 1, img_connect_cb, img_connect_cb);
}

void BarcodeZbarNodelet::connectCb()
{
    std::lock_guard<std::mutex> lock(connect_mutex_);
    if (barcode_pub_.getNumSubscribers() == 0 &&
        image_pub_.getNumSubscribers() == 0)
    {
        NODELET_INFO("Disconnect to Barcode Detector...");
        sub_camera_.shutdown();
    }
    else if (!sub_camera_)
    {
        NODELET_INFO("Connect to Barcode Detector...");
        image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
        sub_camera_ = it_->subscribeCamera(subscriber_image_, static_cast<uint32_t>(queue_size_), &BarcodeZbarNodelet::imageCb, this, hints);
    }
}

void BarcodeZbarNodelet::imageCb(const sensor_msgs::ImageConstPtr &image_msg,
                             const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    cv_bridge::CvImageConstPtr img_ptr;
    try
    {
        img_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    std::vector<Barcode> detectedBarcodes;
    findBarcode(img_ptr->image, detectedBarcodes);

    std_msgs::Header header;
    header.stamp = ros::Time::now();

    if(barcode_pub_.getNumSubscribers() > 0)
    {
        capra_zbar_msgs::BarcodeBoxes msg;
        msg.header = header;
        msg.image_header = img_ptr->header;

        for (auto& barcode : detectedBarcodes) {
            capra_zbar_msgs::BarcodeBox barcode_msg;
            barcode_msg.barcode = barcode.data;

            cv::Rect bounding = cv::boundingRect(barcode.location);

            cv::Point min = bounding.tl();
            cv::Point max = bounding.br();

            barcode_msg.xmin = min.x;
            barcode_msg.ymin = min.y;
            barcode_msg.xmax = max.x;
            barcode_msg.ymax = max.y;

            msg.barcode_boxes.emplace_back(barcode_msg);
        }

        barcode_pub_.publish(msg);
    }

    if(image_pub_.getNumSubscribers() > 0)
    {
        cv_bridge::CvImage img_msg(header, sensor_msgs::image_encodings::BGR8, img_ptr->image);

        for (auto& barcode : detectedBarcodes) {

            size_t size = barcode.location.size();
            for(int j = 0; j < size; j++) {
                cv::line(img_msg.image, barcode.location[j], barcode.location[ (j+1) % size], cv::Scalar(255,0,0), 3);
            }
        }

        image_pub_.publish(img_msg.toImageMsg());
    }
}

void BarcodeZbarNodelet::findBarcode(const cv::Mat& img, std::vector<Barcode>& detectedBarcodes)
{
    cv::Mat img_gray;
    cv::cvtColor(img, img_gray, CV_BGR2GRAY);

    zbar::Image image(img.cols, img.rows, "Y800", img_gray.data, img.cols * img.rows);

    int n = scanner_.scan(image);

    for(zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
    {
        Barcode barcode;

        barcode.type = symbol->get_type_name();
        barcode.data = symbol->get_data();

        std::vector<cv::Point> points;
        for(int i = 0; i< symbol->get_location_size(); i++) {
            points.emplace_back(symbol->get_location_x(i),symbol->get_location_y(i));
        }

        if(barcode.location.size() > 4) {
            cv::convexHull(points, barcode.location);
        } else {
            barcode.location = points;
        }

        detectedBarcodes.push_back(barcode);
    }
}

PLUGINLIB_EXPORT_CLASS(capra::BarcodeZbarNodelet, nodelet::Nodelet)
}
