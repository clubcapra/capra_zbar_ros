#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
#include "nodelet/nodelet.h"
#include "std_msgs/String.h"
#include "capra_zbar_msgs/BarcodeBoxes.h"

#include <pluginlib/class_list_macros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

namespace capra
{

class BarcodeZbarNodelet : public nodelet::Nodelet
{
public:
    BarcodeZbarNodelet() = default;
    void onInit() override;
private:
    // Subscriber
    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::CameraSubscriber sub_camera_;
    int queue_size_{};

    // Publisher
    boost::mutex connect_mutex_;
    image_transport::Publisher image_pub_;
    ros::Publisher barcode_pub_;

    void connectCb();
    void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
                 const sensor_msgs::CameraInfoConstPtr& info_msg);
};

void BarcodeZbarNodelet::onInit()
{
    ros::NodeHandle &nh         = getNodeHandle();
    ros::NodeHandle &private_nh = getPrivateNodeHandle();

    it_.reset(new image_transport::ImageTransport(nh));

    // Read parameters
    private_nh.param("queue_size", queue_size_, 5);

    ros::SubscriberStatusCallback connect_cb = boost::bind(&BarcodeZbarNodelet::connectCb, this);
    image_transport::SubscriberStatusCallback img_connect_cb = boost::bind(&BarcodeZbarNodelet::connectCb, this);
    // Make sure we don't enter connectCb() between advertising and assigning to pub_rect_
    boost::lock_guard<boost::mutex> lock(connect_mutex_);

    barcode_pub_ = nh.advertise<capra_zbar_msgs::BarcodeBoxes>("barcodes", 1, connect_cb, connect_cb);
    image_pub_ = it_->advertise("image", 1, img_connect_cb, img_connect_cb);
}

void BarcodeZbarNodelet::connectCb()
{
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
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
        sub_camera_ = it_->subscribeCamera("/capra/camera_3d/rgb/image_raw", static_cast<uint32_t>(queue_size_), &BarcodeZbarNodelet::imageCb, this, hints);
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

    std_msgs::Header header;
    header.stamp = ros::Time::now();

    if(barcode_pub_.getNumSubscribers() > 0)
    {
        capra_zbar_msgs::BarcodeBoxes barcodes_msg;
        barcodes_msg.header = header;
        barcodes_msg.image_header = img_ptr->header;

        barcode_pub_.publish(barcodes_msg);
    }

    if(image_pub_.getNumSubscribers() > 0)
    {
        cv_bridge::CvImage img_msg(header, sensor_msgs::image_encodings::BGR8, img_ptr->image);

        image_pub_.publish(img_msg.toImageMsg());
    }
}

PLUGINLIB_EXPORT_CLASS(capra::BarcodeZbarNodelet, nodelet::Nodelet)
}
