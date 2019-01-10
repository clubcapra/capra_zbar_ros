#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
#include "nodelet/nodelet.h"
#include "std_msgs/String.h"

#include <pluginlib/class_list_macros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

namespace capra
{

    struct Gaps {
        std::vector<float> angles;
        std::vector<float> radius;
        std::vector<cv::Point2f> centers;
    };

    class ZbarReaderNodelet : public nodelet::Nodelet
    {
    public:
        ZbarReaderNodelet() = default;
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
        ros::Publisher bounding_pub_;

        void connectCb();
        void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
                     const sensor_msgs::CameraInfoConstPtr& info_msg);
    };

    void ZbarReaderNodelet::onInit()
    {
        ros::NodeHandle &nh         = getNodeHandle();
        ros::NodeHandle &private_nh = getPrivateNodeHandle();

        it_.reset(new image_transport::ImageTransport(nh));

        // Read parameters
        private_nh.param("queue_size", queue_size_, 5);

        ros::SubscriberStatusCallback connect_cb = boost::bind(&ZbarReaderNodelet::connectCb, this);
        image_transport::SubscriberStatusCallback img_connect_cb = boost::bind(&ZbarReaderNodelet::connectCb, this);
        // Make sure we don't enter connectCb() between advertising and assigning to pub_rect_
        boost::lock_guard<boost::mutex> lock(connect_mutex_);

        bounding_pub_ = nh.advertise<capra_landolt_msgs::BoundingCircles>("boundings", 1, connect_cb, connect_cb);
        barcode_pub_ = nh.advertise<std_msgs::String>("barcodes", 1, connect_cb, connect_cb);
        image_pub_ = it_->advertise("image", 1, img_connect_cb, img_connect_cb);
    }

    void ZbarReaderNodelet::connectCb()
    {
        boost::lock_guard<boost::mutex> lock(connect_mutex_);
        if (barcode_pub_.getNumSubscribers() == 0 &&
            image_pub_.getNumSubscribers() == 0 &&
            bounding_pub_.getNumSubscribers() == 0)
        {
            NODELET_INFO("Disconnect to Barcode Detector...");
            sub_camera_.shutdown();
        }
        else if (!sub_camera_)
        {
            NODELET_INFO("Connect to Barcode Detector...");
            image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
            sub_camera_ = it_->subscribeCamera("/capra/camera_3d/rgb/image_raw", static_cast<uint32_t>(queue_size_), &ZbarReaderNodelet::imageCb, this, hints);
        }
    }

    void ZbarReaderNodelet::imageCb(const sensor_msgs::ImageConstPtr &image_msg,
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

        Gaps gaps;
        //findLandoltGaps(img_ptr->image, gaps, 12, 0.8f, 10);

        std_msgs::Header header;
        header.stamp = ros::Time::now();

        if(barcode_pub_.getNumSubscribers() > 0)
        {
            capra_landolt_msgs::Landolts landolts;
            landolts.angles = gaps.angles;
            landolts.header = header;

            barcode_pub_.publish(landolts);
        }

        if(bounding_pub_.getNumSubscribers() > 0)
        {
            capra_landolt_msgs::BoundingCircles boundings;
            boundings.header = header;
            boundings.radius = gaps.radius;

            std::vector<capra_landolt_msgs::Point2f> centers;
            for (auto &i : gaps.centers) {
                capra_landolt_msgs::Point2f center;
                center.x = i.x;
                center.y = i.y;
                centers.push_back(center);
            }

            boundings.centers = centers;
            bounding_pub_.publish(boundings);
        }

        if(image_pub_.getNumSubscribers() > 0)
        {
            cv_bridge::CvImage img_msg(header, sensor_msgs::image_encodings::BGR8, img_ptr->image);
            for (int i = 0; i < gaps.angles.size(); i++)
            {
                circle(img_msg.image, gaps.centers[i], static_cast<int>(gaps.radius[i]), cv::Scalar(0, 0, 1), 3);
            }

            image_pub_.publish(img_msg.toImageMsg());
        }
    }

    PLUGINLIB_EXPORT_CLASS(capra::LandoltNodelet, nodelet::Nodelet)
}
