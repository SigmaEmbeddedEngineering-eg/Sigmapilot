#include <gtest/gtest.h>
#include <ros/ros.h>

#include <chrono>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <thread>

#include "dataContainers.hpp"
#include "ground_segmentation.hpp"
#include "sgm_segmentation.hpp"

// using namespace std;

class MyTestSuite : public ::testing::Test {
   public:
    MyTestSuite() {
        configuration config;
        config.ground_segmentation_threshold = 0.1;
        config.use_morphological_filter = false;
        this->gs_ptr = std::make_unique<GroundSegmentation>(config);
        this->sgm_ss_ptr = std::make_unique<SGMSegmentation>(config);
    }
    ~MyTestSuite() {}

    std::unique_ptr<GroundSegmentation> gs_ptr;
    std::unique_ptr<SGMSegmentation> sgm_ss_ptr;
};
TEST_F(MyTestSuite, ground_slopes) {
    SGM sgm(3, 3);
    float x[9] = {0, 1, 1, 0, 1, 1, 0, 1, 1};
    float y[9] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
    float z[9] = {0, 0, 1, 0, 0, 1, 0, 0, 1};
    sgm.x = cv::Mat(3, 3, CV_32F, x);
    sgm.y = cv::Mat(3, 3, CV_32F, y);
    sgm.z = cv::Mat(3, 3, CV_32F, z);
    cv::Mat slopes;
    ErrorCode error_ = this->gs_ptr->spherical_grid_map_slopes(sgm, slopes);

    ASSERT_EQ(error_, ErrorCode::Success);
    ASSERT_EQ(slopes.rows, sgm.height);
    ASSERT_EQ(slopes.cols, sgm.width);
}

TEST_F(MyTestSuite, ground_segmentation) {
    SGM sgm(3, 3);
    float x[9] = {0, 0, 0, 1, 1, 1, 1, 1, 1};
    float y[9] = {0, 0, 0, 1, 1, 1, 2, 2, 2};
    float z[9] = {0, 0, 0, 0, 0, 0, 1, 1, 1};
    float r[9] = {0, 0, 0, 0, 0, 0, 1, 1, 1};
    cv::Mat result = cv::Mat(3, 3, CV_32F, r);

    sgm.x = cv::Mat(3, 3, CV_32F, x);
    sgm.y = cv::Mat(3, 3, CV_32F, y);
    sgm.z = cv::Mat(3, 3, CV_32F, z);

    cv::Mat ground_label;
    ErrorCode error_ = this->gs_ptr->ground_segmentation(sgm, ground_label);

    cv::Mat diff = ground_label != result;

    ASSERT_EQ(error_, ErrorCode::Success);
    ASSERT_EQ(cv::countNonZero(diff), 0);
    ASSERT_EQ(ground_label.rows, sgm.height);
    ASSERT_EQ(ground_label.cols, sgm.width);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "TestNode");

    testing::InitGoogleTest(&argc, argv);

    std::thread t([] {
        while (ros::ok()) ros::spin();
    });

    auto res = RUN_ALL_TESTS();

    ros::shutdown();

    return res;
}
