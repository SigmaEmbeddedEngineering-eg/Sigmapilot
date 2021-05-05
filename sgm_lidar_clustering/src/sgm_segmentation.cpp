/*
 * Desc: SGM LiDAR clustering.
 * Author: khaled elmadawi
 * mail: khalid.elmadawi@robotion.ai
 * Date: 12 September 2020
 * 
 * Copyright 2020 Robotion-AI. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include "sgm_segmentation.hpp"
SGMSegmentation::SGMSegmentation(configuration config)
{
    this->config_ = config;
}
SGMSegmentation::~SGMSegmentation() {}
cv::Mat SGMSegmentation::spherical_grid_map_depth_diff(SGM sgm, cv::Mat ground_label)
{
    cv::Mat depth = sgm.d.mul(ground_label);

    // Step1: calculating gradients Up, Down, left, and right.
    cv::Mat diffUp = cv::Mat::zeros(sgm.height, sgm.width, CV_32F);
    cv::absdiff(depth(cv::Range(1, sgm.height), cv::Range::all()),
                depth(cv::Range(0, sgm.height - 1), cv::Range::all()),
                diffUp(cv::Range(1, sgm.height), cv::Range::all()));

    cv::Mat diffDown = cv::Mat::zeros(sgm.height, sgm.width, CV_32F);
    cv::absdiff(depth(cv::Range(1, sgm.height), cv::Range::all()),
                depth(cv::Range(0, sgm.height - 1), cv::Range::all()),
                diffDown(cv::Range(0, sgm.height - 1), cv::Range::all()));

    cv::Mat diffLeft = cv::Mat::zeros(sgm.height, sgm.width, CV_32F);
    cv::absdiff(depth(cv::Range::all(), cv::Range(1, sgm.width)),
                depth(cv::Range::all(), cv::Range(0, sgm.width - 1)),
                diffLeft(cv::Range::all(), cv::Range(1, sgm.width)));

    cv::Mat diffRight = cv::Mat::zeros(sgm.height, sgm.width, CV_32F);
    cv::absdiff(depth(cv::Range::all(), cv::Range(1, sgm.width)),
                depth(cv::Range::all(), cv::Range(0, sgm.width - 1)),
                diffRight(cv::Range::all(), cv::Range(0, sgm.width - 1)));

    return (diffUp + diffDown + diffLeft + diffRight);
}
cv::Mat SGMSegmentation::euclidean_diff(SGM sgm, cv::Mat ground_label,
                                        cv::Range upper_x, cv::Range lower_x,
                                        cv::Range upper_y, cv::Range lower_y)
{
    cv::Mat x = sgm.x.mul(ground_label);
    cv::Mat x_sq = cv::Mat::zeros(sgm.height, sgm.width, CV_32F);
    cv::absdiff(x(upper_x, upper_y),
                x(lower_x, lower_y),
                x_sq(upper_x, upper_y));
    cv::pow(x_sq, 2, x_sq);

    cv::Mat y = sgm.y.mul(ground_label);
    cv::Mat y_sq = cv::Mat::zeros(sgm.height, sgm.width, CV_32F);
    cv::absdiff(y(upper_x, upper_y),
                y(lower_x, lower_y),
                y_sq(upper_x, upper_y));
    cv::pow(y_sq, 2, y_sq);

    cv::Mat z = sgm.z.mul(ground_label);
    cv::Mat z_sq = cv::Mat::zeros(sgm.height, sgm.width, CV_32F);
    cv::absdiff(z(upper_x, upper_y),
                z(lower_x, lower_y),
                z_sq(upper_x, upper_y));
    cv::pow(z_sq, 2, z_sq);
    cv::Mat equilidian_distance = x_sq + y_sq + z_sq;
    cv::sqrt(equilidian_distance, equilidian_distance);
    return equilidian_distance;
}
cv::Mat SGMSegmentation::spherical_grid_map_euclidean_diff(SGM sgm, cv::Mat ground_label)
{
    cv::Mat diffUp = this->euclidean_diff(sgm, ground_label,
                                          cv::Range(1, sgm.height),
                                          cv::Range(0, sgm.height - 1),
                                          cv::Range::all(), cv::Range::all());
    cv::Mat diffDown = this->euclidean_diff(sgm, ground_label,
                                            cv::Range(0, sgm.height - 1),
                                            cv::Range(1, sgm.height),
                                            cv::Range::all(), cv::Range::all());
    cv::Mat diffLeft = this->euclidean_diff(sgm, ground_label,
                                            cv::Range::all(), cv::Range::all(),
                                            cv::Range(1, sgm.width),
                                            cv::Range(0, sgm.width - 1));
    cv::Mat diffRight = this->euclidean_diff(sgm, ground_label,
                                             cv::Range::all(), cv::Range::all(),
                                             cv::Range(0, sgm.width - 1),
                                             cv::Range(1, sgm.width));
    return (diffUp + diffDown + diffLeft + diffRight);
}

clustered_objects SGMSegmentation::semantic_segmentation(SGM sgm, cv::Mat ground_label)
{
    //cv::Mat range_diff = spherical_grid_map_euclidean_diff(sgm, ground_label);
    cv::Mat range_diff = spherical_grid_map_depth_diff(sgm, ground_label);
    cv::threshold(range_diff, range_diff, this->config_.distance_threshold, 255,
                  this->config_.distance_threshold_type);
    range_diff = range_diff.mul(ground_label);
    range_diff.convertTo(range_diff, CV_8UC1);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(range_diff, contours, hierarchy, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    clustered_objects ss_clustered_objects;

    ss_clustered_objects.labels = cv::Mat::zeros(sgm.height, sgm.width, CV_32F);
    cv::Mat obj_label = cv::Mat::zeros(sgm.height, sgm.width, CV_8UC1);

    for (int i = 0; i < contours.size(); i++)
    {
        if (!this->config_.obj_level_filter_flag)
        {
            int color = 1 + i % 3; // 5 + rand() & 1;
            drawContours(ss_clustered_objects.labels, contours, i, color, cv::FILLED);
            drawContours(ss_clustered_objects.labels, contours, i, color, 2, 8, hierarchy, 0,
                         cv::Point());
        }
        else
        {
            drawContours(obj_label, contours, i, 1, cv::FILLED);
            // get object indces in the sgm to process its dimensions.
            std::vector<cv::Point> indices;
            cv::findNonZero(obj_label, indices);

            object obj = filter_objects(sgm, indices);
            if ((obj.size_x > this->config_.obj_level_x_min &&
                 obj.size_x < this->config_.obj_level_x_max) &&
                (obj.size_y > this->config_.obj_level_y_min &&
                 obj.size_y < this->config_.obj_level_y_max) &&
                (obj.size_z > this->config_.obj_level_z_min &&
                 obj.size_z < this->config_.obj_level_z_max))
            {
                int color = 1 + i % 3; // 5 + rand() & 1;
                drawContours(ss_clustered_objects.labels, contours, i, color, cv::FILLED);
                drawContours(ss_clustered_objects.labels, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
                ss_clustered_objects.objects.push_back(obj);
            }
            obj_label = 0;
        }
    }
    return ss_clustered_objects;
}

object SGMSegmentation::filter_objects(SGM sgm, std::vector<cv::Point> indices)
{
    object obj;
    std::vector<float> x, y, z;
    for (int i = 0; i < indices.size(); i++)
    {
        if (!std::isinf(sgm.x.at<float>(indices[i].y, indices[i].x)) &&
            !std::isnan(sgm.x.at<float>(indices[i].y, indices[i].x)) &&
            !std::isinf(sgm.y.at<float>(indices[i].y, indices[i].x)) &&
            !std::isnan(sgm.y.at<float>(indices[i].y, indices[i].x)) &&
            !std::isinf(sgm.z.at<float>(indices[i].y, indices[i].x)) &&
            !std::isnan(sgm.z.at<float>(indices[i].y, indices[i].x)) && 1)
        {
            x.push_back(sgm.x.at<float>(indices[i].y, indices[i].x));
            y.push_back(sgm.y.at<float>(indices[i].y, indices[i].x));
            z.push_back(sgm.z.at<float>(indices[i].y, indices[i].x));
        }
    }
    if (x.size())
    {
        obj.pos_x = (*std::max_element(x.begin(), x.end()) +
                     *std::min_element(x.begin(), x.end())) /
                    2;

        obj.pos_y = (*std::max_element(y.begin(), y.end()) +
                     *std::min_element(y.begin(), y.end())) /
                    2;

        obj.pos_z = (*std::max_element(z.begin(), z.end()) +
                     *std::min_element(z.begin(), z.end())) /
                    2;

        obj.size_x = abs(*std::max_element(x.begin(), x.end()) -
                         *std::min_element(x.begin(), x.end()));
        obj.size_y = abs(*std::max_element(y.begin(), y.end()) -
                         *std::min_element(y.begin(), y.end()));
        obj.size_z = abs(*std::max_element(z.begin(), z.end()) -
                         *std::min_element(z.begin(), z.end()));
    }
    return obj;
}
