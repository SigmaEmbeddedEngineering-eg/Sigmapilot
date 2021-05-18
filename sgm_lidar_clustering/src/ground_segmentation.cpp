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

#include "ground_segmentation.hpp"
GroundSegmentation::GroundSegmentation(configuration config) { this->config_ = config; }
GroundSegmentation::~GroundSegmentation() {}
cv::Mat GroundSegmentation::spherical_grid_map_slopes(SGM sgm)
{
    // slope up direction
    int rows = sgm.x.rows;
    int cols = sgm.x.cols;
    cv::Mat deltaX = cv::Mat::zeros(sgm.x.rows, sgm.x.cols, CV_32F);

    cv::absdiff(sgm.x(cv::Range(1, rows), cv::Range::all()),
                sgm.x(cv::Range(0, rows - 1), cv::Range::all()),
                deltaX(cv::Range(1, rows), cv::Range::all()));
    cv::pow(deltaX, 2, deltaX);

    cv::Mat deltaY = cv::Mat::zeros(sgm.x.rows, sgm.x.cols, CV_32F);
    cv::absdiff(sgm.y(cv::Range(1, rows), cv::Range::all()),
                sgm.y(cv::Range(0, rows - 1), cv::Range::all()),
                deltaY(cv::Range(1, rows), cv::Range::all()));
    cv::pow(deltaY, 2, deltaY);
    cv::Mat deltaXY = cv::Mat::zeros(sgm.x.rows, sgm.x.cols, CV_32F);
    deltaXY = deltaX + deltaY;
    cv::sqrt(deltaXY, deltaXY);

    cv::Mat deltaZ = cv::Mat::zeros(sgm.x.rows, sgm.x.cols, CV_32F);
    cv::absdiff(sgm.z(cv::Range(1, rows), cv::Range::all()),
                sgm.z(cv::Range(0, rows - 1), cv::Range::all()),
                deltaZ(cv::Range(1, rows), cv::Range::all()));
    cv::pow(deltaZ, 2, deltaZ);
    cv::sqrt(deltaZ, deltaZ);

    cv::Mat slope = deltaZ / deltaXY;
    return slope;
}

cv::Mat GroundSegmentation::ground_segmentation(SGM sgm)
{
    cv::Mat ground_label;
    cv::Mat slope = this->spherical_grid_map_slopes(sgm);
    //taking threshold for the ground vectors
    cv::threshold(slope, ground_label, this->config_.ground_segmentation_threshold,
                  1, 0);
    if (this->config_.use_morphological_filter)
    {
        cv::Mat element = cv::Mat::ones(3, 3, CV_8U);
        cv::erode(ground_label, ground_label, element, cv::Point(-1, -1), 2);
        cv::dilate(ground_label, ground_label, element, cv::Point(-1, -1), 2);
    }

    return ground_label;
}
std::vector<float> GroundSegmentation::freespace(cv::Mat ground_label, SGM sgm)
{
    cv::Mat freespace_edges = cv::Mat::zeros(sgm.height, sgm.width, CV_32F);

    cv::absdiff(ground_label(cv::Range(1, sgm.height), cv::Range::all()),
                ground_label(cv::Range(0, sgm.height - 1), cv::Range::all()),
                freespace_edges(cv::Range(1, sgm.height), cv::Range::all()));

    cv::threshold(freespace_edges, freespace_edges, 0.5, 255, 0);

    std::vector<cv::Point> free_space_idx;
    cv::findNonZero(freespace_edges, free_space_idx);
    std::vector<float> freespace(sgm.width, 0);
    for (int i = 0; i < free_space_idx.size(); i++)
    {
        if (freespace[free_space_idx[i].x] < 0.001)
        {
            freespace[free_space_idx[i].x] = std::sqrt(std::pow(sgm.x.at<float>(free_space_idx[i].y, free_space_idx[i].x), 2) +
                                                       std::pow(sgm.y.at<float>(free_space_idx[i].y, free_space_idx[i].x), 2));
        }
    }
    return freespace;
}
