/*
 * Desc: SGM LiDAR clustering.
 * Author: khaled elmadawi
 * mail: khalid.elmadawi@sigma.se
 * Date: 12 September 2020
 *
 * Copyright 2020 sigma embedded engineering-se. All rights reserved.
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

#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

#include "dataContainers.hpp"
#ifndef GROUND_SEGMENTATION_HH
#define GROUND_SEGMENTATION_HH

class GroundSegmentation {
   public:
    GroundSegmentation(configuration config);
    ~GroundSegmentation();
    ErrorCode spherical_grid_map_slopes(SGM const& sgm, cv::Mat& slope);
    ErrorCode ground_segmentation(SGM const& sgm, cv::Mat& ground_label);
    ErrorCode freespace(cv::Mat const& ground_label, SGM const& sgm, std::vector<float>& freespace_);

   private:
    configuration config_;
};
#endif