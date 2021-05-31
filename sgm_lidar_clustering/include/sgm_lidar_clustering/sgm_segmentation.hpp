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

#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "dataContainers.hpp"
#ifndef SGM_SEGMENTATION_HH
#define SGM_SEGMENTATION_HH
class SGMSegmentation
{
public:
    SGMSegmentation(configuration config);
    ~SGMSegmentation();
    clustered_objects semantic_segmentation(SGM sgm, cv::Mat ground_label);
    object create_object(double x, double y, double z);
    void update_object(object &obj, double x, double y, double z);
    void clear_object(object &obj);

private:
    configuration config_;
};
#endif