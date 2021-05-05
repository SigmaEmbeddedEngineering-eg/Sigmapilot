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

#include "sgm_segmentation.hpp"
SGMSegmentation::SGMSegmentation(configuration config) { this->config_ = config; }
SGMSegmentation::~SGMSegmentation() {}

ErrorCode SGMSegmentation::semantic_segmentation(SGM const& sgm, cv::Mat const& ground_label,
                                                 clustered_objects& ss_clustered_objects) {
    cv::Mat depth = sgm.d.mul(ground_label);

    // Step1: calculating gradients Up, Down, left, and right.
    cv::Mat diffUp = cv::Mat::zeros(sgm.height, sgm.width, CV_32F);
    cv::absdiff(depth(cv::Range(1, sgm.height), cv::Range::all()),
                depth(cv::Range(0, sgm.height - 1), cv::Range::all()),
                diffUp(cv::Range(1, sgm.height), cv::Range::all()));

    cv::Mat diffLeft = cv::Mat::zeros(sgm.height, sgm.width, CV_32F);
    cv::absdiff(depth(cv::Range::all(), cv::Range(1, sgm.width)), depth(cv::Range::all(), cv::Range(0, sgm.width - 1)),
                diffLeft(cv::Range::all(), cv::Range(1, sgm.width)));

    ss_clustered_objects.labels = cv::Mat::zeros(sgm.height, sgm.width, CV_32F);
    std::cout << "-----------------------------------" << std::endl;
    for (int colIdx = 0; colIdx < sgm.width; colIdx++) {
        for (int rowIdx = 0; rowIdx < sgm.height; rowIdx++)

        {
            if (ground_label.at<float>(rowIdx, colIdx) && sgm.d.at<float>(rowIdx, colIdx) > 0) {
                if (rowIdx > 0 && colIdx > 0 &&
                    (diffLeft.at<float>(rowIdx, colIdx) <= this->config_.distance_threshold &&
                     diffUp.at<float>(rowIdx, colIdx) <= this->config_.distance_threshold)) {
                    if (ss_clustered_objects.labels.at<float>(rowIdx - 1, colIdx) > 0 &&
                        ss_clustered_objects.labels.at<float>(rowIdx, colIdx - 1) > 0 &&
                        ss_clustered_objects.labels.at<float>(rowIdx, colIdx - 1) !=
                            ss_clustered_objects.labels.at<float>(rowIdx - 1, colIdx) &&
                        diffLeft.at<float>(rowIdx, colIdx) <= this->config_.distance_threshold &&
                        diffUp.at<float>(rowIdx, colIdx) <= this->config_.distance_threshold) {
                        std::vector<cv::Point> indices1 =
                            ss_clustered_objects
                                .cluster_indices[ss_clustered_objects.labels.at<float>(rowIdx - 1, colIdx) - 1];
                        std::vector<cv::Point> indices2 =
                            ss_clustered_objects
                                .cluster_indices[ss_clustered_objects.labels.at<float>(rowIdx, colIdx - 1) - 1];
                        float cluster1_size =
                            ss_clustered_objects
                                .cluster_indices[ss_clustered_objects.labels.at<float>(rowIdx - 1, colIdx) - 1]
                                .size();
                        float cluster2_size =
                            ss_clustered_objects
                                .cluster_indices[ss_clustered_objects.labels.at<float>(rowIdx, colIdx - 1) - 1]
                                .size();
                        if (cluster1_size > cluster2_size) {
                            for (int pt_idx = 0; pt_idx < cluster2_size; pt_idx++) {
                                ss_clustered_objects.labels.at<float>(indices2[pt_idx].y, indices2[pt_idx].x) =
                                    ss_clustered_objects.labels.at<float>(rowIdx - 1, colIdx);
                                update_object(ss_clustered_objects.objects[ss_clustered_objects.labels.at<float>(
                                                                               indices2[pt_idx].y, indices2[pt_idx].x) -
                                                                           1],
                                              sgm.x.at<float>(indices2[pt_idx].y, indices2[pt_idx].x),
                                              sgm.y.at<float>(indices2[pt_idx].y, indices2[pt_idx].x),
                                              sgm.z.at<float>(indices2[pt_idx].y, indices2[pt_idx].x));
                            }
                            ss_clustered_objects
                                .cluster_indices[ss_clustered_objects.labels.at<float>(rowIdx, colIdx - 1) - 1]
                                .clear();
                            // (void)clear_object(ss_clustered_objects.objects[ss_clustered_objects.labels.at<float>(rowIdx,
                            // colIdx - 1) - 1]);
                        } else {
                            for (int pt_idx = 0; pt_idx < cluster1_size; pt_idx++) {
                                ss_clustered_objects.labels.at<float>(indices1[pt_idx].y, indices1[pt_idx].x) =
                                    ss_clustered_objects.labels.at<float>(rowIdx, colIdx - 1);
                                update_object(ss_clustered_objects.objects[ss_clustered_objects.labels.at<float>(
                                                                               indices1[pt_idx].y, indices1[pt_idx].x) -
                                                                           1],
                                              sgm.x.at<float>(indices1[pt_idx].y, indices1[pt_idx].x),
                                              sgm.y.at<float>(indices1[pt_idx].y, indices1[pt_idx].x),
                                              sgm.z.at<float>(indices1[pt_idx].y, indices1[pt_idx].x));
                            }
                            ss_clustered_objects
                                .cluster_indices[ss_clustered_objects.labels.at<float>(rowIdx - 1, colIdx) - 1]
                                .clear();
                            // (void)clear_object(ss_clustered_objects.objects[ss_clustered_objects.labels.at<float>(rowIdx
                            // - 1, colIdx) - 1]);
                        }
                    } else if (ss_clustered_objects.labels.at<float>(rowIdx - 1, colIdx) > 0) {
                        ss_clustered_objects.labels.at<float>(rowIdx, colIdx) =
                            ss_clustered_objects.labels.at<float>(rowIdx - 1, colIdx);
                        cv::Point p;
                        p.y = rowIdx;
                        p.x = colIdx;
                        ss_clustered_objects.cluster_indices[ss_clustered_objects.labels.at<float>(rowIdx, colIdx) - 1]
                            .push_back(p);
                        update_object(
                            ss_clustered_objects.objects[ss_clustered_objects.labels.at<float>(rowIdx, colIdx) - 1],
                            sgm.x.at<float>(rowIdx, colIdx), sgm.y.at<float>(rowIdx, colIdx),
                            sgm.z.at<float>(rowIdx, colIdx));
                    } else if (ss_clustered_objects.labels.at<float>(rowIdx, colIdx - 1) > 0) {
                        ss_clustered_objects.labels.at<float>(rowIdx, colIdx) =
                            ss_clustered_objects.labels.at<float>(rowIdx, colIdx - 1);
                        cv::Point p;
                        p.y = rowIdx;
                        p.x = colIdx;
                        ss_clustered_objects.cluster_indices[ss_clustered_objects.labels.at<float>(rowIdx, colIdx) - 1]
                            .push_back(p);
                        update_object(
                            ss_clustered_objects.objects[ss_clustered_objects.labels.at<float>(rowIdx, colIdx) - 1],
                            sgm.x.at<float>(rowIdx, colIdx), sgm.y.at<float>(rowIdx, colIdx),
                            sgm.z.at<float>(rowIdx, colIdx));
                    } else {
                        std::vector<cv::Point> indices;
                        ss_clustered_objects.cluster_indices.push_back(indices);
                        ss_clustered_objects.labels.at<float>(rowIdx, colIdx) =
                            static_cast<int>(ss_clustered_objects.cluster_indices.size());  // cluster_idx;
                        cv::Point p;
                        p.y = rowIdx;
                        p.x = colIdx;
                        ss_clustered_objects.cluster_indices[ss_clustered_objects.labels.at<float>(rowIdx, colIdx) - 1]
                            .push_back(p);
                        object obj;
                        (void)create_object(sgm.x.at<float>(rowIdx, colIdx), sgm.y.at<float>(rowIdx, colIdx),
                                            sgm.z.at<float>(rowIdx, colIdx), obj);
                        ss_clustered_objects.objects.push_back(obj);
                    }
                } else if (rowIdx > 0 && diffUp.at<float>(rowIdx, colIdx) <= this->config_.distance_threshold) {
                    if (ss_clustered_objects.labels.at<float>(rowIdx - 1, colIdx) > 0) {
                        ss_clustered_objects.labels.at<float>(rowIdx, colIdx) =
                            ss_clustered_objects.labels.at<float>(rowIdx - 1, colIdx);
                        cv::Point p;
                        p.y = rowIdx;
                        p.x = colIdx;
                        ss_clustered_objects.cluster_indices[ss_clustered_objects.labels.at<float>(rowIdx, colIdx) - 1]
                            .push_back(p);
                        update_object(
                            ss_clustered_objects.objects[ss_clustered_objects.labels.at<float>(rowIdx, colIdx) - 1],
                            sgm.x.at<float>(rowIdx, colIdx), sgm.y.at<float>(rowIdx, colIdx),
                            sgm.z.at<float>(rowIdx, colIdx));
                    } else {
                        std::vector<cv::Point> indices;
                        ss_clustered_objects.cluster_indices.push_back(indices);
                        ss_clustered_objects.labels.at<float>(rowIdx, colIdx) =
                            static_cast<int>(ss_clustered_objects.cluster_indices.size());  // cluster_idx;
                        cv::Point p;
                        p.y = rowIdx;
                        p.x = colIdx;
                        ss_clustered_objects.cluster_indices[ss_clustered_objects.labels.at<float>(rowIdx, colIdx) - 1]
                            .push_back(p);
                        object obj;
                        (void)create_object(sgm.x.at<float>(rowIdx, colIdx), sgm.y.at<float>(rowIdx, colIdx),
                                            sgm.z.at<float>(rowIdx, colIdx), obj);
                        ss_clustered_objects.objects.push_back(obj);
                    }
                } else if (colIdx > 0 && diffLeft.at<float>(rowIdx, colIdx) <= this->config_.distance_threshold) {
                    if (ss_clustered_objects.labels.at<float>(rowIdx, colIdx - 1) > 0) {
                        ss_clustered_objects.labels.at<float>(rowIdx, colIdx) =
                            ss_clustered_objects.labels.at<float>(rowIdx, colIdx - 1);
                        cv::Point p;
                        p.y = rowIdx;
                        p.x = colIdx;
                        ss_clustered_objects.cluster_indices[ss_clustered_objects.labels.at<float>(rowIdx, colIdx) - 1]
                            .push_back(p);
                        (void)update_object(
                            ss_clustered_objects.objects[ss_clustered_objects.labels.at<float>(rowIdx, colIdx) - 1],
                            sgm.x.at<float>(rowIdx, colIdx), sgm.y.at<float>(rowIdx, colIdx),
                            sgm.z.at<float>(rowIdx, colIdx));
                    } else {
                        std::vector<cv::Point> indices;
                        ss_clustered_objects.cluster_indices.push_back(indices);
                        ss_clustered_objects.labels.at<float>(rowIdx, colIdx) =
                            static_cast<int>(ss_clustered_objects.cluster_indices.size());  // cluster_idx;
                        cv::Point p;
                        p.y = rowIdx;
                        p.x = colIdx;
                        ss_clustered_objects.cluster_indices[ss_clustered_objects.labels.at<float>(rowIdx, colIdx) - 1]
                            .push_back(p);
                        object obj;
                        (void)create_object(sgm.x.at<float>(rowIdx, colIdx), sgm.y.at<float>(rowIdx, colIdx),
                                            sgm.z.at<float>(rowIdx, colIdx), obj);
                        ss_clustered_objects.objects.push_back(obj);
                    }
                } else if (diffLeft.at<float>(rowIdx, colIdx) <= this->config_.distance_threshold ||
                           diffUp.at<float>(rowIdx, colIdx) <= this->config_.distance_threshold) {
                    std::vector<cv::Point> indices;
                    ss_clustered_objects.cluster_indices.push_back(indices);
                    ss_clustered_objects.labels.at<float>(rowIdx, colIdx) =
                        static_cast<int>(ss_clustered_objects.cluster_indices.size());  // cluster_idx;
                    cv::Point p;
                    p.y = rowIdx;
                    p.x = colIdx;
                    ss_clustered_objects.cluster_indices[ss_clustered_objects.labels.at<float>(rowIdx, colIdx) - 1]
                        .push_back(p);
                    object obj;
                    (void)create_object(sgm.x.at<float>(rowIdx, colIdx), sgm.y.at<float>(rowIdx, colIdx),
                                        sgm.z.at<float>(rowIdx, colIdx), obj);
                    ss_clustered_objects.objects.push_back(obj);
                }
            }
        }
    }
    return ErrorCode::Success;
}
ErrorCode SGMSegmentation::create_object(double const& x, double const& y, double const& z, object& obj) {
    obj.pos_x = x;
    obj.pos_y = y;
    obj.pos_z = z;
    obj.min_x = x;
    obj.min_y = y;
    obj.min_z = z;
    obj.max_x = x;
    obj.max_y = y;
    obj.max_z = z;
    obj.size_x = 0;
    obj.size_y = 0;
    obj.size_z = 0;
    return ErrorCode::Success;
}
ErrorCode SGMSegmentation::update_object(object& obj, double const& x, double const& y, double const& z) {
    if (obj.min_x > x) {
        obj.min_x = x;
    }
    if (obj.max_x < x) {
        obj.min_x = x;
    }

    if (obj.min_y > y) {
        obj.min_y = y;
    }
    if (obj.max_y < y) {
        obj.min_y = y;
    }

    if (obj.min_z > z) {
        obj.min_z = z;
    }
    if (obj.max_z < z) {
        obj.min_z = z;
    }

    obj.pos_x = (obj.max_x + obj.min_x) / 2;
    obj.pos_y = (obj.max_y + obj.min_y) / 2;
    obj.pos_z = (obj.max_z + obj.min_z) / 2;

    obj.size_x = abs(obj.max_x - obj.min_x);
    obj.size_y = abs(obj.max_y - obj.min_y);
    obj.size_z = abs(obj.max_z - obj.min_z);
    return ErrorCode::Success;
}
ErrorCode SGMSegmentation::clear_object(object& obj) {
    obj.pos_x = 0;
    obj.pos_y = 0;
    obj.pos_z = 0;
    obj.min_x = 0;
    obj.min_y = 0;
    obj.min_z = 0;
    obj.max_x = 0;
    obj.max_y = 0;
    obj.max_z = 0;
    obj.size_x = 0;
    obj.size_y = 0;
    obj.size_z = 0;
    return ErrorCode::Success;
}