/*
 * Desc: Laser Scan fusion.
 * Author: khaled elmadawi
 * mail: khalid.elmadawi@sigma.se
 * Date: 30 march 2021
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

#ifndef DATA_CONTAINER_HH
#define DATA_CONTAINER_HH
#include <string>

class scan {
   public:
    float x;
    float y;
    float yaw;
    float depth;
    int pointIdx;
};
class fusion_config {
   public:
    std::string fusion_frame;
    int number_fused_points;
    float range_min;
    float range_max;
    float dis_threshold;
    float tf_listener_rate;
};

enum ErrorCode { Success, Failed, Unknown, BadArgument };

#endif