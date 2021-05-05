/*
 * Desc: Pointcloud assembler.
 * Author: khaled elmadawi
 * mail: khalid.elmadawi@sigma.se
 * Date: 14 September 2020
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

#include <string>

class configuration {
   public:
    double voxel_size;
    double tf_duration;
    std::string odom_frame;
};

enum ErrorCode { Success, Failed, Unknown, BadArgument };