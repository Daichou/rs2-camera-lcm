/* * BSD 3-Clause License
 *
 * Copyright (c) 2018, Daichou
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _RS2_DRIVER_H_
#define _RS2_DRIVER_H_

#include "librealsense2/rs.hpp"
#include <iostream>
#include <memory>

#include <openni2/image_t.hpp>
#include <openni2/images_t.hpp>
#include <openni2/image_metadata_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <cstdint>

class rs2_driver
{
public:
    rs2_driver(std::shared_ptr<lcm::LCM> &lcm);
    ~rs2_driver();
private:
    void rs2_init_device();
    void rs2_config();
    void rs2_image_grabber();
    std::shared_ptr<openni2::image_t> rs2_depth_lcm_generator(rs2::depth_frame&);
    std::shared_ptr<openni2::image_t> rs2_color_lcm_generator(rs2::video_frame&);
    std::shared_ptr<openni2::images_t> rs2_package_lcm_generator(
            std::shared_ptr<openni2::image_t> &depth_image,
            std::shared_ptr<openni2::image_t> &color_image);
    rs2::device *dev;
    rs2::context ctx;
    rs2::config cfg;
    rs2::pipeline pipe;
    std::shared_ptr<lcm::LCM> rs2_lcm;
    bool initSuccess;
    int jpeg_buf_size;
    uint8_t* jpeg_buf;
    int zlib_buf_size;
    uint8_t* zlib_buf;
};

#endif
