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

#include "rs2_driver.h"
#include "librealsense2/rs.hpp"
#include <iostream>
#include <memory>
#include <cstring>

#include <openni2/image_t.hpp>
#include <openni2/images_t.hpp>
#include <openni2/image_metadata_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <zlib.h>

rs2_driver::rs2_driver(std::shared_ptr<lcm::LCM> &lcm) :
    rs2_lcm(lcm)
{
	initSuccess = true;
    rs2_init_device();
    rs2_config();
    rs2_image_grabber();
}

rs2_driver::~rs2_driver(){};

void rs2_driver::rs2_init_device()
{
	auto list = ctx.query_devices();
    if (list.size() == 0){
        initSuccess = false;
    }

	std::cout << "Total " << list.size() << "devices." << std::endl
			<< "list all valid rs2 devices:" << std::endl;

	//for (auto& it : list ) {
	//	std::cout << it.get_info(RS2_CAMERA_INFO_NAME) << " " <<
   //				it.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
    //}

	std::cout << "!!! default use first device !!!" << std::endl;

    rs2::device tmp_dev = list.front();
    dev = &tmp_dev;
}

void rs2_driver::rs2_config()
{
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8,30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);//PIXEL_FORMAT_GRAY16 in openni?
}

void rs2_driver::rs2_image_grabber()
{
    pipe.start(cfg);

    while (true) {
   		std::shared_ptr<openni2::image_t> depth_image(new openni2::image_t);
   		std::shared_ptr<openni2::image_t> color_image(new openni2::image_t);
		auto frames = pipe.wait_for_frames();
		rs2::depth_frame current_depth_frame = frames.get_depth_frame();
		rs2::video_frame current_color_frame = frames.get_color_frame();

        depth_image->width = current_depth_frame.get_width();
		depth_image->height = current_depth_frame.get_width();
        depth_image->nmetadata = 0;
        depth_image->row_stride = sizeof(unsigned char) * 3 * depth_image->width;//  get_stride_in_bytes() ??
        depth_image->pixelformat = openni2::image_t::PIXEL_FORMAT_INVALID; //depth is not encode in normal way
        int data_size = current_depth_frame.get_width() * current_depth_frame.get_height() * 2;
        std::cout << "data_size = " << data_size << std::endl;
        depth_image->size = data_size;
        depth_image->data.resize(data_size);
        std::cout << "New size = " << depth_image->data.size() << std::endl;
        memcpy(&depth_image->data[0], current_depth_frame.get_data(), data_size);

        rs2_lcm->publish( "OPENNI_DEPTH", depth_image.get());
    }
}

