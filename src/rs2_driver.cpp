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

#include <lcmtypes/openni2/image_t.hpp>
#include <lcmtypes/openni2/images_t.hpp>
#include <lcmtypes/openni2/image_metadata_t.hpp>
#include <lcm/lcm-cpp.hpp>

rs2_driver::rs2_driver(std::shared_ptr<lcm::LCM> &lcm) :
    rs2_lcm(lcm)
{
	initSuccess = true;
    rs2_init_device();
    rs2_config();
    rs2_image_grabber();
}

rs2_driver::~rs2_driver(){};

rs2_driver::rs2_init_device()
{
	auto list = ctx.query_devices();
    if (list.size() == 0){
        errorText = "No device connected.";
        initSuccessful = false;
        return -1;
    }

	std::cout << "Total " << list.size() << "devices." << std::endl
			<< "list all valid rs2 devices:" << std::endl;

	for (auto& it : list ) {
		std::cout << it.get_info(RS2_CAMERA_INFO_NAME) << " " <<
				it.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
	}

	std::cout << "!!! default use first device !!!" << std::endl;

	dev = &list.front();
}

rs2_driver::rs2_config()
{
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8,30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);//PIXEL_FORMAT_GRAY16 in openni?
    std::cout << dev->get_info(RS2_CAMERA_INFO_NAME) << " " << dev->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
}

rs2_driver::rs2_image_grabber()
{
	
}

#endif
