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
#include <cstdlib>
#include "jpeg_utils.h"

rs2_driver::rs2_driver(std::shared_ptr<lcm::LCM> &lcm) :
    rs2_lcm(lcm)
{
    initSuccess = true;
    jpeg_buf_size = 640 * 480 * 10;
    if (0 != posix_memalign((void**) &jpeg_buf, 16, jpeg_buf_size)) {
        fprintf(stderr, "Error allocating image buffer\n");
    }

    // allocate space for zlib compressing depth data
    zlib_buf_size = 640 * 480 * sizeof(int16_t) * 4;
    zlib_buf = (uint8_t*) malloc(zlib_buf_size);
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
        std::shared_ptr<openni2::images_t> images(new openni2::images_t);
        auto frames = pipe.wait_for_frames();
        rs2::depth_frame current_depth_frame = frames.get_depth_frame();
        rs2::video_frame current_color_frame = frames.get_color_frame();
        depth_image = rs2_depth_lcm_generator(current_depth_frame);
        color_image = rs2_color_lcm_generator(current_color_frame);
        images = rs2_package_lcm_generator(depth_image, color_image);
        rs2_lcm->publish( "OPENNI_FRAME", images.get());
    }
}

std::shared_ptr<openni2::image_t> rs2_driver::rs2_depth_lcm_generator(rs2::depth_frame& current_depth_frame)
{
    std::shared_ptr<openni2::image_t> depth_image(new openni2::image_t);
    depth_image->utime = current_depth_frame.get_timestamp();
    depth_image->width = current_depth_frame.get_width();
    depth_image->height = current_depth_frame.get_height();
    depth_image->nmetadata = 0;
    depth_image->row_stride = sizeof(unsigned char) * 2 * depth_image->width;//  get_stride_in_bytes() ??
    
    int uncompressed_size = depth_image->height * depth_image->width * sizeof(short);
    unsigned long int compressed_size = zlib_buf_size;
    compress2(zlib_buf, &compressed_size, (const Bytef*) current_depth_frame.get_data(), uncompressed_size,
            Z_BEST_SPEED);
    depth_image->size =(int)compressed_size;
    depth_image->data.resize(compressed_size);
    memcpy(&depth_image->data[0], zlib_buf, compressed_size);

    depth_image->pixelformat = openni2::image_t::PIXEL_FORMAT_INVALID; //depth is not encode in normal way
    return depth_image;
}

std::shared_ptr<openni2::image_t> rs2_driver::rs2_color_lcm_generator(rs2::video_frame& current_color_frame)
{
    std::shared_ptr<openni2::image_t> color_image(new openni2::image_t);
    color_image->utime = current_color_frame.get_timestamp();
    color_image->width = current_color_frame.get_width();
    color_image->height = current_color_frame.get_height();
    color_image->nmetadata = 0;
    color_image->row_stride = sizeof(unsigned char) * 3 * color_image->width;//  get_stride_in_bytes() ??

    int compressed_size =  color_image->height * color_image->row_stride;//image_buf_size;
    int compression_status = jpeg_compress_rgb((const uint8_t*)current_color_frame.get_data(),
            color_image->width, color_image->height,
            color_image->row_stride, jpeg_buf, &compressed_size, 94);

    if (compression_status) {
        std::cerr << "JPEG compression failed..." << std::endl;
    }

    color_image->data.resize(compressed_size);
    memcpy(&color_image->data[0], jpeg_buf, compressed_size);
    color_image->size = compressed_size;
    //color_image->pixelformat = openni2::image_t::PIXEL_FORMAT_RGB; //RGB888
    color_image->pixelformat = openni2::image_t::PIXEL_FORMAT_MJPEG; //jpeg
    return color_image;
}

std::shared_ptr<openni2::images_t> rs2_driver::rs2_package_lcm_generator(
        std::shared_ptr<openni2::image_t> &depth_image,
        std::shared_ptr<openni2::image_t> &color_image)
{
    std::shared_ptr<openni2::images_t> images(new openni2::images_t);
    images->utime = depth_image->utime;
    images->n_images = 2;
    images->images.push_back(*color_image);
    images->images.push_back(*depth_image);
    images->image_types.push_back(0); //LEFT = 0
    images->image_types.push_back(6); //DEPTH_MM_ZIPPED = 0

    return images;
}
