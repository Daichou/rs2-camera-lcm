#!/bin/sh

set -ex

lcm-gen -x --cpp-hpath=include --lazy lcmtypes/openni2_image_t.lcm
lcm-gen -x --cpp-hpath=include --lazy lcmtypes/openni2_images_t.lcm
lcm-gen -x --cpp-hpath=include --lazy lcmtypes/openni_image_metadata_t.lcm
