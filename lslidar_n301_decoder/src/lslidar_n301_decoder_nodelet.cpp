/*
 * This file is part of lslidar_n301 driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <lslidar_n301_decoder/lslidar_n301_decoder_nodelet.h>

namespace lslidar_n301_decoder {

void LslidarN301DecoderNodelet::onInit() {
  decoder.reset(new LslidarN301Decoder(
        getNodeHandle(), getPrivateNodeHandle()));
  if(!decoder->initialize()) {
    ROS_ERROR("Cannot initialize the lslidar puck decoder...");
    return;
  }
  return;
}

} // end namespace lslidar_n301_decoder

PLUGINLIB_DECLARE_CLASS(lslidar_n301_decoder, LslidarN301Nodelet,
    lslidar_n301_decoder::LslidarN301DecoderNodelet, nodelet::Nodelet);
