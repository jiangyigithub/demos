/* main.cc
 *
 * Copyright (C) 2002 The libxml++ development team
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the Free
 * Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cstdint>

#include <libxml++/libxml++.h>

xmlpp::DomParser parser;

bool found_horizontal_angle_correction_ = false;
bool found_vertical_angle_correction_ = false;
bool found_distance_correction_ = false;
bool found_distance_correction_x_ = false;
bool found_distance_correction_y_ = false;
bool found_horizontal_offset_correction_ = false;
bool found_vertical_offset_correction_ = false;

std::vector<double> horizontal_angle_corrections_;
std::vector<double> vertical_angle_corrections_;
std::vector<double> distance_corrections_;
std::vector<double> distance_corrections_x_;
std::vector<double> distance_corrections_y_;
std::vector<double> horizontal_offset_corrections_;
std::vector<double> vertical_offset_corrections_;

std::vector<uint16_t> min_intensities_;
std::vector<uint16_t> max_intensities_;
std::vector<bool> enabled_;

template<class T>
bool fromString(T& t, const std::string& s, std::ios_base& (*f)(std::ios_base&) = std::dec) {
  std::istringstream iss(s);
  return !(iss >> f >> t).fail();
}

template<class T>
void getItemVector(const xmlpp::Node::NodeList& node_list, std::vector<T>& res) {
  xmlpp::Node::NodeList::const_iterator nlit = node_list.begin(), nlit_end = node_list.end();

  for (; nlit != nlit_end; nlit++) {

    const xmlpp::Node* node = *nlit;

    if (node->get_name() == "item") {
      const xmlpp::Node::NodeList& item_content = node->get_children();
      if (!item_content.empty()) {
        const xmlpp::TextNode* item_text_node = dynamic_cast<const xmlpp::TextNode*> (*item_content.begin());
        if (item_text_node) {
          T val;
          fromString<T> (val, item_text_node->get_content());
          res.push_back(val);
        }
      }
    }
  }
}

void parseConfig(const xmlpp::Node* node) {

  const xmlpp::ContentNode* content_node = dynamic_cast<const xmlpp::ContentNode*> (node);
  const xmlpp::TextNode* text_node = dynamic_cast<const xmlpp::TextNode*> (node);

  if (text_node && text_node->is_white_space()) //Let's ignore the indenting - you don't always want to do this.
  return;

  const Glib::ustring nodename = node->get_name();

  if (nodename == "rotCorrection_") {
    found_horizontal_angle_correction_ = true;
  }
  else if (found_horizontal_angle_correction_) {
    if (text_node) {
      double angle;
      fromString<double> (angle, std::string(text_node->get_content()));
      horizontal_angle_corrections_.push_back(angle);
    }
    found_horizontal_angle_correction_ = false;
  }

  if (nodename == "vertCorrection_") {
    found_vertical_angle_correction_ = true;
  }
  else if (found_vertical_angle_correction_) {
    if (text_node) {
      double angle;
      fromString<double> (angle, std::string(text_node->get_content()));
      vertical_angle_corrections_.push_back(angle);
    }
    found_vertical_angle_correction_ = false;
  }

  if (nodename == "distCorrection_") {
    found_distance_correction_ = true;
  }
  else if (found_distance_correction_) {
    if (text_node) {
      double offset;
      fromString<double> (offset, std::string(text_node->get_content()));
      distance_corrections_.push_back(offset);
    }
    found_distance_correction_ = false;
  }

  if (nodename == "distCorrectionX_") {
    found_distance_correction_x_ = true;
  }
  else if (found_distance_correction_x_) {
    if (text_node) {
      double offset;
      fromString<double> (offset, std::string(text_node->get_content()));
      distance_corrections_x_.push_back(offset);
    }
    found_distance_correction_x_ = false;
  }

  if (nodename == "distCorrectionY_") {
    found_distance_correction_y_ = true;
  }
  else if (found_distance_correction_y_) {
    if (text_node) {
      double offset;
      fromString<double> (offset, std::string(text_node->get_content()));
      distance_corrections_y_.push_back(offset);
    }
    found_distance_correction_y_ = false;
  }

  if (nodename == "horizOffsetCorrection_") {
    found_horizontal_offset_correction_ = true;
  }
  else if (found_horizontal_offset_correction_) {
    if (text_node) {
      double offset;
      fromString<double> (offset, std::string(text_node->get_content()));
      horizontal_offset_corrections_.push_back(offset);
    }
    found_horizontal_offset_correction_ = false;
  }

  if (nodename == "vertOffsetCorrection_") {
    found_vertical_offset_correction_ = true;
  }
  else if (found_vertical_offset_correction_) {
    if (text_node) {
      double offset;
      fromString<double> (offset, std::string(text_node->get_content()));
      vertical_offset_corrections_.push_back(offset);
    }
    found_vertical_offset_correction_ = false;
  }

  if (nodename == "minIntensity_") {
    getItemVector(node->get_children(), min_intensities_);
  }
  else if (nodename == "maxIntensity_") {
    getItemVector(node->get_children(), max_intensities_);
  }
  else if (nodename == "enabled_") {
    getItemVector(node->get_children(), enabled_);
  }

  if (!content_node) {
    xmlpp::Node::NodeList list = node->get_children();
    for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter) {
      parseConfig(*iter);
    }
  }
}

int main(int argc, char* argv[]) {
  // Set the global C++ locale to the user-configured locale,
  // so we can use std::cout with UTF-8, via Glib::ustring, without exceptions.
  std::locale::global(std::locale(""));

  if(argc < 3) {
    std::cout << "Usage: " << argv[0] << " <in:velodyne xml config file> <out:calib file> <out: intensity calib file>\n";
    exit(-5);
  }

  try
  {
    //parser.set_validate();
    parser.set_substitute_entities(); //We just want the text to be resolved/unescaped automatically.
    parser.parse_file(argv[1]);
    if (parser) {
      parseConfig(parser.get_document()->get_root_node());
    }
  }
  catch (const std::exception& ex)
  {
    std::cout << "Exception caught: " << ex.what() << std::endl;
    exit(EXIT_FAILURE);
  }

  size_t num_beams = horizontal_angle_corrections_.size();
  if(vertical_angle_corrections_.size() != num_beams ||
      distance_corrections_.size() != num_beams ||
      distance_corrections_x_.size() != num_beams ||
      distance_corrections_y_.size() != num_beams ||
      horizontal_offset_corrections_.size() != num_beams ||
      vertical_offset_corrections_.size() != num_beams )
  {
    std::cout << "Error in config file: not all parameters could be read.\n";
    return -5;
  }

  std::cout << "index\tangle_h  \tangle_v  \toff_dist\toff_dist_x\toff_dist_y\toffset_v\toffset_h\tin_min\tin_max\tenabled\n";
  std::ios_base::fmtflags old_flags = std::cout.flags();
  for (size_t i = 0; i < num_beams; i++)
  {
    std::cout << i << std::fixed << std::setprecision(6) << "\t" << horizontal_angle_corrections_[i] << "\t" << vertical_angle_corrections_[i] << "\t"
        << distance_corrections_[i]/100.0 << "\t" << distance_corrections_x_[i]/100.0 << "\t" << distance_corrections_y_[i]/100.0 << "\t"
              << vertical_offset_corrections_[i]/100.0 << "\t" << horizontal_offset_corrections_[i]/100.0 << "\t"
              << min_intensities_[i]<< "\t" << max_intensities_[i] << "\t" << enabled_[i] << std::endl;
  }
  std::cout.flags(old_flags); // reset flags

  // XYZ
  std::ofstream calib_file;
  calib_file.open(argv[2]);
  if(!calib_file.is_open())
  {
    std::cout << " Could not open " << argv[2] << " for writing." << std::endl;
    return -10;
  }
  calib_file << "RANGE_MULTIPLIER 1.000\n";
  for (size_t i = 0; i < num_beams; i++)
  {
    calib_file << i << std::fixed << std::setprecision(6) << "\t" << horizontal_angle_corrections_[i] << "\t" << vertical_angle_corrections_[i] << "\t"
        << distance_corrections_[i]/100.0 << "\t" << distance_corrections_x_[i]/100.0 << "\t" << distance_corrections_y_[i]/100.0 << "\t"
              << vertical_offset_corrections_[i]/100.0 << "\t" << horizontal_offset_corrections_[i]/100.0 << "\t"
              << enabled_[i] << std::endl;
  }
  calib_file.close();

  // intensity
  std::ofstream calib_intensity_file;
  calib_intensity_file.open(argv[3]);
  if(!calib_intensity_file.is_open()) {
    std::cout << " Could not open " << argv[3] << " for writing.\n";
    return -10;
  }

  calib_intensity_file << "0 255\n";
  for (size_t i = 0; i < num_beams; i++)
  {
    for(uint16_t j=0; j<256; j++)
    {
      float val = (((float)j))/(255.0-0.0)*(max_intensities_[i]-min_intensities_[i])+min_intensities_[i];
      calib_intensity_file << (uint16_t)(val+0.5) <<" ";
    }
    calib_intensity_file << std::endl;
  }
  calib_intensity_file.close();

  return 0;
}
