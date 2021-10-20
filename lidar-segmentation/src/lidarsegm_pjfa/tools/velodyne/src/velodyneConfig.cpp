/********************************************************
  Stanford Driving Software
  Copyright (c) 2011 Stanford University
  All rights reserved.

  Redistribution and use in source and binary forms, with 
  or without modification, are permitted provided that the 
  following conditions are met:

* Redistributions of source code must retain the above 
  copyright notice, this list of conditions and the 
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the 
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
 ********************************************************/


#include <inttypes.h>
#include <angles/angles.h>

#include <driving_common/math_trig.h>
#include <driving_common/file_tools.h>
#include <velodyne/defaultcal.h>
#include <velodyne/velodyneConfig.h>

namespace drc = driving_common;

namespace velodyne {

typedef struct {
  double angle;
  int idx;
} beam_angle_t;

Config::Config()
{
  autoConfig();
  recomputeAngles();
  findBeamOrder();
}

Config::~Config() {

}

static int beamCompare(const void *a, const void *b) {
  static beam_angle_t v1, v2;
  v1 = *(beam_angle_t *) a;
  v2 = *(beam_angle_t *) b;
  if (v1.angle > v2.angle) return 1;
  else if (v1.angle == v2.angle) return 0;
  else return -1;
}

void Config::findBeamOrder() {
  beam_angle_t beams[VELO_NUM_LASERS];

  for (int32_t i = 0; i < VELO_NUM_LASERS; i++) {
    beams[i].angle = vert_angle[i];
    beams[i].idx = i;
  }

  qsort(beams, VELO_NUM_LASERS, sizeof(beam_angle_t), beamCompare);

  for (int32_t i = 0; i < VELO_NUM_LASERS; i++) {
    beam_order[i] = beams[i].idx;
    inv_beam_order[beams[i].idx] = i;
  }
}

void Config::recomputeAngles() {
  int i, j;
  double angle = 0;
  double laser_offset_yaw = 0.0;

  for (i = 0; i < 64; i++) {
    cos_vert_angle[i] = cos(vert_angle[i]);
    sin_vert_angle[i] = sin(vert_angle[i]);

    for (j = 0; j < VELO_NUM_TICKS; j++) {
      angle = drc::normalizeAngle(laser_offset_yaw - j / (float) VELO_NUM_TICKS * 2.0 * M_PI + rot_angle[i]);
      cos_rot_angle[j][i] = cos(angle);
      sin_rot_angle[j][i] = sin(angle);
    }
  }
  for (j = 0; j < VELO_NUM_TICKS; j++) {
    enc_angle[j] = drc::normalizeAngle(laser_offset_yaw - j / (float) VELO_NUM_TICKS * 2.0 * M_PI);
    cos_enc_angle[j] = cos(enc_angle[j]);
    sin_enc_angle[j] = sin(enc_angle[j]);
  }
}

void Config::autoConfig()
{
  global_range_offset = 0;
  range_multiplier = 1.0;
  min_intensity = 0;
  max_intensity = 0;
  spin_start = 0;

  for (int32_t i = 0; i < VELO_NUM_LASERS; i++) {
    range_offset[i] = 0;
    range_offsetX[i] = 0;
    range_offsetY[i] = 0;
    laser_enabled[i] = 1;
    vert_angle[i] = angles::from_degrees(DEFAULT_VELO_VERT_ANGLE[i]);
    rot_angle[i] = angles::from_degrees(DEFAULT_VELO_ROT_ANGLE[i]);
    h_offset[i] = 0;
    v_offset[i] = 0;
  }
}

#define MAX_LINE_LENGTH    512

bool Config::readIntensity(const std::string& filename) {
  FILE* iop = NULL;

  if(filename.empty()) { // not using calibrated intensities so skip this
    return true;
  }

  std::string expanded_filename = drc::expandFileName(filename);
  if (expanded_filename.empty()) {
    fprintf(stderr, "[31;1m# ERROR: could not expand filename %s[0m\n", filename.c_str());
    return false;
  }
  if ((iop = fopen(expanded_filename.c_str(), "r")) == 0) {
    fprintf(stderr, "[31;1m# ERROR: could not open velodyne intensity calibration file %s[0m\n", expanded_filename.c_str());
    return false;
  }
  fprintf(stderr, "# INFO: read velodyne intensity calibration file %s\n", expanded_filename.c_str());

  if(fscanf(iop, "%d %d\n", &min_intensity, &max_intensity) == EOF)
  {
    fprintf(stderr, "Error reading  min/max intensity values.\n");
    fclose(iop);
    return false;
  }

  if(max_intensity == min_intensity)
  {
    fprintf(stderr, "Invalid min/max intensity values. Values are equal: %d\n", min_intensity);
    fclose(iop);
    return false;
  }

  if (max_intensity < min_intensity)
  {
    fprintf(stderr, "Invalid min/max intensity values. Maximum %d < minimum %d\n", max_intensity, min_intensity);
    fclose(iop);
    return false;
  }

  static const int INTENSITY_BOUNDS_MAX = 255;
  static const int INTENSITY_BOUNDS_MIN = 0;
  if (INTENSITY_BOUNDS_MAX < max_intensity)
  {
    fprintf(stderr, "Maximum intensity %d is above maximum allowed: %d\n", max_intensity, INTENSITY_BOUNDS_MAX);
    fclose(iop);
    return false;
  }

  if (INTENSITY_BOUNDS_MIN > min_intensity)
  {
    fprintf(stderr, "Minimum intensity %d is below minimum allowed: %d\n", min_intensity, INTENSITY_BOUNDS_MIN);
    fclose(iop);
    return false;
  }


  for (uint8_t i = 0; i < 64; i++)
  {
    for (uint16_t j = 0; j < 256; j++)
    {
      if(fscanf(iop, "%lf ", &intensity_map[i][j]) == EOF)
      {
        fprintf(stderr, "Error reading intensity value.\n");
        fclose(iop);
        return false;
      }
      float expanded = (intensity_map[i][j] - min_intensity) / (max_intensity - min_intensity);
      if (expanded < 0) {expanded = 0;}
      if (expanded > 1) {expanded = 1;}
      intensity_map[i][j] = (unsigned char) (255 * expanded);
    }
  }
  fclose(iop);
  //printf("New min:%d    New max: %d\n", min_intensity, max_intensity);
  return true;
}

bool Config::readCalibration(const std::string& filename) {
  FILE * iop;

  int FEnd;
  int linectr = 0;
  int n, id, enabled;
  int i, j;
  double rcf, hcf, hoff, voff, dist, distX, distY;

  char command[MAX_LINE_LENGTH];
  char line[MAX_LINE_LENGTH];
  char str1[MAX_LINE_LENGTH];
  char str2[MAX_LINE_LENGTH];
  char str3[MAX_LINE_LENGTH];
  char str4[MAX_LINE_LENGTH];
  char str5[MAX_LINE_LENGTH];
  char str6[MAX_LINE_LENGTH];
  char str7[MAX_LINE_LENGTH];
  char str8[MAX_LINE_LENGTH];
  char str9[MAX_LINE_LENGTH];

  min_intensity = 0;
  max_intensity = 255;
  for (i = 0; i < 64; i++) {
    for (j = 0; j < 256; j++) {
      intensity_map[i][j] = j;
    }
  }
  range_multiplier = 1.0;
  spin_start = VELO_SPIN_START;

  std::string expanded_filename = drc::expandFileName(filename);
  if (expanded_filename.empty()) {
    fprintf(stderr, "[31;1m# ERROR: could not expand filename %s[0m\n", filename.c_str());
    return false;
  }
  if ((iop = fopen(expanded_filename.c_str(), "r")) == 0) {
    fprintf(stderr, "[31;1m# ERROR: could not open velodyne calibration file %s[0m\n", expanded_filename.c_str());
    return false;
  }
  fprintf(stderr, "# INFO: read velodyne calibration file %s\n", expanded_filename.c_str());

  FEnd = 0;
  do {
    if (fgets(line, MAX_LINE_LENGTH, iop) == NULL) FEnd = 1;
    else {
      linectr++;
      if (sscanf(line, "%s", command) == 0) {
        fclose(iop);
        return false;
      }
      else {
        if (command[0] != '#') {
          n = sscanf(line, "%s %s %s %s %s %s %s %s %s", str1, str2, str3, str4, str5, str6, str7, str8, str9);
          if (n == 9) {
            id = atoi(str1);
            rcf = atof(str2);
            hcf = atof(str3);
            dist = atof(str4);
            distX = atof(str5);
            distY = atof(str6);
            voff = atof(str7);
            hoff = atof(str8);
            enabled = atoi(str9);
            if (id < 0 || id > 63) {
              fprintf(stderr, "[31;1m# ERROR: wrong id '%d' in line %d[0m\n", id, linectr);
              fclose(iop);
              return false;
            }
            else {
              rot_angle[id] = angles::from_degrees(rcf);
              vert_angle[id] = angles::from_degrees(hcf);
              range_offset[id] = dist;
              range_offsetX[id] = distX;
              range_offsetY[id] = distY;
              laser_enabled[id] = enabled;
              v_offset[id] = voff;
              h_offset[id] = hoff;
            }
          }
          else if (n == 2) {
            if (!strcasecmp(str1, "RANGE_MULTIPLIER")) {
              range_multiplier = atof(str2);
            }
            else if (!strcasecmp(str1, "SPIN_START")) {
              spin_start = atoi(str2);
            }
            else {
              fprintf(stderr, "[31;1m# ERROR: unknown keyword '%s' in line %d[0m\n", str1, linectr);
              fclose(iop);
              return false;
            }
          }
          else {
            fprintf(stderr, "[31;1m# ERROR: error in line %d: %s[0m\n", linectr, line);
            fclose(iop);
            return false;
          }
        }
      }
    }
  } while (!FEnd);

  fclose(iop);

  recomputeAngles();
  findBeamOrder();

  return true;
}

void Config::printCalibrationData() {
  int i;

  printf("\ndouble VELO_ROT_ANGLE2[64] = { \n");
  for (i = 0; i < 16; i++) {
    printf("                                    %12.9f,%12.9f,%12.9f,%12.9f,\n", angles::to_degrees(rot_angle[4 * i + 0]), angles::to_degrees(rot_angle[4 * i + 1]),
        angles::to_degrees(rot_angle[4 * i + 2]), angles::to_degrees(rot_angle[4 * i + 3]));
  }
  printf("                                  };\n");

  printf("double VELO_VERT_ANGLE2[64] = { \n");
  for (i = 0; i < 16; i++) {
    printf("                                    %12.9f,%12.9f,%12.9f,%12.9f,\n", angles::to_degrees(vert_angle[4 * i + 0]),
        angles::to_degrees(vert_angle[4 * i + 1]), angles::to_degrees(vert_angle[4 * i + 2]), angles::to_degrees(vert_angle[4 * i + 3]));
  }
  printf("                                  };\n");

  printf("int VELO_RANGE_OFFSET2[64] = { \n");
  for (i = 0; i < 16; i++) {
    printf("                                    %3f,%3f,%3f,%3f,\n", range_offset[4 * i + 0], range_offset[4 * i + 1], range_offset[4
        * i + 2], range_offset[4 * i + 3]);
  }
  printf("                                  };\n");

  printf("char VELO_LASER_ENABLED2[64] = { \n");
  for (i = 0; i < 16; i++) {
    printf("                                    %2d,%2d,%2d,%2d,\n", laser_enabled[4 * i + 0], laser_enabled[4 * i + 1],
        laser_enabled[4 * i + 2], laser_enabled[4 * i + 3]);
  }
  printf("                                  };\n");
}

} // namespace velodyne
