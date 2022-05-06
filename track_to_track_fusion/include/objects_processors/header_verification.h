#pragma once

#include "objects_processors/objects_processors.h"

namespace objects_processors
{
template <class T>
class HeaderVerification : public ObjectsProcessor<T>
{
public:
  explicit HeaderVerification(std::string const& expected_frame) : expected_frame_(expected_frame)
  {
  }

  bool process(T& msg, std::string const& sensor_modality) const override
  {
    if (msg.header.frame_id != expected_frame_)
    {
      std::stringstream sstr;
      sstr << "[" << sensor_modality << "]: wrong header frame [" << msg.header.frame_id << "]. Expected ["
           << expected_frame_ << "]";
      throw std::runtime_error(sstr.str());
    }
    return true;
  }

private:
  std::string const expected_frame_{};
};
}  // namespace objects_processors