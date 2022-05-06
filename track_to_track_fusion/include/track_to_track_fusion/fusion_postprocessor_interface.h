#pragma once

#include "objects_processors/objects_processors.h"
#include <list>

namespace track_to_track_fusion
{
class FusionPostprocessorInterface
{
public:
  void postprocess(PerceptionKitObjects& objects);

  void register_postprocessor(::objects_processors::SingleObjectProcessorConstPtr postprocessor);
  void register_postprocessor(::objects_processors::ObjectListProcessorConstPtr postprocessor);

private:
  std::list<::objects_processors::SingleObjectProcessorConstPtr> single_object_post_processors_;
  std::list<::objects_processors::ObjectListProcessorConstPtr> object_list_post_processors_;
};
}  // namespace track_to_track_fusion