#include "track_to_track_fusion/fusion_postprocessor_interface.h"

namespace track_to_track_fusion
{
void FusionPostprocessorInterface::postprocess(PerceptionKitObjects& object_container)
{
  if (object_list_post_processors_.size())
  {
    throw std::runtime_error("Post processing is not implemented yet!");
  }

  // @todo: same code than in preprocessing -> put into function
  auto object_it = std::begin(object_container.objects);
  while (object_it != std::end(object_container.objects))
  {
    bool object_is_valid = true;
    for (auto const& single_object_preprocessor : single_object_post_processors_)
    {
      object_is_valid = object_is_valid && single_object_preprocessor->process(*object_it, "fusion");
    }

    if (object_is_valid)
    {
      ++object_it;
    }
    else
    {
      object_it = object_container.objects.erase(object_it);
    }
  }
}

void FusionPostprocessorInterface::register_postprocessor(
    objects_processors::SingleObjectProcessorConstPtr postprocessor)
{
  single_object_post_processors_.push_back(std::move(postprocessor));
}
void FusionPostprocessorInterface::register_postprocessor(objects_processors::ObjectListProcessorConstPtr postprocessor)
{
  object_list_post_processors_.push_back(std::move(postprocessor));
}

}  // namespace track_to_track_fusion