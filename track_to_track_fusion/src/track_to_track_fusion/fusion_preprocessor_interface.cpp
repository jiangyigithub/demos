#include "track_to_track_fusion/fusion_interface.h"

namespace track_to_track_fusion
{
namespace internal
{
void preprocessSingleObjects(
    std::list<::objects_processors::SingleObjectProcessorConstPtr> const& single_object_processors,
    std::string const& sensor_modality, NonCopyableObjects& object_container)
{
  auto object_it = std::begin(object_container.objects);
  while (object_it != std::end(object_container.objects))
  {
    bool object_is_valid = true;
    for (auto const& single_object_preprocessor : single_object_processors)
    {
      object_is_valid = object_is_valid && single_object_preprocessor->process(*object_it, sensor_modality);
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

bool preprocessObjectList(std::list<::objects_processors::ObjectListProcessorConstPtr> const& object_list_processors,
                          std::string const& sensor_modality, NonCopyableObjects& object_list)
{
  bool object_list_valid = true;
  for (auto const& object_list_preprocessor : object_list_processors)
  {
    object_list_valid = object_list_valid && object_list_preprocessor->process(object_list, sensor_modality);
  }
  return object_list_valid;
}

}  // namespace internal

void FusionPreprocessorInterface::init(std::set<std::string> const& sensor_modalities)
{
  for (auto const& sensor_modality : sensor_modalities)
  {
    data_[sensor_modality] = nullptr;
    preprocessed_data_[sensor_modality] = nullptr;
  }
}

bool FusionPreprocessorInterface::preprocess(track_to_track_fusion::NonCopyableObjects& objects,
                                             std::string const& sensor_modality) const
{
  if (::track_to_track_fusion::internal::preprocessObjectList(object_list_pre_processors_, sensor_modality, objects))
  {
    ::track_to_track_fusion::internal::preprocessSingleObjects(single_object_pre_processors_, sensor_modality, objects);
    return true;
  }
  return false;
}

void FusionPreprocessorInterface::addInputData(PerceptionKitObjectsConstSharedPtr objects,
                                               std::string const& sensor_modality)
{
  try
  {
    data_.at(sensor_modality) = objects;
    preprocessed_data_.at(sensor_modality) = nullptr;
  }
  catch (std::out_of_range const& out_of_range_exception)
  {
    (void)out_of_range_exception;
    std::stringstream sstr;
    sstr << "tried to add data for sensor " << sensor_modality
         << " which was not initialized. Call FusionInterface::init() first" << std::endl;
    throw std::runtime_error(sstr.str());
  }
}

FusionInterface::NonCopyableObjectsContainer FusionPreprocessorInterface::getData()
{
  for (auto data : data_)
  {
    // this is the only case we need to do sth. When there is new input data (data.second) and the data is not already
    // processed (!preprocessed data)
    if (data.second && !preprocessed_data_.at(data.first))
    {
      // this is the only place in code, where we need to (deep) copy the data
      auto preprocessed_data = std::make_shared<track_to_track_fusion::NonCopyableObjects>(*data.second);
      if (preprocess(*preprocessed_data, data.first))
      {
        preprocessed_data_.at(data.first) = preprocessed_data;
      }
    }
  }

  return preprocessed_data_;
}

void FusionPreprocessorInterface::register_preprocessor(objects_processors::SingleObjectProcessorConstPtr preprocessor)
{
  single_object_pre_processors_.push_back(std::move(preprocessor));
}

void FusionPreprocessorInterface::register_preprocessor(objects_processors::ObjectListProcessorConstPtr preprocessor)
{
  object_list_pre_processors_.push_back(std::move(preprocessor));
}

}  // namespace track_to_track_fusion