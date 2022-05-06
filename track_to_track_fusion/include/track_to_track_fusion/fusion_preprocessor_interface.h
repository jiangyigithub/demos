#pragma once

#include "objects_processors/objects_processors.h"

#include <string>
#include <map>
#include <list>
#include <set>

#ifdef ROS2
using PerceptionKitObjectsConstSharedPtr = PerceptionKitObjects::ConstSharedPtr;
using PerceptionKitObjectsSharedPtr = PerceptionKitObjects::SharedPtr;
#else
using PerceptionKitObjectsConstSharedPtr = PerceptionKitObjects::ConstPtr;
using PerceptionKitObjectsSharedPtr = PerceptionKitObjects::Ptr;
#endif

namespace track_to_track_fusion
{
class FusionPreprocessorInterface
{
public:
  using NonCopyableObjectsContainer = std::map<std::string const, NonCopyableObjects::Ptr>;
  using DataContainer = std::map<std::string const, PerceptionKitObjectsConstSharedPtr>;

  void register_preprocessor(::objects_processors::SingleObjectProcessorConstPtr preprocessor);
  void register_preprocessor(::objects_processors::ObjectListProcessorConstPtr preprocessor);
  void addInputData(PerceptionKitObjectsConstSharedPtr objects, std::string const& sensor_modality);

protected:
  NonCopyableObjectsContainer getData();
  void init(std::set<std::string> const& sensor_modalities);

private:
  /** returns true, if the object list is valid after preprocessing */
  bool preprocess(track_to_track_fusion::NonCopyableObjects& objects, std::string const& sensor_modality) const;

  std::list<::objects_processors::SingleObjectProcessorConstPtr> single_object_pre_processors_;
  std::list<::objects_processors::ObjectListProcessorConstPtr> object_list_pre_processors_;

  DataContainer data_;
  NonCopyableObjectsContainer preprocessed_data_;
};

}  // namespace track_to_track_fusion
