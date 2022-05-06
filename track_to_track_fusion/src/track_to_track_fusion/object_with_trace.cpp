#include "track_to_track_fusion/object_with_trace.h"
#include <iostream>

namespace track_to_track_fusion
{
std::ostream& operator<<(std::ostream& os, const ObjectWithTrace& object_with_trace)
{
  os << "Object[" << object_with_trace.internal_fusion_id() << "]: { ";
  for (auto const trace : object_with_trace.trace_)
  {
    os << trace.first << ":" << trace.second << " ";
  }
  os << "}";
  return os;
}

ObjectWithTrace::ObjectWithTrace(ObjectIt const& object, Trace trace, uint32_t internal_fusion_id)
  : object_ptr_(nullptr), object_(object), trace_(trace), internal_fusion_id_(internal_fusion_id)
{
}

ObjectWithTrace::ObjectWithTrace(std::unique_ptr<PerceptionKitObject const> object, Trace trace,
                                 uint32_t internal_fusion_id)
  : object_ptr_(std::move(object)), object_(*object_ptr_), trace_(trace), internal_fusion_id_(internal_fusion_id)
{
}

uint32_t ObjectWithTrace::internal_fusion_id() const
{
  return internal_fusion_id_;
}

bool ObjectWithTrace::isSameObject(ObjectWithTrace const& other) const
{
  auto const a = &other.object_;
  auto const b = &object_;
  return a == b;
}

bool ObjectWithTrace::hasCommonSensorTrace(ObjectWithTrace const& other) const
{
  auto trace_union = trace_;
  trace_union.insert(other.trace_.begin(), other.trace_.end());
  return trace_.size() + other.trace_.size() != trace_union.size();
}

PerceptionKitObject const& ObjectWithTrace::object() const
{
  return object_;
}

ObjectWithTrace::Trace const& ObjectWithTrace::trace() const
{
  return trace_;
}

}  // namespace track_to_track_fusion
