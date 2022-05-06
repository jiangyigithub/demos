#include "track_to_track_fusion/trace_storage.h"
#include <set>

namespace track_to_track_fusion
{
namespace internal
{
bool traceWasAlreadySeen(ObjectWithTrace::Trace const& trace_previous_cycle,
                         ObjectWithTrace::Trace const& trace_this_cycle)
{
  // True cases:
  // a) everything is the same
  // b) one or more sensors vanished, but the ids of the remaining sensors are identical
  // c) one or more sensors are new, but the ids of the previous ones are the same

  if (trace_this_cycle.size() == 0 || trace_previous_cycle.size() == 0)
    throw std::runtime_error("Traces must not be empty");

  std::set<std::string> sensor_modalities;
  for (auto const& trace : { trace_previous_cycle, trace_this_cycle })
  {
    std::transform(trace.begin(), trace.end(), std::inserter(sensor_modalities, sensor_modalities.begin()),
                   [](ObjectWithTrace::Trace::value_type const& pair) { return pair.first; });
  }

  auto const disjunct = sensor_modalities.size() == trace_this_cycle.size() + trace_previous_cycle.size();

  auto ret = !disjunct;
  for (auto const& sensor_modality : sensor_modalities)
  {
    if (!ret)
      break;

    ret &= trace_previous_cycle.count(sensor_modality) == 0 || trace_this_cycle.count(sensor_modality) == 0 ||
           trace_this_cycle.at(sensor_modality) == trace_previous_cycle.at(sensor_modality);
  }

#if 0
  if (ret)
  {
    std::cout << "SAME TRACE" << std::endl;
    for (auto const& p : trace_previous_cycle)
    {
      std::cout << "Prev: " << p.first << ":" << p.second << std::endl;
    }
    for (auto const& p : trace_this_cycle)
    {
      std::cout << "This: " << p.first << ":" << p.second << std::endl;
    }
  }
#endif
  return ret;
}

}  // namespace internal

PerceptionKitObject::_id_type TraceStorage::getIdFromTrace(ObjectWithTrace::Trace const& trace_this_cycle)
{
  static PerceptionKitObject::_id_type id{ 0 };
  static std::map<int, ObjectWithTrace::Trace> traces;

  for (auto trace : traces)
  {
    if (internal::traceWasAlreadySeen(trace.second, trace_this_cycle))
    {
      traces.at(trace.first) = trace_this_cycle;
      return trace.first;
    }
  }

  // @todo: cleanup old traces

  ++id;
  traces[id] = trace_this_cycle;
  return id;
}

}  // namespace track_to_track_fusion