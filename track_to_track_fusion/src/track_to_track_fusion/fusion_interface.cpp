#include "track_to_track_fusion/fusion_interface.h"
#include <numeric>
#include "track_to_track_fusion/cost_cell.h"
#include "track_to_track_fusion/weights.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#if __GNUC__ > 8
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#endif
#ifdef ROS2
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using PerceptionKitAttribute = perception_kit_msgs::msg::Attribute;
#else
#include <geometry_msgs/PointStamped.h>
using PerceptionKitAttribute = perception_kit_msgs::Attribute;
#endif
#pragma GCC diagnostic pop

#include "track_to_track_fusion/feature/feature_fusion_interfaces.h"
#include "track_to_track_fusion/trace_storage.h"

// @TODO: variance = sigma * sigma

namespace track_to_track_fusion
{
namespace internal
{
void initializeFirstColumnOfCostMatrix(FusionInterface::NonCopyableObjectsContainer const& data,
                                       ObjectWithTrace::Container& objects_with_trace_storage,
                                       std::set<uint32_t>& elements, uint32_t& internal_fusion_id)
{
  for (auto const& sensor_data : data)
  {
    if (!sensor_data.second)
      continue;

    for (auto objects_iterator = std::cbegin(sensor_data.second->objects);
         objects_iterator != std::cend(sensor_data.second->objects); ++objects_iterator)
    {
      objects_with_trace_storage.emplace_front(
          *objects_iterator, ObjectWithTrace::Trace({ std::make_pair(sensor_data.first, objects_iterator->id) }),
          ++internal_fusion_id);
      elements.insert(objects_with_trace_storage.front().internal_fusion_id());
    }
  }
}

void createCostMatrix(ObjectWithTrace::Container const& objects_with_trace_storage, CostCell::LMatrix& cost_matrix,
                      CostCell::CostFunction const& cost_function)
{
  for (auto const& row : objects_with_trace_storage)
  {
    for (auto const& col : objects_with_trace_storage)
    {
      auto const inserted_element = cost_matrix.emplace(row, col, cost_function);

      if (inserted_element->isDiagonalElement())
        break;
    }
  }
}

void createNewObjectFromCellFusion(CostCell const& lowest_cost_element, ObjectFusionExecuter const& fusion,
                                   ObjectWithTrace::Container& objects_with_trace_storage, uint32_t& internal_fusion_id)
{
  auto fused_object = std::make_unique<PerceptionKitObject>();

  // set header information by simply taking it from one object (the headers are equal)
  fused_object->header = lowest_cost_element.row_object().object().header;

  fusion.fuse(lowest_cost_element, *fused_object);

  // insert the new object into the list
  objects_with_trace_storage.emplace_front(std::move(fused_object), lowest_cost_element.fused_traits(),
                                           ++internal_fusion_id);
}

void removeCellFromCostMatrixAndElements(CostCell::LMatrix& cost_matrix, std::set<uint32_t>& elements,
                                         CostCell const& lowest_cost_element)
{
  // after the fusion, these two elements are not required anymore
  auto const& row_to_remove = lowest_cost_element.row_object();
  auto const& col_to_remove = lowest_cost_element.col_object();

  // remove the 'row' and the 'col' from the LMatrix which held the two objects fused
  auto it = cost_matrix.begin();
  while (it != cost_matrix.end())
  {
    if (it->contains(col_to_remove) || it->contains(row_to_remove))
      it = cost_matrix.erase(it);
    else
      ++it;
  }
  // remove the objects form the elements container
  elements.erase(row_to_remove.internal_fusion_id());
  elements.erase(col_to_remove.internal_fusion_id());
}

void calculateCostOfNewElement(CostCell::LMatrix& cost_matrix,
                               ObjectWithTrace::Container const& objects_with_trace_storage,
                               std::set<uint32_t>& elements, CostCell::CostFunction const& cost_function)
{
  cost_matrix.emplace(objects_with_trace_storage.front(), objects_with_trace_storage.front(), cost_function);
  for (auto const& row : objects_with_trace_storage)
  {
    if (elements.count(row.internal_fusion_id()))
    {
      cost_matrix.emplace(row, objects_with_trace_storage.front(), cost_function);
    }
  }
}

void addNewObjectToElements(std::set<uint32_t>& elements, ObjectWithTrace::Container const& objects_with_trace_storage)
{
  // finally add the new object to the elements list
  elements.insert(objects_with_trace_storage.front().internal_fusion_id());
}

void copyFusionResultToObjects(ObjectWithTrace::Container const& objects_with_trace_storage,
                               std::set<uint32_t> const& elements, PerceptionKitObjects::_objects_type& objects)
{
  objects.clear();
  for (auto& row : objects_with_trace_storage)
  {
    if (elements.count(row.internal_fusion_id()))
    {
      objects.push_back(row.object());
      objects.back().id = TraceStorage::getIdFromTrace(row.trace());

      for (auto const& trace : row.trace())
      {
        PerceptionKitAttribute attribute;
        attribute.name = "internal.fusion." + trace.first;
        attribute.value = { static_cast<float>(trace.second) };
        objects.back().attributes.push_back(attribute);
      }
    }
  }
}

void fuseCostMatrixElements(CostCell::LMatrix& cost_matrix, ObjectWithTrace::Container& objects_with_trace_storage,
                            std::set<uint32_t>& elements, uint32_t& internal_fusion_id,
                            CostCell::CostFunction const& cost_function, double const& threshold,
                            ObjectFusionExecuter const& fusion)
{
// Fuse the first element. Recalculate list then
#if 0
  auto iteration = 0;
#endif

  auto lowest_cost_element = std::begin(cost_matrix);
  while (lowest_cost_element != std::end(cost_matrix))
  {
#if 0
    // print
    for (auto const& cost_matrix_cell : cost_matrix)
    {
      ROS_DEBUG_STREAM("TEMP[" << iteration << "]: " << cost_matrix_cell);
    }
    ROS_DEBUG_STREAM("Elements[" << iteration << "]: " << elements.size());
    ++iteration;
#endif

    if (lowest_cost_element->cost() > threshold)
    {
      break;
    }

    // fuse the content of the matrix cell. The fused object will be pushed front into objects_with_trace_storage. The
    // objects_with_trace_storage is increasing only. Elements are not deleted here. Therefore, the elements set is kept
    // up to date with all elements which are remaining
    internal::createNewObjectFromCellFusion(*lowest_cost_element, fusion, objects_with_trace_storage,
                                            internal_fusion_id);

    internal::removeCellFromCostMatrixAndElements(cost_matrix, elements, *lowest_cost_element);

#if 0
    // print
    for (auto const& cost_matrix_cell : cost_matrix)
    {
      ROS_DEBUG_STREAM("AFTER ERASE TEMP[" << iteration << "]: " << cost_matrix_cell);
    }
    ROS_DEBUG_STREAM("AFTER ERASE Elements[" << iteration << "]: " << elements.size());
#endif

    internal::calculateCostOfNewElement(cost_matrix, objects_with_trace_storage, elements, cost_function);

    internal::addNewObjectToElements(elements, objects_with_trace_storage);

    // as long as the fusion matrix contains fuseable objects - continue
    lowest_cost_element = std::begin(cost_matrix);
  }
}

}  // namespace internal

void FusionInterface::setPredictor(objects_prediction::ObjectsPredictor::ConstPtr predictor)
{
  predictor_ = std::move(predictor);
}

void FusionInterface::init(std::set<std::string> const& sensor_modalities,
                           track_to_track_fusion::ObjectFusionExecuter::ConstPtr fusion,
                           cost_calculation::CostCalculation::ConstPtr cost_calculation)
{
  FusionPreprocessorInterface::init(sensor_modalities);

  fusion_ = std::move(fusion);
  cost_function_ = std::move(cost_calculation);
}

void FusionInterface::predict(FusionInterface::NonCopyableObjectsContainer& data_container,
                              RosTime const& requested_prediction_time) const
{
  if (!predictor_)
    throw std::runtime_error("No data predictor available in the fusion");

  for (auto& data : data_container)
  {
    if (data.second)
      predictor_->predict(*data.second, requested_prediction_time);
  }
}

double calculateCost(cost_calculation::CostCalculation const& cost_calculation, CostCell::Row const& row,
                     CostCell::Col const& col)
{
  // Either the objects originate from the same sensor modalities or where fused with the same sensor modalities
  if (row.hasCommonSensorTrace(col))
    return std::numeric_limits<double>::infinity();

  return cost_calculation.cost()(row.object(), col.object());
}

void associate(FusionInterface::NonCopyableObjectsContainer const& data, PerceptionKitObjects::_objects_type& objects,
               ObjectFusionExecuter const& fusion, cost_calculation::CostCalculation const& cost_calculation)
{
  auto const calculate_cost_function =
      std::bind(calculateCost, cost_calculation, std::placeholders::_1, std::placeholders::_2);

  // Append trace to all objects and put them into a single list
  ObjectWithTrace::Container objects_with_trace_storage;
  std::set<uint32_t> elements;
  uint32_t internal_fusion_id = 0;
  internal::initializeFirstColumnOfCostMatrix(data, objects_with_trace_storage, elements, internal_fusion_id);

  // Create a LMatrix of the objects sorted by the cost (not really a LMatrix though).
  CostCell::LMatrix cost_matrix;
  internal::createCostMatrix(objects_with_trace_storage, cost_matrix, calculate_cost_function);

  // fuse the cells of the matrix as long as everything is over the threshold
  internal::fuseCostMatrixElements(cost_matrix, objects_with_trace_storage, elements, internal_fusion_id,
                                   calculate_cost_function, cost_calculation.threshold(), fusion);

#if 0
  // print
  for (auto const& cost_matrix_cell : cost_matrix)
  {
    ROS_DEBUG_STREAM("FINAL: " << cost_matrix_cell);
  }
    ROS_DEBUG_STREAM("RETURN: " << objects.size() << " ELEMENTS: " << elements.size());
#endif

  internal::copyFusionResultToObjects(objects_with_trace_storage, elements, objects);
}

PerceptionKitObjects FusionInterface::fuse(RosTime const& fusion_time)
{
  auto preprocessed_data = FusionPreprocessorInterface::getData();

#if 0
  for (auto const data : preprocessed_data)
  {
    if (!data.second)
      continue;

    ROS_DEBUG_STREAM("After preprocssing: [" << data.first << "]: " << data.second->objects.size());
  }
#endif

  predict(preprocessed_data, fusion_time);

  PerceptionKitObjects ret;
  ret.header.stamp = fusion_time;

  associate(preprocessed_data, ret.objects, *fusion_, *cost_function_);

  FusionPostprocessorInterface::postprocess(ret);

  return ret;
}

}  // namespace track_to_track_fusion
