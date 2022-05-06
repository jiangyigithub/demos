#include "ros2/fusion_node.hpp"
#include <memory>
#include "track_to_track_fusion/weights.h"
#include "objects_processors/existence_threshold_filter.h"
#include "objects_processors/header_verification.h"

#include "track_to_track_fusion/feature/feature_fusion_elements.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
// #include <ros/ros.h>
// #include <tf/transform_listener.h>
#pragma GCC diagnostic pop

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace perception_kit_msgs;
using namespace std;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("track_to_track_fusion.fusion_node");

namespace track_to_track_fusion
{
namespace internal
{
bool parameterIsInvalid(double const& value)
{
  return !std::isfinite(value);
}

bool parameterIsInvalid(std::string const& value)
{
  return value.empty();
}

template <typename ParameterType>
void readMandatoryParameter(FusionNode& node, std::string const& parameter_name, ParameterType& output)
{
  if (!node.get_parameter(parameter_name, output) || parameterIsInvalid(output))
  {
    RCLCPP_FATAL_STREAM(LOGGER, "Parameter '" << parameter_name << "' not defined.");
    rclcpp::shutdown();
  }
  RCLCPP_INFO_STREAM(LOGGER, "Successfully read parameter '" << parameter_name << "' (value: " << output << ").");
}

void readFusionFrame(FusionNode& node, std::string& operation_frame)
{
  readMandatoryParameter(node, "operation_frame", operation_frame);
}

void readWeightsFrame(FusionNode& node, std::string& weights_frame, std::string const& operation_frame)
{
  if (node.get_parameter("weights_frame", weights_frame))
  {
    weights_frame = operation_frame;
  }
}

void readOutputConfiguration(FusionNode& node, FusionNode::OutputConfiguration& output_configuration)
{
  readMandatoryParameter(node, "output.topic_name", output_configuration.topic_name);
  readMandatoryParameter(node, "output.min_existence_probability", output_configuration.min_existence_probability);
}

void appendPolygonMarker(std::string const& sensor_modality, ObjectFusion::Feature const& feature,
                         Weights::boost_polygon const& polygon, float weight,
                         visualization_msgs::msg::MarkerArray& markers)
{
  (void)weight;

  if (!polygon.outer().size())
    return;

  visualization_msgs::msg::Marker marker_polygon;
  marker_polygon.id = markers.markers.size();
  marker_polygon.header.frame_id = "base_link";
  marker_polygon.type = visualization_msgs::msg::Marker::LINE_STRIP;
  for (auto const& p : polygon.outer())
  {
    geometry_msgs::msg::Point point;
    point.x = boost::geometry::get<0>(p);
    point.y = boost::geometry::get<1>(p);
    point.z = 0.0;
    marker_polygon.points.push_back(point);
    marker_polygon.ns = sensor_modality;
    marker_polygon.pose.orientation.w = 1.0;
    marker_polygon.color.a = 1.0;
    marker_polygon.color.r = 1.0;
    marker_polygon.color.g = 1.0;
    marker_polygon.color.b = 1.0;
    marker_polygon.scale.x = 0.05;
  }
  markers.markers.push_back(marker_polygon);

  visualization_msgs::msg::Marker marker_text;
  marker_text.header.frame_id = "base_link";
  marker_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker_text.text = feature;
  marker_text.ns = sensor_modality;
  marker_text.pose.position = marker_polygon.points.front();
  marker_text.pose.orientation.w = 1;
  marker_text.id = markers.markers.size();
  marker_text.color.a = 1.0;
  marker_text.color.r = 1.0;
  marker_text.color.g = 1.0;
  marker_text.color.b = 1.0;
  marker_text.scale.z = 0.2;
  markers.markers.push_back(marker_text);
}

void publishMarkersOnce(FusionNode& node, visualization_msgs::msg::MarkerArray const& markers)
{
  static rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr p =
      node.create_publisher<visualization_msgs::msg::MarkerArray>("weights_marker", 1);
  p->publish(markers);
}

void readWeights(FusionNode& node, Weights& weights, std::set<std::string> const& sensor_modalities)
{
  visualization_msgs::msg::MarkerArray markers;

  // for (auto const& sensor_modality : sensor_modalities)
  // {
  //   for (auto const& feature : ObjectFusion::AllFeatures())
  //   {
  //     std::string const key = std::string("weights/") + sensor_modality + std::string("/") + feature;

  //     XmlRpc::XmlRpcValue rpc_value;
  //     if (node.get_parameter(key, rpc_value))
  //     {
  //       RCLCPP_INFO_STREAM(LOGGER, "Got a rpc_value parameter: ");
  //       size_t polygon_id = 0;
  //       while (polygon_id != std::numeric_limits<size_t>::max())
  //       {
  //         double double_value;
  //         std::string const polygon_key_default = key + "/default";
  //         if (node.get_parameter(polygon_key_default, double_value))
  //         {
  //           RCLCPP_INFO_STREAM(LOGGER, "Set weight: " << sensor_modality << ":" << feature << ": " << double_value);
  //           weights.setWeight(sensor_modality, feature, double_value);
  //         }

  //         bool publish_marker = false;
  //         std::string const polygon_key_marker = key + "/publish_marker";
  //         node.get_parameter(polygon_key_marker, publish_marker);

  //         std::string const polygon_key = key + "/polygon_" + std::to_string(polygon_id);
  //         if (node.get_parameter(polygon_key, rpc_value))
  //         {
  //           double polygon_weight;
  //           std::string polygon_wkt;
  //           if (node.get_parameter(polygon_key + "/value", polygon_weight) &&
  //               node.get_parameter(polygon_key + "/wkt", polygon_wkt))
  //           {
  //             Weights::boost_polygon polygon;
  //             boost::geometry::read_wkt(polygon_wkt, polygon);

  //             RCLCPP_INFO_STREAM(LOGGER, "SetPolygonWeight: " << sensor_modality << ":" << feature << " Weight: " <<
  //             polygon_weight
  //                                                  << " Wkt: " << polygon_wkt);
  //             weights.setPolygonWeight(sensor_modality, feature, polygon, polygon_weight);
  //             if (publish_marker)
  //             {
  //               appendPolygonMarker(sensor_modality, feature, polygon, polygon_weight, markers);
  //             }

  //             ++polygon_id;
  //           }
  //           else
  //           {
  //             RCLCPP_ERROR_STREAM(LOGGER, "Could not read polygon " << polygon_id);
  //             throw std::invalid_argument("Could not read polygon. Check your config file.");
  //           }
  //         }
  //         else
  //         {
  //           polygon_id = std::numeric_limits<size_t>::max();
  //         }
  //       }
  //     }

  //     double double_value;
  //     if (node.get_parameter(key, double_value))
  //     {
  //       RCLCPP_INFO_STREAM(LOGGER, "Set weight: " << sensor_modality << ":" << feature << ": " << double_value);
  //       weights.setWeight(sensor_modality, feature, double_value);
  //     }

  //     std::string string_value;
  //     if (node.get_parameter(key, string_value))
  //     {
  //       // weights.setWeight(sensor_modality,feature,string_value);
  //       RCLCPP_FATAL_STREAM(LOGGER, "Got a string parameter: " << string_value << "(" << key << ")");
  //       throw std::runtime_error("Got a string value. Which is not implemented yet (but will be soon)!");
  //     }
  //   }
  // }

  // if (markers.markers.size())
  // {
  //   publishMarkersOnce(node, markers);
  // }
}

void readInputConfiguration(FusionNode& node, std::vector<FusionNode::Input>& inputs,
                            std::map<std::string const, float const>& existence_thresholds,
                            std::set<std::string>& sensor_modalities)
{
  int key = 0;
  std::map<int, FusionNode::Input> input_map;
  try
  {
    std::vector<std::string> sensor_names;
    node.get_parameter("inputs.sensor_names", sensor_names);

    for (auto& sensor_name : sensor_names)
    {
      FusionNode::Input input;
      input.name = sensor_name;
      sensor_modalities.insert(input.name);

      float min_existence_probability;

      if (!node.get_parameter("inputs." + sensor_name + ".topic_name", input.topic_name) ||
          !node.get_parameter("inputs." + sensor_name + ".min_existence_probability", min_existence_probability))
      {
        RCLCPP_FATAL_STREAM(node.get_logger(), "Could not read sensor input: " << input.name);
        RCLCPP_FATAL_STREAM(node.get_logger(), "either 'topic_name' or 'min_existence_probability' missing on config "
                                               "file entry 'inputs'.");
        rclcpp::shutdown();
      }

      if (min_existence_probability < 0.0 || min_existence_probability > 1.0)
      {
        RCLCPP_FATAL_STREAM(node.get_logger(), "Got existence probability of " << min_existence_probability
                                                                               << " which is out of range "
                                                                                  "[0:1].");
        rclcpp::shutdown();
      }
      existence_thresholds.emplace(input.name, min_existence_probability);

      // int const key = item.second.hasMember("order") ? int(item.second["order"]) : default_key--;
      input_map[key++] = input;

      RCLCPP_INFO_STREAM(node.get_logger(), "Successfully read parameters for sensor '"
                                                << sensor_name << "':\n"
                                                << "    topic name: '" << input.topic_name << "'\n"
                                                << "    minimal ex. prob.: '" << min_existence_probability << "'");
    }
  }
  catch (...)
  {
    RCLCPP_FATAL(node.get_logger(), "Could not read sensor inputs. Most likely, your config file is not valid. Check "
                                    "the example config file "
                                    "in the fusion package for reference.");
    rclcpp::shutdown();
  }

  for (auto const& pair : input_map)
  {
    inputs.push_back(pair.second);
    RCLCPP_DEBUG_STREAM(node.get_logger(), "Input " << pair.second.name << " added (order: " << pair.first << ")");
  }

  if (inputs.size() == 0)
  {
    RCLCPP_FATAL(node.get_logger(), "No inputs specified. Most likely, your config file is not valid. Check the "
                                    "example config file in the "
                                    "fusion package for reference.");
    rclcpp::shutdown();
  }
}

Weights::ConstPtr setupWeights(FusionNode& node, std::set<std::string> const& sensor_modalities)
{
  auto weights = std::make_unique<Weights>();
  internal::readWeights(node, *weights, sensor_modalities);
  return weights;
}

}  // namespace internal

FusionNode::~FusionNode()
{
  // call destructors of the subscribers prior to the destructors of the inputs to avoid having dangling pointers in the
  // callback binds
  subscribers_.clear();
}

PerceptionKitObject::_position_type
FusionNode::toWeightsFrame(PerceptionKitObject::_position_type const& position_in_operation_frame) const
{
  // if (operation_frame_ == weights_frame_)
  return position_in_operation_frame;

  // geometry_msgs::msg::Point position_in_base_link;
  // position_in_base_link.x = std::numeric_limits<double>::infinity();
  // position_in_base_link.y = std::numeric_limits<double>::infinity();
  // position_in_base_link.z = std::numeric_limits<double>::infinity();
  // tf::StampedTransform trafo;
  // try
  // {
  //   transform_listener_.lookupTransform(weights_frame_, operation_frame_, rclcpp::Time(0), trafo);

  //   auto const transformed = trafo * tf::Vector3(position_in_operation_frame.x, position_in_operation_frame.y,
  //                                                position_in_operation_frame.z);
  //   position_in_base_link.x = transformed.getX();
  //   position_in_base_link.y = transformed.getY();
  //   position_in_base_link.z = transformed.getZ();
  // }
  // catch (tf::LookupException const& error)
  // {
  //   RCLCPP_WARN_STREAM(LOGGER, "Transformation error: " << error.what());
  // }
  // return position_in_base_link;
}

void FusionNode::advertiseOutputs()
{
  // advertise outputs
  fusion_publisher_ = this->create_publisher<PerceptionKitObjects>(output_configuration_.topic_name, 10);
  RCLCPP_INFO_STREAM(this->get_logger(), "Publishing fused objects on " << output_configuration_.topic_name);
}

void FusionNode::subscribeToInputs()
{
  for (auto const& input : inputs_)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "connecting to " << input.name << " on topic " << input.topic_name);

    std::function<void(const PerceptionKitObjects::SharedPtr)> bound_callback_func =
        std::bind(&track_to_track_fusion::FusionNode::onObjectsCallback, this, std::placeholders::_1, input);

    subscribers_[input.name] = this->create_subscription<PerceptionKitObjects>(
        input.topic_name, rclcpp::SystemDefaultsQoS(), bound_callback_func);
  }
}

FusionNode::FusionNode(const std::string& node_name, const rclcpp::NodeOptions& options) : Node(node_name, options)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Setting up node: " << node_name);

  declareParameters();

  internal::readFusionFrame(*this, operation_frame_);

  internal::readOutputConfiguration(*this, output_configuration_);

  std::set<std::string> sensor_modalities;
  std::map<std::string const, float const> existence_thresholds;
  internal::readInputConfiguration(*this, inputs_, existence_thresholds, sensor_modalities);

  auto weights = internal::setupWeights(*this, sensor_modalities);
  internal::readWeightsFrame(*this, weights_frame_, operation_frame_);

  std::map<ObjectFusion::Feature, ObjectFusion::ConstPtr> fusers;
  fusers["dimension"] = std::make_unique<DimensionFusionWeightedNormalDistribution>();
  fusers["position"] = std::make_unique<PoseFusionWeightedNormalDistribution>();
  fusers["orientation"] = std::make_unique<OrientationFusionWeightedNormalDistribution>();
  fusers["classification"] = std::make_unique<UnionClassificationFusion>();
  fusers["existence_probability"] = std::make_unique<ExistenceProbabilityWeightedFusion>();
  fusers["dynamics"] = std::make_unique<DynamicsFusionWeightedNormalDistribution>();

  auto fusion = std::make_unique<ObjectFusionExecuter>(
      std::move(fusers), std::move(weights),
      [this](PerceptionKitObject::_position_type const& object_position_in_operation_frame) {
        return this->toWeightsFrame(object_position_in_operation_frame);
      });

  // @todo: put into config files
  auto cost_function = std::make_unique<cost_calculation::CostCalculation>(cost_calculation::euclideanDistance, 10.0);

  fusion_interface_.init(sensor_modalities, std::move(fusion), std::move(cost_function));

  fusion_interface_.register_preprocessor(
      std::make_unique<objects_processors::HeaderVerification<NonCopyableObjects>>(operation_frame_));

  fusion_interface_.register_preprocessor(
      std::make_unique<objects_processors::HeaderVerification<PerceptionKitObject>>(operation_frame_));

  fusion_interface_.register_preprocessor(
      std::make_unique<objects_processors::ExistenceThresoldFilter>(existence_thresholds));

  fusion_interface_.register_postprocessor(
      std::make_unique<objects_processors::ExistenceThresoldFilter>(output_configuration_.min_existence_probability));

  fusion_interface_.setPredictor(std::make_unique<objects_prediction::ConstantVelocityPredictor>());

  advertiseOutputs();

  subscribeToInputs();

  timer_ = this->create_wall_timer(66.667ms, std::bind(&FusionNode::publishObjects, this));
}

void FusionNode::declareParameters()
{
  /* Declare sensor_names parameter first */
  this->declare_parameter<std::vector<std::string>>("inputs.sensor_names", std::vector<std::string>{});
  std::vector<std::string> sensor_names;
  this->get_parameter("inputs.sensor_names", sensor_names);

  RCLCPP_INFO_STREAM(this->get_logger(), "Declaring parameters ...");

  /* Declare input parameters based on list of sensor names */
  for (const std::string& sensor_name : sensor_names)
  {
    this->declare_parameter<std::string>("inputs." + sensor_name + ".topic_name", "");
    this->declare_parameter<double>("inputs." + sensor_name + ".min_existence_probability", 1.0);
  }

  /* Declare output parameters */
  this->declare_parameter<std::string>("output.topic_name", "");
  this->declare_parameter<double>("output.min_existence_probability", 1.0);

  /* Declare operation frame */
  this->declare_parameter<std::string>("operation_frame", "");

  RCLCPP_INFO_STREAM(this->get_logger(), "Declared parameters.");
}

void FusionNode::publishObjects()
{
  auto fused_objects = fusion_interface_.fuse(this->now());
  fused_objects.header.frame_id = operation_frame_;

  fusion_publisher_->publish(fused_objects);
}

void FusionNode::onObjectsCallback(const PerceptionKitObjects::ConstSharedPtr& objects, const Input& input)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Received data from " << input.name
                                                               << " Num Objects: " << objects->objects.size());
  fusion_interface_.addInputData(objects, input.name);
}

}  // namespace track_to_track_fusion
