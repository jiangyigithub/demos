#pragma once

#include "track_to_track_fusion/object_with_trace.h"
#include <set>
#include <functional>

namespace track_to_track_fusion
{
class CostCell
{
public:
  using LMatrix = std::multiset<CostCell>;

  using Row = ObjectWithTrace;
  using Col = ObjectWithTrace;
  using CostFunction = std::function<double(Row const&, Col const&)>;

  CostCell(Row const& row, Col const& col, CostFunction cost_function);
  bool operator<(CostCell const& other) const;

  bool isDiagonalElement() const;

  double cost() const;

  ObjectWithTrace::Trace fused_traits() const;
  bool contains(CostCell const& cell) const;

  bool contains(ObjectWithTrace const& object) const;
  Row const& row_object() const;
  Col const& col_object() const;

private:
  Row const& row_;
  Col const& col_;
  double const cost_;

  CostCell() = delete;
  CostCell(CostCell const&) = delete;
  CostCell operator=(CostCell const&) = delete;

  friend std::ostream& operator<<(std::ostream& os, const CostCell& cost_cell);
};
}  // namespace track_to_track_fusion