#include "track_to_track_fusion/cost_cell.h"
#include <ostream>

namespace track_to_track_fusion
{
std::ostream& operator<<(std::ostream& os, const CostCell& cost_cell)
{
  os << "ROW: " << cost_cell.row_ << " ";
  os << "COL: " << cost_cell.col_ << " ";
  os << "COST: " << cost_cell.cost_;

  return os;
}

CostCell::CostCell(Row const& row, Col const& col, CostFunction cost_function)
  : row_(row), col_(col), cost_(cost_function(row, col))
{
}

bool CostCell::operator<(CostCell const& other) const
{
  return cost_ < other.cost_;
}

bool CostCell::isDiagonalElement() const
{
  return row_.isSameObject(col_);
}

double CostCell::cost() const
{
  return cost_;
}

ObjectWithTrace::Trace CostCell::fused_traits() const
{
  ObjectWithTrace::Trace a;
  for (auto const& c : col_.trace())
  {
    a.insert(c);
  }
  for (auto const& c : row_.trace())
  {
    a.insert(c);
  }

  if (col_.trace().size() + row_.trace().size() != a.size())
  {
    throw std::runtime_error("Fusing traits of same sensor modality");
  }

  return a;
}

bool CostCell::contains(CostCell const& cell) const
{
  return cell.row_.isSameObject(row_) || cell.row_.isSameObject(col_) || cell.col_.isSameObject(row_) ||
         cell.col_.isSameObject(col_);
}

bool CostCell::contains(ObjectWithTrace const& object) const
{
  return object.isSameObject(row_) || object.isSameObject(col_);
}

CostCell::Row const& CostCell::row_object() const
{
  return row_;
}
CostCell::Col const& CostCell::col_object() const
{
  return col_;
}

}  // namespace track_to_track_fusion