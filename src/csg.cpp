
#include <algorithm>

#include "csg.hpp"
#include "tuple.hpp"


namespace geo {

  CSG::CSG(const SetOperation set_op, const std::shared_ptr<geo::Shape> le, const std::shared_ptr<geo::Shape> ri)
    : operation {set_op}, left {le}, right {ri} {}

  CSG::~CSG() {}

  Intersections CSG::local_intersects(const ray::Ray& local_ray) {

    auto ixs = left->intersects(local_ray);
    auto ixs2 = right->intersects(local_ray);

    // sort intersections by t
    ixs.insert(ixs.end(), ixs2.begin(), ixs2.end());
    std::sort(ixs.begin(), ixs.end(), [&](const Intersection& ix1, const Intersection& ix2){return ix1.t < ix2.t;});

    return filter_intersections(ixs);
  }

  math::Tuple CSG::local_normal_at(const math::Tuple& local_point, const Intersection& ix) const {

    return math::Tuple(0, 0, 0, 0);
  }

  bool CSG::local_equality_predicate(const Shape* shape) const {

    auto other_csg = dynamic_cast<const CSG*>(shape);

    return (operation == other_csg->operation && *left == *(other_csg->left) && *right == *(other_csg->right));
  }

  BoundingBox CSG::get_bounds() const {

    return BoundingBox();
  }

  bool CSG::includes(const Shape* shape) const {
    
    if (*left == *shape || *right == *shape)
	return true;
    // If left or right is a Group
    if (auto group = dynamic_cast<const Group*>(left.get()); group != nullptr)
      if (group->includes(shape))
	return true;
    if (auto group = dynamic_cast<const Group*>(right.get()); group != nullptr)
      if (group->includes(shape))
	return true;
    // Same if left or right is a CSG
    if (auto csg = dynamic_cast<const CSG*>(left.get()); csg != nullptr)
      if (csg->includes(shape))
	return true;
    if (auto csg = dynamic_cast<const CSG*>(right.get()); csg != nullptr)
      if (csg->includes(shape))
	return true;
    
    return false;
  }

  Intersections CSG::filter_intersections(const Intersections& ixs) {

    Intersections result;

    // We begin outside both shapes
    bool in_left = false;
    bool in_right = false;
    
    for (const auto& xs : ixs) {
      bool left_hit = false;
      // If left is a Group
      if (auto group = dynamic_cast<const Group*>(left.get()); group != nullptr) {
    	// If the Group includes the intersected geometry
    	if (group->includes(xs.geometry.get()))
    	  left_hit = true;
      // If left is a CSG Shape, check the children
      } else if (auto csg = dynamic_cast<const CSG*>(left.get()); csg != nullptr) {
	if (csg->includes(xs.geometry.get()))
	  left_hit = true;
      } else
	if (*left == *(xs.geometry))
	  left_hit = true;
	
      if (intersection_allowed(operation, left_hit, in_left, in_right))
	result.push_back(xs);

      // Keep track of each shape currently inside
      if (left_hit)
	in_left = ! in_left;
      else
	in_right = ! in_right;
    }

    return result;
  }

  std::shared_ptr<geo::CSG> operator|(const std::shared_ptr<geo::Shape> first, const std::shared_ptr<geo::Shape> second) {

    auto csg = std::make_shared<geo::CSG>(SetOperation::Union, first, second);
    
    first->parent = csg->get_weak_ptr();
    second->parent = csg->get_weak_ptr();
    
    return csg;
  }

  std::shared_ptr<geo::CSG> operator&(const std::shared_ptr<geo::Shape> first, const std::shared_ptr<geo::Shape> second) {

    auto csg = std::make_shared<geo::CSG>(SetOperation::Intersection, first, second);
    
    first->parent = csg->get_weak_ptr();
    second->parent = csg->get_weak_ptr();
    
    return csg;    
  }
  
  std::shared_ptr<geo::CSG> operator-(const std::shared_ptr<geo::Shape> first, const std::shared_ptr<geo::Shape> second) {
    
    auto csg = std::make_shared<geo::CSG>(SetOperation::Difference, first, second);
    
    first->parent = csg->get_weak_ptr();
    second->parent = csg->get_weak_ptr();
    
    return csg;        
  }

  bool intersection_allowed(const SetOperation operation, const bool left_hit, const bool in_left, const bool in_right) {

    if (operation == SetOperation::Union)
      // We want the Intersections that are not in two Shapes at the same time
      return (left_hit && !in_right) or (!left_hit && !in_left);
    else if (operation == SetOperation::Intersection) 
      // We want the Intersections that are where two Shapes overlap
      return (left_hit && in_right) or (!left_hit && in_left);
    else if (operation == SetOperation::Difference)
      // The Intersections on left not inside right, or right inside left
      return (left_hit && !in_right) or (!left_hit && in_left);
    
    throw std::runtime_error{"passed set operation is not permit"};
  }
}

