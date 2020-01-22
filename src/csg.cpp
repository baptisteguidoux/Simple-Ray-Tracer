
#include "csg.hpp"
#include "tuple.hpp"


namespace geo {

  CSG::CSG(const std::string& set_op, const std::shared_ptr<geo::Shape> le, const std::shared_ptr<geo::Shape> ri)
    : operation {set_op}, left {le}, right {ri} {}

  CSG::~CSG() {}

  Intersections CSG::local_intersects(const ray::Ray& local_ray) {

    return Intersections{};
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
  
  std::shared_ptr<geo::CSG> operator|(const std::shared_ptr<geo::Shape> first, const std::shared_ptr<geo::Shape> second) {

    auto csg = std::make_shared<geo::CSG>("union", first, second);
    
    first->parent = csg->get_weak_ptr();
    second->parent = csg->get_weak_ptr();
    
    return csg;
  }
}

