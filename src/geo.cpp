#include <optional>
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>
#include <optional>
#include <memory>

#include "geo.hpp"
#include "ray.hpp"
#include "matrix.hpp"
#include "tuple.hpp"


namespace geo {

  // Necessary to provide a definition for virtual destructors
  Shape::~Shape() {};

  Intersections Shape::intersects(const ray::Ray& ray) {

    // The ray passed is transformed by the inverse of the sphere transform matrix
    // Every sphere stays at 0, 0, 0, the sphere transform matrices are inversely applied to the ray
    // (because translating the sphere away from the ray == translating the ray away from the sphere)
    auto local_ray = ray.transform(math::inverse(transform));

    return local_intersects(local_ray);
  }

  math::Tuple Shape::normal_at(const math::Tuple& world_point, const Intersection& ix) {

    // The vector from the sphere's origin to the point on the sphere is the normal at the point where it intersects (for a sphere centered at origin)

    // From world space to object space
    auto local_point = world_to_object(world_point);
    // The object's normal vector we get is in object space
    auto local_normal = local_normal_at(local_point, ix);

    return normal_to_world(local_normal);
  }

  color::Color Shape::pattern_at(const math::Tuple& world_point) const {

    auto object_point = world_to_object(world_point);

    return material.pattern->pattern_at(object_point);
  }

  math::Tuple Shape::world_to_object(const math::Tuple& world_point) const {

    // It the Shape has a parent, first convert the point to its parent space
    if (auto parentptr = parent.lock()) {
      //if (parent != nullptr) {
      auto parent_point = parentptr->world_to_object(world_point);
      return math::inverse(transform) * parent_point;
    }
  
    return math::inverse(transform) * world_point;
  }

  math::Tuple Shape::normal_to_world(const math::Tuple& object_normal) const {

     // When the sphere has been transformed, to keep the normals perpendicular to the surface, we must multiply the normal by the inverse transpose matrix
    auto normal = math::transpose(math::inverse(transform)) * object_normal;
    normal.w = 0;
    normal = math::normalize(normal);

    //if (parent != nullptr)
    if (auto parentptr = parent.lock())
      normal = parentptr->normal_to_world(normal);

    return normal;
  }

  std::shared_ptr<Shape> Shape::get_shared_ptr() {

    return shared_from_this();
  }

  std::weak_ptr<Shape> Shape::get_weak_ptr() {

    return weak_from_this();
  }

  void Shape::divide(const size_t threshold) {
    // Nothing happens for primitive Shapes. Group overrides this function
  }


  BoundingBox Shape::get_parent_space_bounds() const {

    return get_bounds().transform(transform);
  }

  bool operator==(const Shape& first, const Shape& second) {

    if (typeid(first) == typeid(second) && // same class?
	first.transform == second.transform &&
	first.material == second.material)
      return first.local_equality_predicate(&second);

    return false;
  }

  bool operator!=(const Shape& first, const Shape& second) {

    return ! (first == second);
  }

  TestShape::~TestShape() {};

  Intersections TestShape::local_intersects(const ray::Ray& local_ray) {

    saved_ray = local_ray;

    return Intersections();
  }

  math::Tuple TestShape::local_normal_at(const math::Tuple& local_point, const Intersection& ix) const {

    return math::Vector(local_point.x, local_point.y, local_point.z);
  }

  bool TestShape::local_equality_predicate(const Shape* shape) const {
    // If the transforms and material are equal, the TestShapes are equal
    return true;
  }

  BoundingBox TestShape::get_bounds() const {

    return BoundingBox(math::Point(-1, -1, -1),
		       math::Point(1, 1, 1));
  }
  
  Sphere::~Sphere() {};
  
  Intersections Sphere::local_intersects(const ray::Ray& local_ray) {
      
    // When `ray` intersects with this Sphere, returns a vector with the two intersections distances: between the ray origin and the intersection points
	  
    // First, we find the discriminant, that will tell us whether the ray intersects the sphere
    // Good read: https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection
    // Vector from the sphere center to the ray origin
    auto sphere_to_ray = local_ray.origin - math::Point(0, 0, 0); // The sphere is by default at the center of the world and has a radius = 1

    auto a = math::dot(local_ray.direction, local_ray.direction);
    auto b = 2 * math::dot(local_ray.direction, sphere_to_ray);
    auto c = math::dot(sphere_to_ray, sphere_to_ray) - 1;

    auto discriminant = (b * b) - (4 * a * c);

    // If the discriminant is negative, the ray misses
    if (discriminant < 0)
      return Intersections{};

    // Compute the intersections. If t1 == t2, it means the intersection is a perfect tangent
    auto t1 = (-b - sqrt(discriminant)) / (2 * a); // t stands for time and is the convention to designate a distance along a ray
    auto t2 = (-b + sqrt(discriminant)) / (2 * a); // (if you think of the ray direction vector as its speed)

    // Return the intersections in increasing order
    auto result = (t1 < t2) ?
      Intersections{Intersection(t1, this), Intersection(t2, this)}
    : Intersections{Intersection(t2, this), Intersection(t1, this)};

    return result;      
  }

  math::Tuple Sphere::local_normal_at(const math::Tuple& object_point, const Intersection& ix) const {

      return object_point - math::Point(0.0, 0.0, 0.0);
  }

  bool Sphere::local_equality_predicate(const Shape* shape) const {
    // If the transforms and material are equal, the Spheres are equal
    return true;
  }

  BoundingBox Sphere::get_bounds() const {

    return BoundingBox(math::Point(-1, -1, -1),
		       math::Point(1, 1, 1));
  }

  std::shared_ptr<Sphere> build_glass_sphere() {

    auto sphere = std::make_shared<geo::Sphere>();
    sphere->material.transparency = 1.0;
    sphere->material.refractive_index = 1.5;

    return sphere;
  }

  Plane::~Plane() {};

  Intersections Plane::local_intersects(const ray::Ray& local_ray) {

    // If a ray is parallel to a Plane, it won't intersect
    // We would consider the same if the ray is coplanar to the Plane, even if it actually intersects infinitely with the Plane
    // To determine if a ray is parallel, we need to find if it's purely in the space xz, no slope in y
    if (std::abs(local_ray.direction.y) < math::EPSILON)
      return Intersections();

    auto t = - local_ray.origin.y / local_ray.direction.y; // Always assuming the Plane is in xz space only
    
    return Intersections {Intersection(t, this)};
  }

  math::Tuple Plane::local_normal_at(const math::Tuple& local_point, const Intersection& ix) const {

    static const math::Tuple default_normal = math::Vector(0, 1, 0);

    return default_normal;
  }

  bool Plane::local_equality_predicate(const Shape* shape) const {
    // If the transforms and material are equal, the Plane are equal
    return true;
  }

  BoundingBox Plane::get_bounds() const {

    return BoundingBox(math::Point(-INFINITY, 0, -INFINITY),
		       math::Point(INFINITY, 0, INFINITY));
  }

  Cube::~Cube() {};

  Intersections Cube::local_intersects(const ray::Ray& local_ray) {

    auto [xtmin, xtmax] = check_axis(local_ray.origin.x, local_ray.direction.x);
    auto [ytmin, ytmax] = check_axis(local_ray.origin.y, local_ray.direction.y);
    auto [ztmin, ztmax] = check_axis(local_ray.origin.z, local_ray.direction.z);

    // find the value closest to the cube
    //https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection
    auto tmin = std::max(xtmin, std::max(ytmin, ztmin));
    auto tmax = std::min(xtmax, std::min(ytmax, ztmax));

    // When the ray misses the cube
    if (tmin > tmax)
      return Intersections {};
    
    Intersections ixs {Intersection(tmin, this)};
    ixs.push_back(Intersection(tmax, this));
		  
    return ixs;
  };
  
  math::Tuple Cube::local_normal_at(const math::Tuple& local_point, const Intersection& ix) const {

    // Find the component with the highest absolute value,
    // then return a vector pointing in that direction

    // Implementation could be optimized
    auto max_component = std::max(std::abs(local_point.x),
  				  std::max(std::abs(local_point.y), std::abs(local_point.z)));

    if (max_component == std::abs(local_point.x))
      return math::Vector(local_point.x, 0, 0);
    else if (max_component == std::abs(local_point.y))
      return math::Vector(0, local_point.y, 0);
    else
      return math::Vector(0, 0, local_point.z);
  };

  bool Cube::local_equality_predicate(const Shape* shape) const {
    // If the transforms and material are equal, the Cubes are equal
    return true;
  }

  BoundingBox Cube::get_bounds() const {

    return BoundingBox(math::Point(-1, -1, -1),
		       math::Point(1, 1, 1));
  }

  Cylinder::~Cylinder() {};
  
  Intersections Cylinder::local_intersects(const ray::Ray& local_ray) {

    auto a = pow(local_ray.direction.x, 2) + pow(local_ray.direction.z, 2);

    // ray is parallel to the y axis, just check if it intersects with the Cylinder'caps
    if (math::almost_equal(a, 0))
      return intersects_caps(local_ray, Intersections{});

    auto  b = 2 * local_ray.origin.x * local_ray.direction.x +
              2 * local_ray.origin.z * local_ray.direction.z;

    auto c = pow(local_ray.origin.x, 2) + pow(local_ray.origin.z, 2) - 1;

    // discriminant
    auto disc = pow(b, 2) - 4 * a * c;
    if (disc < 0)
      return Intersections{};

    auto t0 = (-b - sqrt(disc)) / (2 * a);
    auto t1 = (-b + sqrt(disc)) / (2 * a);
    if (t0 > t1)
      std::swap(t0, t1);

    Intersections xs;

    // Check if the ray is above the minimum
    auto y0 = local_ray.origin.y + t0 * local_ray.direction.y;
    if (minimum < y0 && y0 < maximum)
      xs.push_back(Intersection(t0, this));

    // Check if the ray is below the maximum
    auto y1 = local_ray.origin.y + t1 * local_ray.direction.y;
    if (minimum < y1 && y1 < maximum)
      xs.push_back(Intersection(t1, this));
    
    return intersects_caps(local_ray, xs);
  }

  Intersections Cylinder::intersects_caps(const ray::Ray& local_ray, Intersections ixs) {

    // If the  Cylinder is not closed or the ray has no possibility to intersects the Cylinder
    if (! closed || math::almost_equal(std::abs(local_ray.direction.y), 0))
      return ixs;

    // Check for an intersection with the lower cap
    double t = (minimum - local_ray.origin.y) / local_ray.direction.y;
    if (check_cap(local_ray, t))
      ixs.push_back(Intersection(t, this));

    // Check for an intersection with the upper cap
    t = (maximum - local_ray.origin.y) / local_ray.direction.y;
    if (check_cap(local_ray, t))
      ixs.push_back(Intersection(t, this));

    return ixs;    
  }

  math::Tuple Cylinder::local_normal_at(const math::Tuple& local_point, const Intersection& ix) const {
    
    // Compute the square of the distance from the y axis
    auto dist_sq = pow(local_point.x, 2) + pow(local_point.z, 2);

    if (dist_sq < 1 && local_point.y >= maximum - math::EPSILON)
      return math::Vector(0, 1, 0);

    if (dist_sq < 1 && local_point.y <= minimum + math::EPSILON)
      return math::Vector(0, -1, 0);

    return math::Vector(local_point.x, 0, local_point.z);
  }

  bool Cylinder::local_equality_predicate(const Shape* shape) const {

    auto cyl = dynamic_cast<const Cylinder*>(shape);

    return (math::almost_equal(minimum, cyl->minimum) &&
	    math::almost_equal(maximum, cyl->maximum) &&
	    closed == cyl->closed);
  }

  BoundingBox Cylinder::get_bounds() const {

    return BoundingBox(math::Point(-1, minimum, -1),
		       math::Point(1, maximum, 1));
  }
  
  DoubleCone::~DoubleCone() {};

  Intersections DoubleCone::local_intersects(const ray::Ray& local_ray) {

    // same algorithm as Cylinder, except for the calculation of a, b and c

    auto a = pow(local_ray.direction.x, 2) - pow(local_ray.direction.y, 2) + pow(local_ray.direction.z, 2);

    auto b = 2 * (local_ray.origin.x * local_ray.direction.x) - 2 * (local_ray.origin.y * local_ray.direction.y) + 2 * (local_ray.origin.z * local_ray.direction.z);
    
    auto c = pow(local_ray.origin.x, 2) - pow(local_ray.origin.y, 2) + pow(local_ray.origin.z, 2);

    // ray is parallel to one of the double cones' halves
    if (math::almost_equal(a, 0)) {
      // this means the ray may still intersects with the other halve
      if (math::almost_equal(b, 0))
	return intersects_caps(local_ray, Intersections{});
      else // if b is not zero, there is one point of intersection
	return intersects_caps(local_ray, Intersections{Intersection((-c / (2 * b)), this)});
    }
    
    // discriminant
    auto disc = pow(b, 2) - 4 * a * c;
    if (disc < 0)
      return Intersections{};

    auto t0 = (-b - sqrt(disc)) / (2 * a);
    auto t1 = (-b + sqrt(disc)) / (2 * a);
    if (t0 > t1)
      std::swap(t0, t1);

    Intersections xs;

    // Check if the ray is above the minimum
    auto y0 = local_ray.origin.y + t0 * local_ray.direction.y;
    if (minimum < y0 && y0 < maximum)
      xs.push_back(Intersection(t0, this));

    // Check if the ray is below the maximum
    auto y1 = local_ray.origin.y + t1 * local_ray.direction.y;
    if (minimum < y1 && y1 < maximum)
      xs.push_back(Intersection(t1, this));

    //return xs;
    return intersects_caps(local_ray, xs);    
  }

  Intersections DoubleCone::intersects_caps(const ray::Ray& local_ray, Intersections ixs) {

    // same algorithm as Cylinder
    
    // If the  DoubleCone is not closed or the ray has no possibility to intersects the DoubleCone
    if (! closed || math::almost_equal(std::abs(local_ray.direction.y), 0))
      return ixs;

    // Check for an intersection with the lower cap
    double t = (minimum - local_ray.origin.y) / local_ray.direction.y;
    if (check_cap(local_ray, t, std::abs(minimum)))
      ixs.push_back(Intersection(t, this));

    // Check for an intersection with the upper cap
    t = (maximum - local_ray.origin.y) / local_ray.direction.y;
    if (check_cap(local_ray, t, std::abs(maximum)))
      ixs.push_back(Intersection(t, this));

    return ixs;    
  }  

  math::Tuple DoubleCone::local_normal_at(const math::Tuple& local_point, const Intersection& ix) const {

    // Compute the square of the distance from the y axis
    auto dist_sq = pow(local_point.x, 2) + pow(local_point.z, 2);

    if (dist_sq < 1 && local_point.y >= maximum - math::EPSILON)
      return math::Vector(0, 1, 0);

    if (dist_sq < 1 && local_point.y <= minimum + math::EPSILON)
      return math::Vector(0, -1, 0);

    auto y = sqrt(pow(local_point.x, 2) + pow(local_point.z, 2));
    if (local_point.y > 0)
      y = -y;

    return math::Vector(local_point.x, y, local_point.z);    
  }
  
  bool DoubleCone::local_equality_predicate(const Shape* shape) const {

    auto cone = dynamic_cast<const DoubleCone*>(shape);
    
    return (math::almost_equal(minimum, cone->minimum) &&
	    math::almost_equal(maximum, cone->maximum) &&
	    closed == cone->closed);
  }

  BoundingBox DoubleCone::get_bounds() const {

    auto limit = std::max(std::abs(minimum), std::abs(maximum));

    return BoundingBox(math::Point(-limit, minimum, -limit),
		       math::Point(limit, maximum, limit));
  }
  
  math::Tuple reflect(const math::Tuple& vec, const math::Tuple& normal) {

    // The velocity vec is reflected around the normal
  
    return vec - normal * 2 * math::dot(vec, normal);
  }  

  std::pair<double, double> check_axis(const double origin, const double direction, const double min_val, const double max_val) {

    // each pair of planes is offset 1 unit in opposing direction
    double tmin_numerator = min_val - origin;
    double tmax_numerator = max_val - origin;

    double tmin, tmax;

    // to avoid dividing by zero
    if (std::abs(direction) >= math::EPSILON) {
      tmin = tmin_numerator / direction;
      tmax = tmax_numerator / direction;
    } else {
      tmin = tmin_numerator * INFINITY;
      tmax = tmax_numerator * INFINITY;
    }

    if (tmin > tmax)
      return std::make_pair(tmax, tmin);
    
    return std::make_pair(tmin, tmax);
  }

  bool check_cap(const ray::Ray& ray, const double t, const double radius) {
    double x = ray.origin.x + t * ray.direction.x;
    double z = ray.origin.z + t * ray.direction.z;

    return (pow(x, 2) + pow(z, 2)) <= radius; // The radius of the Cylinder is 1
  }

  Triangle::Triangle(const math::Tuple& p1_, const math::Tuple& p2_, const math::Tuple& p3_) {

    p1 = p1_;
    p2 = p2_;
    p3 = p3_;
    e1 = p2 - p1;
    e2 = p3 - p1;
    normal = math::normalize(math::cross(e2, e1));
  }

  Triangle::~Triangle() {}

  Intersections Triangle::local_intersects(const ray::Ray& local_ray) {

    // If the ray is parallel to the Triangleit misses
    auto dir_cross_e2 = math::cross(local_ray.direction, e2);
    auto determinant = math::dot(e1, dir_cross_e2);
    if (std::abs(determinant) < math::EPSILON)
      return Intersections {};

    // Check the p1-p3 edge
    auto p1_to_origin = local_ray.origin - p1;
    auto f = 1.0 / determinant;
    auto u = f * math::dot(p1_to_origin, dir_cross_e2);
    if (u < 0 || u > 1)
      return Intersections {};

    // Check the p1-p2 and p2-p3 edges
    auto origin_cross_e1 = math::cross(p1_to_origin, e1);
    auto v = f * math::dot(local_ray.direction, origin_cross_e1);
    if (v < 0 || (u + v) > 1)
      return Intersections{};

    auto t = f * math::dot(e2, origin_cross_e1);
    
    return Intersections {Intersection(t, this, u, v)};
  }

  math::Tuple Triangle::local_normal_at(const math::Tuple& local_point, const Intersection& ix) const {

    return normal;
  }

  bool Triangle::local_equality_predicate(const Shape* shape) const {

    auto other = dynamic_cast<const Triangle*>(shape);

    return (other->p1 == p1 && other->p2 == p2 && other->p3 == p3 &&
	    other->e1 == e1 && other->e2 == e2 && other->normal == normal);
    
    return false;
  }

  BoundingBox Triangle::get_bounds() const {

    BoundingBox bounds;

    bounds.include(p1);
    bounds.include(p2);
    bounds.include(p3);

    return bounds;
  }

  SmoothTriangle::SmoothTriangle(const math::Tuple& p1_, const math::Tuple& p2_,
				 const math::Tuple& p3_, const math::Tuple& n1_,
				 const math::Tuple& n2_, const math::Tuple& n3_)
    : Triangle(p1_, p2_ , p3_), n1 {n1_}, n2 {n2_}, n3 {n3_} {}

  SmoothTriangle::~SmoothTriangle() {}

  math::Tuple SmoothTriangle::local_normal_at(const math::Tuple& local_point, const Intersection& ix) const {

    return n2 * ix.u + n3 * ix.v + n1 * (1 - ix.u - ix.v);
  }

  bool SmoothTriangle::local_equality_predicate(const Shape* shape) const {

    auto other = dynamic_cast<const SmoothTriangle*>(shape);

    return (other->p1 == p1 && other->p2 == p2 && other->p3 == p3 &&
	    other->e1 == e1 && other->e2 == e2 && other->normal == normal &&
	    other->n1 == n1 && other->n2 == n2 && other->n3 == n3);
    
    return false;
  }
    
  Group::~Group() {}

  Intersections Group::local_intersects(const ray::Ray& local_ray) {

    auto group_intersections = Intersections{};

    if (get_bounds().intersects(local_ray)) {
      // Call intersects for each Shape of the Group
      for (const auto& shrd_shape_ptr : shapes)
	if (shrd_shape_ptr != nullptr) {
	  auto shape_intersections = shrd_shape_ptr->intersects(local_ray);
	  group_intersections.insert(group_intersections.end(), shape_intersections.begin(), shape_intersections.end());
	}    

      // sort intersections
      std::sort(group_intersections.begin(), group_intersections.end(),
		[&](const Intersection& inter1, const Intersection& inter2){return inter1.t < inter2.t;});      
    }
    
    return group_intersections;
  }

  math::Tuple Group::local_normal_at(const math::Tuple& local_point, const Intersection& ix) const {

    // In a Group, normals are computed by calling children Shapes local_normal_at
    throw std::runtime_error {"Group.local_normal_at function should not be called"};
  }

  void Group::add_child(std::shared_ptr<Shape> shape) {

    shapes.push_back(shape->get_shared_ptr());
    shape->parent = get_weak_ptr();
  }

  bool Group::local_equality_predicate(const Shape* shape) const {

    auto group = dynamic_cast<const Group*>(shape);

    // Check there are as much shapes in both Groups
    if (shapes.size() != group->shapes.size())
      return false;

    // Check that every shape in this Group can be found in the other Group
    for (const auto& sub_shape : group->shapes) {
      auto p = std::find_if(shapes.begin(), shapes.end(),
			    [&](const std::shared_ptr<geo::Shape> shp){
			      if (*shp == *sub_shape) {
				  return true;
			      };
			      return false;
			    });
      
      // If the sub shape is not found
      if (p == shapes.end())
	return false;
    }

    return true;
  }

  BoundingBox Group::get_bounds() const {

    BoundingBox box;

    for (const auto& child : shapes)
      box.add(child->get_parent_space_bounds());

    return box;
  }

  void Group::divide(const size_t threshold) {

    if (threshold <= shapes.size()) {
      auto [left, right] = partition_children();
      if (left.size() > 0)
	make_subgroup(left);
      if (right.size() > 0)
	make_subgroup(right);
    }

    for (const auto& child : shapes)
      child->divide(threshold);
  }

  std::pair<std::vector<std::shared_ptr<Shape>>, std::vector<std::shared_ptr<Shape>>> Group::partition_children() {

    auto [left_box, right_box] = get_bounds().split();
    // containers for the shapes that can be contain in the lef tor right BB
    std::vector<std::shared_ptr<Shape>> left_shapes;
    std::vector<std::shared_ptr<Shape>> right_shapes;
    // the shapes remaining in this group (they do not fit entirely in a sub BB)
    std::vector<std::shared_ptr<Shape>> group_shapes;
    
    // Check the Shapes that can be contain in either sub BB
    for (const auto& shape : shapes) {
      auto shape_bounds = shape->get_parent_space_bounds();
      if (left_box.contains(shape_bounds))
	  left_shapes.push_back(shape);
      else if (right_box.contains(shape_bounds))
	right_shapes.push_back(shape);
      else
	group_shapes.push_back(std::move(shape));
    }

    shapes = std::move(group_shapes);

    return std::make_pair(left_shapes, right_shapes);
  }

  void Group::make_subgroup(const std::vector<std::shared_ptr<Shape>> shape_vec) {

    auto subgroup = std::make_shared<Group>();
    for (const auto& shapeptr : shape_vec) {
      subgroup->add_child(shapeptr);
    }

    shapes.push_back(std::move(subgroup));
  }

  
  Intersection::Intersection(const float t_, geo::Shape* geo, const float u_, const float v_) :
    t {t_}, geometry {geo->get_shared_ptr()}, u {u_}, v {v_} {}

  Intersection& Intersection::operator=(const Intersection& source) {

    t = source.t;
    geometry = source.geometry;

    return *this;
  }

  std::optional<Intersection> hit(const Intersections& intersections) {
    //std::sort(intersections.begin(), intersections.end(), [&](const Intersection& first, const Intersection& second) {return first.t < second.t;});
    // The hit is the lowest nonegative intersection
  
    // First filter out the intersection with negative t values;
    Intersections nonneg_integers;
    std::copy_if(intersections.begin(), intersections.end(), std::back_inserter(nonneg_integers), [&](const Intersection& inter) {return inter.t >= 0;});
  
    if (nonneg_integers.size()) {
      auto res = std::min_element(nonneg_integers.begin(), nonneg_integers.end(),
				  [&](const Intersection& first, const Intersection& second) {return first.t < second.t;});
      return *res;
    }
  
    return {}; // ie std::nullopt
  }  

  bool operator==(const Intersection& first, const Intersection& second) {

    return (first.t == second.t) && (*first.geometry == *second.geometry);
  }

  bool operator!=(const Intersection& first, const Intersection& second) {

    return ! (first == second);
  }

  float Computations::schlick() const {

    // find cosine of the angle between the eye and normal vectors
    auto cos = math::dot(eye_vector, normal_vector);
    // total internal reflection can only occur if n1 > n2
    if (n1 > n2) {
	auto n_ratio = n1 / n2;
	auto sin2_t = pow(n_ratio, 2) * (1 - pow(cos, 2));
	if (sin2_t > 1)
	  return 1;

	// compute cosine of theta t
	auto cos_t = sqrt(1 - sin2_t);

	// when n1 > n2, use cos of theta t instead
	cos = cos_t;
      }

    // Schlick's approximation
    auto r0 = pow((n1 - n2) / (n1 + n2), 2);
    return r0 + (1 - r0) * pow((1 - cos), 5);
}
  
  Computations prepare_computations(const Intersection& ix, const ray::Ray r, const Intersections& ixs) {

    auto comps = Computations();

    // Copy the intersection's properties for convenience
    comps.t = ix.t;
    comps.geometry = ix.geometry;
    
    // Precompute useful values
    comps.point = r.position(comps.t);
    comps.eye_vector = -r.direction;
    comps.normal_vector = comps.geometry->normal_at(comps.point, ix);

    // Find if the normal points away from the eye vector, ie intersection occured inside object
    if (math::dot(comps.eye_vector, comps.normal_vector) < 0) {
      comps.inside = true;
      comps.normal_vector = -comps.normal_vector;
    } else
      comps.inside = false;

    // Reflect the ray around object's normal
    comps.reflect_vector = geo::reflect(r.direction, comps.normal_vector);
    
    //Bump the point in direction of the normal
    comps.over_point = comps.point + comps.normal_vector * math::EPSILON;
    comps.under_point = comps.point - comps.normal_vector * math::EPSILON;
    
    auto containers = std::vector<std::shared_ptr<geo::Shape>>(); // will contain objects encountered but not (yet) exited

    for (const auto& ix_ : ixs) {

      // If the intersection is the hit, ie the given intersection
      if (ix_ == ix) {
    	if (containers.size() == 0)
    	  comps.n1 = 1.0;
    	else
    	  comps.n1 = containers.at(containers.size() - 1)->material.refractive_index;
      }

      // If the intersection object already in containers
      const auto shpp = std::find_if(containers.begin(), containers.end(),
				     [&](const std::shared_ptr<geo::Shape> shp) {return *shp.get() == *ix_.geometry.get();});
      if (shpp != containers.end())
    	// Then intersection is leaving the object, erase it from `containers`
    	containers.erase(containers.begin() + std::distance(containers.begin(), shpp));
      else
    	// intersection is entering object
    	containers.push_back(ix_.geometry);

      // If the intersection is the hit
      if (ix_ == ix) {
    	if (containers.size() == 0)
    	  comps.n2 = 1.0;
    	else
    	  comps.n2 = containers.at(containers.size() - 1)->material.refractive_index;
    	// terminate the loop
    	break;
      }
    }
    
    return comps;
  }

  BoundingBox::BoundingBox(const math::Tuple& min, const math::Tuple& max) :
    minimum {min}, maximum {max} {}

  void BoundingBox::include(const math::Tuple& point) {

    point.x < minimum.x ? minimum.x = point.x : minimum.x;
    point.y < minimum.y ? minimum.y = point.y : minimum.y;
    point.z < minimum.z ? minimum.z = point.z : minimum.z;

    point.x > maximum.x ? maximum.x = point.x : maximum.x;
    point.y > maximum.y ? maximum.y = point.y : maximum.y;
    point.z > maximum.z ? maximum.z = point.z : maximum.z;
  }

  void BoundingBox::add(const BoundingBox& box) {

    include(box.minimum);
    include(box.maximum);
  }

  bool BoundingBox::contains(const math::Tuple& point) const {

    return (minimum.x <= point.x && maximum.x >= point.x &&
	    minimum.y <= point.y && maximum.y >= point.y &&
	    minimum.z <= point.z && maximum.z >= point.z);
  }

  bool BoundingBox::contains(const BoundingBox& box) const {

    return (contains(box.minimum) && contains(box.maximum));
  }

  BoundingBox BoundingBox::transform(const math::Matrix& transform) {

    // Transform the eight points to extend the BoundingBox
    auto p1 = minimum;
    auto p2 = math::Point(minimum.x, minimum.y, maximum.z);
    auto p3 = math::Point(minimum.x, maximum.y, minimum.z);
    auto p4 = math::Point(minimum.x, maximum.y, maximum.z);
    auto p5 = math::Point(maximum.x, minimum.y, minimum.z);
    auto p6 = math::Point(maximum.x, minimum.y, maximum.z);
    auto p7 = math::Point(maximum.x, maximum.y, minimum.z);
    auto p8 = maximum;

    BoundingBox box;

    for (const auto& p : std::vector<math::Tuple>{p1, p2, p3, p4, p5, p6, p7, p8})
      box.include(transform * p);

    return box;
  }

  bool BoundingBox::intersects(const ray::Ray& ray) const {

    // Same logic as for the Cube
    
    // Check if ray intersects the boundingbox
    auto [xtmin, xtmax] = check_axis(ray.origin.x, ray.direction.x, minimum.x, maximum.x);
    auto [ytmin, ytmax] = check_axis(ray.origin.y, ray.direction.y, minimum.y, maximum.y);
    auto [ztmin, ztmax] = check_axis(ray.origin.z, ray.direction.z, minimum.z, maximum.z);
    auto tmin = std::max(xtmin, std::max(ytmin, ztmin));
    auto tmax = std::min(xtmax, std::min(ytmax, ztmax));
    // When the ray misses the box
    if (tmin > tmax)
      return false;

    return true;
  }

  std::pair<BoundingBox, BoundingBox> BoundingBox::split() const {

    // Find largest dimension
    auto dx = maximum.x - minimum.x;
    auto dy = maximum.y - minimum.y;
    auto dz = maximum.z - minimum.z;
    auto greatest = std::max(dx, std::max(dy, dz));

    auto x0 = minimum.x;
    auto y0 = minimum.y;
    auto z0 = minimum.z;
    auto x1 = maximum.x;
    auto y1 = maximum.y;
    auto z1 = maximum.z;    
    // Adjust Points so that they lie on the dividing plane
    if (greatest == dx) {
      x0 = x0 + dx / 2.0;
      x1 = x0;
    } else if (greatest == dy) {
      y0 = y0 + dy / 2.0;
      y1 = y0;
    } else {
      z0 = z0 + dz / 2.0;
      z1 = z0;
    }
    
    auto mid_min = math::Point(x0, y0, z0);
    auto mid_max = math::Point(x1, y1, z1);    

    BoundingBox left(minimum, mid_max);
    BoundingBox right(mid_min, maximum);

    return std::make_pair(left, right);
  }
      
}

