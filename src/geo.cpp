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

  math::Tuple Shape::normal_at(const math::Tuple& world_point) {

    // The vector from the sphere's origin to the point on the sphere is the normal at the point where it intersects (for a sphere centered at origin)

    // From world space to object space
    auto local_point = math::inverse(transform) * world_point;
    // The object's normal vector we get is in object space
    auto local_normal = local_normal_at(local_point);
    // When the sphere has been transformed, to keep the normals perpendicular to the surface, we must multiply the normal by the inverse transpose matrix
    auto world_normal = math::transpose(math::inverse(transform)) * local_normal;
    // If the transform includes any kind of translation, multiplying by its transpose will mess with w coordinates in the vector
    world_normal.w = 0;
       
    return math::normalize(world_normal);
  }

  color::Color Shape::pattern_at(const math::Tuple& world_point) const {

    auto object_point = math::inverse(transform) * world_point;

    return material.pattern->pattern_at(object_point);
  }

  math::Tuple Shape::world_to_object(const math::Tuple& world_point) const {

    // It the Shape has a parent, first convert the point to its parent space
    if (parent != nullptr) {
      auto parent_point = parent->world_to_object(world_point);
      return math::inverse(transform) * parent_point;
    }
  
    return math::inverse(transform) * world_point;
  }

  math::Tuple Shape::normal_to_world(const math::Tuple& object_normal) const {

    auto normal = math::transpose(math::inverse(transform)) * object_normal;
    normal.w = 0;
    normal = math::normalize(normal);

    if (parent != nullptr)
      normal = parent->normal_to_world(normal);

    return normal;
  }

  std::shared_ptr<Shape> Shape::get_shared_ptr() {

    return shared_from_this();
  }

  std::weak_ptr<Shape> Shape::get_weak_ptr() {

    return weak_from_this();
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

  math::Tuple TestShape::local_normal_at(const math::Tuple& local_point) const {

    return math::Vector(local_point.x, local_point.y, local_point.z);
  }

  bool TestShape::local_equality_predicate(const Shape* shape) const {
    // If the transforms and material are equal, the TestShapes are equal
    return true;
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

  math::Tuple Sphere::local_normal_at(const math::Tuple& object_point) const {

      return object_point - math::Point(0.0, 0.0, 0.0);
  }

  bool Sphere::local_equality_predicate(const Shape* shape) const {
    // If the transforms and material are equal, the Spheres are equal
    return true;
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

  math::Tuple Plane::local_normal_at(const math::Tuple& local_point) const {

    static const math::Tuple default_normal = math::Vector(0, 1, 0);

    return default_normal;
  }

  bool Plane::local_equality_predicate(const Shape* shape) const {
    // If the transforms and material are equal, the Plane are equal
    return true;
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
  
  math::Tuple Cube::local_normal_at(const math::Tuple& local_point) const {

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

  math::Tuple Cylinder::local_normal_at(const math::Tuple& local_point) const {
    
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

  math::Tuple DoubleCone::local_normal_at(const math::Tuple& local_point) const {

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
  
  math::Tuple reflect(const math::Tuple& vec, const math::Tuple& normal) {

    // The velocity vec is reflected around the normal
  
    return vec - normal * 2 * math::dot(vec, normal);
  }  

  std::pair<double, double> check_axis(const double origin, const double direction) {

    // each pair of planes is offset 1 unit in opposing direction
    double tmin_numerator = -1 - origin;
    double tmax_numerator = 1 - origin;

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

  Group::~Group() {};

  Intersections Group::local_intersects(const ray::Ray& local_ray) {

    auto group_intersections = Intersections{};

    // Call intersects for each Shape of the Group
    for (const auto& shape_weak_ptr : shapes)
      // Check the weak_ptr can be converted to a shared_ptr
      if (auto shrd_shape_ptr = shape_weak_ptr.lock()) {
	auto shape_intersections = shrd_shape_ptr->intersects(local_ray);
	group_intersections.insert(group_intersections.end(), shape_intersections.begin(), shape_intersections.end());
    }

    // sort intersections
    std::sort(group_intersections.begin(), group_intersections.end(),
	      [&](const Intersection& inter1, const Intersection& inter2){return inter1.t < inter2.t;});
    
    return group_intersections;
  }

  math::Tuple Group::local_normal_at(const math::Tuple& local_point) const {

    return math::Vector(0, 0, 0);
  }

  void Group::add_child(Shape* shape) {

    shapes.push_back(shape->get_weak_ptr());
    shape->parent = get_shared_ptr();
  }

  bool Group::local_equality_predicate(const Shape* shape) const {

    auto group = dynamic_cast<const Group*>(shape);

    // Check there are as much shapes in both Groups
    if (shapes.size() != group->shapes.size())
      return false;

    // Check that every shape in this Group can be found in the other Group
    for (const auto& sub_shape : group->shapes) {
      auto p = std::find_if(shapes.begin(), shapes.end(),
			    [&](const std::weak_ptr<geo::Shape> shp){
			      if (auto shrdptr = shp.lock()){
				if (*shrdptr == *sub_shape.lock())
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

  Intersection::Intersection(const float t_, geo::Shape* geo) :
    t {t_}, geometry {geo->get_shared_ptr()} {}

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
    // std::back_inserter(nonneg_integers) and not nonneg_integers.begin():
    // copy_if does not do a push_back, it just copy the value on the position where the destination iterator is and then increments the iterator
    // --> can have seg faults
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
    comps.normal_vector = comps.geometry->normal_at(comps.point);

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

}

