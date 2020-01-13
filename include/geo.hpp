/*! \file geo.hpp
 */

#ifndef GEO_H
#define GEO_H

#include <vector>
#include <optional>
#include <memory>
#include <utility>
#include <cmath>

#include "ray.hpp"
#include "tuple.hpp"
#include "matrix.hpp"
#include "material.hpp"


/*! \namespace geo
 */
namespace geo {

  // Forward declaration
  struct Intersection;
  typedef std::vector<Intersection> Intersections;

  struct BoundingBox;

  /*! \class Shape
   *  \brief Base class, with some virtual functions
   */
  class Shape : public std::enable_shared_from_this<Shape> {
  public:

    /* virtual destructor
     */
    virtual ~Shape() = 0;

    math::Matrix transform = math::IDENTITY_MATRIX; /*!< default transform*/
    material::Material material = material::Material();
    std::weak_ptr<Shape> parent; /*!< A shape has an optional parent (should be only nullptr or a shared_ptr to a Group)*/

    /*! \fn Intersections intersects(const ray::Ray& ray)
     *  \brief Computes the Intersections of the Ray and Shape, calls local_intersects
     *  \param ray the Ray to intersect with
     *  \return Intersections
     */
    Intersections intersects(const ray::Ray& ray);

    /*! \fn virtual Intersections local_intersects(const ray::Ray& local_ray) = 0
     *  \brief Shape dereived classes must implement this function to locally compute the Intersections of the Ray and Shape
     *  \param ray the Ray to intersect with
     *  \return Intersections
     */
    virtual Intersections local_intersects(const ray::Ray& local_ray) = 0;

    /*! \fn math::Tuple normal_at(const math::Tuple& world_point, const Intersection& ix)
     *  \brief Calculates the normal on the Shape at the given point (in world space)
     *  \param world_point
     *  \param ix Intersection data struct
     *  \return a Tuple, the normal Vector (wolrd space)
     */    
    math::Tuple normal_at(const math::Tuple& world_point, const Intersection& ix);

    /*! \fn virtual math::Tuple local_normal_at(const math::Tuple& local_point, const Intersection& ix) const = 0
     *  \brief Shape dereived classes must implement this function to locally calculate the normal on the Shape
     *  \param local_point the point in local space
     *  \param ix Intersection data struct     
     *  \return a Tuple, the normal Vector (local space)
     */     
    virtual math::Tuple local_normal_at(const math::Tuple& local_point, const Intersection& ix) const = 0;

    /*! \fn color::Color pattern_at(const math::Tuple& world_point) const
     *  \brief Use this on Shape with a Pattern, to retrive the Color at the given Point
     *  \param world_point Point in world coordinates 
     *  \return the Color of the Pattern at the given Point
     */
    color::Color pattern_at(const math::Tuple& world_point) const;

    /*! \fn math::Point world_to_object(const math::Tuple& world_point) const
     *  \brief Pass through all parents recursively to transform given world space Point into object space Point
     *  \param world_point Point to convert to local space
     *  \return local space Point
     */ 
    math::Tuple world_to_object(const math::Tuple& world_point) const;

    /*! \fn math::Tuple normal_to_world(const math::Tuple& object_normal) const
     *  \brief Convert the normal in object space to world space, recursively if any parents
     *  \param object_normal object space normal vector
     *  \return world space normal vector
     */    
    math::Tuple normal_to_world(const math::Tuple& object_normal) const;

    /*! \fn std::shared_ptr<Shape> get_shared_ptr()
     *  \brief Get a shared_ptr which shares ownership of this Shape
     *  \return a shared_ptr to *this
     */
    std::shared_ptr<Shape> get_shared_ptr();

    /*! \fn std::weak_ptr<Shape> get_weak_ptr()
     *  \brief Get a weak_ptr which has a weak reference to this Shape
     *  \return a weak_ptr to *this
     */    
    std::weak_ptr<Shape> get_weak_ptr();

    /*! \fn virtual bool local_equality_predicate(const Shape* shape) const = 0
     *  \brief After having check the two Shapes belong to the same class and have same transforms and Material, check if their  member variables are similar
     *  \return true if the two Shape are equal
     */     
    virtual bool local_equality_predicate(const Shape* shape) const = 0;

    /*! \fn virtual BoundingBox get_bounds() const = 0
     *  \brief Get the bounding box coordinates for each untransformed Shape, in object space
     *  \return BoundingBox struct for this Shape
     */    
    virtual BoundingBox get_bounds() const = 0;

    /*! \fn BoundingBox get_parent_space_bounds() const
     *  \brief Get the SHape's bounds in its parent space
     *  \return a BoundingBox
     */    
    BoundingBox get_parent_space_bounds() const;

    /*! \fn virtual void divide(const int threshold)
     *  \brief Nothing happens for primitive Shape. Overriden by Group
     *  \param threshold minimum number of children to have before it will be divided
     */ 
    virtual void divide(const int threshold);

  };
  
  /*! \fn bool operator==(const Shape& first, const Shape& second)
   *  \brief Compares the equality of two Shape
   *  \param first a Shape
   *  \param second another Shape
   *  \return true if the two Shape are equal, false otherwise
   */  
  bool operator==(const Shape& first, const Shape& second);

  /*! \fn bool operator!=(const Shape& first, const Shape& second)
   *  \brief Compares the unequality of two Shape
   *  \param first a Shape
   *  \param second another Shape
   *  \return true if the two Shape are not equal, false otherwise
   */   
  bool operator!=(const Shape& first, const Shape& second);

  /*! \class TestShape
   */  
  class TestShape : public Shape {
  public:

    ~TestShape() override;
   
    ray::Ray saved_ray = ray::Ray(math::Point(0, 0, 0), math::Vector(0, 0, 0));

    Intersections local_intersects(const ray::Ray& local_ray) override;

    math::Tuple local_normal_at(const math::Tuple& local_point, const Intersection& ix) const override;

    bool local_equality_predicate(const Shape* shape) const override;

    BoundingBox get_bounds() const override;

  };

  /*! \class Sphere
   */  
  class Sphere : public Shape {
  public:

    ~Sphere() override;
    
    Intersections local_intersects(const ray::Ray& local_ray) override;

    math::Tuple local_normal_at(const math::Tuple& local_point, const Intersection& ix) const override;

    bool local_equality_predicate(const Shape* shape) const override;

    BoundingBox get_bounds() const override;

  };

  std::shared_ptr<Sphere> build_glass_sphere();

  /*! \class Plane
   */  
  class Plane : public Shape {
  public:

    ~Plane() override;
    
    Intersections local_intersects(const ray::Ray& local_ray) override;

    math::Tuple local_normal_at(const math::Tuple& local_point, const Intersection& ix) const override;

    bool local_equality_predicate(const Shape* shape) const override;

    BoundingBox get_bounds() const override;

  };
  
  /*! \class Cube
   */
  class Cube : public Shape {
  public:

    ~Cube() override;
    
    Intersections local_intersects(const ray::Ray& local_ray) override;

    math::Tuple local_normal_at(const math::Tuple& local_point, const Intersection& ix) const override;

    bool local_equality_predicate(const Shape* shape) const override;

    BoundingBox get_bounds() const override;

  };
  
  /*! \class Cylinder
   */
  class Cylinder : public Shape {
  public:

    float minimum = -INFINITY; /*<! minimum value on the y axis (lower bound) (default not truncated)*/
    float maximum = INFINITY; /*<! maximum value on the y axis (upper bound) (default not truncated)*/
    bool closed = false; /*<! if the cylinder is capped */

    ~Cylinder() override;
    
    Intersections local_intersects(const ray::Ray& local_ray) override;

    math::Tuple local_normal_at(const math::Tuple& local_point, const Intersection& ix) const override;

    /*! \fn Intersections intersects_caps(const ray::Ray& ray, Intersections ixs)
     *  \brief Checks if the Ray intersects with the end caps of the Cylinder
     *  \param local_ray Ray to check
     *  \param ixs existing Intersections
     *  \return The given Intersections + the possible point of intersections with end caps
     */ 
    Intersections intersects_caps(const ray::Ray& local_ray, Intersections ixs);

    bool local_equality_predicate(const Shape* shape) const override;

    BoundingBox get_bounds() const override;

  };

  /*! \class DoubleCone
   *  Can obtain a simple cone by truncating it at min = -1 and max = 0 and translate y 1
   */
  class DoubleCone : public Shape {
  public:

    float minimum = -INFINITY; /*<! minimum value on the y axis (lower bound) (default not truncated)*/
    float maximum = INFINITY; /*<! maximum value on the y axis (upper bound) (default not truncated)*/
    bool closed = false; /*<! if the cone is capped */

    ~DoubleCone() override;
    
    Intersections local_intersects(const ray::Ray& local_ray) override;

    math::Tuple local_normal_at(const math::Tuple& local_point, const Intersection& ix) const override;

    /*! \fn Intersections intersects_caps(const ray::Ray& ray, Intersections ixs)
     *  \brief Checks if the Ray intersects with the end caps of the DoubleCone
     *  \param local_ray Ray to check
     *  \param ixs existing Intersections
     *  \return The given Intersections + the possible point of intersections with end caps
     */ 
    Intersections intersects_caps(const ray::Ray& local_ray, Intersections ixs);

    bool local_equality_predicate(const Shape* shape) const override;

    BoundingBox get_bounds() const override;
    
  };

  /*! \class Triangle
   */
  class Triangle : public Shape {
  public:

    math::Tuple p1;
    math::Tuple p2;
    math::Tuple p3;
    math::Vector e1; /// edge vector
    math::Vector e2;
    math::Vector normal;    

    Triangle(const math::Tuple& p1_, const math::Tuple& p2_, const math::Tuple& p3_);

    ~Triangle() override;
    
    Intersections local_intersects(const ray::Ray& local_ray) override;

    math::Tuple local_normal_at(const math::Tuple& local_point, const Intersection& ix) const override;

    bool local_equality_predicate(const Shape* shape) const override;

    BoundingBox get_bounds() const override;
    
  };

  /*! \class Triangle
   */
  class SmoothTriangle : public Shape {
  public:

    math::Tuple p1;
    math::Tuple p2;
    math::Tuple p3;
    math::Tuple n1; /// normal vectors
    math::Tuple n2;
    math::Tuple n3;
    math::Tuple e1; /// edge vector  -- > USEFUL FOR SMOOTHTRIANGLE?
    math::Tuple e2;
    math::Vector normal;    

    SmoothTriangle(const math::Tuple& p1_, const math::Tuple& p2_, const math::Tuple& p3_, const math::Tuple& n1_, const math::Tuple& n2_, const math::Tuple& n3_);

    ~SmoothTriangle() override;
    
    Intersections local_intersects(const ray::Ray& local_ray) override;

    math::Tuple local_normal_at(const math::Tuple& local_point, const Intersection& ix) const override;

    bool local_equality_predicate(const Shape* shape) const override;

    BoundingBox get_bounds() const override;
    
  };    

  /*! \class Group
   *  \brief A Shape that is a collection of Shapes
   */
  class Group : public Shape {
  public:

    std::vector<std::shared_ptr<Shape>> shapes;

    ~Group() override;
    
    Intersections local_intersects(const ray::Ray& local_ray) override;

    math::Tuple local_normal_at(const math::Tuple& local_point, const Intersection& ix) const override;

    /*! \fn void add_child(std::shared_ptr<Shape> shape)
     *  \brief Add the shape to the group of Shapes 
     *  \param shape the Shape to add to the Group
     */
    void add_child(std::shared_ptr<Shape> shape);

    bool local_equality_predicate(const Shape* shape) const override;

    /*! \fn  BoundingBox get_bounds() const 
     *  \brief Convert the BoundingBox of its children into group space
     *  \return Group's BoundingBox
     */
    BoundingBox get_bounds() const override;

    /*! \fn std::pair<std::vector<std::shared_ptr<Shape>>, std::vector<std::shared_ptr<Shape>>> partition_children()
     *  \brief Splits the shapes betwen two vector, one for the left sided objects, another one for the right sided shapes. Shapes that are not entirely in one sub BB stays in this Group
     *  \return a pair of vector of shapes ([left, right])
     */
    std::pair<std::vector<std::shared_ptr<Shape>>, std::vector<std::shared_ptr<Shape>>> partition_children();
    /*! \fn void make_subgroup(const std::vector<std::shared_ptr<Shape>> shape_vec)
     *  \brief Create a child Group and add the elements of shape_vec to it
     *  \param shape_vec
     */ 
    void make_subgroup(const std::vector<std::shared_ptr<Shape>> shape_vec);

    /*! \fn void divide(const int threshold) override
     *  \brief recursively splits a group and its children. 
     *  \param threshold minimum number of children to have before it will be divided
     */ 
    void divide(const int threshold) override;    
  };
  
  /*! \fn math::Tuple reflect(const math::Tuple& vec, const math::Tuple& normal)
   *  \brief Reflects the vector around the normal
   *  \param the vector to reflect
   *  \param the normal vector
   *  \return a Tuple, the reflected vector
   */ 
  math::Tuple reflect(const math::Tuple& vec, const math::Tuple& normal);

  /*! \fn std::pair<double, double> check_axis(const double origin, const double direction)
   *  \brief helper function to find intersection points in Cube.local_intersects
   *  \param origin ray origin (one axis)
   *  \param direction  ray direction(one axis)
   *  \param min_val min value for the tested axis
   *  \param max_val max value for the tested axis
   *  \return a pair, min and max t values
   */  
  std::pair<double, double> check_axis(const double origin, const double direction, const double min_val = -1, const double max_val = 1);

  /*! \fn bool check_cap(const ray::Ray& ray, const double t)
   *  \biref helper function to Cylinder/DoubleCone.intersects_caps, to check if the intersection lies in the Cylinder radius 
   *  \param ray
   *  \param t time
   *  \param radius the radius of the cylinder or cone, default 1 for cylinder
   *  \return true if intersects with cylinder cap
   */
  bool check_cap(const ray::Ray& ray, const double t, const double radius = 1);

  
  /*! \struct Intersection
   */
  struct Intersection {

    float t;
    std::shared_ptr<geo::Shape> geometry;
    float u; /// Only useful for smooth triangle, to help identify where the intersections occured on a triangle
    float v;

    /*! Intersection constructor
     *  \param t_ the t value of the intersection, "when" the ray intersects the geo
     *  \param geo the object that's been intersected
     */   
    Intersection(const float t_, geo::Shape* geo, const float u_ = 0, const float v_ = 0);

    Intersection() = default;

    /*! \fn Intersection& operator=(const Intersection& source)
     *  \param source the Intersection to copied, the source
     *  \return Intersection copy destination
     */       
    Intersection& operator=(const Intersection& source);

  };
  
  /*! \fn std::optional<Intersection> hit(const Intersections& intersections)
   *  \brief Find the hit among all the Intersections, if there is any. The hit is the lowest nonegative intersection.
   *  \param the Intersections to look at
   *  \return an optional Intersection
   */
  std::optional<Intersection> hit(const Intersections& intersections);

  /*! \fn bool operator==(const Intersection& first, const Intersection& second)
   *  \brief Compares the equality of two Intersection
   *  \param first an Intersection
   *  \param second another Intersection
   *  \return true if the two Intersection are equal, false otherwise
   */
  bool operator==(const Intersection& first, const Intersection& second);

  /*! \fn bool operator!=(const Intersection& first, const Intersection& second)
   *  \brief Compares the unequality of two Intersection
   *  \param first an Intersection
   *  \param second another Intersection
   *  \return true if the two Intersection are not equal, false otherwise
   */  
  bool operator!=(const Intersection& first, const Intersection& second);

  
  /*! \struct Computations
   *  \brief Store some precomputed values relative to an intersection
   */
  struct Computations {

    double t;
    std::shared_ptr<geo::Shape> geometry;
    math::Tuple point;
    math::Tuple eye_vector;
    math::Tuple normal_vector;
    bool inside; /*!< true if the intersection occured into an object*/
    math::Tuple over_point = math::Point(0, 0, 0); /*!< Adjust the point slightly above the normal to prevent self-shadowing and acne due to float rounding errors*/
    math::Tuple under_point = math::Point(0, 0, 0); 
    math::Tuple reflect_vector;
    double n1; /*<! refractive index of the material being exited */
    double n2; /*<! refractive index of the material being entered  */
    
    /*! Computations' constuctor
     */
    Computations() = default;

    /*! \fn float schlick() const
     *  \brief Schlick approximation to Fresnel equaltion to calculate the reflectance
     *  \return schlick approximation
     */
    float schlick() const;
  };  

  /*! \fn Computations prepare_computations(const inter::Intersection& ixs, const ray::Ray r)
   *  \param ixs an Intersection
   *  \param r a Ray
   *  \param ixs Intersections
   *  \return a Computations
   */
  Computations prepare_computations(const Intersection& ix, const ray::Ray r, const Intersections& ixs = Intersections{});

  /*! \struct BoundingBox
   *  \brief Stores the coordinates of a bounding box
   */
  struct BoundingBox {

    math::Tuple minimum = math::Point(INFINITY, INFINITY, INFINITY); /// minumum coordinates of the bounding box
    math::Tuple maximum = math::Point(-INFINITY, -INFINITY, -INFINITY);; /// maximum coordinates of the bounding box

    /*! \fn BoundingBox()
     *  \brief Constructs a default BoundingBox (min = infinity, max = -infinity)
     */     
    BoundingBox() = default;
    
    /*! \fn BoundingBox(const math::Tuple& min, const math::Tuple& max)
     *  \brief Constructs a BoundingBox by passing two Points
     */    
    BoundingBox(const math::Tuple& min, const math::Tuple& max);

    /*! \fn void include(const math::Tuple& point)
     *  \brief Make sure the BoundingBox can include the given Point
     *  \param point the Point to include
     */
    void include(const math::Tuple& point);

    /*! \fn void add(const BoundingBox& box)
     *  \brief Extend this BoundingBox with another one
     *  \param box the other BoundingBox
     */    
    void add(const BoundingBox& box);

    /*! \fn bool contains(const math::Tuple& point) const
     *  \brief Check if the BoundingBox contains the given point
     *  \param point the Point to check if inside
     */    
    bool contains(const math::Tuple& point) const;

    /*! \fn bool contains(const BoundingBox& box) const
     *  \brief Check if the BoundingBox contains the given BoundingBox
     *  \param point the PBoundingBox to check if inside
     */        
    bool contains(const BoundingBox& box) const;

    /*! \fn BoundingBox transform(const math::Matrix& transform)
     *  \brief Transforms the BoundingBox with the given transformation Matrix
     *  \param transform A transformation Matrix
     *  \return a new, transformed, BoundingBox
     */    
    BoundingBox transform(const math::Matrix& transform);

    /*! \fn bool intersects(const ray::Ray& r) const
     *  \brief Check if the Ray intersects with the BoundingBox
     *  \param r the Ray
     *  \return true if the Ray intersects the box, false otherwise
     */
    bool intersects(const ray::Ray& r) const;

    /*! \fn std::pair<BoundingBox, BoundingBox> split() const
     *  \brief Split the BoundingBox on the longest axis
     *  \return a pair of BoundingBox
     */    
    std::pair<BoundingBox, BoundingBox> split() const;
  };
  
}

#endif

