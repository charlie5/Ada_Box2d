with
     b2_Math,
     b2_Collision,
     b2_Types,
     b2_block_Allocator;


package b2_Shape
is

   --  class b2BlockAllocator;
   --

   use b2_Math,
       b2_block_Allocator,
       b2_Collision,
       b2_Types;



   -------------
   -- b2MassData
   --

   --  This holds the mass data computed for a shape.
   --

   --  struct b2MassData
   --  {
   --    The mass of the shape, usually in kilograms.
   --    float mass;
   --
   --    The position of the shape's centroid relative to the shape's origin.
   --    b2Vec2 center;
   --
   --    The rotational inertia of the shape about the local origin.
   --    float I;
   --  };

   type b2massData is
      record
         Mass   : Real;       -- The mass of the shape, usually in kilograms.
         Center : b2Vec2;     -- The position of the shape's centroid relative to the shape's origin.
         I      : Real;       -- The rotational inertia of the shape about the local origin.
      end record;




   ----------
   -- b2Shape
   --

   --  A shape is used for collision detection. You can create a shape however you like.
   --  Shapes used for simulation in b2World are created automatically when a b2Fixture
   --  is created. Shapes may encapsulate a one or more child shapes.
   --

   --    enum Type
   --    {
   --       e_circle = 0,
   --       e_edge = 1,
   --       e_polygon = 2,
   --       e_chain = 3,
   --       e_typeCount = 4
   --    };

   type shape_Type is (e_circle,
                       e_edge,
                       e_polygon,
                       e_chain,
                       e_typeCount);

   for shape_Type use (e_circle    => 0,
                       e_edge      => 1,
                       e_polygon   => 2,
                       e_chain     => 3,
                       e_typeCount => 4);

   --  class b2Shape
   --  {
   --  public:
   --
   --
   --    Type m_type;
   --
   --    Radius of a shape. For polygonal shapes this must be b2_polygonRadius. There is no support for
   --    making rounded polygons.
   --    float m_radius;
   --  };


   type b2Shape is abstract tagged
      record
         m_Type   : shape_Type;
         m_Radius : Real;
      end record;

   type b2Shape_view is access all b2Shape'Class;



   --    virtual ~b2Shape() {}
   --

   procedure destruct (Self : in out b2Shape) is null;



   --    Clone the concrete shape using the provided allocator.
   --
   --    virtual b2Shape* Clone(b2BlockAllocator* allocator) const = 0;
   --

   function clone (Self : in b2Shape;   Allocator : in out b2BlockAllocator) return b2Shape_view
                   is abstract;



   --    Get the type of this shape. You can use this to down cast to the concrete shape.
   --    @return the shape type.
   --
   --    Type GetType() const;

   function getType (Self : in b2Shape) return shape_Type
     with Inline;



   --    Get the number of child primitives.
   --
   --    virtual int32 GetChildCount() const = 0;
   --

   function getChildCount (Self : in b2Shape) return int32
     is abstract;



   --    Test a point for containment in this shape. This only works for convex shapes.
   --    @param xf the shape world transform.
   --    @param p a point in world coordinates.
   --
   --    virtual bool TestPoint(const b2Transform& xf, const b2Vec2& p) const = 0;

   function testPoint (Self : in b2Shape;   xf : in b2Transform;
                                            p  : in b2Vec2) return Boolean
                       is abstract;



   --    Cast a ray against a child shape.
   --    @param output the ray-cast results.
   --    @param input the ray-cast input parameters.
   --    @param transform the transform to be applied to the shape.
   --    @param childIndex the child shape index
   --
   --    virtual bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
   --                         const b2Transform& transform, int32 childIndex) const = 0;

   function rayCast (Self : in b2Shape;   Output     :    out b2RayCastOutput;
                                          Input      : in     b2RayCastInput;
                                          Transform  : in     b2Transform;
                                          childIndex : in     int32)          return Boolean
                     is abstract;



   --    Given a transform, compute the associated axis aligned bounding box for a child shape.
   --    @param aabb returns the axis aligned box.
   --    @param xf the world transform of the shape.
   --    @param childIndex the child shape
   --
   --    virtual void ComputeAABB(b2AABB* aabb, const b2Transform& xf, int32 childIndex) const = 0;

   procedure computeAABB (Self : in b2Shape;   aabb       :    out b2AABB;
                                               Transform  : in     b2Transform;
                                               childIndex : in     int32)
   is abstract;



   --    Compute the mass properties of this shape using its dimensions and density.
   --    The inertia tensor is computed about the local origin.
   --    @param massData returns the mass data for this shape.
   --    @param density the density in kilograms per meter squared.
   --
   --    virtual void ComputeMass(b2MassData* massData, float density) const = 0;

   procedure computeMass (Self : in b2Shape;   massData :    out b2MassData;
                                               Density  : in     Real)
   is abstract;


end b2_Shape;
