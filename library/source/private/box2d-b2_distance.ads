with
     box2d.b2_Shape,
     box2d.b2_Math,
     box2d.b2_Types,
     box2d.b2_Pointers,
     box2d.b2_Settings;


package box2d.b2_Distance
is
   use b2_Shape,
       b2_Math,
       b2_Types,
       b2_Pointers,
       b2_Settings;



   --  A distance proxy is used by the GJK algorithm.
   --  It encapsulates any shape.
   --
   --  struct b2DistanceProxy
   --  {
   --    b2Vec2 m_buffer[2];
   --    const b2Vec2* m_vertices;
   --    int32 m_count;
   --    float m_radius;
   --  };
   --

   type b2DistanceProxy is tagged
      record
         m_Buffer   : b2Vec2s (0 .. 1);
         m_Vertices : b2Vec2_ptr;
         --  m_Vertices : b2Vec2s (0 .. b2_maxPolygonVertices - 1);
         m_Count    : Natural      := 0;
         m_Radius   : Real := 0.0;
      end record;


   --    b2DistanceProxy() : m_vertices(nullptr), m_count(0), m_radius(0.0f) {}


   --    Initialize the proxy using the given shape. The shape
   --    must remain in scope while the proxy is in use.
   --
   --    void Set(const b2Shape* shape, int32 index);

   procedure set (Self : access b2DistanceProxy;   Shape : in b2Shape_ptr;
                                                   Index : in Natural);


   --      Initialize the proxy using a vertex cloud and radius. The vertices
   --      must remain in scope while the proxy is in use.
   --
   --      void Set(const b2Vec2* vertices, int32 count, float radius);

   procedure set (Self : in out b2DistanceProxy;   Vertices : in b2Vec2_ptr;
                                                   Count    : in Positive;
                                                   Radius   : in Real);


   --    Get the supporting vertex index in the given direction.
   --
   --    int32 GetSupport(const b2Vec2& d) const;

   function getSupport (Self : in b2DistanceProxy;   d : in b2Vec2) return Natural
     with Inline;



   --    Get the supporting vertex in the given direction.
   --
   --    const b2Vec2& GetSupportVertex(const b2Vec2& d) const;

   function getSupportVertex (Self : in b2DistanceProxy;   d : in b2Vec2) return b2Vec2
     with Inline;



   --    Get the vertex count.
   --
   --    int32 GetVertexCount() const;

   function getVertexCount (Self : in b2DistanceProxy) return Positive
     with Inline;



   --    Get a vertex by index. Used by b2Distance.
   --
   --    const b2Vec2& GetVertex(int32 index) const;

   function getVertex (Self : in b2DistanceProxy;   Index : in Natural) return b2Vec2
     with Inline;






   --  Used to warm start b2Distance.
   --  Set count to zero on first call.
   --
   --  struct b2SimplexCache
   --  {
   --    float metric;     ///< length or area
   --    uint16 count;
   --    uint8 indexA[3];  ///< vertices on shape A
   --    uint8 indexB[3];  ///< vertices on shape B
   --  };
   --

   type uint8s   is array (Natural range <>) of uint8;

   type b2SimplexCache is
      record
         Metric : Real;     -- Length or area.
         Count  : Natural;
         indexA : Naturals (0 .. 3);  -- Vertices on shape A.
         indexB : Naturals (0 .. 3);  -- Vertices on shape B.
      end record;





   --  Input for b2Distance.
   --
   --  You have the option to use the shape radii in the computation. Even
   --
   --  struct b2DistanceInput
   --  {
   --    b2DistanceProxy proxyA;
   --    b2DistanceProxy proxyB;
   --    b2Transform transformA;
   --    b2Transform transformB;
   --    bool useRadii;
   --  };
   --

   type b2DistanceInput is
      record
         proxyA     : aliased b2DistanceProxy;
         proxyB     : aliased b2DistanceProxy;

         transformA : b2Transform;
         transformB : b2Transform;

         useRadii   : Boolean;
      end record;





   --  Output for b2Distance.
   --
   --  struct b2DistanceOutput
   --  {
   --    b2Vec2 pointA;    ///< closest point on shapeA
   --    b2Vec2 pointB;    ///< closest point on shapeB
   --    float distance;
   --    int32 iterations; ///< number of GJK iterations used
   --  };
   --

   type b2DistanceOutput is
      record
         pointA     : b2Vec2;           -- Closest point on shapeA.
         pointB     : b2Vec2;           -- Closest point on shapeB.

         Distance   : Real;
         Iterations : Positive;         -- Number of GJK iterations used.
      end record;


   --  Compute the closest points between two shapes. Supports any combination
   --  of: b2CircleShape, b2PolygonShape, b2EdgeShape.
   --  The simplex cache is input/output.
   --
   --  On the first call set 'b2SimplexCache.count' to zero.
   --
   --  void b2Distance (b2DistanceOutput* output,
   --                   b2SimplexCache* cache,
   --                   const b2DistanceInput* input);
   --

   procedure b2Distance (Output :    out b2DistanceOutput;
                         Cache  : in out b2SimplexCache;
                         Input  : in     b2DistanceInput);





   --  Input parameters for b2ShapeCast
   --
   --  struct b2ShapeCastInput
   --  {
   --    b2DistanceProxy proxyA;
   --    b2DistanceProxy proxyB;
   --    b2Transform transformA;
   --    b2Transform transformB;
   --    b2Vec2 translationB;
   --  };
   --

   type b2ShapeCastInput is
      record
         proxyA       : b2DistanceProxy;
         proxyB       : b2DistanceProxy;

         transformA   : b2Transform;
         transformB   : b2Transform;

         translationB : b2Vec2;
      end record;



   --  Output results for b2ShapeCast
   --
   --  struct b2ShapeCastOutput
   --  {
   --    b2Vec2 point;
   --    b2Vec2 normal;
   --    float lambda;
   --    int32 iterations;
   --  };
   --

   type b2ShapeCastOutput is
      record
         Point      : b2Vec2;
         Normal     : b2Vec2;
         Lambda     : Real;
         Iterations : Natural;
      end record;


   --  Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
   --
   --  @returns true if hit, false if there is no hit or an initial overlap
   --
   --  bool b2ShapeCast(b2ShapeCastOutput* output, const b2ShapeCastInput* input);
   --

   function b2ShapeCast (Output :    out b2ShapeCastOutput;
                         Input  : in     b2ShapeCastInput) return Boolean;


end box2d.b2_Distance;
