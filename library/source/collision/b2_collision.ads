with
     b2_Math,
     b2_Types,
     b2_Common,
     b2_Settings,

     interfaces.C;


package b2_Collision
--
--  Structures and functions used for computing contact points,
--  distance queries, and TOI queries.
--
is
   use b2_Math,
       b2_Types,
       b2_Common,
       b2_Settings,
       Interfaces;


   --  class b2Shape;
   --  class b2CircleShape;
   --  class b2EdgeShape;
   --  class b2PolygonShape;


   --  const uint8 b2_nullFeature = UCHAR_MAX;
   --
   b2_nullFeature : constant uint8 := interfaces.C.UCHAR_MAX;





   -------------------
   -- b2contactFeature
   --

   --  The features that intersect to form the contact point
   --  This must be 4 bytes or less.
   --
   --  struct b2ContactFeature
   --  {
   --    enum Type
   --    {
   --       e_vertex = 0,
   --       e_face = 1
   --    };
   --
   --    uint8 indexA;     ///< Feature index on shapeA
   --    uint8 indexB;     ///< Feature index on shapeB
   --    uint8 typeA;      ///< The feature type on shapeA
   --    uint8 typeB;      ///< The feature type on shapeB
   --  };

   type feature_Type      is  (e_vertex,       e_face);
   for  feature_Type      use (e_vertex => 0,  e_face => 1);
   for  feature_Type'Size use 8;

   type b2contactFeature is
      record
         indexA : Unsigned_8;       -- Feature index on shapeA.
         indexB : Unsigned_8;       -- Feature index on shapeB.
         typeA  : feature_Type;     -- The feature type on shapeA.
         typeB  : feature_Type;     -- The feature type on shapeB.
      end record;

   pragma assert (b2contactFeature'Size <= 4 * 8);





   --------------
   -- b2ContactID
   --

   --  Contact Ids to facilitate warm starting.
   --
   --  union b2ContactID
   --  {
   --    b2ContactFeature cf;
   --    uint32 key;             // Used to quickly compare contact ids.
   --  };

   type b2ContactID (null_Discriminant : Boolean := False) is
      record
         case null_Discriminant is
            when True  => cF  : b2ContactFeature;
            when False => Key : Natural;               -- Used to quickly compare contact ids.
         end case;
      end record
      with unchecked_Union;





   ------------------
   -- b2manifoldPoint
   --

   --  A manifold point is a contact point belonging to a contact
   --  manifold. It holds details related to the geometry and dynamics
   --  of the contact points.
   --
   --  The local point usage depends on the manifold type:
   --
   --       e_circles: the local center of circleB
   --       e_faceA:   the local center of cirlceB or the clip point of polygonB
   --       e_faceB:   the clip point of polygonA
   --
   --  This structure is stored across time steps, so we keep it small.
   --
   --  Note: The impulses are used for internal caching and may not provide
   --        reliable contact forces, especially for high speed collisions.

   --  struct b2ManifoldPoint
   --  {
   --    b2Vec2 localPoint;      // usage depends on manifold type
   --    float normalImpulse;    // the non-penetration impulse
   --    float tangentImpulse;   // the friction impulse
   --    b2ContactID id;         // uniquely identifies a contact point between two shapes
   --  };

   type b2manifoldPoint is
      record
         localPoint     : b2Vec2;          -- Usage depends on manifold type.
         normalImpulse  : Real;            -- The non-penetration impulse.
         tangentImpulse : Real;            -- The friction impulse.
         Id             : b2ContactID;     -- Uniquely identifies a contact point between two shapes.
      end record;

   type b2manifoldPoints is array (Natural range <>) of aliased b2manifoldPoint;



   -------------
   -- b2Manifold
   --

   --  A manifold for two touching convex shapes.
   --
   --  Box2D supports multiple types of contact:
   --
   --       clip point versus plane with radius
   --       point versus point with radius (circles)
   --
   --  The local point usage depends on the manifold type:
   --
   --       e_circles: The local center of circleA.
   --       e_faceA:   The center of faceA.
   --       e_faceB:   The center of faceB.
   --
   --  Similarly the local normal usage:
   --
   --       e_circles: Not used
   --       e_faceA:   The normal on polygonA.
   --       e_faceB:   The normal on polygonB.
   --
   --  We store contacts in this way so that position correction can
   --  account for movement, which is critical for continuous physics.
   --
   --  All contact scenarios must be expressed in one of these types.
   --
   --  This structure is stored across time steps, so we keep it small.

   --  struct b2Manifold
   --  {
   --    enum Type
   --    {
   --       e_circles,
   --       e_faceA,
   --       e_faceB
   --    };
   --
   --    b2ManifoldPoint points[b2_maxManifoldPoints];   ///< the points of contact
   --    b2Vec2 localNormal;                       ///< not use for Type::e_points
   --    b2Vec2 localPoint;                        ///< usage depends on manifold type
   --    Type type;
   --    int32 pointCount;                      ///< the number of manifold points
   --  };

   type b2manifold_Type is (e_Circles, e_faceA, e_faceB);

   type b2Manifold is
      record
        Points      : b2manifoldPoints (0 .. b2_maxManifoldPoints - 1);     -- The points of contact.
        localNormal : b2Vec2;                                           -- Not use for Type::e_points.
        localPoint  : b2Vec2;                                           -- Usage depends on manifold type.
        Kind        : b2manifold_Type;
        pointCount  : Natural;                                            -- The number of manifold points.
      end record;





   ------------------
   -- b2WorldManifold
   --

   --  This is used to compute the current state of a contact manifold.
   --
   --  struct b2WorldManifold
   --  {
   --    Evaluate the manifold with supplied transforms. This assumes
   --    modest motion from the original state. This does not change the
   --    point count, impulses, etc. The radii must come from the shapes
   --    that generated the manifold.
   --
   --    b2Vec2 normal;                                -- world vector pointing from A to B
   --    b2Vec2 points     [b2_maxManifoldPoints];    -- world contact point (point of intersection)
   --    float  separations[b2_maxManifoldPoints];     -- a negative value indicates overlap, in meters
   --  };

   type b2WorldManifold is
      record
         Normal      : b2Vec2;                                      -- World vector pointing from A to B.
         Points      : b2Vec2s (0 .. b2_maxManifoldPoints - 1);     -- World contact point (point of intersection).
         Separations : Reals   (0 .. b2_maxManifoldPoints - 1);     -- A negative value indicates overlap, in meters.
      end record;


   --    void Initialize(const b2Manifold*    manifold,
   --                    const b2Transform&   xfA, float radiusA,
   --                    const b2Transform&   xfB, float radiusB);
   --
   procedure initialize (Self : out b2WorldManifold;   Manifold : in b2Manifold;
                                                       xfA      : in b2Transform;   radiusA : in Real;
                                                       xfB      : in b2Transform;   radiusB : in Real);



   ---------------
   -- b2PointState
   --

   --  This is used for determining the state of contact points.
   --
   --  enum b2PointState
   --  {
   --    b2_nullState,     ///< point does not exist
   --    b2_addState,      ///< point was added in the update
   --    b2_persistState,  ///< point persisted across the update
   --    b2_removeState    ///< point was removed in the update
   --  };

   type b2PointState  is (b2_nullState,        -- Point does not exist.
                          b2_addState,         -- Point was added in the update.
                          b2_persistState,     -- Point persisted across the update.
                          b2_removeState);     -- Point was removed in the update.

   type b2PointStates is array (Natural range 0 .. b2_maxManifoldPoints - 1) of b2PointState;


   --  Compute the point states given two manifolds. The states pertain to the transition from manifold1
   --  to manifold2. So state1 is either persist or remove while state2 is either add or persist.
   --
   --  void b2GetPointStates(b2PointState state1[b2_maxManifoldPoints], b2PointState state2[b2_maxManifoldPoints],
   --                  const b2Manifold* manifold1, const b2Manifold* manifold2);

   procedure b2getPointStates (State1    :    out b2PointStates;
                               State2    :    out b2PointStates;
                               Manifold1 : in     b2Manifold;
                               Manifold2 : in     b2Manifold);


   ---------------
   -- b2ClipVertex
   --

   --  Used for computing contact manifolds.
   --
   --  struct b2ClipVertex
   --  {
   --    b2Vec2 v;
   --    b2ContactID id;
   --  };

   type b2ClipVertex is
      record
         v  : b2Vec2;
         Id : b2ContactID;
      end record;




   -----------------
   -- b2RayCastInput
   --

   --  Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
   --  struct b2RayCastInput
   --  {
   --    b2Vec2 p1, p2;
   --    float maxFraction;
   --  };

   type b2RayCastInput is
      record
         p1, p2      : b2Vec2;
         maxFraction : Real;
      end record;




   ------------------
   -- b2RayCastOutput
   --
   --  Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
   --  come from b2RayCastInput.
   --
   --  struct b2RayCastOutput
   --  {
   --    b2Vec2 normal;
   --    float fraction;
   --  };

   type b2RayCastOutput is
      record
         Normal   : b2Vec2;
         Fraction : Real;
      end record;




   ---------
   -- b2AABB
   --

   --  An axis aligned bounding box.
   --
   --  struct b2AABB
   --  {
   --    b2Vec2 lowerBound;   ///< the lower vertex
   --    b2Vec2 upperBound;   ///< the upper vertex
   --  };

   type b2AABB is
      record
         lowerBound : b2Vec2;     -- The lower vertex.
         upperBound : b2Vec2;     -- The upper vertex.
      end record;


   --    Verify that the bounds are sorted.
   --
   --    bool IsValid() const;
   --
   --  inline bool b2AABB::IsValid() const
   --  {
   --    b2Vec2 d = upperBound - lowerBound;
   --    bool valid = d.x >= 0.0f && d.y >= 0.0f;
   --    valid = valid && lowerBound.IsValid() && upperBound.IsValid();
   --    return valid;
   --  }

   function isValid (Self : in b2AABB) return Boolean;



   --    Get the center of the AABB.
   --
   --    b2Vec2 GetCenter() const
   --    {
   --       return 0.5f * (lowerBound + upperBound);
   --    }

   function getCenter (Self : in b2AABB) return b2Vec2;


   --    Get the extents of the AABB (half-widths).
   --
   --    b2Vec2 GetExtents() const
   --    {
   --       return 0.5f * (upperBound - lowerBound);
   --    }

   function getExtents (Self : in b2AABB) return b2Vec2;


   --    Get the perimeter length.
   --
   --    float GetPerimeter() const
   --    {
   --       float wx = upperBound.x - lowerBound.x;
   --       float wy = upperBound.y - lowerBound.y;
   --       return 2.0f * (wx + wy);
   --    }

   function getPerimeter (Self : in b2AABB) return Real;


   --    Combine an AABB into this one.
   --
   --    void Combine(const b2AABB& aabb)
   --    {
   --       lowerBound = b2Min(lowerBound, aabb.lowerBound);
   --       upperBound = b2Max(upperBound, aabb.upperBound);
   --    }

   procedure combine (Self : in out b2AABB;   aabb : in b2AABB);


   --    Combine two AABBs into this one.
   --
   --    void Combine(const b2AABB& aabb1, const b2AABB& aabb2)
   --    {
   --       lowerBound = b2Min(aabb1.lowerBound, aabb2.lowerBound);
   --       upperBound = b2Max(aabb1.upperBound, aabb2.upperBound);
   --    }

   procedure combine (Self : in out b2AABB;   aabb1 : in b2AABB;
                                              aabb2 : in b2AABB);


   --    Does this aabb contain the provided AABB.
   --
   --    bool Contains(const b2AABB& aabb) const
   --    {
   --       bool result = true;
   --       result = result && lowerBound.x <= aabb.lowerBound.x;
   --       result = result && lowerBound.y <= aabb.lowerBound.y;
   --       result = result && aabb.upperBound.x <= upperBound.x;
   --       result = result && aabb.upperBound.y <= upperBound.y;
   --       return result;
   --    }

   function contains (Self : in b2AABB;   aabb : in b2AABB) return Boolean;


   --    bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input) const;

   function rayCast (Self : in b2AABB;   output : out b2RayCastOutput;
                                         input  : in  b2RayCastInput) return Boolean;


   --  inline bool b2TestOverlap(const b2AABB& a, const b2AABB& b)
   --  {
   --    b2Vec2 d1, d2;
   --    d1 = b.lowerBound - a.upperBound;
   --    d2 = a.lowerBound - b.upperBound;
   --
   --    if (d1.x > 0.0f || d1.y > 0.0f)
   --       return false;
   --
   --    if (d2.x > 0.0f || d2.y > 0.0f)
   --       return false;
   --
   --    return true;
   --  }

   function b2testOverlap (a, b : in b2AABB) return Boolean;




   ------------
   -- Colliders
   --


   --  Compute the collision manifold between two circles.
   --
   --  void b2CollideCircles(b2Manifold* manifold,
   --                        const b2CircleShape* circleA, const b2Transform& xfA,
   --                        const b2CircleShape* circleB, const b2Transform& xfB);
   --
   -- Moved to 'b2_collide_Circle' spec.



   --  Compute the collision manifold between a polygon and a circle.
   --
   --  void b2CollidePolygonAndCircle(b2Manifold* manifold,
   --                                 const b2PolygonShape* polygonA, const b2Transform& xfA,
   --                                 const b2CircleShape*  circleB,  const b2Transform& xfB);
   --
   -- Moved to 'b2_collide_Circle' spec.



   --  Compute the collision manifold between two polygons.
   --
   --  void b2CollidePolygons(b2Manifold* manifold,
   --                         const b2PolygonShape* polygonA, const b2Transform& xfA,
   --                         const b2PolygonShape* polygonB, const b2Transform& xfB);
   --
   -- Moved to 'b2_collide_Polygon' spec.



   --  Compute the collision manifold between an edge and a circle.
   --
   --  void b2CollideEdgeAndCircle(b2Manifold* manifold,
   --                              const b2EdgeShape*   polygonA, const b2Transform& xfA,
   --                              const b2CircleShape* circleB,  const b2Transform& xfB);
   --
   -- Moved to 'b2_collide_Edge' spec.



   --  Compute the collision manifold between an edge and a polygon.
   --
   --  void b2CollideEdgeAndPolygon (b2Manifold* manifold,
   --                               const b2EdgeShape*    edgeA,   const b2Transform& xfA,
   --                               const b2PolygonShape* circleB, const b2Transform& xfB);
   --
   -- Moved to 'b2_collide_Edge' spec.



   --  Clipping for contact manifolds.
   --
   --  int32 b2ClipSegmentToLine (b2ClipVertex vOut[2], const b2ClipVertex vIn[2],
   --                             const b2Vec2& normal, float offset, int32 vertexIndexA);

   type b2ClipVertex_Pair is array (0 .. 1) of b2ClipVertex;

   function b2ClipSegmentToLine (vOut         :    out b2ClipVertex_Pair;
                                 vIn          : in     b2ClipVertex_Pair;
                                 Normal       : in     b2Vec2;
                                 Offset       : in     Real;
                                 vertexIndexA : in     Natural) return Natural;


   --  Determine if two generic shapes overlap.
   --
   --  bool b2TestOverlap (const b2Shape* shapeA, int32 indexA,
   --                      const b2Shape* shapeB, int32 indexB,
   --                      const b2Transform& xfA, const b2Transform& xfB);
   --
   -- Moved to 'b2_collision.overlap' spec.


end b2_Collision;
