with
     b2_Shape,
     b2_Collision,
     b2_Math,
     b2_Types,
     b2_Settings;


package b2_polygon_Shape
--
--  A solid convex polygon. It is assumed that the interior of the polygon is to
--  the left of each edge.
--  Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
--  In most cases you should not need many vertices for a convex polygon.
--
is
   use b2_Shape,
       b2_Collision,
       b2_Math,
       b2_Types,
       b2_Settings;


   --  class b2PolygonShape : public b2Shape
   --  {
   --  public:
   --    b2Vec2 m_centroid;
   --    b2Vec2 m_vertices[b2_maxPolygonVertices];
   --    b2Vec2 m_normals[b2_maxPolygonVertices];
   --    int32 m_count;
   --  };

   type b2polygonShape is new b2Shape with
      record
         m_Centroid : b2Vec2;
         m_Vertices : b2Vec2s (0 .. b2_maxPolygonVertices - 1);
         m_Normals  : b2Vec2s (0 .. b2_maxPolygonVertices - 1);
         m_Count    : Natural;
      end record;



   --    b2PolygonShape();
   --

   function to_b2polygonShape return b2polygonShape
     with Inline;



   -- Implement b2Shape.
   --
   -- b2Shape* Clone(b2BlockAllocator* allocator) const override;
   --

   overriding
   function clone (Self : in b2polygonShape) return b2Shape_ptr;



   -- @see b2Shape::GetChildCount.
   --
   --   int32 GetChildCount() const override;
   --

   overriding
   function getChildCount (Self : in b2polygonShape) return Natural;



   --    @see b2Shape::TestPoint
   --
   --    bool TestPoint (const b2Transform& transform, const b2Vec2& p) const override;
   --

   overriding
   function testPoint (Self : in b2polygonShape;   Transform : in b2Transform;
                                                   p         : in b2Vec2) return Boolean;



   --    Implement b2Shape.
   --
   --    @note because the polygon is solid, rays that start inside do not hit because the normal is
   --    not defined.
   --
   --    bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
   --                 const b2Transform& transform, int32 childIndex) const override;
   --

   overriding
   function raycast (Self : in b2polygonShape;   Output     :    out b2RayCastOutput;
                                                 Input      : in     b2RayCastInput;
                                                 Transform  : in     b2Transform;
                                                 childIndex : in     Natural)        return Boolean;



   --    @see b2Shape::ComputeAABB
   --
   --    void ComputeAABB(b2AABB* aabb, const b2Transform& transform, int32 childIndex) const override;
   --

   overriding
   procedure computeAABB (Self : in b2polygonShape;   aabb       :    out b2AABB;
                                                      Transform  : in     b2Transform;
                                                      childIndex : in     Natural);



   --    @see b2Shape::ComputeMass
   --
   --    void ComputeMass (b2MassData* massData, float density) const override;
   --

   overriding
   procedure computeMass (Self : in b2polygonShape;   massData :    out b2MassData;
                                                      Density  : in     b2_Math.Real);



   --    Create a convex hull from the given array of local points.
   --    The count must be in the range [3, b2_maxPolygonVertices].
   --    @warning the points may be re-ordered, even if they form a convex polygon
   --    @warning collinear points are handled but not removed. Collinear points
   --    may lead to poor stacking behavior.
   --
   --    void Set (const b2Vec2* points, int32 count);
   --

   procedure set (Self : in out b2polygonShape;   Vertices : in b2Vec2s;
                                                  Count    : in Natural);



   --    Build vertices to represent an axis-aligned box centered on the local origin.
   --
   --    @param hx the half-width.
   --    @param hy the half-height.
   --
   --    void SetAsBox (float hx, float hy);
   --

   procedure setAsBox (Self : in out b2polygonShape;   hx, hy : in b2_Math.Real);




   --    Build vertices to represent an oriented box.
   --
   --    @param hx the half-width.
   --    @param hy the half-height.
   --    @param center the center of the box in local coordinates.
   --    @param angle the rotation of the box in local coordinates.
   --
   --    void SetAsBox(float hx, float hy, const b2Vec2& center, float angle);
   --

   procedure setAsBox (Self : in out b2polygonShape;   hx, hy : in b2_Math.Real;
                                                       Center : in b2Vec2;
                                                       Angle  : in b2_Math.Real);



   --    Validate convexity. This is a very time consuming operation.
   --
   --    @returns true if valid
   --
   --    bool Validate() const;
   --

   function validate (Self : in b2polygonShape) return Boolean;



end b2_polygon_Shape;
