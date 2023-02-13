with
     b2_Shape,
     b2_Collision,
     b2_Math,
     b2_Types;


package b2_edge_Shape
is
   use b2_Shape,
       b2_Collision,
       b2_Math,
       b2_Types;



--   A line segment (edge) shape. These can be connected in chains or loops
--   to other edge shapes. Edges created independently are two-sided and do
--   no provide smooth movement across junctions.
--
--  class b2EdgeShape : public b2Shape
--  {
--  public:
--
--     These are the edge vertices
--    b2Vec2 m_vertex1, m_vertex2;
--
--     Optional adjacent vertices. These are used for smooth collision.
--    b2Vec2 m_vertex0, m_vertex3;
--
--     Uses m_vertex0 and m_vertex3 to create smooth collision.
--    bool m_oneSided;
--  };

   type b2edgeShape is new b2Shape with
      record
         m_Vertex1  : aliased b2Vec2;      -- These are the edge vertices.
         m_Vertex2  :         b2Vec2;

         m_Vertex0,
         m_Vertex3  :         b2Vec2;      -- Optional adjacent vertices. These are used for smooth collision.

         m_oneSided :         Boolean;     -- Uses m_vertex0 and m_vertex3 to create smooth collision.
      end record;



   --    b2EdgeShape();
   --

   function to_b2EdgeShape return b2EdgeShape
     with Inline;



   --     Set this as a part of a sequence. Vertex v0 precedes the edge and vertex v3
   --     follows. These extra vertices are used to provide smooth movement
   --     across junctions. This also makes the collision one-sided. The edge
   --     normal points to the right looking from v1 to v2.
   --
   --    void SetOneSided(const b2Vec2& v0, const b2Vec2& v1,const b2Vec2& v2, const b2Vec2& v3);
   --

   procedure setOneSided (Self : in out b2edgeShape;   v0, v1, v2, v3 : in b2Vec2);



   --     Set this as an isolated edge. Collision is two-sided.
   --
   --    void SetTwoSided(const b2Vec2& v1, const b2Vec2& v2);
   --

   procedure setTwoSided (Self : in out b2edgeShape;   v1, v2 : in b2Vec2);



   --     Implement b2Shape.
   --
   --    b2Shape* Clone(b2BlockAllocator* allocator) const override;
   --

   overriding
   function clone (Self : in b2edgeShape) return b2Shape_ptr;



   --     @see b2Shape::GetChildCount
   --
   --    int32 GetChildCount() const override;
   --

   overriding
   function getChildCount (Self : in b2edgeShape) return Natural;



   --     @see b2Shape::TestPoint
   --
   --    bool TestPoint(const b2Transform& transform, const b2Vec2& p) const override;
   --

   overriding
   function testPoint (Self : in b2edgeShape;   Transform : in b2Transform;
                                                p         : in b2Vec2) return Boolean;



   --     Implement b2Shape.
   --
   --    bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
   --             const b2Transform& transform, int32 childIndex) const override;
   --

   overriding
   function rayCast (Self : in b2edgeShape;   Output     :    out b2RayCastOutput;
                                              Input      : in     b2RayCastInput;
                                              Transform  : in     b2Transform;
                                              childIndex : in     Natural) return Boolean;


   --     @see b2Shape::ComputeAABB
   --
   --    void ComputeAABB(b2AABB* aabb, const b2Transform& transform, int32 childIndex) const override;
   --

   overriding
   procedure computeAABB (Self : in b2edgeShape;   aabb       :    out b2AABB;
                                                   Transform  : in     b2Transform;
                                                   childIndex : in     Natural);


   --     @see b2Shape::ComputeMass
   --
   --    void ComputeMass(b2MassData* massData, float density) const override;
   --

   overriding
   procedure computeMass (Self : in b2edgeShape;   massData :    out b2MassData;
                                                   Density  : in     Real);


end b2_edge_Shape;
