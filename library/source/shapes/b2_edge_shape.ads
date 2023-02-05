with
     b2_Shape;


package b2_edge_Shape
is
   procedure dummy;


   --
--  /// A line segment (edge) shape. These can be connected in chains or loops
--  /// to other edge shapes. Edges created independently are two-sided and do
--  /// no provide smooth movement across junctions.
--  class b2EdgeShape : public b2Shape
--  {
--  public:
--    b2EdgeShape();
--
--    /// Set this as a part of a sequence. Vertex v0 precedes the edge and vertex v3
--    /// follows. These extra vertices are used to provide smooth movement
--    /// across junctions. This also makes the collision one-sided. The edge
--    /// normal points to the right looking from v1 to v2.
--    void SetOneSided(const b2Vec2& v0, const b2Vec2& v1,const b2Vec2& v2, const b2Vec2& v3);
--
--    /// Set this as an isolated edge. Collision is two-sided.
--    void SetTwoSided(const b2Vec2& v1, const b2Vec2& v2);
--
--    /// Implement b2Shape.
--    b2Shape* Clone(b2BlockAllocator* allocator) const override;
--
--    /// @see b2Shape::GetChildCount
--    int32 GetChildCount() const override;
--
--    /// @see b2Shape::TestPoint
--    bool TestPoint(const b2Transform& transform, const b2Vec2& p) const override;
--
--    /// Implement b2Shape.
--    bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
--             const b2Transform& transform, int32 childIndex) const override;
--
--    /// @see b2Shape::ComputeAABB
--    void ComputeAABB(b2AABB* aabb, const b2Transform& transform, int32 childIndex) const override;
--
--    /// @see b2Shape::ComputeMass
--    void ComputeMass(b2MassData* massData, float density) const override;
--
--    /// These are the edge vertices
--    b2Vec2 m_vertex1, m_vertex2;
--
--    /// Optional adjacent vertices. These are used for smooth collision.
--    b2Vec2 m_vertex0, m_vertex3;
--
--    /// Uses m_vertex0 and m_vertex3 to create smooth collision.
--    bool m_oneSided;
--  };


   type b2edgeShape is
      record
         null;
      end record;


--
--  inline b2EdgeShape::b2EdgeShape()
--  {
--    m_type = e_edge;
--    m_radius = b2_polygonRadius;
--    m_vertex0.x = 0.0f;
--    m_vertex0.y = 0.0f;
--    m_vertex3.x = 0.0f;
--    m_vertex3.y = 0.0f;
--    m_oneSided = false;
--  }
--
end b2_edge_Shape;
