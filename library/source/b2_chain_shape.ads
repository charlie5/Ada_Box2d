with
     b2_Shape;


package b2_chain_Shape
is
   procedure dummy;


   --
--  class b2EdgeShape;
--
--  /// A chain shape is a free form sequence of line segments.
--  /// The chain has one-sided collision, with the surface normal pointing to the right of the edge.
--  /// This provides a counter-clockwise winding like the polygon shape.
--  /// Connectivity information is used to create smooth collisions.
--  /// @warning the chain will not collide properly if there are self-intersections.
--  class B2_API b2ChainShape : public b2Shape
--  {
--  public:
--    b2ChainShape();
--
--    /// The destructor frees the vertices using b2Free.
--    ~b2ChainShape();
--
--    /// Clear all data.
--    void Clear();
--
--    /// Create a loop. This automatically adjusts connectivity.
--    /// @param vertices an array of vertices, these are copied
--    /// @param count the vertex count
--    void CreateLoop(const b2Vec2* vertices, int32 count);
--
--    /// Create a chain with ghost vertices to connect multiple chains together.
--    /// @param vertices an array of vertices, these are copied
--    /// @param count the vertex count
--    /// @param prevVertex previous vertex from chain that connects to the start
--    /// @param nextVertex next vertex from chain that connects to the end
--    void CreateChain(const b2Vec2* vertices, int32 count,
--       const b2Vec2& prevVertex, const b2Vec2& nextVertex);
--
--    /// Implement b2Shape. Vertices are cloned using b2Alloc.
--    b2Shape* Clone(b2BlockAllocator* allocator) const override;
--
--    /// @see b2Shape::GetChildCount
--    int32 GetChildCount() const override;
--
--    /// Get a child edge.
--    void GetChildEdge(b2EdgeShape* edge, int32 index) const;
--
--    /// This always return false.
--    /// @see b2Shape::TestPoint
--    bool TestPoint(const b2Transform& transform, const b2Vec2& p) const override;
--
--    /// Implement b2Shape.
--    bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
--                const b2Transform& transform, int32 childIndex) const override;
--
--    /// @see b2Shape::ComputeAABB
--    void ComputeAABB(b2AABB* aabb, const b2Transform& transform, int32 childIndex) const override;
--
--    /// Chains have zero mass.
--    /// @see b2Shape::ComputeMass
--    void ComputeMass(b2MassData* massData, float density) const override;
--
--    /// The vertices. Owned by this class.
--    b2Vec2* m_vertices;
--
--    /// The vertex count.
--    int32 m_count;
--
--    b2Vec2 m_prevVertex, m_nextVertex;
--  };
--
--  inline b2ChainShape::b2ChainShape()
--  {
--    m_type = e_chain;
--    m_radius = b2_polygonRadius;
--    m_vertices = nullptr;
--    m_count = 0;
--  }
--
end b2_chain_Shape;
