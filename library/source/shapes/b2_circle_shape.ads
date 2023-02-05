with
     b2_Shape;


package b2_circle_Shape
is
   procedure dummy;


   --  /// A solid circle shape
--  class b2CircleShape : public b2Shape
--  {
--  public:
--    b2CircleShape();
--
--    /// Implement b2Shape.
--    b2Shape* Clone(b2BlockAllocator* allocator) const override;
--
--    /// @see b2Shape::GetChildCount
--    int32 GetChildCount() const override;
--
--    /// Implement b2Shape.
--    bool TestPoint(const b2Transform& transform, const b2Vec2& p) const override;
--
--    /// Implement b2Shape.
--    /// @note because the circle is solid, rays that start inside do not hit because the normal is
--    /// not defined.
--    bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
--             const b2Transform& transform, int32 childIndex) const override;
--
--    /// @see b2Shape::ComputeAABB
--    void ComputeAABB(b2AABB* aabb, const b2Transform& transform, int32 childIndex) const override;
--
--    /// @see b2Shape::ComputeMass
--    void ComputeMass(b2MassData* massData, float density) const override;
--
--    /// Position
--    b2Vec2 m_p;
--  };

   type b2circleShape is
      record
         null;
      end record;

--
--  inline b2CircleShape::b2CircleShape()
--  {
--    m_type = e_circle;
--    m_radius = 0.0f;
--    m_p.SetZero();
--  }
--
end b2_circle_Shape;
