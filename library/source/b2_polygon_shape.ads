--  #include "b2_api.h"
--  #include "b2_shape.h"
--
--  /// A solid convex polygon. It is assumed that the interior of the polygon is to
--  /// the left of each edge.
--  /// Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
--  /// In most cases you should not need many vertices for a convex polygon.
--  class B2_API b2PolygonShape : public b2Shape
--  {
--  public:
--  	b2PolygonShape();
--
--  	/// Implement b2Shape.
--  	b2Shape* Clone(b2BlockAllocator* allocator) const override;
--
--  	/// @see b2Shape::GetChildCount
--  	int32 GetChildCount() const override;
--
--  	/// Create a convex hull from the given array of local points.
--  	/// The count must be in the range [3, b2_maxPolygonVertices].
--  	/// @warning the points may be re-ordered, even if they form a convex polygon
--  	/// @warning collinear points are handled but not removed. Collinear points
--  	/// may lead to poor stacking behavior.
--  	void Set(const b2Vec2* points, int32 count);
--
--  	/// Build vertices to represent an axis-aligned box centered on the local origin.
--  	/// @param hx the half-width.
--  	/// @param hy the half-height.
--  	void SetAsBox(float hx, float hy);
--
--  	/// Build vertices to represent an oriented box.
--  	/// @param hx the half-width.
--  	/// @param hy the half-height.
--  	/// @param center the center of the box in local coordinates.
--  	/// @param angle the rotation of the box in local coordinates.
--  	void SetAsBox(float hx, float hy, const b2Vec2& center, float angle);
--
--  	/// @see b2Shape::TestPoint
--  	bool TestPoint(const b2Transform& transform, const b2Vec2& p) const override;
--
--  	/// Implement b2Shape.
--  	/// @note because the polygon is solid, rays that start inside do not hit because the normal is
--  	/// not defined.
--  	bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
--  					const b2Transform& transform, int32 childIndex) const override;
--
--  	/// @see b2Shape::ComputeAABB
--  	void ComputeAABB(b2AABB* aabb, const b2Transform& transform, int32 childIndex) const override;
--
--  	/// @see b2Shape::ComputeMass
--  	void ComputeMass(b2MassData* massData, float density) const override;
--
--  	/// Validate convexity. This is a very time consuming operation.
--  	/// @returns true if valid
--  	bool Validate() const;
--
--  	b2Vec2 m_centroid;
--  	b2Vec2 m_vertices[b2_maxPolygonVertices];
--  	b2Vec2 m_normals[b2_maxPolygonVertices];
--  	int32 m_count;
--  };
--
--  inline b2PolygonShape::b2PolygonShape()
--  {
--  	m_type = e_polygon;
--  	m_radius = b2_polygonRadius;
--  	m_count = 0;
--  	m_centroid.SetZero();
--  }
--