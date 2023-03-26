with
     box2d.b2_chain_Shape,
     box2d.b2_Common,
     --  b2_Settings,
     ada.unchecked_Conversion;


package body box2d.b2_chain_Shape
is

   --  inline b2chainShape::b2chainShape()
   --  {
   --    m_type = e_chain;
   --    m_radius = b2_polygonRadius;
   --    m_vertices = nullptr;
   --    m_count = 0;
   --  }
   --

   function to_b2chainShape return b2chainShape
   is
      use b2_Common;

      Self : b2chainShape;
   begin
      Self.m_Type     := e_Chain;
      Self.m_Radius   := b2_polygonRadius;
      Self.m_Count    := 0;

      return Self;
   end to_b2chainShape;





   --  b2chainShape::~b2chainShape()
   --  {
   --    Clear();
   --  }
   --

   overriding
   procedure destruct (Self : in out b2chainShape)
   is
   begin
      Self.clear;
   end destruct;





   --    Clear all data.
   --
   --  void b2chainShape::Clear()
   --  {
   --    b2Free(m_vertices);
   --    m_vertices = nullptr;
   --    m_count = 0;
   --  }
   --


   procedure clear (Self : in out b2chainShape)
   is
   begin
      Self.m_count := 0;
   end clear;





   --  void b2chainShape::CreateLoop (const b2Vec2* vertices,
   --                                 int32         count)
   --  {
   --    b2Assert(m_vertices == nullptr && m_count == 0);
   --    b2Assert(count >= 3);
   --    if (count < 3)
   --    {
   --       return;
   --    }
   --
   --    for (int32 i = 1; i < count; ++i)
   --    {
   --       b2Vec2 v1 = vertices[i-1];
   --       b2Vec2 v2 = vertices[i];
   --       // If the code crashes here, it means your vertices are too close together.
   --       b2Assert(b2DistanceSquared(v1, v2) > b2_linearSlop * b2_linearSlop);
   --    }
   --
   --    m_count = count + 1;
   --    m_vertices = (b2Vec2*)b2Alloc(m_count * sizeof(b2Vec2));
   --    memcpy(m_vertices, vertices, count * sizeof(b2Vec2));
   --    m_vertices[count] = m_vertices[0];
   --    m_prevVertex = m_vertices[m_count - 2];
   --    m_nextVertex = m_vertices[1];
   --  }
   --

   procedure createLoop (Self : in out b2chainShape;   Vertices : in b2Vec2s;
                                                       Count    : in int32)
   is
      use type int32;

      pragma assert (Self.m_Count  = 0);
      pragma assert (Count        >= 3);

      C : constant Natural := Natural (Count);

   begin

      if C < 3
      then
         return;
      end if;


      for i in 1 .. C - 1
      loop
         declare
            use b2_Common;

            v1 : constant b2Vec2 := Vertices (i - 1);
            v2 : constant b2Vec2 := Vertices (i);

            -- If the code crashes here, it means your vertices are too close together.
            --
            pragma assert (  b2DistanceSquared (v1, v2)
                           > b2_linearSlop * b2_linearSlop);
         begin
            null;
         end;
      end loop;


      Self.m_Count                 := C + 1;
      Self.m_Vertices (0 .. C - 1) := Vertices;

      Self.m_Vertices (C) := Self.m_Vertices (0);
      Self.m_prevVertex   := Self.m_Vertices (Self.m_Count - 2);
      Self.m_nextVertex   := Self.m_Vertices (1);
   end createLoop;





   --    Create a chain with ghost vertices to connect multiple chains together.
   --
   --    @param vertices an array of vertices, these are copied
   --    @param count the vertex count
   --    @param prevVertex previous vertex from chain that connects to the start
   --    @param nextVertex next vertex from chain that connects to the end
   --
   --  void CreateChain (const b2Vec2* vertices, int32 count,
   --                    const b2Vec2& prevVertex, const b2Vec2& nextVertex);
   --  {
   --    b2Assert(m_vertices == nullptr && m_count == 0);
   --    b2Assert(count >= 2);
   --    for (int32 i = 1; i < count; ++i)
   --    {
   --       // If the code crashes here, it means your vertices are too close together.
   --       b2Assert(b2DistanceSquared(vertices[i-1], vertices[i]) > b2_linearSlop * b2_linearSlop);
   --    }
   --
   --    m_count = count;
   --    m_vertices = (b2Vec2*)b2Alloc(count * sizeof(b2Vec2));
   --    memcpy(m_vertices, vertices, m_count * sizeof(b2Vec2));
   --
   --    m_prevVertex = prevVertex;
   --    m_nextVertex = nextVertex;
   --  }
   --


   procedure createChain (Self : in out b2chainShape;   Vertices   : in b2Vec2s;
                                                        Count      : in Natural;
                                                        prevVertex,
                                                        nextVertex : in b2Vec2)
   is
      use b2_Common;

      C : constant Natural := Count;

      pragma assert (Self.m_Count  = 0);
      pragma assert (C            >= 2);

   begin
      for i in 1 .. C - 1
      loop
         -- If the code crashes here, it means your vertices are too close together.
         --
         pragma assert (  b2DistanceSquared (Vertices (i - 1),
                                             Vertices (i))
                        > b2_linearSlop * b2_linearSlop);
      end loop;

      Self.m_Count                 := C;
      Self.m_Vertices (0 .. C - 1) := Vertices (0 .. C - 1);

      Self.m_prevVertex := prevVertex;
      Self.m_nextVertex := nextVertex;
   end createChain;





   --    Implement b2Shape. Vertices are cloned using b2Alloc.
   --
   --  b2Shape* b2chainShape::Clone(b2BlockAllocator* allocator) const
   --  {
   --    void* mem = allocator->Allocate(sizeof(b2chainShape));
   --    b2chainShape* clone = new (mem) b2chainShape;
   --    clone->CreateChain(m_vertices, m_count, m_prevVertex, m_nextVertex);
   --    return clone;
   --  }
   --

   type b2chainShape_ptr is access all b2chainShape;

   overriding
   function clone (Self : in b2chainShape) return b2Shape_ptr
   is
      Clone : constant b2chainShape_ptr := new b2chainShape;
   begin
      Clone.createChain (Self.m_vertices,
                         Self.m_count,
                         Self.m_prevVertex,
                         Self.m_nextVertex);
     return Clone.all'Access;
   end clone;






   --    @see b2Shape::GetChildCount
   --
   --  int32 b2chainShape::GetChildCount() const
   --  {
   --    // edge count = vertex count - 1
   --    return m_count - 1;
   --  }
   --

   overriding
   function getChildCount (Self : in b2chainShape) return Natural
   is
   begin
      return Self.m_count - 1;     -- Edge count = vertex count - 1.
   end getChildCount;




   --    Get a child edge.
   --
   --  void b2chainShape::GetChildEdge(b2EdgeShape* edge, int32 index) const
   --  {
   --    b2Assert(0 <= index && index < m_count - 1);
   --    edge->m_type = b2Shape::e_edge;
   --    edge->m_radius = m_radius;
   --
   --    edge->m_vertex1 = m_vertices[index + 0];
   --    edge->m_vertex2 = m_vertices[index + 1];
   --    edge->m_oneSided = true;
   --
   --    if (index > 0)
   --    {
   --       edge->m_vertex0 = m_vertices[index - 1];
   --    }
   --    else
   --    {
   --       edge->m_vertex0 = m_prevVertex;
   --    }
   --
   --    if (index < m_count - 2)
   --    {
   --       edge->m_vertex3 = m_vertices[index + 2];
   --    }
   --    else
   --    {
   --       edge->m_vertex3 = m_nextVertex;
   --    }
   --  }
   --

   procedure getChildEdge (Self  : in b2chainShape;   Edge  :    out b2EdgeShape;     --  Get a child edge.
                                                      Index : in     Natural)

   is
      pragma assert (0 <= Index and Index < Self.m_Count - 1);
   begin
      Edge.m_Type   := b2_Shape.e_Edge;
      Edge.m_Radius := Self.m_Radius;

      Edge.m_Vertex1  := Self.m_vertices (Index + 0);
      Edge.m_Vertex2  := Self.m_vertices (Index + 1);
      Edge.m_oneSided := True;


      if index > 0
      then
         Edge.m_Vertex0 := Self.m_vertices (Index - 1);
      else
         Edge.m_Vertex0 := Self.m_prevVertex;
      end if;


      if Index < Self.m_Count - 2
      then
         Edge.m_Vertex3 := Self.m_Vertices (Index + 2);
      else
         Edge.m_Vertex3 := Self.m_nextVertex;
      end if;

   end getChildEdge;





   --    This always return false.
   --
   --    @see b2Shape::TestPoint
   --
   --  bool b2chainShape::TestPoint(const b2Transform& xf, const b2Vec2& p) const
   --  {
   --    B2_NOT_USED(xf);
   --    B2_NOT_USED(p);
   --    return false;
   --  }
   --

   overriding
   function testPoint (Self : in b2chainShape;   Transform : in b2Transform;                     -- @see b2Shape::TestPoint
                                                 p         : in b2Vec2)      return Boolean
   is
      pragma Unreferenced (Transform);
      pragma Unreferenced (p);
   begin
      return False;
   end testPoint;






   --    Implement b2Shape.
   --
   --  bool b2chainShape::RayCast (b2RayCastOutput* output, const b2RayCastInput& input,
   --                              const b2Transform& xf, int32 childIndex) const
   --  {
   --    b2Assert(childIndex < m_count);
   --
   --    b2EdgeShape edgeShape;
   --
   --    int32 i1 = childIndex;
   --    int32 i2 = childIndex + 1;
   --    if (i2 == m_count)
   --    {
   --       i2 = 0;
   --    }
   --
   --    edgeShape.m_vertex1 = m_vertices[i1];
   --    edgeShape.m_vertex2 = m_vertices[i2];
   --
   --    return edgeShape.RayCast(output, input, xf, 0);
   --  }
   --

   --

   overriding
   function raycast (Self : in b2chainShape;   Output     :    out b2RayCastOutput;     -- Implement b2Shape.
                                               Input      : in     b2RayCastInput;
                                               Transform  : in     b2Transform;
                                               childIndex : in     Natural)         return Boolean
   is
      pragma assert (childIndex < Self.m_Count);

      edgeShape : b2EdgeShape;

      i1 : constant Natural := childIndex;
      i2 :          Natural := childIndex + 1;

   begin
      if i2 = Self.m_Count
      then
         i2 := 0;
      end if;

      edgeShape.m_vertex1 := Self.m_Vertices (i1);
      edgeShape.m_vertex2 := Self.m_Vertices (i2);

      return edgeShape.raycast (Output, Input, Transform, 0);
   end rayCast;





   --    @see b2Shape::ComputeAABB
   --
   --    Chains have zero mass.
   --
   --    @see b2Shape::ComputeMass
   --
   --  void b2chainShape::ComputeAABB (b2AABB* aabb, const b2Transform& xf, int32 childIndex) const
   --  {
   --    b2Assert(childIndex < m_count);
   --
   --    int32 i1 = childIndex;
   --    int32 i2 = childIndex + 1;
   --    if (i2 == m_count)
   --    {
   --       i2 = 0;
   --    }
   --
   --    b2Vec2 v1 = b2Mul(xf, m_vertices[i1]);
   --    b2Vec2 v2 = b2Mul(xf, m_vertices[i2]);
   --
   --    b2Vec2 lower = b2Min(v1, v2);
   --    b2Vec2 upper = b2Max(v1, v2);
   --
   --    b2Vec2 r(m_radius, m_radius);
   --    aabb->lowerBound = lower - r;
   --    aabb->upperBound = upper + r;
   --  }
   --

   overriding
   procedure computeAABB (Self : in b2chainShape;   aabb       :    out b2AABB;     --  @see b2Shape::ComputeAABB.
                                                    Transform  : in     b2Transform;
                                                    childIndex : in     Natural)
   is
      pragma assert (childIndex < Self.m_Count);

      i1 : constant Natural := childIndex;
      i2 : Natural := childIndex + 1;
   begin
      if i2 = Self.m_Count
      then
         i2 := 0;
      end if;

      declare
         v1    : constant b2Vec2 := b2Mul (Transform, Self.m_Vertices (i1));
         v2    : constant b2Vec2 := b2Mul (Transform, Self.m_Vertices (i2));

         Lower : constant b2Vec2 := b2_Math.b2Min (v1, v2);
         Upper : constant b2Vec2 := b2_Math.b2Max (v1, v2);

         r     : constant b2Vec2 := (Self.m_Radius, Self.m_Radius);
      begin
         aabb.lowerBound := Lower - r;
         aabb.upperBound := Upper + r;
      end;
   end computeAABB;





   --  void b2chainShape::ComputeMass (b2MassData* massData, float density) const
   --  {
   --    B2_NOT_USED(density);
   --
   --    massData->mass = 0.0f;
   --    massData->center.SetZero();
   --    massData->I = 0.0f;
   --  }
   --

   overriding
   procedure computeMass (Self : in b2chainShape;   massData :    out b2MassData;
                                                    Density  : in     Real)
   is
      pragma Unreferenced (Density);
   begin
      massData.Mass := 0.0;
      massData.I    := 0.0;

      setZero (massData.Center);
   end computeMass;



end box2d.b2_chain_Shape;
