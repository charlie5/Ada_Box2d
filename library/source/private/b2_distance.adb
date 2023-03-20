with
     b2_circle_Shape,
     b2_edge_Shape,
     b2_chain_Shape,
     b2_polygon_Shape,
     b2_Common,
     interfaces.C.Pointers;


package body b2_Distance
is
   use b2_circle_Shape,
       b2_edge_Shape,
       b2_polygon_Shape,
       b2_chain_Shape;

   use type int8;


   --  GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.
   --

   --  int32 b2_gjkCalls, b2_gjkIters, b2_gjkMaxIters;
   --
   b2_gjkCalls,
   b2_gjkIters,
   b2_gjkMaxIters : Natural := 0;


   --  void b2DistanceProxy::Set(const b2Shape* shape, int32 index)
   --  {
   --    switch (shape->GetType())
   --    {
   --    case b2Shape::e_circle:
   --       {
   --          const b2CircleShape* circle = static_cast<const b2CircleShape*>(shape);
   --          m_vertices = &circle->m_p;
   --          m_count = 1;
   --          m_radius = circle->m_radius;
   --       }
   --       break;
   --
   --    case b2Shape::e_polygon:
   --       {
   --          const b2PolygonShape* polygon = static_cast<const b2PolygonShape*>(shape);
   --          m_vertices = polygon->m_vertices;
   --          m_count = polygon->m_count;
   --          m_radius = polygon->m_radius;
   --       }
   --       break;
   --
   --    case b2Shape::e_chain:
   --       {
   --          const b2ChainShape* chain = static_cast<const b2ChainShape*>(shape);
   --          b2Assert(0 <= index && index < chain->m_count);
   --
   --          m_buffer[0] = chain->m_vertices[index];
   --          if (index + 1 < chain->m_count)
   --          {
   --             m_buffer[1] = chain->m_vertices[index + 1];
   --          }
   --          else
   --          {
   --             m_buffer[1] = chain->m_vertices[0];
   --          }
   --
   --          m_vertices = m_buffer;
   --          m_count = 2;
   --          m_radius = chain->m_radius;
   --       }
   --       break;
   --
   --    case b2Shape::e_edge:
   --       {
   --          const b2EdgeShape* edge = static_cast<const b2EdgeShape*>(shape);
   --          m_vertices = &edge->m_vertex1;
   --          m_count = 2;
   --          m_radius = edge->m_radius;
   --       }
   --       break;
   --
   --    default:
   --       b2Assert(false);
   --    }
   --  }
   --

   procedure set (Self : access b2DistanceProxy;   Shape : in b2Shape_ptr;
                                                   Index : in Natural)
   is
   begin
      case Shape.getType
      is
         when e_Circle =>
            declare
               Circle : b2CircleShape renames b2circleShape (Shape.all);
            begin
               Self.m_Vertices := Circle.m_p'Access;
               Self.m_Count    := 1;
               Self.m_Radius   := Circle.m_Radius;
            end;

         when e_Polygon =>
            declare
               Polygon : b2PolygonShape renames b2polygonShape (Shape.all);
            begin
               Self.m_Vertices := Polygon.m_Vertices (Polygon.m_Vertices'First)'Access;
               Self.m_Count    := Polygon.m_Count;
               Self.m_Radius   := Polygon.m_Radius;
            end;

         when e_Chain =>
            declare
               Chain : b2ChainShape renames b2chainShape (Shape.all);
            begin
               pragma assert (        0 <= Index
                              and Index < Chain.m_Count);

               Self.m_buffer (0) := Chain.m_Vertices (Index);

               if Index + 1 < Chain.m_Count
               then
                  Self.m_Buffer (1) := Chain.m_Vertices (Index + 1);
               else
                  Self.m_Buffer (1) := Chain.m_Vertices (0);
               end if;

               Self.m_Vertices := Self.m_Buffer (Self.m_Buffer'First)'Access;
               Self.m_Count    := 2;
               Self.m_Radius   := Chain.m_Radius;
          end;

         when e_Edge =>
            declare
               Edge : b2EdgeShape renames b2edgeShape (Shape.all);
            begin
               Self.m_Vertices := Edge.m_Vertex1'Access;
               Self.m_Count    := 2;
               Self.m_Radius   := Edge.m_Radius;
            end;

       when others =>
          pragma assert (False);
      end case;
   end set;




   --  void b2DistanceProxy::Set(const b2Vec2* vertices, int32 count, float radius)
   --  {
   --      m_vertices = vertices;
   --      m_count = count;
   --      m_radius = radius;
   --  }
   --

   procedure set (Self : in out b2DistanceProxy;   Vertices : in b2Vec2_ptr;
                                                   Count    : in Positive;
                                                   Radius   : in Real)
   is
   begin
      Self.m_Vertices := Vertices;
      Self.m_Count    := Count;
      Self.m_Radius   := Radius;
   end set;




   --  inline int32 b2DistanceProxy::GetSupport(const b2Vec2& d) const
   --  {
   --    int32 bestIndex = 0;
   --    float bestValue = b2Dot(m_vertices[0], d);
   --
   --    for (int32 i = 1; i < m_count; ++i)
   --    {
   --       float value = b2Dot(m_vertices[i], d);
   --       if (value > bestValue)
   --       {
   --          bestIndex = i;
   --          bestValue = value;
   --       }
   --    }
   --
   --    return bestIndex;
   --  }
   --

   function getSupport (Self : in b2DistanceProxy;   d : in b2Vec2) return Natural
   is
      use b2_Pointers.b2Vec2_Pointers,
          interfaces.C;

      my_Vertices_ptr : constant b2Vec2_ptr   := Self.m_Vertices.all'Access;
      my_Vertices     : constant b2Vec2_array := Value (my_Vertices_ptr,
                                                        ptrdiff_t (Self.m_Count));
      bestIndex : Natural      := 0;
      bestValue : Real := b2Dot (my_Vertices (0),
                                         d);
   begin
      for i in 1 .. Self.m_Count - 1
      loop
         declare
            Value : constant Real := b2Dot (my_Vertices (i),
                                                    d);
         begin
            if Value > bestValue
            then
               bestIndex := i;
               bestValue := Value;
            end if;
         end;
      end loop;

      return bestIndex;
   end getSupport;




   --  inline const b2Vec2& b2DistanceProxy::GetSupportVertex(const b2Vec2& d) const
   --  {
   --    int32 bestIndex = 0;
   --    float bestValue = b2Dot(m_vertices[0], d);
   --    for (int32 i = 1; i < m_count; ++i)
   --    {
   --       float value = b2Dot(m_vertices[i], d);
   --       if (value > bestValue)
   --       {
   --          bestIndex = i;
   --          bestValue = value;
   --       }
   --    }
   --
   --    return m_vertices[bestIndex];
   --  }

   function getSupportVertex (Self : in b2DistanceProxy;   d : in b2Vec2) return b2Vec2
   is
      use b2_Pointers.b2Vec2_Pointers,
          interfaces.C;

      my_Vertices_ptr : constant b2Vec2_ptr   := Self.m_Vertices.all'Access;
      my_Vertices     : constant b2Vec2_array := Value (my_Vertices_ptr,
                                                        ptrdiff_t (Self.m_Count));
      Value     : Real;
      bestIndex : Natural      := 0;
      bestValue : Real := b2Dot (my_Vertices (0),
                                         d);
   begin
      for i in 1 .. Self.m_Count - 1
      loop
         Value := b2Dot (my_Vertices (i),
                         d);

        if Value > bestValue
         then
            bestIndex := i;
            bestValue := Value;
         end if;
      end loop;

     return my_Vertices (bestIndex);
   end getSupportVertex;




   --  inline int32 b2DistanceProxy::GetVertexCount() const
   --  {
   --    return m_count;
   --  }
   --

   function getVertexCount (Self : in b2DistanceProxy) return Positive
   is
   begin
      return Self.m_Count;
   end getVertexCount;





   --  inline const b2Vec2& b2DistanceProxy::GetVertex(int32 index) const
   --  {
   --    b2Assert(0 <= index && index < m_count);
   --    return m_vertices[index];
   --  }
   --

   function getVertex (Self : in b2DistanceProxy;   Index : in Natural) return b2Vec2
   is
      pragma assert (    0     <= Index
                     and Index <  Self.m_count);

      use b2_Pointers.b2Vec2_Pointers,
          interfaces.C;

      my_Vertices_ptr : constant b2Vec2_ptr   := Self.m_Vertices.all'Access;
      my_Vertices     : constant b2Vec2_array := Value (my_Vertices_ptr,
                                                        ptrdiff_t (Self.m_Count));
   begin
     return my_Vertices (Index);
   end getVertex;





   --  struct b2SimplexVertex
   --  {
   --    b2Vec2 wA;     support point in proxyA
   --    b2Vec2 wB;     support point in proxyB
   --    b2Vec2 w;      wB - wA
   --    float a;    barycentric coordinate for closest point
   --    int32 indexA;  wA index
   --    int32 indexB;  wB index
   --  };
   --

   type b2simplexVertex is
      record
           wA     : b2Vec2;      -- Support point in proxyA.
           wB     : b2Vec2;      -- Support point in proxyB.
           w      : b2Vec2;      -- wB - wA
           a      : Real;        -- Barycentric coordinate for closest point.
           indexA : Natural;     -- wA index.
           indexB : Natural;     -- wB index.
      end record;

   type b2simplexVertices is array (0 .. 2) of b2SimplexVertex;


   --  struct b2Simplex
   --  {
   --    b2SimplexVertex m_v1, m_v2, m_v3;
   --    int32 m_count;
   --  };
   --

   type b2Simplex is
      record
         --  m_v1,
         --  m_v2,
         --  m_v3    : b2SimplexVertex;

         m_Vertices : b2simplexVertices;

         m_Count : Natural;
      end record;



   --    float GetMetric() const
   --    {
   --       switch (m_count)
   --       {
   --       case 0:
   --          b2Assert(false);
   --          return 0.0f;
   --
   --       case 1:
   --          return 0.0f;
   --
   --       case 2:
   --          return b2Distance(m_v1.w, m_v2.w);
   --
   --       case 3:
   --          return b2Cross(m_v2.w - m_v1.w, m_v3.w - m_v1.w);
   --
   --       default:
   --          b2Assert(false);
   --          return 0.0f;
   --       }
   --    }
   --

   function getMetric (Self : in b2Simplex) return Real
   is
      m_v1 : b2SimplexVertex renames Self.m_Vertices (0);
      m_v2 : b2SimplexVertex renames Self.m_Vertices (1);
      m_v3 : b2SimplexVertex renames Self.m_Vertices (2);
   begin
      case Self.m_Count
      is
         when 0 =>
            pragma assert (False);
            return 0.0;

         when 1 =>
            return 0.0;

         when 2 =>
            return b2Distance (m_v1.w,
                               m_v2.w);

         when 3 =>
            return b2Cross (m_v2.w - m_v1.w,
                            m_v3.w - m_v1.w);

         when others =>
            pragma assert (False);
            return 0.0;
      end case;
   end getMetric;




   --    void b2Simplex::ReadCache (const b2SimplexCache* cache,
   --                               const b2DistanceProxy* proxyA, const b2Transform& transformA,
   --                               const b2DistanceProxy* proxyB, const b2Transform& transformB)
   --    {
   --       b2Assert(cache->count <= 3);
   --
   --       Copy data from cache.
   --       m_count = cache->count;
   --       b2SimplexVertex* vertices = &m_v1;
   --       for (int32 i = 0; i < m_count; ++i)
   --       {
   --          b2SimplexVertex* v = vertices + i;
   --          v->indexA = cache->indexA[i];
   --          v->indexB = cache->indexB[i];
   --          b2Vec2 wALocal = proxyA->GetVertex(v->indexA);
   --          b2Vec2 wBLocal = proxyB->GetVertex(v->indexB);
   --          v->wA = b2Mul(transformA, wALocal);
   --          v->wB = b2Mul(transformB, wBLocal);
   --          v->w = v->wB - v->wA;
   --          v->a = 0.0f;
   --       }
   --
   --       Compute the new simplex metric, if it is substantially different than
   --       old metric then flush the simplex.
   --       if (m_count > 1)
   --       {
   --          float metric1 = cache->metric;
   --          float metric2 = GetMetric();
   --          if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < b2_epsilon)
   --          {
   --             Reset the simplex.
   --             m_count = 0;
   --          }
   --       }
   --
   --       If the cache is empty or invalid ...
   --       if (m_count == 0)
   --       {
   --          b2SimplexVertex* v = vertices + 0;
   --          v->indexA = 0;
   --          v->indexB = 0;
   --          b2Vec2 wALocal = proxyA->GetVertex(0);
   --          b2Vec2 wBLocal = proxyB->GetVertex(0);
   --          v->wA = b2Mul(transformA, wALocal);
   --          v->wB = b2Mul(transformB, wBLocal);
   --          v->w = v->wB - v->wA;
   --          v->a = 1.0f;
   --          m_count = 1;
   --       }
   --    }
   --

   procedure readCache (Self : in out b2Simplex;   Cache  : in b2SimplexCache;
                                                   proxyA : in b2DistanceProxy;   transformA : in b2Transform;
                                                   proxyB : in b2DistanceProxy;   transformB : in b2Transform)
   is
      use b2_Common;

      pragma assert (Cache.Count <= 3);


      --  b2SimplexVertex* vertices = &m_v1;
      Vertices : b2simplexVertices renames Self.m_Vertices;

   begin
      Self.m_Count := Cache.Count;

      -- Copy data from cache.
      --
      for i in 0 .. Self.m_Count - 1
      loop
         declare
            --  b2SimplexVertex* v = vertices + i;
            v : b2SimplexVertex renames Vertices (i);

            wALocal,
            wBLocal : b2Vec2;
         begin
            v.indexA := Cache.indexA (i);
            v.indexB := Cache.indexB (i);

            wALocal  := getVertex (proxyA, v.indexA);
            wBLocal  := getVertex (proxyB, v.indexB);

            v.wA := b2Mul (transformA, wALocal);
            v.wB := b2Mul (transformB, wBLocal);
            v.w  := v.wB - v.wA;
            v.a  := 0.0;
         end;
      end loop;

        -- Compute the new simplex metric, if it is substantially different than
        -- old metric then flush the simplex.
        --
      if Self.m_count > 1
      then
         declare
            Metric1 : constant Real := Cache.Metric;
            Metric2 : constant Real := getMetric (Self);
         begin
            if         Metric2 < Metric1 * 0.5
              or 2.0 * Metric1 < Metric2
              or       Metric2 < b2_Epsilon
            then
               Self.m_Count := 0;     -- Reset the simplex.
            end if;
         end;
      end if;

      -- If the cache is empty or invalid ...
      --
      if Self.m_Count = 0
      then
         declare
            --  b2SimplexVertex* v = vertices + 0;
            v : b2SimplexVertex renames Vertices (0);

            wALocal : b2Vec2;
            wBLocal : b2Vec2;
         begin
            v.indexA := 0;
            v.indexB := 0;

            wALocal  := getVertex (proxyA, 0);
            wBLocal  := getVertex (proxyB, 0);

            v.wA := b2Mul (transformA, wALocal);
            v.wB := b2Mul (transformB, wBLocal);

            v.w  := v.wB - v.wA;
            v.a  := 1.0;

            Self.m_Count := 1;
         end;
      end if;
   end readCache;





   --  Solve a line segment using barycentric coordinates.
   --
   --  p = a1 * w1 + a2 * w2
   --  a1 + a2 = 1
   --
   --  The vector from the origin to the closest point on the line is
   --  perpendicular to the line.
   --  e12 = w2 - w1
   --  dot(p, e) = 0
   --  a1 * dot(w1, e) + a2 * dot(w2, e) = 0
   --
   --  2-by-2 linear system
   --  [1      1     ][a1] = [1]
   --  [w1.e12 w2.e12][a2] = [0]
   --
   --  Define
   --  d12_1 =  dot(w2, e12)
   --  d12_2 = -dot(w1, e12)
   --  d12 = d12_1 + d12_2
   --
   --  Solution
   --  a1 = d12_1 / d12
   --  a2 = d12_2 / d12
   --
   --
   --  void b2Simplex::Solve2()
   --  {
   --    b2Vec2 w1 = m_v1.w;
   --    b2Vec2 w2 = m_v2.w;
   --    b2Vec2 e12 = w2 - w1;
   --
   --    w1 region
   --    float d12_2 = -b2Dot(w1, e12);
   --    if (d12_2 <= 0.0f)
   --    {
   --       a2 <= 0, so we clamp it to 0
   --       m_v1.a = 1.0f;
   --       m_count = 1;
   --       return;
   --    }
   --
   --    w2 region
   --    float d12_1 = b2Dot(w2, e12);
   --    if (d12_1 <= 0.0f)
   --    {
   --       a1 <= 0, so we clamp it to 0
   --       m_v2.a = 1.0f;
   --       m_count = 1;
   --       m_v1 = m_v2;
   --       return;
   --    }
   --
   --    Must be in e12 region.
   --    float inv_d12 = 1.0f / (d12_1 + d12_2);
   --    m_v1.a = d12_1 * inv_d12;
   --    m_v2.a = d12_2 * inv_d12;
   --    m_count = 2;
   --  }
   --

   procedure solve2 (Self : in out b2Simplex)
   is
      m_v1 : b2SimplexVertex renames Self.m_Vertices (0);
      m_v2 : b2SimplexVertex renames Self.m_Vertices (1);
      m_v3 : b2SimplexVertex renames Self.m_Vertices (2);

      w1  : constant b2Vec2  := m_v1.w;
      w2  : constant b2Vec2  := m_v2.w;
      e12 : constant b2Vec2  := w2 - w1;

      -- w1 region
      --
      d12_1 :          Real;
      d12_2 : constant Real := -b2Dot (w1, e12);

   begin
      if d12_2 <= 0.0
      then                        -- a2 <= 0, so we clamp it to 0.
         m_v1.a       := 1.0;
         Self.m_count := 1;
         return;
      end if;

      -- w2 region
      --
      d12_1 := b2Dot (w2, e12);

      if d12_1 <= 0.0
         then                     -- a1 <= 0, so we clamp it to 0.
            m_v2.a       := 1.0;
            Self.m_count := 1;
            m_v1         := m_v2;

            return;
         end if;

      -- Must be in e12 region.
      --
      declare
         inv_d12 : constant Real := 1.0 / (d12_1 + d12_2);
      begin
         m_v1.a       := d12_1 * inv_d12;
         m_v2.a       := d12_2 * inv_d12;
         Self.m_Count := 2;
      end;
   end solve2;




   --  Possible regions:
   --  - points[2]
   --  - edge points[0]-points[2]
   --  - edge points[1]-points[2]
   --  - inside the triangle
   --
   --  void b2Simplex::Solve3()
   --  {
   --    b2Vec2 w1 = m_v1.w;
   --    b2Vec2 w2 = m_v2.w;
   --    b2Vec2 w3 = m_v3.w;
   --
   --    Edge12
   --    [1      1     ][a1] = [1]
   --    [w1.e12 w2.e12][a2] = [0]
   --    a3 = 0
   --    b2Vec2 e12 = w2 - w1;
   --    float w1e12 = b2Dot(w1, e12);
   --    float w2e12 = b2Dot(w2, e12);
   --    float d12_1 = w2e12;
   --    float d12_2 = -w1e12;
   --
   --    Edge13
   --    [1      1     ][a1] = [1]
   --    [w1.e13 w3.e13][a3] = [0]
   --    a2 = 0
   --    b2Vec2 e13 = w3 - w1;
   --    float w1e13 = b2Dot(w1, e13);
   --    float w3e13 = b2Dot(w3, e13);
   --    float d13_1 = w3e13;
   --    float d13_2 = -w1e13;
   --
   --    Edge23
   --    [1      1     ][a2] = [1]
   --    [w2.e23 w3.e23][a3] = [0]
   --    a1 = 0
   --    b2Vec2 e23 = w3 - w2;
   --    float w2e23 = b2Dot(w2, e23);
   --    float w3e23 = b2Dot(w3, e23);
   --    float d23_1 = w3e23;
   --    float d23_2 = -w2e23;
   --
   --    Triangle123
   --    float n123 = b2Cross(e12, e13);
   --
   --    float d123_1 = n123 * b2Cross(w2, w3);
   --    float d123_2 = n123 * b2Cross(w3, w1);
   --    float d123_3 = n123 * b2Cross(w1, w2);
   --
   --    w1 region
   --    if (d12_2 <= 0.0f && d13_2 <= 0.0f)
   --    {
   --       m_v1.a = 1.0f;
   --       m_count = 1;
   --       return;
   --    }
   --
   --    e12
   --    if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f)
   --    {
   --       float inv_d12 = 1.0f / (d12_1 + d12_2);
   --       m_v1.a = d12_1 * inv_d12;
   --       m_v2.a = d12_2 * inv_d12;
   --       m_count = 2;
   --       return;
   --    }
   --
   --    e13
   --    if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f)
   --    {
   --       float inv_d13 = 1.0f / (d13_1 + d13_2);
   --       m_v1.a = d13_1 * inv_d13;
   --       m_v3.a = d13_2 * inv_d13;
   --       m_count = 2;
   --       m_v2 = m_v3;
   --       return;
   --    }
   --
   --    w2 region
   --    if (d12_1 <= 0.0f && d23_2 <= 0.0f)
   --    {
   --       m_v2.a = 1.0f;
   --       m_count = 1;
   --       m_v1 = m_v2;
   --       return;
   --    }
   --
   --    w3 region
   --    if (d13_1 <= 0.0f && d23_1 <= 0.0f)
   --    {
   --       m_v3.a = 1.0f;
   --       m_count = 1;
   --       m_v1 = m_v3;
   --       return;
   --    }
   --
   --    e23
   --    if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
   --    {
   --       float inv_d23 = 1.0f / (d23_1 + d23_2);
   --       m_v2.a = d23_1 * inv_d23;
   --       m_v3.a = d23_2 * inv_d23;
   --       m_count = 2;
   --       m_v1 = m_v3;
   --       return;
   --    }
   --
   --    Must be in triangle123
   --    float inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
   --    m_v1.a = d123_1 * inv_d123;
   --    m_v2.a = d123_2 * inv_d123;
   --    m_v3.a = d123_3 * inv_d123;
   --    m_count = 3;
   --  }
   --

   procedure solve3 (Self : in out b2Simplex)
   is
      m_v1 : b2SimplexVertex renames Self.m_Vertices (0);
      m_v2 : b2SimplexVertex renames Self.m_Vertices (1);
      m_v3 : b2SimplexVertex renames Self.m_Vertices (2);

      w1 : constant b2Vec2 := m_v1.w;
      w2 : constant b2Vec2 := m_v2.w;
      w3 : constant b2Vec2 := m_v3.w;

      --    Edge12
      --
      --    [1      1     ][a1] = [1]
      --    [w1.e12 w2.e12][a2] = [0]
      --    a3 = 0
      --
      e12   : constant b2Vec2 :=  w2 - w1;
      w1e12 : constant Real   :=  b2Dot (w1, e12);
      w2e12 : constant Real   :=  b2Dot (w2, e12);
      d12_1 : constant Real   :=  w2e12;
      d12_2 : constant Real   := -w1e12;

      --    Edge13
      --
      --    [1      1     ][a1] = [1]
      --    [w1.e13 w3.e13][a3] = [0]
      --    a2 = 0
      --
      e13   : constant b2Vec2 :=  w3 - w1;
      w1e13 : constant Real   :=  b2Dot (w1, e13);
      w3e13 : constant Real   :=  b2Dot (w3, e13);
      d13_1 : constant Real   :=  w3e13;
      d13_2 : constant Real   := -w1e13;

      --    Edge23
      --
      --    [1      1     ][a2] = [1]
      --    [w2.e23 w3.e23][a3] = [0]
      --    a1 = 0
      --
      e23   : constant b2Vec2 :=  w3 - w2;
      w2e23 : constant Real   :=  b2Dot(w2, e23);
      w3e23 : constant Real   :=  b2Dot(w3, e23);
      d23_1 : constant Real   :=  w3e23;
      d23_2 : constant Real   := -w2e23;

      --    Triangle123
      --
      n123   : constant Real := b2Cross (e12, e13);

      d123_1 : constant Real := n123 * b2Cross (w2, w3);
      d123_2 : constant Real := n123 * b2Cross (w3, w1);
      d123_3 : constant Real := n123 * b2Cross (w1, w2);

   begin
      --    w1 region
      --
      if    d12_2 <= 0.0
        and d13_2 <= 0.0
      then
         m_v1.a       := 1.0;
         Self.m_Count := 1;
         return;
      end if;

      --    e12
      --
      if    d12_1  >  0.0
        and d12_2  >  0.0
        and d123_3 <= 0.0
      then
         declare
            inv_d12 : constant Real := 1.0 / (d12_1 + d12_2);
         begin
            m_v1.a := d12_1 * inv_d12;
            m_v2.a := d12_2 * inv_d12;

            Self.m_Count := 2;
            return;
         end;
      end if;

      --    e13
      --
      if    d13_1  >  0.0
        and d13_2  >  0.0
        and d123_2 <= 0.0
      then
         declare
            inv_d13 : constant Real := 1.0 / (d13_1 + d13_2);
         begin
            m_v1.a := d13_1 * inv_d13;
            m_v3.a := d13_2 * inv_d13;
            m_v2   := m_v3;

            Self.m_Count := 2;
            return;
         end;
      end if;

      --    w2 region
      --
      if      d12_1 <= 0.0
          and d23_2 <= 0.0
        then
         m_v2.a := 1.0;
         m_v1   := m_v2;

         Self.m_Count := 1;
         return;
      end if;

      --    w3 region
      --
      if    d13_1 <= 0.0
        and d23_1 <= 0.0
      then
         m_v3.a := 1.0;
         m_v1   := m_v3;

         Self.m_Count := 1;
         return;
      end if;

      --    e23
      --
      if    d23_1  >  0.0
        and d23_2  >  0.0
        and d123_1 <= 0.0
      then
         declare
            inv_d23 : constant Real := 1.0 / (d23_1 + d23_2);
         begin
            m_v2.a := d23_1 * inv_d23;
            m_v3.a := d23_2 * inv_d23;
            m_v1   := m_v3;

            Self.m_Count := 2;
            return;
         end;
      end if;

      --    Must be in triangle123
      --
      declare
         inv_d123 : constant Real := 1.0 / (d123_1 + d123_2 + d123_3);
      begin
         m_v1.a := d123_1 * inv_d123;
         m_v2.a := d123_2 * inv_d123;
         m_v3.a := d123_3 * inv_d123;

         Self.m_Count := 3;
      end;

   end solve3;




   --    b2Vec2 b2Simplex::GetSearchDirection() const
   --    {
   --       switch (m_count)
   --       {
   --       case 1:
   --          return -m_v1.w;
   --
   --       case 2:
   --          {
   --             b2Vec2 e12 = m_v2.w - m_v1.w;
   --             float sgn = b2Cross(e12, -m_v1.w);
   --             if (sgn > 0.0f)
   --             {
   --                Origin is left of e12.
   --                return b2Cross(1.0f, e12);
   --             }
   --             else
   --             {
   --                Origin is right of e12.
   --                return b2Cross(e12, 1.0f);
   --             }
   --          }
   --
   --       default:
   --          b2Assert(false);
   --          return b2Vec2_zero;
   --       }
   --    }
   --

   function getSearchDirection (Self : in b2Simplex) return b2Vec2
   is
      m_v1 : b2SimplexVertex renames Self.m_Vertices (0);
      m_v2 : b2SimplexVertex renames Self.m_Vertices (1);
      m_v3 : b2SimplexVertex renames Self.m_Vertices (2);
   begin
      case Self.m_count
      is
         when 1 =>
            return -m_v1.w;


         when 2 =>
            declare
               e12 : constant b2Vec2 := m_v2.w - m_v1.w;
               sgn : constant Real   := b2Cross (e12, -m_v1.w);
            begin
               if sgn > 0.0
               then
                  --  Origin is left of e12.
                  --
                  return b2Cross (1.0, e12);

               else
                  --  Origin is right of e12.
                  --
                  return b2Cross (e12, 1.0);
               end if;
            end;


         when others =>
            pragma assert (False);
            return b2Vec2_zero;
      end case;
   end getSearchDirection;





   --    void b2Simplex::GetWitnessPoints(b2Vec2* pA, b2Vec2* pB) const
   --    {
   --       switch (m_count)
   --       {
   --       case 0:
   --          b2Assert(false);
   --          break;
   --
   --       case 1:
   --          *pA = m_v1.wA;
   --          *pB = m_v1.wB;
   --          break;
   --
   --       case 2:
   --          *pA = m_v1.a * m_v1.wA + m_v2.a * m_v2.wA;
   --          *pB = m_v1.a * m_v1.wB + m_v2.a * m_v2.wB;
   --          break;
   --
   --       case 3:
   --          *pA = m_v1.a * m_v1.wA + m_v2.a * m_v2.wA + m_v3.a * m_v3.wA;
   --          *pB = *pA;
   --          break;
   --
   --       default:
   --          b2Assert(false);
   --          break;
   --       }
   --    }
   --

   procedure getWitnessPoints (Self : in b2Simplex;   pA, pB : out b2Vec2)
   is
      m_v1 : b2SimplexVertex renames Self.m_Vertices (0);
      m_v2 : b2SimplexVertex renames Self.m_Vertices (1);
      m_v3 : b2SimplexVertex renames Self.m_Vertices (2);
   begin
      case Self.m_count
      is
         when 0 =>
            pragma assert (False);

         when 1 =>
            pA := m_v1.wA;
            pB := m_v1.wB;

         when 2 =>
            pA :=   m_v1.a * m_v1.wA
                  + m_v2.a * m_v2.wA;
            pB :=   m_v1.a * m_v1.wB
                  + m_v2.a * m_v2.wB;

         when 3 =>
            pA :=    m_v1.a * m_v1.wA
                  +  m_v2.a * m_v2.wA
                  +  m_v3.a * m_v3.wA;
            pB := pA;

         when others =>
            pragma assert (False);
      end case;
   end getWitnessPoints;





   --    void b2Simplex::WriteCache(b2SimplexCache* cache) const
   --    {
   --       cache->metric = GetMetric();
   --       cache->count = uint16(m_count);
   --       const b2SimplexVertex* vertices = &m_v1;
   --       for (int32 i = 0; i < m_count; ++i)
   --       {
   --          cache->indexA[i] = uint8(vertices[i].indexA);
   --          cache->indexB[i] = uint8(vertices[i].indexB);
   --       }
   --    }
   --

   procedure writeCache (Self : in b2Simplex;   cache : out b2SimplexCache)
   is
      m_v1 : b2SimplexVertex renames Self.m_Vertices (0);
      m_v2 : b2SimplexVertex renames Self.m_Vertices (1);
      m_v3 : b2SimplexVertex renames Self.m_Vertices (2);

      vertices : b2SimplexVertices renames Self.m_Vertices;

   begin
      cache.metric := getMetric (Self);
      cache.count  := Self.m_count;

      for i in 0 .. Self.m_count - 1
      loop
         cache.indexA (i) := vertices (i).indexA;
         cache.indexB (i) := vertices (i).indexB;
      end loop;
   end writeCache;





   --  Compute the closest points between two shapes. Supports any combination
   --  of: b2CircleShape, b2PolygonShape, b2EdgeShape.
   --
   --  The simplex cache is input/output.
   --
   --  On the first call set 'b2SimplexCache.count' to zero.
   --
   --  void b2Distance (b2DistanceOutput* output,
   --                   b2SimplexCache* cache,
   --                   const b2DistanceInput* input)
   --  {
   --    ++b2_gjkCalls;
   --
   --    const b2DistanceProxy* proxyA = &input->proxyA;
   --    const b2DistanceProxy* proxyB = &input->proxyB;
   --
   --    b2Transform transformA = input->transformA;
   --    b2Transform transformB = input->transformB;
   --
   --    Initialize the simplex.
   --    b2Simplex simplex;
   --    simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);
   --
   --    Get simplex vertices as an array.
   --    b2SimplexVertex* vertices = &simplex.m_v1;
   --    const int32 k_maxIters = 20;
   --
   --    These store the vertices of the last simplex so that we
   --    can check for duplicates and prevent cycling.
   --    int32 saveA[3], saveB[3];
   --    int32 saveCount = 0;
   --
   --    Main iteration loop.
   --    int32 iter = 0;
   --    while (iter < k_maxIters)
   --    {
   --       Copy simplex so we can identify duplicates.
   --       saveCount = simplex.m_count;
   --       for (int32 i = 0; i < saveCount; ++i)
   --       {
   --          saveA[i] = vertices[i].indexA;
   --          saveB[i] = vertices[i].indexB;
   --       }
   --
   --       switch (simplex.m_count)
   --       {
   --       case 1:
   --          break;
   --
   --       case 2:
   --          simplex.Solve2();
   --          break;
   --
   --       case 3:
   --          simplex.Solve3();
   --          break;
   --
   --       default:
   --          b2Assert(false);
   --       }
   --
   --       If we have 3 points, then the origin is in the corresponding triangle.
   --       if (simplex.m_count == 3)
   --       {
   --          break;
   --       }
   --
   --       Get search direction.
   --       b2Vec2 d = simplex.GetSearchDirection();
   --
   --       Ensure the search direction is numerically fit.
   --       if (d.LengthSquared() < b2_epsilon * b2_epsilon)
   --       {
   --          The origin is probably contained by a line segment
   --          or triangle. Thus the shapes are overlapped.
   --
   --          We can't return zero here even though there may be overlap.
   --          In case the simplex is a point, segment, or triangle it is difficult
   --          to determine if the origin is contained in the CSO or very close to it.
   --          break;
   --       }
   --
   --       Compute a tentative new simplex vertex using support points.
   --       b2SimplexVertex* vertex = vertices + simplex.m_count;
   --       vertex->indexA = proxyA->GetSupport(b2MulT(transformA.q, -d));
   --       vertex->wA = b2Mul(transformA, proxyA->GetVertex(vertex->indexA));
   --       vertex->indexB = proxyB->GetSupport(b2MulT(transformB.q, d));
   --       vertex->wB = b2Mul(transformB, proxyB->GetVertex(vertex->indexB));
   --       vertex->w = vertex->wB - vertex->wA;
   --
   --       Iteration count is equated to the number of support point calls.
   --       ++iter;
   --       ++b2_gjkIters;
   --
   --       Check for duplicate support points. This is the main termination criteria.
   --       bool duplicate = false;
   --       for (int32 i = 0; i < saveCount; ++i)
   --       {
   --          if (vertex->indexA == saveA[i] && vertex->indexB == saveB[i])
   --          {
   --             duplicate = true;
   --             break;
   --          }
   --       }
   --
   --       If we found a duplicate support point we must exit to avoid cycling.
   --       if (duplicate)
   --       {
   --          break;
   --       }
   --
   --       New vertex is ok and needed.
   --       ++simplex.m_count;
   --    }
   --
   --    b2_gjkMaxIters = b2Max(b2_gjkMaxIters, iter);
   --
   --    Prepare output.
   --    simplex.GetWitnessPoints(&output->pointA, &output->pointB);
   --    output->distance = b2Distance(output->pointA, output->pointB);
   --    output->iterations = iter;
   --
   --    Cache the simplex.
   --    simplex.WriteCache(cache);
   --
   --    Apply radii if requested.
   --    if (input->useRadii)
   --    {
   --       float rA = proxyA->m_radius;
   --       float rB = proxyB->m_radius;
   --
   --       if (output->distance > rA + rB && output->distance > b2_epsilon)
   --       {
   --          Shapes are still no overlapped.
   --          Move the witness points to the outer surface.
   --          output->distance -= rA + rB;
   --          b2Vec2 normal = output->pointB - output->pointA;
   --          normal.Normalize();
   --          output->pointA += rA * normal;
   --          output->pointB -= rB * normal;
   --       }
   --       else
   --       {
   --          Shapes are overlapped when radii are considered.
   --          Move the witness points to the middle.
   --          b2Vec2 p = 0.5f * (output->pointA + output->pointB);
   --          output->pointA = p;
   --          output->pointB = p;
   --          output->distance = 0.0f;
   --       }
   --    }
   --  }
   --

   procedure b2Distance (Output :    out b2DistanceOutput;
                         Cache  : in out b2SimplexCache;
                         Input  : in     b2DistanceInput)
   is
      proxyA : b2DistanceProxy renames Input.proxyA;
      proxyB : b2DistanceProxy renames Input.proxyB;

      transformA : b2Transform renames Input.transformA;
      transformB : b2Transform renames Input.transformB;

      Simplex : b2Simplex;

   begin
      readCache (Simplex,   Cache,                  -- Initialize the simplex.
                            proxyA, transformA,
                            proxyB, transformB);
      declare
         -- Get simplex vertices as an array.
         Vertices   : b2simplexVertices renames Simplex.m_Vertices;     -- m_v1'Access;
         k_maxIters : constant := 20;

         -- These store the vertices of the last simplex so that we
         -- can check for duplicates and prevent cycling.
         --
         saveA,
         saveB     : array (0 .. 2) of Natural;
         saveCount : Integer := 0;

         iter      : Integer := 0;

      begin
         b2_gjkCalls := b2_gjkCalls + 1;

         -- Main iteration loop.
         --
         while Iter < k_maxIters
         loop
            -- Copy simplex so we can identify duplicates.
            --
            saveCount := Simplex.m_Count;


            for i in 0 .. saveCount - 1
            loop
               saveA (i) := Vertices (i).indexA;
               saveB (i) := Vertices (i).indexB;
            end loop;


            case Simplex.m_Count
            is
               when 1 => null;
               when 2 => Solve2 (Simplex);
               when 3 => Solve3 (Simplex);

               when others => pragma assert (False);
            end case;


            -- If we have 3 points, then the origin is in the corresponding triangle.
            --
            if Simplex.m_Count = 3
            then
               exit;
            end if;


            -- Get search direction.
            --
            declare
               use b2_Common;

               d : constant b2Vec2 := getSearchDirection (Simplex);
            begin
               -- Ensure the search direction is numerically fit.
               --
               if lengthSquared (d) < b2_Epsilon * b2_Epsilon
               then
                  -- The origin is probably contained by a line segment
                  -- or triangle. Thus the shapes are overlapped.

                  -- We can't return zero here even though there may be overlap.
                  -- In case the simplex is a point, segment, or triangle it is difficult
                  -- to determine if the origin is contained in the CSO or very close to it.
                  --
                  exit;
               end if;

               -- Compute a tentative new simplex vertex using support points.
               --
               declare
                  vertex : b2SimplexVertex renames Vertices (Simplex.m_Count);
               begin
                  vertex.indexA := getSupport (proxyA, b2MulT (transformA.q, -d));
                  vertex.indexB := getSupport (proxyB, b2MulT (transformB.q,  d));

                  vertex.wA     := b2Mul (transformA,  getVertex (proxyA, vertex.indexA));
                  vertex.wB     := b2Mul (transformB,  getVertex (proxyB, vertex.indexB));

                  vertex.w      := vertex.wB - vertex.wA;

                  -- Iteration count is equated to the number of support point calls.
                  --
                  iter        := iter        + 1;
                  b2_gjkIters := b2_gjkIters + 1;


                  -- Check for duplicate support points. This is the main termination criteria.
                  --
                  declare
                     Duplicate : Boolean := False;
                  begin
                     for i in 0 .. saveCount - 1
                     loop
                        if    vertex.indexA = saveA (i)
                          and vertex.indexB = saveB (i)
                        then
                           Duplicate := True;
                           exit;
                        end if;
                     end loop;


                     -- If we found a duplicate support point we must exit to avoid cycling.
                     --
                     if Duplicate
                     then
                        exit;
                     end if;
                  end;
               end;
            end;


            -- New vertex is ok and needed.
            --
            simplex.m_count := simplex.m_count + 1;
         end loop;


         b2_gjkMaxIters := Natural'Max (b2_gjkMaxIters, iter);

         -- Prepare output.
         --
         getWitnessPoints (simplex, output.pointA, output.pointB);

         output.distance   := b2Distance (output.pointA, output.pointB);
         output.iterations := iter;

         -- Cache the simplex.
         --
         writeCache (simplex, cache);


         -- Apply radii if requested.
         --
         if input.useRadii
         then
            declare
               use b2_Common;

               rA     : constant Real  := proxyA.m_radius;
               rB     : constant Real  := proxyB.m_radius;
               Normal :          b2Vec2;
               p      :          b2Vec2;
            begin
               if    output.distance > rA + rB
                 and output.distance > b2_epsilon
               then
                  -- Shapes are still no overlapped.
                  -- Move the witness points to the outer surface.
                  --
                  Normal := output.pointB - output.pointA;
                  normalize (Normal);

                  output.Distance := output.distance - rA + rB;
                  output.pointA   := output.pointA   + rA * Normal;
                  output.pointB   := output.pointB   - rB * Normal;
               else
                  -- Shapes are overlapped when radii are considered.
                  -- Move the witness points to the middle.
                  --
                  p := 0.5 * (output.pointA + output.pointB);

                  output.pointA   := p;
                  output.pointB   := p;
                  output.distance := 0.0;
               end if;
            end;
         end if;
      end;
   end b2Distance;





   --    b2Vec2 b2Simplex::GetClosestPoint() const
   --    {
   --       switch (m_count)
   --       {
   --       case 0:
   --          b2Assert(false);
   --          return b2Vec2_zero;
   --
   --       case 1:
   --          return m_v1.w;
   --
   --       case 2:
   --          return m_v1.a * m_v1.w + m_v2.a * m_v2.w;
   --
   --       case 3:
   --          return b2Vec2_zero;
   --
   --       default:
   --          b2Assert(false);
   --          return b2Vec2_zero;
   --       }
   --    }
   --

   function getClosestPoint (Self : in b2Simplex) return b2Vec2
   is
      m_v1 : b2SimplexVertex renames Self.m_Vertices (0);
      m_v2 : b2SimplexVertex renames Self.m_Vertices (1);
   begin
      case Self.m_count
      is
         when 0 =>
            pragma assert (False);
            return b2Vec2_zero;

         when 1 =>
            return m_v1.w;

         when 2 =>
            return   m_v1.a * m_v1.w
                   + m_v2.a * m_v2.w;

         when 3 =>
            return b2Vec2_zero;

         when others =>
            pragma assert (False);
            return b2Vec2_zero;
      end case;
   end getClosestPoint;





   --  GJK-raycast
   --  Algorithm by Gino van den Bergen.
   --  "Smooth Mesh Contacts with GJK" in Game Physics Pearls. 2010
   --
   --  bool b2ShapeCast(b2ShapeCastOutput * output, const b2ShapeCastInput * input)
   --  {
   --      output->iterations = 0;
   --      output->lambda = 1.0f;
   --      output->normal.SetZero();
   --      output->point.SetZero();
   --
   --    const b2DistanceProxy* proxyA = &input->proxyA;
   --    const b2DistanceProxy* proxyB = &input->proxyB;
   --
   --      float radiusA = b2Max(proxyA->m_radius, b2_polygonRadius);
   --      float radiusB = b2Max(proxyB->m_radius, b2_polygonRadius);
   --      float radius = radiusA + radiusB;
   --
   --    b2Transform xfA = input->transformA;
   --    b2Transform xfB = input->transformB;
   --
   --    b2Vec2 r = input->translationB;
   --    b2Vec2 n(0.0f, 0.0f);
   --    float lambda = 0.0f;
   --
   --    Initial simplex
   --    b2Simplex simplex;
   --    simplex.m_count = 0;
   --
   --    Get simplex vertices as an array.
   --    b2SimplexVertex* vertices = &simplex.m_v1;
   --
   --    Get support point in -r direction
   --    int32 indexA = proxyA->GetSupport(b2MulT(xfA.q, -r));
   --    b2Vec2 wA = b2Mul(xfA, proxyA->GetVertex(indexA));
   --    int32 indexB = proxyB->GetSupport(b2MulT(xfB.q, r));
   --    b2Vec2 wB = b2Mul(xfB, proxyB->GetVertex(indexB));
   --      b2Vec2 v = wA - wB;
   --
   --      Sigma is the target distance between polygons
   --      float sigma = b2Max(b2_polygonRadius, radius - b2_polygonRadius);
   --    const float tolerance = 0.5f * b2_linearSlop;
   --
   --    Main iteration loop.
   --    const int32 k_maxIters = 20;
   --    int32 iter = 0;
   --    while (iter < k_maxIters && v.Length() - sigma > tolerance)
   --    {
   --       b2Assert(simplex.m_count < 3);
   --
   --          output->iterations += 1;
   --
   --       Support in direction -v (A - B)
   --       indexA = proxyA->GetSupport(b2MulT(xfA.q, -v));
   --       wA = b2Mul(xfA, proxyA->GetVertex(indexA));
   --       indexB = proxyB->GetSupport(b2MulT(xfB.q, v));
   --       wB = b2Mul(xfB, proxyB->GetVertex(indexB));
   --          b2Vec2 p = wA - wB;
   --
   --          -v is a normal at p
   --          v.Normalize();
   --
   --          Intersect ray with plane
   --       float vp = b2Dot(v, p);
   --          float vr = b2Dot(v, r);
   --       if (vp - sigma > lambda * vr)
   --       {
   --          if (vr <= 0.0f)
   --          {
   --             return false;
   --          }
   --
   --          lambda = (vp - sigma) / vr;
   --          if (lambda > 1.0f)
   --          {
   --             return false;
   --          }
   --
   --              n = -v;
   --              simplex.m_count = 0;
   --       }
   --
   --          Reverse simplex since it works with B - A.
   --          Shift by lambda * r because we want the closest point to the current clip point.
   --          Note that the support point p is not shifted because we want the plane equation
   --          to be formed in unshifted space.
   --       b2SimplexVertex* vertex = vertices + simplex.m_count;
   --       vertex->indexA = indexB;
   --       vertex->wA = wB + lambda * r;
   --       vertex->indexB = indexA;
   --       vertex->wB = wA;
   --       vertex->w = vertex->wB - vertex->wA;
   --       vertex->a = 1.0f;
   --       simplex.m_count += 1;
   --
   --       switch (simplex.m_count)
   --       {
   --       case 1:
   --          break;
   --
   --       case 2:
   --          simplex.Solve2();
   --          break;
   --
   --       case 3:
   --          simplex.Solve3();
   --          break;
   --
   --       default:
   --          b2Assert(false);
   --       }
   --
   --       If we have 3 points, then the origin is in the corresponding triangle.
   --       if (simplex.m_count == 3)
   --       {
   --          Overlap
   --          return false;
   --       }
   --
   --       Get search direction.
   --       v = simplex.GetClosestPoint();
   --
   --       Iteration count is equated to the number of support point calls.
   --       ++iter;
   --    }
   --
   --    if (iter == 0)
   --    {
   --       Initial overlap
   --       return false;
   --    }
   --
   --    Prepare output.
   --    b2Vec2 pointA, pointB;
   --    simplex.GetWitnessPoints(&pointB, &pointA);
   --
   --    if (v.LengthSquared() > 0.0f)
   --    {
   --          n = -v;
   --       n.Normalize();
   --    }
   --
   --      output->point = pointA + radiusA * n;
   --    output->normal = n;
   --    output->lambda = lambda;
   --    output->iterations = iter;
   --    return true;
   --  }
   --

   function b2ShapeCast (Output :    out b2ShapeCastOutput;
                         Input  : in     b2ShapeCastInput) return Boolean
   is
      iter : Natural  := 0;
   begin
      output.iterations := 0;
      output.lambda     := 1.0;

      setZero (output.normal);
      setZero (output.point);

      declare
         use b2_Common;

         proxyA  : b2DistanceProxy renames input.proxyA;
         proxyB  : b2DistanceProxy renames input.proxyB;

         radiusA : constant Real := Real'max (proxyA.m_radius, b2_polygonRadius);
         radiusB : constant Real := Real'max (proxyB.m_radius, b2_polygonRadius);
         radius  : constant Real := radiusA + radiusB;

         xfA     : constant b2Transform := input.transformA;
         xfB     : constant b2Transform := input.transformB;

         r       : constant b2Vec2 := input.translationB;
         n       :          b2Vec2 := (0.0, 0.0);
         lambda  :          Real   :=  0.0;

         simplex : b2Simplex := (m_count => 0,                        -- Initial simplex.
                                 others  => <>);

         vertices : b2SimplexVertices renames simplex.m_Vertices;     -- Get simplex vertices as an array.

         --    Get support point in -r direction.
         --
         indexA : Natural := getSupport (proxyA,  b2MulT (xfA.q, -r));
         indexB : Natural := getSupport (proxyB,  b2MulT (xfB.q, r));

         wA     : b2Vec2  := b2Mul (xfA,  getVertex (proxyA, indexA));
         wB     : b2Vec2  := b2Mul (xfB,  getVertex (proxyB, indexB));

         v      : b2Vec2  := wA - wB;

         sigma  : constant Real := Real'max (b2_polygonRadius,               -- Sigma is the target distance between polygons.
                                             radius - b2_polygonRadius);

         tolerance  : constant := 0.5 * b2_linearSlop;
         k_maxIters : constant := 20;

      begin
         --    Main iteration loop.
         --
         while iter               < k_maxIters
           and Length (v) - sigma > tolerance
         loop
            declare
               pragma assert (Simplex.m_count < 3);

               p      : b2Vec2;
               vp, vr : Real;
            begin
               output.iterations := output.iterations + 1;

               -- Support in direction -v (A - B)
               --
               indexA := getSupport (proxyA,  b2MulT (xfA.q, -v));
               indexB := getSupport (proxyB,  b2MulT (xfB.q,  v));

               wA := b2Mul (xfA,  getVertex (proxyA, indexA));
               wB := b2Mul (xfB,  getVertex (proxyB, indexB));

               p  := wA - wB;

               -- -v is a normal at p.
               --
              normalize (v);

               -- Intersect ray with plane.
               --
               vp := b2Dot (v, p);
               vr := b2Dot (v, r);

               if vp - sigma > lambda * vr
               then
                  if vr <= 0.0
                  then
                     return False;
                  end if;

                  lambda := (vp - sigma) / vr;

                  if lambda > 1.0
                  then
                     return False;
                  end if;

                  n               := -v;
                  simplex.m_count := 0;
               end if;

               -- Reverse simplex since it works with B - A.
               -- Shift by lambda * r because we want the closest point to the current clip point.
               -- Note that the support point p is not shifted because we want the plane equation
               -- to be formed in unshifted space.
               --
               declare
                  vertex : b2SimplexVertex renames vertices (simplex.m_count);
               begin
                  vertex.indexA := indexB;
                  vertex.indexB := indexA;

                  vertex.wA := wB + lambda * r;
                  vertex.wB := wA;

                  vertex.w  := vertex.wB - vertex.wA;
                  vertex.a  := 1.0;

                  simplex.m_count := simplex.m_count + 1;

                  case (simplex.m_count)
                  is
                     when 1      => null;
                     when 2      => solve2 (simplex);
                     when 3      => solve3 (simplex);
                     when others => pragma assert (False);
                  end case;

                  -- If we have 3 points, then the origin is in the corresponding triangle.
                  --
                  if simplex.m_count = 3
                  then                          -- Overlap
                     return False;
                  end if;

                  -- Get search direction.
                  --
                  v := getClosestPoint (simplex);

                  -- Iteration count is equated to the number of support point calls.
                  --
                  iter := iter + 1;
               end;
            end;
         end loop;


         if iter = 0
         then              -- Initial overlap.
            return False;
         end if;

         --    Prepare output.
         --
         declare
            pointA,
            pointB : b2Vec2;
         begin
            getWitnessPoints (simplex, pointB,
                              pointA);
            if lengthSquared (v) > 0.0
            then
               n := -v;
               normalize (n);
            end if;

            output.point      := pointA  +  radiusA * n;
            output.normal     := n;
            output.lambda     := lambda;
            output.iterations := iter;
         end;
      end;


      return True;
   end b2ShapeCast;


end b2_Distance;
