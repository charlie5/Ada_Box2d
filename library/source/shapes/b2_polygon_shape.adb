with
     b2_polygon_Shape,
     b2_Common;


package body b2_polygon_Shape
is
   use b2_Common;


   --  inline b2PolygonShape::b2PolygonShape()
   --  {
   --    m_type = e_polygon;
   --    m_radius = b2_polygonRadius;
   --    m_count = 0;
   --    m_centroid.SetZero();
   --  }

   function to_b2polygonShape return b2polygonShape
   is
      Self : b2polygonShape;
   begin
      Self.m_Type   := e_Polygon;
      Self.m_Radius := b2_polygonRadius;
      Self.m_Count  := 0;

      setZero (Self.m_Centroid);

      return Self;
   end to_b2polygonShape;




   -- Implement b2Shape.
   --
   --  b2Shape* b2PolygonShape::Clone(b2BlockAllocator* allocator) const
   --  {
   --    void* mem = allocator->Allocate(sizeof(b2PolygonShape));
   --    b2PolygonShape* clone = new (mem) b2PolygonShape;
   --    *clone = *this;
   --    return clone;
   --  }
   --

   type b2polygonShape_ptr is access all b2polygonShape;

   overriding
   function clone (Self : in b2polygonShape) return b2Shape_ptr
   is
      Clone : constant b2polygonShape_ptr := new b2polygonShape;
   begin
      Clone.all := Self;
      return Clone.all'Access;
   end clone;




   -- @see b2Shape::GetChildCount.
   --
   --  int32 b2PolygonShape::GetChildCount() const
   --  {
   --    return 1;
   --  }
   --

   overriding
   function getChildCount (Self : in b2polygonShape) return Natural
   is
   begin
      return 1;
   end getChildCount;




   --    @see b2Shape::TestPoint
   --
   --  bool b2PolygonShape::TestPoint(const b2Transform& xf, const b2Vec2& p) const
   --  {
   --    b2Vec2 pLocal = b2MulT(xf.q, p - xf.p);
   --
   --    for (int32 i = 0; i < m_count; ++i)
   --    {
   --       float dot = b2Dot(m_normals[i], pLocal - m_vertices[i]);
   --       if (dot > 0.0f)
   --       {
   --          return false;
   --       }
   --    }
   --
   --    return true;
   --  }
   --

   overriding
   function testPoint (Self : in b2polygonShape;   Transform : in b2Transform;
                                                   p         : in b2Vec2) return Boolean
   is
      Dot    :          Real;
      pLocal : constant b2Vec2 := b2MulT (Transform.q,
                                          p - Transform.p);
   begin
      for i in 0 .. Self.m_Count - 1
      loop
         Dot := b2Dot (Self.m_Normals (i),
                       pLocal - Self.m_Vertices (i));

         if Dot > 0.0
         then
            return False;
         end if;
      end loop;

      return True;
   end testPoint;




   --    Implement b2Shape.
   --
   --    @note because the polygon is solid, rays that start inside do not hit because the normal is
   --    not defined.
   --
   --  bool b2PolygonShape::RayCast (b2RayCastOutput* output, const b2RayCastInput& input,
   --                                const b2Transform& xf, int32 childIndex) const
   --  {
   --    B2_NOT_USED(childIndex);
   --
   --    -- Put the ray into the polygon's frame of reference.
   --    b2Vec2 p1 = b2MulT(xf.q, input.p1 - xf.p);
   --    b2Vec2 p2 = b2MulT(xf.q, input.p2 - xf.p);
   --    b2Vec2 d = p2 - p1;
   --
   --    float lower = 0.0f, upper = input.maxFraction;
   --
   --    int32 index = -1;
   --
   --    for (int32 i = 0; i < m_count; ++i)
   --    {
   --       -- p = p1 + a * d
   --       -- dot(normal, p - v) = 0
   --       -- dot(normal, p1 - v) + a * dot(normal, d) = 0
   --       float numerator = b2Dot(m_normals[i], m_vertices[i] - p1);
   --       float denominator = b2Dot(m_normals[i], d);
   --
   --       if (denominator == 0.0f)
   --       {
   --          if (numerator < 0.0f)
   --          {
   --             return false;
   --          }
   --       }
   --       else
   --       {
   --          -- Note: we want this predicate without division:
   --          -- lower < numerator / denominator, where denominator < 0
   --          -- Since denominator < 0, we have to flip the inequality:
   --          -- lower < numerator / denominator <==> denominator * lower > numerator.
   --          if (denominator < 0.0f && numerator < lower * denominator)
   --          {
   --             -- Increase lower.
   --             -- The segment enters this half-space.
   --             lower = numerator / denominator;
   --             index = i;
   --          }
   --          else if (denominator > 0.0f && numerator < upper * denominator)
   --          {
   --             -- Decrease upper.
   --             -- The segment exits this half-space.
   --             upper = numerator / denominator;
   --          }
   --       }
   --
   --       -- The use of epsilon here causes the assert on lower to trip
   --       -- in some cases. Apparently the use of epsilon was to make edge
   --       -- shapes work, but now those are handled separately.
   --       //if (upper < lower - b2_epsilon)
   --       if (upper < lower)
   --       {
   --          return false;
   --       }
   --    }
   --
   --    b2Assert(0.0f <= lower && lower <= input.maxFraction);
   --
   --    if (index >= 0)
   --    {
   --       output->fraction = lower;
   --       output->normal = b2Mul(xf.q, m_normals[index]);
   --       return true;
   --    }
   --
   --    return false;
   --  }
   --

   overriding
   function raycast (Self : in b2polygonShape;   Output     :    out b2RayCastOutput;
                                                 Input      : in     b2RayCastInput;
                                                 Transform  : in     b2Transform;
                                                 childIndex : in     Natural)        return Boolean
   is
      pragma Unreferenced (childIndex);

      -- Put the ray into the polygon's frame of reference.
      --
      p1    : constant b2Vec2  := b2MulT (Transform.q, input.p1 - Transform.p);
      p2    : constant b2Vec2  := b2MulT (Transform.q, input.p2 - Transform.p);
      d     : constant b2Vec2  := p2 - p1;

      Lower : Real := 0.0;
      Upper : Real := Input.maxFraction;

      Index : Integer := -1;

   begin
      for i in 0 .. Self.m_count
      loop
         -- p = p1 + a * d
         -- dot(normal, p - v) = 0
         -- dot(normal, p1 - v) + a * dot(normal, d) = 0
         --
         declare
            Numerator   : constant Real := b2Dot (Self.m_Normals (i),  Self.m_Vertices (i) - p1);
            Denominator : constant Real := b2Dot (Self.m_Normals (i),  d);
         begin
            if Denominator = 0.0
            then
               if Numerator < 0.0
               then
                  return False;
               end if;

            else
               -- Note: we want this predicate without division:
               -- lower < numerator / denominator, where denominator < 0
               -- Since denominator < 0, we have to flip the inequality:
               -- lower < numerator / denominator <==> denominator * lower > numerator.
               --
               if    Denominator < 0.0
                 and Numerator   < Lower * Denominator
               then
                  -- Increase lower.
                  -- The segment enters this half-space.
                  Lower := Numerator / Denominator;
                  Index := i;

               elsif     Denominator > 0.0
                     and Numerator   < Upper * Denominator
               then
                  -- Decrease upper.
                  -- The segment exits this half-space.
                  --
                  Upper := Numerator / Denominator;
               end if;
            end if;
         end;

         -- The use of epsilon here causes the assert on lower to trip
         -- in some cases. Apparently the use of epsilon was to make edge
         -- shapes work, but now those are handled separately.
         --
         -- if (upper < lower - b2_epsilon)
         if Upper < Lower
         then
            return False;
         end if;
      end loop;


      pragma assert (0.0 <= Lower and Lower <= Input.maxFraction);

      if Index >= 0
      then
         Output.Fraction := Lower;
         Output.Normal   := b2Mul (Transform.q, Self.m_Normals (Index));

         return True;
      end if;

      return False;
   end raycast;




   --    @see b2Shape::ComputeAABB
   --
   --  void b2PolygonShape::ComputeAABB(b2AABB* aabb, const b2Transform& xf, int32 childIndex) const
   --  {
   --    B2_NOT_USED(childIndex);
   --
   --    b2Vec2 lower = b2Mul(xf, m_vertices[0]);
   --    b2Vec2 upper = lower;
   --
   --    for (int32 i = 1; i < m_count; ++i)
   --    {
   --       b2Vec2 v = b2Mul(xf, m_vertices[i]);
   --       lower = b2Min(lower, v);
   --       upper = b2Max(upper, v);
   --    }
   --
   --    b2Vec2 r(m_radius, m_radius);
   --    aabb->lowerBound = lower - r;
   --    aabb->upperBound = upper + r;
   --  }
   --

   overriding
   procedure computeAABB (Self : in b2polygonShape;   aabb       :    out b2AABB;
                                                      Transform  : in     b2Transform;
                                                      childIndex : in     Natural)
   is
      pragma Unreferenced (childIndex);

      Lower : b2Vec2 := b2Mul (Transform, Self.m_Vertices (0));
      Upper : b2Vec2 := Lower;
      r, v  : b2Vec2;
   begin
      for i in 0 .. Self.m_Count - 1
      loop
         v     := b2Mul (Transform, Self.m_Vertices (i));
         Lower := b2Min (Lower, v);
         Upper := b2Max (Upper, v);
      end loop;

      r := (Self.m_Radius,
            Self.m_Radius);

      aabb.lowerBound := Lower - r;
      aabb.upperBound := Upper + r;
   end computeAABB;




   --    @see b2Shape::ComputeMass
   --
   --  void b2PolygonShape::ComputeMass(b2MassData* massData, float density) const
   --  {
   --    -- Polygon mass, centroid, and inertia.
   --    -- Let rho be the polygon density in mass per unit area.
   --    -- Then:
   --    -- mass = rho * int(dA)
   --    -- centroid.x = (1/mass) * rho * int(x * dA)
   --    -- centroid.y = (1/mass) * rho * int(y * dA)
   --    -- I = rho * int((x*x + y*y) * dA)
   --    //
   --    -- We can compute these integrals by summing all the integrals
   --    -- for each triangle of the polygon. To evaluate the integral
   --    -- for a single triangle, we make a change of variables to
   --    -- the (u,v) coordinates of the triangle:
   --    -- x = x0 + e1x * u + e2x * v
   --    -- y = y0 + e1y * u + e2y * v
   --    -- where 0 <= u && 0 <= v && u + v <= 1.
   --    //
   --    -- We integrate u from [0,1-v] and then v from [0,1].
   --    -- We also need to use the Jacobian of the transformation:
   --    -- D = cross(e1, e2)
   --    //
   --    -- Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
   --    //
   --    -- The rest of the derivation is handled by computer algebra.
   --
   --    b2Assert(m_count >= 3);
   --
   --    b2Vec2 center(0.0f, 0.0f);
   --    float area = 0.0f;
   --    float I = 0.0f;
   --
   --    -- Get a reference point for forming triangles.
   --    -- Use the first vertex to reduce round-off errors.
   --    b2Vec2 s = m_vertices[0];
   --
   --    const float k_inv3 = 1.0f / 3.0f;
   --
   --    for (int32 i = 0; i < m_count; ++i)
   --    {
   --       -- Triangle vertices.
   --       b2Vec2 e1 = m_vertices[i] - s;
   --       b2Vec2 e2 = i + 1 < m_count ? m_vertices[i+1] - s : m_vertices[0] - s;
   --
   --       float D = b2Cross(e1, e2);
   --
   --       float triangleArea = 0.5f * D;
   --       area += triangleArea;
   --
   --       -- Area weighted centroid
   --       center += triangleArea * k_inv3 * (e1 + e2);
   --
   --       float ex1 = e1.x, ey1 = e1.y;
   --       float ex2 = e2.x, ey2 = e2.y;
   --
   --       float intx2 = ex1*ex1 + ex2*ex1 + ex2*ex2;
   --       float inty2 = ey1*ey1 + ey2*ey1 + ey2*ey2;
   --
   --       I += (0.25f * k_inv3 * D) * (intx2 + inty2);
   --    }
   --
   --    -- Total mass
   --    massData->mass = density * area;
   --
   --    -- Center of mass
   --    b2Assert(area > b2_epsilon);
   --    center *= 1.0f / area;
   --    massData->center = center + s;
   --
   --    -- Inertia tensor relative to the local origin (point s).
   --    massData->I = density * I;
   --
   --    -- Shift to center of mass then to original body origin.
   --    massData->I += massData->mass * (b2Dot(massData->center, massData->center) - b2Dot(center, center));
   --  }
   --

   overriding
   procedure computeMass (Self : in b2polygonShape;   massData :    out b2MassData;
                                                      Density  : in     Real)
   is
      --    Polygon mass, centroid, and inertia.
      --    Let rho be the polygon density in mass per unit area.
      --    Then:
      --    mass = rho * int(dA)
      --    centroid.x = (1/mass) * rho * int(x * dA)
      --    centroid.y = (1/mass) * rho * int(y * dA)
      --    I = rho * int((x*x + y*y) * dA)
      --
      --    We can compute these integrals by summing all the integrals
      --    for each triangle of the polygon. To evaluate the integral
      --    for a single triangle, we make a change of variables to
      --    the (u,v) coordinates of the triangle:
      --       x = x0 + e1x * u + e2x * v
      --       y = y0 + e1y * u + e2y * v
      --       where 0 <= u && 0 <= v && u + v <= 1.
      --
      --    We integrate u from [0,1-v] and then v from [0,1].
      --    We also need to use the Jacobian of the transformation:
      --    D = cross(e1, e2)
      --
      --    Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
      --
      --    The rest of the derivation is handled by computer algebra.
      --

      pragma assert (Self.m_Count >= 3);

      Center  : b2Vec2       := (0.0, 0.0);
      Area    : Real := 0.0;
      Inertia : Real := 0.0;

      -- Get a reference point for forming triangles.
      -- Use the first vertex to reduce round-off errors.
      --
      s      : constant b2Vec2       := Self.m_Vertices (0);
      k_inv3 : constant Real := 1.0 / 3.0;

   begin
      for i in 0 .. Self.m_Count - 1
      loop
         declare
            -- Triangle vertices.
            --
            e1 : constant b2Vec2 := Self.m_Vertices (i) - s;
            e2 : constant b2Vec2 := (if i + 1 < Self.m_Count then Self.m_Vertices (i + 1) - s
                                                             else Self.m_Vertices (0)     - s);

            D            : constant Real := b2Cross (e1, e2);
            triangleArea : constant Real := 0.5 * D;
         begin
            Area := Area + triangleArea;

            Center := Center +  triangleArea * k_inv3 * (e1 + e2);     -- Area weighted centroid.

            declare
               ex1 : constant Real := e1.x;
               ey1 : constant Real := e1.y;
               ex2 : constant Real := e2.x;
               ey2 : constant Real := e2.y;

               intx2 : constant Real := ex1*ex1 + ex2*ex1 + ex2*ex2;
               inty2 : constant Real := ey1*ey1 + ey2*ey1 + ey2*ey2;
            begin
               Inertia := Inertia  +  (0.25 * k_inv3 * D)  *  (intx2 + inty2);
            end;
         end;
      end loop;

      massData.Mass := Density * Area;     -- Total mass.

      -- Center of mass.
      --
      pragma assert (Area > b2_Epsilon);

      multiply (Center, 1.0 / Area);
      massData.Center := Center + s;

      massData.I      := Density * Inertia;     -- Inertia tensor relative to the local origin (point s).

      -- Shift to center of mass then to original body origin.
      massData.I :=   massData.I
                    + massData.Mass * (  b2Dot (massData.Center, massData.Center)
                                       - b2Dot (Center, Center));
   end computeMass;





   --  static b2Vec2 ComputeCentroid(const b2Vec2* vs, int32 count)
   --  {
   --    b2Assert(count >= 3);
   --
   --    b2Vec2 c(0.0f, 0.0f);
   --    float area = 0.0f;
   --
   --    -- Get a reference point for forming triangles.
   --    -- Use the first vertex to reduce round-off errors.
   --    b2Vec2 s = vs[0];
   --
   --    const float inv3 = 1.0f / 3.0f;
   --
   --    for (int32 i = 0; i < count; ++i)
   --    {
   --       -- Triangle vertices.
   --       b2Vec2 p1 = vs[0] - s;
   --       b2Vec2 p2 = vs[i] - s;
   --       b2Vec2 p3 = i + 1 < count ? vs[i+1] - s : vs[0] - s;
   --
   --       b2Vec2 e1 = p2 - p1;
   --       b2Vec2 e2 = p3 - p1;
   --
   --       float D = b2Cross(e1, e2);
   --
   --       float triangleArea = 0.5f * D;
   --       area += triangleArea;
   --
   --       -- Area weighted centroid
   --       c += triangleArea * inv3 * (p1 + p2 + p3);
   --    }
   --
   --    -- Centroid
   --    b2Assert(area > b2_epsilon);
   --    c = (1.0f / area) * c + s;
   --    return c;
   --  }

   function computeCentroid (Self : in b2polygonShape;   vs    : in b2Vec2s;
                                                         Count : in Natural) return b2Vec2
   is
      pragma assert (Count >= 3,
                     "Count =" & Count'Image);

      c    : b2Vec2       := (0.0, 0.0);
      Area : Real :=  0.0;

      -- Get a reference point for forming triangles.
      -- Use the first vertex to reduce round-off errors.
      --
      s    : constant b2Vec2       := vs (0);
      inv3 : constant Real := 1.0 / 3.0;
   begin
      for i in 0 .. Count - 1
      loop
         declare
            -- Triangle vertices.
            --
            p1 : constant b2Vec2 := vs (0) - s;
            p2 : constant b2Vec2 := vs (i) - s;
            p3 : constant b2Vec2 := (if i + 1 < Count then vs (i+1) - s
                            else vs (0)   - s);
            e1 : constant b2Vec2 := p2 - p1;
            e2 : constant b2Vec2 := p3 - p1;

            D            : constant Real := b2Cross (e1, e2);
            triangleArea : constant Real := 0.5 * D;
         begin
            Area := Area + triangleArea;
            c    := c    + triangleArea * inv3 * (p1 + p2 + p3);     -- Area weighted centroid.
         end;
      end loop;

      -- Centroid
      --
      pragma assert (Area > b2_Epsilon);

      c := (1.0 / area) * c  +  s;
      return c;
   end ComputeCentroid;




   --    Create a convex hull from the given array of local points.
   --    The count must be in the range [3, b2_maxPolygonVertices].
   --    @warning the points may be re-ordered, even if they form a convex polygon
   --    @warning collinear points are handled but not removed. Collinear points
   --    may lead to poor stacking behavior.
   --
   --  void b2PolygonShape::Set(const b2Vec2* vertices, int32 count)
   --  {
   --    b2Assert(3 <= count && count <= b2_maxPolygonVertices);
   --    if (count < 3)
   --    {
   --       SetAsBox(1.0f, 1.0f);
   --       return;
   --    }
   --
   --    int32 n = b2Min(count, b2_maxPolygonVertices);
   --
   --    -- Perform welding and copy vertices into local buffer.
   --    b2Vec2 ps[b2_maxPolygonVertices];
   --    int32 tempCount = 0;
   --    for (int32 i = 0; i < n; ++i)
   --    {
   --       b2Vec2 v = vertices[i];
   --
   --       bool unique = true;
   --       for (int32 j = 0; j < tempCount; ++j)
   --       {
   --          if (b2DistanceSquared(v, ps[j]) < ((0.5f * b2_linearSlop) * (0.5f * b2_linearSlop)))     ***** TODO: ps has not been initialised !!! *****
   --          {
   --             unique = false;
   --             break;
   --          }
   --       }
   --
   --       if (unique)
   --       {
   --          ps[tempCount++] = v;
   --       }
   --    }
   --
   --    n = tempCount;
   --    if (n < 3)
   --    {
   --       -- Polygon is degenerate.
   --       b2Assert(false);
   --       SetAsBox(1.0f, 1.0f);
   --       return;
   --    }
   --
   --    -- Create the convex hull using the Gift wrapping algorithm
   --    -- http://en.wikipedia.org/wiki/Gift_wrapping_algorithm
   --
   --    -- Find the right most point on the hull
   --    int32 i0 = 0;
   --    float x0 = ps[0].x;
   --    for (int32 i = 1; i < n; ++i)
   --    {
   --       float x = ps[i].x;
   --       if (x > x0 || (x == x0 && ps[i].y < ps[i0].y))
   --       {
   --          i0 = i;
   --          x0 = x;
   --       }
   --    }
   --
   --    int32 hull[b2_maxPolygonVertices];
   --    int32 m = 0;
   --    int32 ih = i0;
   --
   --    for (;;)
   --    {
   --       b2Assert(m < b2_maxPolygonVertices);
   --       hull[m] = ih;
   --
   --       int32 ie = 0;
   --       for (int32 j = 1; j < n; ++j)
   --       {
   --          if (ie == ih)
   --          {
   --             ie = j;
   --             continue;
   --          }
   --
   --          b2Vec2 r = ps[ie] - ps[hull[m]];
   --          b2Vec2 v = ps[j] - ps[hull[m]];
   --          float c = b2Cross(r, v);
   --          if (c < 0.0f)
   --          {
   --             ie = j;
   --          }
   --
   --          -- Collinearity check
   --          if (c == 0.0f && v.LengthSquared() > r.LengthSquared())
   --          {
   --             ie = j;
   --          }
   --       }
   --
   --       ++m;
   --       ih = ie;
   --
   --       if (ie == i0)
   --       {
   --          break;
   --       }
   --    }
   --
   --    if (m < 3)
   --    {
   --       -- Polygon is degenerate.
   --       b2Assert(false);
   --       SetAsBox(1.0f, 1.0f);
   --       return;
   --    }
   --
   --    m_count = m;
   --
   --    -- Copy vertices.
   --    for (int32 i = 0; i < m; ++i)
   --    {
   --       m_vertices[i] = ps[hull[i]];
   --    }
   --
   --    -- Compute normals. Ensure the edges have non-zero length.
   --    for (int32 i = 0; i < m; ++i)
   --    {
   --       int32 i1 = i;
   --       int32 i2 = i + 1 < m ? i + 1 : 0;
   --       b2Vec2 edge = m_vertices[i2] - m_vertices[i1];
   --       b2Assert(edge.LengthSquared() > b2_epsilon * b2_epsilon);
   --       m_normals[i] = b2Cross(edge, 1.0f);
   --       m_normals[i].Normalize();
   --    }
   --
   --    -- Compute the polygon centroid.
   --    m_centroid = ComputeCentroid(m_vertices, m);
   --  }
   --

   procedure set (Self : in out b2polygonShape;   Vertices : in b2Vec2s;
                                                  Count    : in Natural)
   is
      pragma assert (        3 <= Count
                     and Count <= b2_maxPolygonVertices);

      n  : Natural;
      ps : array (0 .. b2_maxPolygonVertices - 1) of b2Vec2;
   begin
      if Count < 3
      then
         Self.setAsBox (1.0, 1.0);
         return;
      end if;

      n := Natural'min (Count, b2_maxPolygonVertices);

      -- Perform welding and copy vertices into local buffer.
      --
      declare
         tempCount : Integer := 0;
         v         : b2Vec2;
         Unique    : Boolean;
      begin
         for i in 0 .. n - 1
         loop
            v      := Vertices (i);
            Unique := True;

            for j in 0 .. tempCount - 1
            loop
               if   b2DistanceSquared (v,  ps (j))
                 < (0.5 * b2_linearSlop) * (0.5 * b2_linearSlop)
               then
                  Unique := False;
                  exit;
               end if;
            end loop;

            if Unique
            then
               ps (tempCount) := v;
               tempCount      := tempCount + 1;
            end if;
         end loop;

         n := tempCount;

         if n < 3
         then     -- Polygon is degenerate.
            pragma assert (False);

            Self.setAsBox (1.0, 1.0);
            return;
         end if;
      end;

      -- Create the convex hull using the Gift wrapping algorithm.
      -- http://en.wikipedia.org/wiki/Gift_wrapping_algorithm
      --

      -- Find the right most point on the hull.
      --
      declare
         i0 : Natural := 0;
         x0 : Real    := ps (0).x;
         x  : Real;
      begin
         for i in 1 .. n - 1
         loop
            x := ps (i).x;

            if   x  > x0
              or (    x        = x0
                  and ps (i).y < ps (i0).y)
            then
               i0 := i;
               x0 := x;
            end if;
         end loop;

         declare
            Hull : array (0 .. b2_maxPolygonVertices - 1) of Natural;
            m    : Natural := 0;
            ih   : Natural := i0;
            ie   : Natural;
         begin
            loop
               pragma assert (m < b2_maxPolygonVertices);

               Hull (m) := ih;
               ie       := 0;

               for j in 1 .. n - 1
               loop
                  if ie = ih
                  then
                     ie := j;
                  else
                     declare
                        r : constant b2Vec2 := ps (ie) - ps (Hull (m));
                        v : constant b2Vec2 := ps (j)  - ps (hull (m));
                        c : constant Real   := b2Cross (r, v);
                     begin
                        if c < 0.0
                        then
                           ie := j;
                        end if;

                        -- Collinearity check.
                        --
                        if                    c = 0.0
                          and lengthSquared (v) > lengthSquared (r)
                        then
                           ie := j;
                        end if;
                     end;
                  end if;
               end loop;

               m  := m + 1;
               ih := ie;

               if ie = i0
               then
                  exit;
               end if;
            end loop;


            if m < 3
            then     -- Polygon is degenerate.
               pragma assert (False, "m =" & m'Image);

               Self.setAsBox (1.0, 1.0);
               return;
            end if;

            Self.m_Count := m;

            -- Copy vertices.
            --
            for i in 0 .. m - 1
            loop
               Self.m_Vertices (i) := ps (Hull (i));
            end loop;

            -- Compute normals. Ensure the edges have non-zero length.
            --
            for i in 0 .. m - 1
            loop
               declare
                  i1 : constant Natural := i;
                  i2 : constant Natural := (if i + 1 < m then i + 1
                                                         else 0);

                  Edge : constant b2Vec2 :=   Self.m_Vertices (i2)
                                            - Self.m_Vertices (i1);
               begin
                  pragma assert (lengthSquared (Edge) > b2_Epsilon * b2_Epsilon);

                  Self.m_Normals (i) := b2Cross (Edge, 1.0);
                  Normalize (Self.m_Normals (i));
               end;
            end loop;

            -- Compute the polygon centroid.
            --
            Self.m_Centroid := Self.computeCentroid (Self.m_Vertices, m);
         end;
      end;
   end set;



   --    Build vertices to represent an axis-aligned box centered on the local origin.
   --
   --    @param hx the half-width.
   --    @param hy the half-height.
   --
   --  void b2PolygonShape::SetAsBox(float hx, float hy)
   --  {
   --    m_count = 4;
   --    m_vertices[0].Set(-hx, -hy);
   --    m_vertices[1].Set( hx, -hy);
   --    m_vertices[2].Set( hx,  hy);
   --    m_vertices[3].Set(-hx,  hy);
   --    m_normals[0].Set(0.0f, -1.0f);
   --    m_normals[1].Set(1.0f, 0.0f);
   --    m_normals[2].Set(0.0f, 1.0f);
   --    m_normals[3].Set(-1.0f, 0.0f);
   --    m_centroid.SetZero();
   --  }
   --

   procedure setAsBox (Self : in out b2polygonShape;   hx, hy : in Real)
   is
   begin
      Self.m_Count := 4;

      set (Self.m_Vertices (0), -hx, -hy);
      set (Self.m_Vertices (1),  hx, -hy);
      set (Self.m_Vertices (2),  hx,  hy);
      set (Self.m_Vertices (3), -hx,  hy);

      set (Self.m_Normals (0),  0.0, -1.0);
      set (Self.m_Normals (1),  1.0,  0.0);
      set (Self.m_Normals (2),  0.0,  1.0);
      set (Self.m_Normals (3), -1.0,  0.0);

      setZero (Self.m_Centroid);
   end setAsBox;




   --    Build vertices to represent an oriented box.
   --
   --    @param hx the half-width.
   --    @param hy the half-height.
   --    @param center the center of the box in local coordinates.
   --    @param angle the rotation of the box in local coordinates.
   --
   --  void b2PolygonShape::SetAsBox(float hx, float hy, const b2Vec2& center, float angle)
   --  {
   --    m_count = 4;
   --    m_vertices[0].Set(-hx, -hy);
   --    m_vertices[1].Set( hx, -hy);
   --    m_vertices[2].Set( hx,  hy);
   --    m_vertices[3].Set(-hx,  hy);
   --    m_normals[0].Set(0.0f, -1.0f);
   --    m_normals[1].Set(1.0f, 0.0f);
   --    m_normals[2].Set(0.0f, 1.0f);
   --    m_normals[3].Set(-1.0f, 0.0f);
   --    m_centroid = center;
   --
   --    b2Transform xf;
   --    xf.p = center;
   --    xf.q.Set(angle);
   --
   --    -- Transform vertices and normals.
   --    for (int32 i = 0; i < m_count; ++i)
   --    {
   --       m_vertices[i] = b2Mul(xf, m_vertices[i]);
   --       m_normals[i] = b2Mul(xf.q, m_normals[i]);
   --    }
   --  }
   --

   procedure setAsBox (Self : in out b2polygonShape;   hx, hy : in Real;
                                                       Center : in b2Vec2;
                                                       Angle  : in Real)
   is
      Transform : b2Transform;
   begin
      Self.m_count := 4;

      set (Self.m_Vertices (0), -hx, -hy);
      set (Self.m_Vertices (1),  hx, -hy);
      set (Self.m_Vertices (2),  hx,  hy);
      set (Self.m_Vertices (3), -hx,  hy);

      set (Self.m_Normals (0),  0.0, -1.0);
      set (Self.m_Normals (1),  1.0,  0.0);
      set (Self.m_Normals (2),  0.0,  1.0);
      set (Self.m_Normals (3), -1.0,  0.0);

      Self.m_Centroid := Center;

      Transform.p := Center;
      set (Transform.q, Angle);

      -- Transform vertices and normals.
      --
      for i in 0 .. Self.m_Count - 1
      loop
         Self.m_Vertices (i) := b2Mul (Transform,   Self.m_Vertices (i));
         Self.m_Normals  (i) := b2Mul (Transform.q, Self.m_Normals  (i));
      end loop;
   end setAsBox;



   --    Validate convexity. This is a very time consuming operation.
   --
   --    @returns true if valid
   --
   --  bool b2PolygonShape::Validate() const
   --  {
   --    for (int32 i = 0; i < m_count; ++i)
   --    {
   --       int32 i1 = i;
   --       int32 i2 = i < m_count - 1 ? i1 + 1 : 0;
   --       b2Vec2 p = m_vertices[i1];
   --       b2Vec2 e = m_vertices[i2] - p;
   --
   --       for (int32 j = 0; j < m_count; ++j)
   --       {
   --          if (j == i1 || j == i2)
   --          {
   --             continue;
   --          }
   --
   --          b2Vec2 v = m_vertices[j] - p;
   --          float c = b2Cross(e, v);
   --          if (c < 0.0f)
   --          {
   --             return false;
   --          }
   --       }
   --    }
   --
   --    return true;
   --  }

   function validate (Self : in b2polygonShape) return Boolean
   is
   begin
     for i in 0 .. Self.m_Count - 1
      loop
         declare
            i1 : constant Natural := i;
            i2 : constant Natural := (if i < Self.m_Count - 1 then i1 + 1
                                                              else 0);
            p  : constant b2Vec2  := Self.m_Vertices (i1);
            e  : constant b2Vec2  := Self.m_Vertices (i2) - p;
         begin
            for j in 0 .. Self.m_Count - 1
            loop
               if not (   j = i1
                       or j = i2)
               then
                  declare
                     v : constant b2Vec2       := Self.m_Vertices (j) - p;
                     c : constant Real := b2Cross (e, v);
                  begin
                     if c < 0.0
                     then
                        return False;
                     end if;
                  end;
               end if;
            end loop;
         end;
      end loop;

      return True;
   end validate;


end b2_polygon_Shape;
