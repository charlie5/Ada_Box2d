with
     b2_Types,
     b2_Common;


package body b2_collide_Edge
is
   use b2_Common;



   --  static b2EPAxis b2ComputeEdgeSeparation (const b2TempPolygon& polygonB, const b2Vec2& v1, const b2Vec2& normal1)
   --  {
   --    b2EPAxis axis;
   --    axis.type = b2EPAxis::e_edgeA;
   --    axis.index = -1;
   --    axis.separation = -FLT_MAX;
   --    axis.normal.SetZero();
   --
   --    b2Vec2 axes[2] = { normal1, -normal1 };
   --
   --    // Find axis with least overlap (min-max problem)
   --    for (int32 j = 0; j < 2; ++j)
   --    {
   --       float sj = FLT_MAX;
   --
   --       // Find deepest polygon vertex along axis j
   --       for (int32 i = 0; i < polygonB.count; ++i)
   --       {
   --          float si = b2Dot(axes[j], polygonB.vertices[i] - v1);
   --          if (si < sj)
   --          {
   --             sj = si;
   --          }
   --       }
   --
   --       if (sj > axis.separation)
   --       {
   --          axis.index = j;
   --          axis.separation = sj;
   --          axis.normal = axes[j];
   --       }
   --    }
   --
   --    return axis;
   --  }

   function b2ComputeEdgeSeparation (polygonB : in b2TempPolygon;
                                     v1       : in b2Vec2;
                                     normal1  : in b2Vec2) return b2EPAxis
   is
      axis : b2EPAxis;
      axes : constant b2Vec2s (0 .. 1) := [normal1, -normal1];
   begin
      axis.Kind       := e_edgeA;
      axis.index      := -1;
      axis.separation := -Real'Last;
      setZero (axis.normal);

      -- Find axis with least overlap (min-max problem).
      --
      for j in 0 .. 1
      loop
         declare
            sj : Real := Real'Last;
            si : Real;
         begin
            -- Find deepest polygon vertex along axis j.
            --
            for i in 0 .. polygonB.count - 1
            loop
               si := b2Dot (axes (j),
                            polygonB.vertices (i) - v1);

               if si < sj
               then
                  sj := si;
               end if;
            end loop;

            if sj > axis.separation
            then
               axis.index      := j;
               axis.separation := sj;
               axis.normal     := axes (j);
            end if;
         end;
      end loop;

      return axis;
   end b2ComputeEdgeSeparation;



   --  static b2EPAxis b2ComputePolygonSeparation(const b2TempPolygon& polygonB, const b2Vec2& v1, const b2Vec2& v2)
   --  {
   --    b2EPAxis axis;
   --    axis.type = b2EPAxis::e_unknown;
   --    axis.index = -1;
   --    axis.separation = -FLT_MAX;
   --    axis.normal.SetZero();
   --
   --    for (int32 i = 0; i < polygonB.count; ++i)
   --    {
   --       b2Vec2 n = -polygonB.normals[i];
   --
   --       float s1 = b2Dot(n, polygonB.vertices[i] - v1);
   --       float s2 = b2Dot(n, polygonB.vertices[i] - v2);
   --       float s = b2Min(s1, s2);
   --
   --       if (s > axis.separation)
   --       {
   --          axis.type = b2EPAxis::e_edgeB;
   --          axis.index = i;
   --          axis.separation = s;
   --          axis.normal = n;
   --       }
   --    }
   --
   --    return axis;
   --  }
   --

   function b2computePolygonSeparation (polygonB : in b2TempPolygon;
                                        v1, v2   : in b2Vec2) return b2EPAxis
   is
      axis : b2EPAxis;
   begin
      axis.Kind       := e_unknown;
      axis.index      := -1;
      axis.separation := -Real'Last;
      setZero (axis.normal);

      for i in 0 .. polygonB.count - 1
      loop
         declare
            n  : constant b2Vec2 := -polygonB.normals (i);

            s1 : constant Real := b2Dot (n,  polygonB.vertices (i) - v1);
            s2 : constant Real := b2Dot (n,  polygonB.vertices (i) - v2);
            s  : constant Real := Real'min (s1, s2);
         begin
            if s > axis.separation
            then
               axis.Kind       := e_edgeB;
               axis.index      := Integer (i);
               axis.separation := s;
               axis.normal     := n;
            end if;
         end;
      end loop;

      return axis;
   end b2computePolygonSeparation;






   --  Compute contact points for edge versus circle.
   --  This accounts for edge connectivity.
   --
   --  void b2CollideEdgeAndCircle (b2Manifold* manifold,
   --                               const b2EdgeShape*   edgeA,   const b2Transform& xfA,
   --                               const b2CircleShape* circleB, const b2Transform& xfB)
   --  {
   --    manifold->pointCount = 0;
   --
   --    // Compute circle in frame of edge
   --    b2Vec2 Q = b2MulT(xfA, b2Mul(xfB, circleB->m_p));
   --
   --    b2Vec2 A = edgeA->m_vertex1, B = edgeA->m_vertex2;
   --    b2Vec2 e = B - A;
   --
   --    // Normal points to the right for a CCW winding
   --    b2Vec2 n(e.y, -e.x);
   --    float offset = b2Dot(n, Q - A);
   --
   --    bool oneSided = edgeA->m_oneSided;
   --    if (oneSided && offset < 0.0f)
   --    {
   --       return;
   --    }
   --
   --    // Barycentric coordinates
   --    float u = b2Dot(e, B - Q);
   --    float v = b2Dot(e, Q - A);
   --
   --    float radius = edgeA->m_radius + circleB->m_radius;
   --
   --    b2ContactFeature cf;
   --    cf.indexB = 0;
   --    cf.typeB = b2ContactFeature::e_vertex;
   --
   --    // Region A
   --    if (v <= 0.0f)
   --    {
   --       b2Vec2 P = A;
   --       b2Vec2 d = Q - P;
   --       float dd = b2Dot(d, d);
   --       if (dd > radius * radius)
   --       {
   --          return;
   --       }
   --
   --       // Is there an edge connected to A?
   --       if (edgeA->m_oneSided)
   --       {
   --          b2Vec2 A1 = edgeA->m_vertex0;
   --          b2Vec2 B1 = A;
   --          b2Vec2 e1 = B1 - A1;
   --          float u1 = b2Dot(e1, B1 - Q);
   --
   --          // Is the circle in Region AB of the previous edge?
   --          if (u1 > 0.0f)
   --          {
   --             return;
   --          }
   --       }
   --
   --       cf.indexA = 0;
   --       cf.typeA = b2ContactFeature::e_vertex;
   --       manifold->pointCount = 1;
   --       manifold->type = b2Manifold::e_circles;
   --       manifold->localNormal.SetZero();
   --       manifold->localPoint = P;
   --       manifold->points[0].id.key = 0;
   --       manifold->points[0].id.cf = cf;
   --       manifold->points[0].localPoint = circleB->m_p;
   --       return;
   --    }
   --
   --    // Region B
   --    if (u <= 0.0f)
   --    {
   --       b2Vec2 P = B;
   --       b2Vec2 d = Q - P;
   --       float dd = b2Dot(d, d);
   --       if (dd > radius * radius)
   --       {
   --          return;
   --       }
   --
   --       // Is there an edge connected to B?
   --       if (edgeA->m_oneSided)
   --       {
   --          b2Vec2 B2 = edgeA->m_vertex3;
   --          b2Vec2 A2 = B;
   --          b2Vec2 e2 = B2 - A2;
   --          float v2 = b2Dot(e2, Q - A2);
   --
   --          // Is the circle in Region AB of the next edge?
   --          if (v2 > 0.0f)
   --          {
   --             return;
   --          }
   --       }
   --
   --       cf.indexA = 1;
   --       cf.typeA = b2ContactFeature::e_vertex;
   --       manifold->pointCount = 1;
   --       manifold->type = b2Manifold::e_circles;
   --       manifold->localNormal.SetZero();
   --       manifold->localPoint = P;
   --       manifold->points[0].id.key = 0;
   --       manifold->points[0].id.cf = cf;
   --       manifold->points[0].localPoint = circleB->m_p;
   --       return;
   --    }
   --
   --    // Region AB
   --    float den = b2Dot(e, e);
   --    b2Assert(den > 0.0f);
   --    b2Vec2 P = (1.0f / den) * (u * A + v * B);
   --    b2Vec2 d = Q - P;
   --    float dd = b2Dot(d, d);
   --    if (dd > radius * radius)
   --    {
   --       return;
   --    }
   --
   --    if (offset < 0.0f)
   --    {
   --       n.Set(-n.x, -n.y);
   --    }
   --    n.Normalize();
   --
   --    cf.indexA = 0;
   --    cf.typeA = b2ContactFeature::e_face;
   --    manifold->pointCount = 1;
   --    manifold->type = b2Manifold::e_faceA;
   --    manifold->localNormal = n;
   --    manifold->localPoint = A;
   --    manifold->points[0].id.key = 0;
   --    manifold->points[0].id.cf = cf;
   --    manifold->points[0].localPoint = circleB->m_p;
   --  }
   --

   procedure b2CollideEdgeAndCircle (manifold :    out b2Manifold;
                                     edgeA    : in     b2EdgeShape;     xfA : in b2Transform;
                                     circleB  : in     b2CircleShape;   xfB : in b2Transform)
   is
      -- Compute circle in frame of edge.
      --
      Q : constant b2Vec2 := b2MulT (xfA,
                                     b2Mul (xfB, circleB.m_p));

      A : constant b2Vec2 := edgeA.m_vertex1;
      B : constant b2Vec2 := edgeA.m_vertex2;
      e : constant b2Vec2 := B - A;

      -- Normal points to the right for a CCW winding.
      --
      n        :          b2Vec2  := (e.y, -e.x);
      offset   : constant Real    := b2Dot (n,  Q - A);

      oneSided : constant Boolean := edgeA.m_oneSided;

   begin
      manifold.pointCount := 0;

      if oneSided and offset < 0.0
      then
         return;
      end if;

      -- Barycentric coordinates.
      --
      declare
         u : constant Real := b2Dot (e,  B - Q);
         v : constant Real := b2Dot (e,  Q - A);

         radius : constant Real :=   edgeA  .m_radius
                                   + circleB.m_radius;

         cf : b2ContactFeature;
      begin
        cf.indexB := 0;
        cf.typeB  := e_vertex;


         -- Region A.
         --
         if v <= 0.0
         then
            declare
               P  : constant b2Vec2 := A;
               d  : constant b2Vec2 := Q - P;
               dd : constant Real   := b2Dot (d, d);
            begin
               if dd > radius * radius
               then
                  return;
               end if;

               -- Is there an edge connected to A?
               --
               if edgeA.m_oneSided
               then
                  declare
                     A1 : constant b2Vec2 := edgeA.m_vertex0;
                     B1 : constant b2Vec2 := A;
                     e1 : constant b2Vec2 := B1 - A1;
                     u1 : constant Real   := b2Dot (e1,  B1 - Q);
                  begin
                     -- Is the circle in Region AB of the previous edge?
                     --
                     if u1 > 0.0
                     then
                        return;
                     end if;
                  end;
               end if;

               cf.indexA := 0;
               cf.typeA  := e_vertex;

               manifold.pointCount            := 1;
               manifold.Kind                  := e_circles;
               manifold.localPoint            := P;
               manifold.points (0).id.key     := 0;
               manifold.points (0).id.cf      := cf;
               manifold.points (0).localPoint := circleB.m_p;
               setZero (manifold.localNormal);

               return;
            end;
         end if;

         -- Region B.
         --
         if u <= 0.0
         then
            declare
               P  : constant b2Vec2 := B;
               d  : constant b2Vec2 := Q - P;
               dd : constant Real   := b2Dot (d, d);
            begin
               if dd > radius * radius
               then
                  return;
               end if;

               -- Is there an edge connected to B?
               --
               if edgeA.m_oneSided
               then
                  declare
                     B2 : constant b2Vec2 := edgeA.m_vertex3;
                     A2 : constant b2Vec2 := B;
                     e2 : constant b2Vec2 := B2 - A2;
                     v2 : constant Real   := b2Dot (e2,  Q - A2);
                  begin
                     -- Is the circle in Region AB of the next edge?
                     --
                     if v2 > 0.0
                     then
                        return;
                     end if;
                  end;
               end if;

               cf.indexA := 1;
               cf.typeA  := e_vertex;

               manifold.pointCount            := 1;
               manifold.Kind                  := e_circles;
               manifold.localPoint            := P;
               manifold.points (0).id.key     := 0;
               manifold.points (0).id.cf      := cf;
               manifold.points (0).localPoint := circleB.m_p;
               setZero (manifold.localNormal);

               return;
            end;
         end if;

         -- Region AB.
         --
         declare
            den : constant Real := b2Dot (e, e);
            pragma assert (den > 0.0);

            p  : constant b2Vec2 :=   (1.0 / den)
                                    * (u * A  +  v * B);
            d  : constant b2Vec2 := Q - P;
            dd : constant Real   := b2Dot (d, d);
         begin
            if dd > radius * radius
            then
               return;
            end if;

            if offset < 0.0
            then
               set (n, -n.x, -n.y);
            end if;

            normalize (n);
         end;

         cf.indexA := 0;
         cf.typeA  := e_face;

         manifold.pointCount            := 1;
         manifold.Kind                  := e_faceA;
         manifold.localNormal           := n;
         manifold.localPoint            := A;
         manifold.points (0).id.key     := 0;
         manifold.points (0).id.cf      := cf;
         manifold.points (0).localPoint := circleB.m_p;
      end;
   end b2CollideEdgeAndCircle;





   --  void b2CollideEdgeAndPolygon (b2Manifold* manifold,
   --                                const b2EdgeShape* edgeA,       const b2Transform& xfA,
   --                                const b2PolygonShape* polygonB, const b2Transform& xfB)
   --  {
   --    manifold->pointCount = 0;
   --
   --    b2Transform xf = b2MulT(xfA, xfB);
   --
   --    b2Vec2 centroidB = b2Mul(xf, polygonB->m_centroid);
   --
   --    b2Vec2 v1 = edgeA->m_vertex1;
   --    b2Vec2 v2 = edgeA->m_vertex2;
   --
   --    b2Vec2 edge1 = v2 - v1;
   --    edge1.Normalize();
   --
   --    // Normal points to the right for a CCW winding
   --    b2Vec2 normal1(edge1.y, -edge1.x);
   --    float offset1 = b2Dot(normal1, centroidB - v1);
   --
   --    bool oneSided = edgeA->m_oneSided;
   --    if (oneSided && offset1 < 0.0f)
   --    {
   --       return;
   --    }
   --
   --    // Get polygonB in frameA
   --    b2TempPolygon tempPolygonB;
   --    tempPolygonB.count = polygonB->m_count;
   --    for (int32 i = 0; i < polygonB->m_count; ++i)
   --    {
   --       tempPolygonB.vertices[i] = b2Mul(xf, polygonB->m_vertices[i]);
   --       tempPolygonB.normals[i] = b2Mul(xf.q, polygonB->m_normals[i]);
   --    }
   --
   --    float radius = polygonB->m_radius + edgeA->m_radius;
   --
   --    b2EPAxis edgeAxis = b2ComputeEdgeSeparation(tempPolygonB, v1, normal1);
   --    if (edgeAxis.separation > radius)
   --    {
   --       return;
   --    }
   --
   --    b2EPAxis polygonAxis = b2ComputePolygonSeparation(tempPolygonB, v1, v2);
   --    if (polygonAxis.separation > radius)
   --    {
   --       return;
   --    }
   --
   --    // Use hysteresis for jitter reduction.
   --    const float k_relativeTol = 0.98f;
   --    const float k_absoluteTol = 0.001f;
   --
   --    b2EPAxis primaryAxis;
   --    if (polygonAxis.separation - radius > k_relativeTol * (edgeAxis.separation - radius) + k_absoluteTol)
   --    {
   --       primaryAxis = polygonAxis;
   --    }
   --    else
   --    {
   --       primaryAxis = edgeAxis;
   --    }
   --
   --    if (oneSided)
   --    {
   --       // Smooth collision
   --       // See https://box2d.org/posts/2020/06/ghost-collisions/
   --
   --       b2Vec2 edge0 = v1 - edgeA->m_vertex0;
   --       edge0.Normalize();
   --       b2Vec2 normal0(edge0.y, -edge0.x);
   --       bool convex1 = b2Cross(edge0, edge1) >= 0.0f;
   --
   --       b2Vec2 edge2 = edgeA->m_vertex3 - v2;
   --       edge2.Normalize();
   --       b2Vec2 normal2(edge2.y, -edge2.x);
   --       bool convex2 = b2Cross(edge1, edge2) >= 0.0f;
   --
   --       const float sinTol = 0.1f;
   --       bool side1 = b2Dot(primaryAxis.normal, edge1) <= 0.0f;
   --
   --       // Check Gauss Map
   --       if (side1)
   --       {
   --          if (convex1)
   --          {
   --             if (b2Cross(primaryAxis.normal, normal0) > sinTol)
   --             {
   --                // Skip region
   --                return;
   --             }
   --
   --             // Admit region
   --          }
   --          else
   --          {
   --             // Snap region
   --             primaryAxis = edgeAxis;
   --          }
   --       }
   --       else
   --       {
   --          if (convex2)
   --          {
   --             if (b2Cross(normal2, primaryAxis.normal) > sinTol)
   --             {
   --                // Skip region
   --                return;
   --             }
   --
   --             // Admit region
   --          }
   --          else
   --          {
   --             // Snap region
   --             primaryAxis = edgeAxis;
   --          }
   --       }
   --    }
   --
   --    b2ClipVertex clipPoints[2];
   --    b2ReferenceFace ref;
   --    if (primaryAxis.type == b2EPAxis::e_edgeA)
   --    {
   --       manifold->type = b2Manifold::e_faceA;
   --
   --       // Search for the polygon normal that is most anti-parallel to the edge normal.
   --       int32 bestIndex = 0;
   --       float bestValue = b2Dot(primaryAxis.normal, tempPolygonB.normals[0]);
   --       for (int32 i = 1; i < tempPolygonB.count; ++i)
   --       {
   --          float value = b2Dot(primaryAxis.normal, tempPolygonB.normals[i]);
   --          if (value < bestValue)
   --          {
   --             bestValue = value;
   --             bestIndex = i;
   --          }
   --       }
   --
   --       int32 i1 = bestIndex;
   --       int32 i2 = i1 + 1 < tempPolygonB.count ? i1 + 1 : 0;
   --
   --       clipPoints[0].v = tempPolygonB.vertices[i1];
   --       clipPoints[0].id.cf.indexA = 0;
   --       clipPoints[0].id.cf.indexB = static_cast<uint8>(i1);
   --       clipPoints[0].id.cf.typeA = b2ContactFeature::e_face;
   --       clipPoints[0].id.cf.typeB = b2ContactFeature::e_vertex;
   --
   --       clipPoints[1].v = tempPolygonB.vertices[i2];
   --       clipPoints[1].id.cf.indexA = 0;
   --       clipPoints[1].id.cf.indexB = static_cast<uint8>(i2);
   --       clipPoints[1].id.cf.typeA = b2ContactFeature::e_face;
   --       clipPoints[1].id.cf.typeB = b2ContactFeature::e_vertex;
   --
   --       ref.i1 = 0;
   --       ref.i2 = 1;
   --       ref.v1 = v1;
   --       ref.v2 = v2;
   --       ref.normal = primaryAxis.normal;
   --       ref.sideNormal1 = -edge1;
   --       ref.sideNormal2 = edge1;
   --    }
   --    else
   --    {
   --       manifold->type = b2Manifold::e_faceB;
   --
   --       clipPoints[0].v = v2;
   --       clipPoints[0].id.cf.indexA = 1;
   --       clipPoints[0].id.cf.indexB = static_cast<uint8>(primaryAxis.index);
   --       clipPoints[0].id.cf.typeA = b2ContactFeature::e_vertex;
   --       clipPoints[0].id.cf.typeB = b2ContactFeature::e_face;
   --
   --       clipPoints[1].v = v1;
   --       clipPoints[1].id.cf.indexA = 0;
   --       clipPoints[1].id.cf.indexB = static_cast<uint8>(primaryAxis.index);
   --       clipPoints[1].id.cf.typeA = b2ContactFeature::e_vertex;
   --       clipPoints[1].id.cf.typeB = b2ContactFeature::e_face;
   --
   --       ref.i1 = primaryAxis.index;
   --       ref.i2 = ref.i1 + 1 < tempPolygonB.count ? ref.i1 + 1 : 0;
   --       ref.v1 = tempPolygonB.vertices[ref.i1];
   --       ref.v2 = tempPolygonB.vertices[ref.i2];
   --       ref.normal = tempPolygonB.normals[ref.i1];
   --
   --       // CCW winding
   --       ref.sideNormal1.Set(ref.normal.y, -ref.normal.x);
   --       ref.sideNormal2 = -ref.sideNormal1;
   --    }
   --
   --    ref.sideOffset1 = b2Dot(ref.sideNormal1, ref.v1);
   --    ref.sideOffset2 = b2Dot(ref.sideNormal2, ref.v2);
   --
   --    // Clip incident edge against reference face side planes
   --    b2ClipVertex clipPoints1[2];
   --    b2ClipVertex clipPoints2[2];
   --    int32 np;
   --
   --    // Clip to side 1
   --    np = b2ClipSegmentToLine(clipPoints1, clipPoints, ref.sideNormal1, ref.sideOffset1, ref.i1);
   --
   --    if (np < b2_maxManifoldPoints)
   --    {
   --       return;
   --    }
   --
   --    // Clip to side 2
   --    np = b2ClipSegmentToLine(clipPoints2, clipPoints1, ref.sideNormal2, ref.sideOffset2, ref.i2);
   --
   --    if (np < b2_maxManifoldPoints)
   --    {
   --       return;
   --    }
   --
   --    // Now clipPoints2 contains the clipped points.
   --    if (primaryAxis.type == b2EPAxis::e_edgeA)
   --    {
   --       manifold->localNormal = ref.normal;
   --       manifold->localPoint = ref.v1;
   --    }
   --    else
   --    {
   --       manifold->localNormal = polygonB->m_normals[ref.i1];
   --       manifold->localPoint = polygonB->m_vertices[ref.i1];
   --    }
   --
   --    int32 pointCount = 0;
   --    for (int32 i = 0; i < b2_maxManifoldPoints; ++i)
   --    {
   --       float separation;
   --
   --       separation = b2Dot(ref.normal, clipPoints2[i].v - ref.v1);
   --
   --       if (separation <= radius)
   --       {
   --          b2ManifoldPoint* cp = manifold->points + pointCount;
   --
   --          if (primaryAxis.type == b2EPAxis::e_edgeA)
   --          {
   --             cp->localPoint = b2MulT(xf, clipPoints2[i].v);
   --             cp->id = clipPoints2[i].id;
   --          }
   --          else
   --          {
   --             cp->localPoint = clipPoints2[i].v;
   --             cp->id.cf.typeA = clipPoints2[i].id.cf.typeB;
   --             cp->id.cf.typeB = clipPoints2[i].id.cf.typeA;
   --             cp->id.cf.indexA = clipPoints2[i].id.cf.indexB;
   --             cp->id.cf.indexB = clipPoints2[i].id.cf.indexA;
   --          }
   --
   --          ++pointCount;
   --       }
   --    }
   --
   --    manifold->pointCount = pointCount;
   --  }

   procedure b2CollideEdgeAndPolygon (manifold :    out b2Manifold;
                                      edgeA    : in     b2edgeShape;      xfA : in b2Transform;
                                      polygonB : in     b2polygonShape;   xfB : in b2Transform)
   is
      xf        : constant b2Transform := b2MulT (xfA, xfB);
      centroidB : constant b2Vec2      := b2Mul (xf, polygonB.m_centroid);

      v1 : constant b2Vec2 := edgeA.m_vertex1;
      v2 : constant b2Vec2 := edgeA.m_vertex2;

      edge1 : b2Vec2 := v2 - v1;

      -- Normal points to the right for a CCW winding.
      --
      normal1  : b2Vec2;
      offset1  : Real;

      oneSided : constant Boolean := edgeA.m_oneSided;

   begin
      normalize (edge1);

      -- Normal points to the right for a CCW winding.
      --
      normal1 := (edge1.y, -edge1.x);
      offset1 := b2Dot (normal1,  centroidB - v1);

      manifold.pointCount := 0;

      if oneSided and offset1 < 0.0
      then
         return;
      end if;

      -- Get polygonB in frameA.
      --
      declare
         tempPolygonB : b2TempPolygon;
         Radius       : Real;
         edgeAxis     : b2EPAxis;
         polygonAxis  : b2EPAxis;
         primaryAxis  : b2EPAxis;

         k_relativeTol : constant := 0.98;
         k_absoluteTol : constant := 0.001;
      begin
         tempPolygonB.count := Unsigned_8 (polygonB.m_count);

         for i in 0 .. polygonB.m_count - 1
         loop
            tempPolygonB.vertices (Unsigned_8 (i)) := b2Mul (xf,    polygonB.m_vertices (i));
            tempPolygonB.normals  (Unsigned_8 (i)) := b2Mul (xf.q,  polygonB.m_normals  (i));
         end loop;

         radius   := polygonB.m_radius + edgeA.m_radius;
         edgeAxis := b2ComputeEdgeSeparation (tempPolygonB, v1, normal1);

         if edgeAxis.separation > radius
         then
            return;
         end if;

         polygonAxis := b2ComputePolygonSeparation (tempPolygonB, v1, v2);

         if polygonAxis.separation > radius
         then
            return;
         end if;

         -- Use hysteresis for jitter reduction.
         --
         if   polygonAxis.separation - radius
            > k_relativeTol * (edgeAxis.separation - radius) + k_absoluteTol
         then
            primaryAxis := polygonAxis;
         else
            primaryAxis := edgeAxis;
         end if;

        if oneSided
        then
           -- Smooth collision.
           -- See https://box2d.org/posts/2020/06/ghost-collisions.
           --
            declare
               edge0   : b2Vec2;
               normal0 : b2Vec2;
               convex1 : Boolean;

               edge2   : b2Vec2;
               normal2 : b2Vec2;
               convex2 : Boolean;

               sinTol  : constant := 0.1;
               side1   : Boolean;

            begin
               edge0 := v1 - edgeA.m_vertex0;
               normalize (edge0);

               normal0 := (edge0.y, -edge0.x);
               convex1 := b2Cross (edge0, edge1) >= 0.0;

               edge2 := edgeA.m_vertex3 - v2;
               normalize (edge2);

               normal2 := (edge2.y, -edge2.x);
               convex2 := b2Cross (edge1, edge2)              >= 0.0;
               side1   := b2Dot   (primaryAxis.normal, edge1) <= 0.0;

               -- Check Gauss Map.
               --
               if side1
               then
                  if convex1
                  then
                     if b2Cross (primaryAxis.normal, normal0) > sinTol
                     then
                        -- Skip region.
                        return;
                     end if;

                     -- Admit region.
                  else
                     -- Snap region.
                     primaryAxis := edgeAxis;
                  end if;

               else
                  if convex2
                  then
                     if b2Cross (normal2, primaryAxis.normal) > sinTol
                     then
                        -- Skip region.
                        return;
                     end if;

                     -- Admit region.
                  else
                     -- Snap region.
                     primaryAxis := edgeAxis;
                  end if;
               end if;
            end;
         end if;

         declare
            use b2_Types;

            --  clipPoints : b2ClipVertices (0 .. 1);
            clipPoints : b2ClipVertex_Pair;
            ref        : b2ReferenceFace;

            bestIndex  : Unsigned_8 := 0;
            bestValue  : Real       := b2Dot (primaryAxis.normal,
                                              tempPolygonB.normals (0));
         begin
            if primaryAxis.Kind = e_edgeA
            then
               manifold.Kind := e_faceA;

               -- Search for the polygon normal that is most anti-parallel to the edge normal.
               --
               for i in 1 .. tempPolygonB.count - 1
               loop
                  declare
                     value : constant Real := b2Dot (primaryAxis.normal,
                                                     tempPolygonB.normals (i));
                  begin
                     if value < bestValue
                     then
                        bestValue := value;
                        bestIndex := i;
                     end if;
                  end;
               end loop;

               declare
                  i1 : constant Unsigned_8 := bestIndex;
                  i2 : constant Unsigned_8 := (if i1 + 1 < tempPolygonB.count then i1 + 1
                                                                              else 0);
               begin
                  clipPoints (0).v            := tempPolygonB.vertices (i1);
                  clipPoints (0).id.cf.indexA := 0;
                  clipPoints (0).id.cf.indexB := i1;
                  clipPoints (0).id.cf.typeA  := e_face;
                  clipPoints (0).id.cf.typeB  := e_vertex;

                  clipPoints (1).v            := tempPolygonB.vertices (i2);
                  clipPoints (1).id.cf.indexA := 0;
                  clipPoints (1).id.cf.indexB := i2;
                  clipPoints (1).id.cf.typeA  := e_face;
                  clipPoints (1).id.cf.typeB  := e_vertex;

                  ref.i1 := 0;
                  ref.i2 := 1;
                  ref.v1 := v1;
                  ref.v2 := v2;
                  ref.normal      := primaryAxis.normal;
                  ref.sideNormal1 := -edge1;
                  ref.sideNormal2 :=  edge1;
               end;
            else
               manifold.Kind := e_faceB;

               clipPoints (0).v            := v2;
               clipPoints (0).id.cf.indexA := 1;
               clipPoints (0).id.cf.indexB := Unsigned_8 (primaryAxis.index);
               clipPoints (0).id.cf.typeA  := e_vertex;
               clipPoints (0).id.cf.typeB  := e_face;

               clipPoints (1).v            := v1;
               clipPoints (1).id.cf.indexA := 0;
               clipPoints (1).id.cf.indexB := Unsigned_8 (primaryAxis.index);
               clipPoints (1).id.cf.typeA  := e_vertex;
               clipPoints (1).id.cf.typeB  := e_face;

               ref.i1     := primaryAxis.index;
               ref.i2     := (if ref.i1 + 1 < Natural (tempPolygonB.count) then ref.i1 + 1
                                                                           else 0);
               ref.v1     := tempPolygonB.vertices (Unsigned_8 (ref.i1));
               ref.v2     := tempPolygonB.vertices (Unsigned_8 (ref.i2));
               ref.normal := tempPolygonB.normals  (Unsigned_8 (ref.i1));

               -- CCW winding.
               --
               set (ref.sideNormal1,  ref.normal.y,
                                     -ref.normal.x);
               ref.sideNormal2 := -ref.sideNormal1;
            end if;

            ref.sideOffset1 := b2Dot (ref.sideNormal1, ref.v1);
            ref.sideOffset2 := b2Dot (ref.sideNormal2, ref.v2);

            -- Clip incident edge against reference face side planes.
            --
            declare
               clipPoints1 : b2ClipVertex_Pair;
               clipPoints2 : b2ClipVertex_Pair;

               np          : Natural;
               pointCount  : Natural;
            begin
               -- Clip to side 1.
               --
               np := b2clipSegmentToLine (clipPoints1,
                                          clipPoints,
                                          ref.sideNormal1,
                                          ref.sideOffset1,
                                          ref.i1);
               if np < b2_maxManifoldPoints
               then
                  return;
               end if;

               -- Clip to side 2
               --
               np := b2clipSegmentToLine (clipPoints2,
                                          clipPoints1,
                                          ref.sideNormal2,
                                          ref.sideOffset2,
                                          ref.i2);
               if np < b2_maxManifoldPoints
               then
                  return;
               end if;

               -- Now clipPoints2 contains the clipped points.
               --
               if primaryAxis.Kind = e_edgeA
               then
                  manifold.localNormal := ref.normal;
                  manifold.localPoint  := ref.v1;
               else
                  manifold.localNormal := polygonB.m_normals  (ref.i1);
                  manifold.localPoint  := polygonB.m_vertices (ref.i1);
               end if;

               pointCount := 0;

               for i in 0 .. b2_maxManifoldPoints - 1
               loop
                  declare
                     cp         : access   b2ManifoldPoint;
                     separation : constant Real := b2Dot (ref.normal,
                                                          clipPoints2 (i).v - ref.v1);
                  begin
                     if separation <= radius
                     then
                        cp := manifold.points (pointCount - 1)'Access;

                        if primaryAxis.Kind = e_edgeA
                        then
                           cp.localPoint := b2MulT (xf, clipPoints2 (i).v);
                           cp.id         := clipPoints2 (i).id;
                        else
                           cp.localPoint   := clipPoints2 (i).v;
                           cp.id.cf.typeA  := clipPoints2 (i).id.cf.typeB;
                           cp.id.cf.typeB  := clipPoints2 (i).id.cf.typeA;
                           cp.id.cf.indexA := clipPoints2 (i).id.cf.indexB;
                           cp.id.cf.indexB := clipPoints2 (i).id.cf.indexA;
                        end if;

                        pointCount := pointCount + 1;
                     end if;
                  end;
               end loop;

               manifold.pointCount := pointCount;
            end;
         end;
      end;
   end b2CollideEdgeAndPolygon;


end b2_collide_Edge;
