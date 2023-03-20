with
     b2_Common,
     Interfaces;


package body b2_collide_Polygon
is
   use Interfaces;


   --  // Find the max separation between poly1 and poly2 using edge normals from poly1.
   --
   --  static float b2FindMaxSeparation (int32* edgeIndex,
   --                                    const b2PolygonShape* poly1, const b2Transform& xf1,
   --                                    const b2PolygonShape* poly2, const b2Transform& xf2)
   --  {
   --    int32 count1 = poly1->m_count;
   --    int32 count2 = poly2->m_count;
   --    const b2Vec2* n1s = poly1->m_normals;
   --    const b2Vec2* v1s = poly1->m_vertices;
   --    const b2Vec2* v2s = poly2->m_vertices;
   --    b2Transform xf = b2MulT(xf2, xf1);
   --
   --    int32 bestIndex = 0;
   --    float maxSeparation = -b2_maxFloat;
   --    for (int32 i = 0; i < count1; ++i)
   --    {
   --       // Get poly1 normal in frame2.
   --       b2Vec2 n = b2Mul(xf.q, n1s[i]);
   --       b2Vec2 v1 = b2Mul(xf, v1s[i]);
   --
   --       // Find deepest point for normal i.
   --       float si = b2_maxFloat;
   --       for (int32 j = 0; j < count2; ++j)
   --       {
   --          float sij = b2Dot(n, v2s[j] - v1);
   --          if (sij < si)
   --          {
   --             si = sij;
   --          }
   --       }
   --
   --       if (si > maxSeparation)
   --       {
   --          maxSeparation = si;
   --          bestIndex = i;
   --       }
   --    }
   --
   --    *edgeIndex = bestIndex;
   --    return maxSeparation;
   --  }
   --

   function b2FindMaxSeparation (edgeIndex : out Natural;   poly1 : in b2PolygonShape;   xf1 : in b2Transform;
                                                            poly2 : in b2PolygonShape;   xf2 : in b2Transform) return Real
   is
      count1 : constant Natural := poly1.m_count;
      count2 : constant Natural := poly2.m_count;

      n1s    : b2Vec2s renames poly1.m_normals;
      v1s    : b2Vec2s renames poly1.m_vertices;
      v2s    : b2Vec2s renames poly2.m_vertices;

      xf     : constant b2Transform := b2MulT (xf2, xf1);

      bestIndex     : Natural := 0;
      maxSeparation : Real    := -Real'Last;

   begin
      for i in 0 .. count1 - 1
      loop
         declare
            -- Get poly1 normal in frame2.
            --
            n   : constant b2Vec2 := b2Mul (xf.q, n1s (i));
            v1  : constant b2Vec2 := b2Mul (xf,   v1s (i));

            si  : Real   := Real'Last;
            sij : Real;
         begin
            -- Find deepest point for normal i.
            for j in 0 .. count2 - 1
            loop
               sij := b2Dot (n,
                             v2s (j) - v1);
               if sij < si
               then
                  si := sij;
               end if;
            end loop;

            if si > maxSeparation
            then
               maxSeparation := si;
               bestIndex     := i;
            end if;
         end;
      end loop;

      edgeIndex := bestIndex;

      return maxSeparation;
   end b2FindMaxSeparation;




   --  static void b2FindIncidentEdge (b2ClipVertex c[2],
   --                                  const b2PolygonShape* poly1, const b2Transform& xf1, int32 edge1,
   --                                  const b2PolygonShape* poly2, const b2Transform& xf2)
   --  {
   --    const b2Vec2* normals1 = poly1->m_normals;
   --
   --    int32 count2 = poly2->m_count;
   --    const b2Vec2* vertices2 = poly2->m_vertices;
   --    const b2Vec2* normals2 = poly2->m_normals;
   --
   --    b2Assert(0 <= edge1 && edge1 < poly1->m_count);
   --
   --    // Get the normal of the reference edge in poly2's frame.
   --    b2Vec2 normal1 = b2MulT(xf2.q, b2Mul(xf1.q, normals1[edge1]));
   --
   --    // Find the incident edge on poly2.
   --    int32 index = 0;
   --    float minDot = b2_maxFloat;
   --    for (int32 i = 0; i < count2; ++i)
   --    {
   --       float dot = b2Dot(normal1, normals2[i]);
   --       if (dot < minDot)
   --       {
   --          minDot = dot;
   --          index = i;
   --       }
   --    }
   --
   --    // Build the clip vertices for the incident edge.
   --    int32 i1 = index;
   --    int32 i2 = i1 + 1 < count2 ? i1 + 1 : 0;
   --
   --    c[0].v = b2Mul(xf2, vertices2[i1]);
   --    c[0].id.cf.indexA = (uint8)edge1;
   --    c[0].id.cf.indexB = (uint8)i1;
   --    c[0].id.cf.typeA = b2ContactFeature::e_face;
   --    c[0].id.cf.typeB = b2ContactFeature::e_vertex;
   --
   --    c[1].v = b2Mul(xf2, vertices2[i2]);
   --    c[1].id.cf.indexA = (uint8)edge1;
   --    c[1].id.cf.indexB = (uint8)i2;
   --    c[1].id.cf.typeA = b2ContactFeature::e_face;
   --    c[1].id.cf.typeB = b2ContactFeature::e_vertex;
   --  }
   --

   procedure b2FindIncidentEdge (c : out b2ClipVertex_Pair;   poly1 : in b2PolygonShape;   xf1 : in b2Transform;
                                                              Edge1 : in Natural;
                                                              poly2 : in b2PolygonShape;   xf2 : in b2Transform)
   is
      normals1  :          b2Vec2s renames poly1.m_normals;
      count2    : constant Natural :=      poly2.m_count;

      vertices2 : b2Vec2s renames poly2.m_vertices;
      normals2  : b2Vec2s renames poly2.m_normals;

      pragma assert (0 <= edge1 and edge1 < poly1.m_count);

      -- Get the normal of the reference edge in poly2's frame.
      --
      normal1 : constant b2Vec2 := b2MulT (xf2.q,
                                           b2Mul (xf1.q,
                                         normals1 (edge1)));
      -- Find the incident edge on poly2.
      --
      index  : Natural := 0;
      minDot : Real    := Real'Last;
      dot    : Real;
   begin
      for i in 0 .. count2 - 1
      loop
         dot := b2Dot (normal1,
                       normals2 (i));
         if dot < minDot
         then
            minDot := dot;
            index  := i;
         end if;
      end loop;

      -- Build the clip vertices for the incident edge.
      --
      declare
         i1 : constant Natural := index;
         i2 : constant Natural := (if i1 + 1 < count2 then i1 + 1
                                                      else 0);
      begin
         c (0).v          := b2Mul (xf2,  vertices2 (i1));
         c (0).id.cf.indexA := Unsigned_8 (edge1);
         c (0).id.cf.indexB := Unsigned_8 (i1);
         c (0).id.cf.typeA  := e_face;
         c (0).id.cf.typeB  := e_vertex;

         c (1).v            := b2Mul (xf2,  vertices2 (i2));
         c (1).id.cf.indexA := Unsigned_8 (edge1);
         c (1).id.cf.indexB := Unsigned_8 (i2);
         c (1).id.cf.typeA  := e_face;
         c (1).id.cf.typeB  := e_vertex;
      end;
   end b2FindIncidentEdge;




   --  // Find edge normal of max separation on A - return if separating axis is found
   --  // Find edge normal of max separation on B - return if separation axis is found
   --  // Choose reference edge as min(minA, minB)
   --  // Find incident edge
   --  // Clip
   --
   --  // The normal points from 1 to 2
   --
   --  void b2CollidePolygons (b2Manifold* manifold,
   --                          const b2PolygonShape* polyA, const b2Transform& xfA,
   --                          const b2PolygonShape* polyB, const b2Transform& xfB)
   --  {
   --    manifold->pointCount = 0;
   --    float totalRadius = polyA->m_radius + polyB->m_radius;
   --
   --    int32 edgeA = 0;
   --    float separationA = b2FindMaxSeparation(&edgeA, polyA, xfA, polyB, xfB);
   --    if (separationA > totalRadius)
   --       return;
   --
   --    int32 edgeB = 0;
   --    float separationB = b2FindMaxSeparation(&edgeB, polyB, xfB, polyA, xfA);
   --    if (separationB > totalRadius)
   --       return;
   --
   --    const b2PolygonShape* poly1;  // reference polygon
   --    const b2PolygonShape* poly2;  // incident polygon
   --    b2Transform xf1, xf2;
   --    int32 edge1;               // reference edge
   --    uint8 flip;
   --    const float k_tol = 0.1f * b2_linearSlop;
   --
   --    if (separationB > separationA + k_tol)
   --    {
   --       poly1 = polyB;
   --       poly2 = polyA;
   --       xf1 = xfB;
   --       xf2 = xfA;
   --       edge1 = edgeB;
   --       manifold->type = b2Manifold::e_faceB;
   --       flip = 1;
   --    }
   --    else
   --    {
   --       poly1 = polyA;
   --       poly2 = polyB;
   --       xf1 = xfA;
   --       xf2 = xfB;
   --       edge1 = edgeA;
   --       manifold->type = b2Manifold::e_faceA;
   --       flip = 0;
   --    }
   --
   --    b2ClipVertex incidentEdge[2];
   --    b2FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);
   --
   --    int32 count1 = poly1->m_count;
   --    const b2Vec2* vertices1 = poly1->m_vertices;
   --
   --    int32 iv1 = edge1;
   --    int32 iv2 = edge1 + 1 < count1 ? edge1 + 1 : 0;
   --
   --    b2Vec2 v11 = vertices1[iv1];
   --    b2Vec2 v12 = vertices1[iv2];
   --
   --    b2Vec2 localTangent = v12 - v11;
   --    localTangent.Normalize();
   --
   --    b2Vec2 localNormal = b2Cross(localTangent, 1.0f);
   --    b2Vec2 planePoint = 0.5f * (v11 + v12);
   --
   --    b2Vec2 tangent = b2Mul(xf1.q, localTangent);
   --    b2Vec2 normal = b2Cross(tangent, 1.0f);
   --
   --    v11 = b2Mul(xf1, v11);
   --    v12 = b2Mul(xf1, v12);
   --
   --    // Face offset.
   --    float frontOffset = b2Dot(normal, v11);
   --
   --    // Side offsets, extended by polytope skin thickness.
   --    float sideOffset1 = -b2Dot(tangent, v11) + totalRadius;
   --    float sideOffset2 = b2Dot(tangent, v12) + totalRadius;
   --
   --    // Clip incident edge against extruded edge1 side edges.
   --    b2ClipVertex clipPoints1[2];
   --    b2ClipVertex clipPoints2[2];
   --    int np;
   --
   --    // Clip to box side 1
   --    np = b2ClipSegmentToLine(clipPoints1, incidentEdge, -tangent, sideOffset1, iv1);
   --
   --    if (np < 2)
   --       return;
   --
   --    // Clip to negative box side 1
   --    np = b2ClipSegmentToLine(clipPoints2, clipPoints1,  tangent, sideOffset2, iv2);
   --
   --    if (np < 2)
   --    {
   --       return;
   --    }
   --
   --    // Now clipPoints2 contains the clipped points.
   --    manifold->localNormal = localNormal;
   --    manifold->localPoint = planePoint;
   --
   --    int32 pointCount = 0;
   --    for (int32 i = 0; i < b2_maxManifoldPoints; ++i)
   --    {
   --       float separation = b2Dot(normal, clipPoints2[i].v) - frontOffset;
   --
   --       if (separation <= totalRadius)
   --       {
   --          b2ManifoldPoint* cp = manifold->points + pointCount;
   --          cp->localPoint = b2MulT(xf2, clipPoints2[i].v);
   --          cp->id = clipPoints2[i].id;
   --          if (flip)
   --          {
   --             // Swap features
   --             b2ContactFeature cf = cp->id.cf;
   --             cp->id.cf.indexA = cf.indexB;
   --             cp->id.cf.indexB = cf.indexA;
   --             cp->id.cf.typeA = cf.typeB;
   --             cp->id.cf.typeB = cf.typeA;
   --          }
   --          ++pointCount;
   --       }
   --    }
   --
   --    manifold->pointCount = pointCount;
   --  }

   procedure b2CollidePolygons (manifold : out b2Manifold;   polygonA : in b2PolygonShape;   xfA : in b2Transform;
                                                             polygonB : in b2PolygonShape;   xfB : in b2Transform)
   is
      totalRadius : constant Real :=   polygonA.m_radius
                                     + polygonB.m_radius;
      edgeA       :          Natural := 0;
      separationA : constant Real    := b2findMaxSeparation (edgeA, polygonA, xfA,
                                                                    polygonB, xfB);
      edgeB       : Natural;
      separationB : Real;

   begin
      manifold.pointCount := 0;

      if separationA > totalRadius
      then
         return;
      end if;

      separationB := b2findMaxSeparation (edgeB, polygonB, xfB,
                                                 polygonA, xfA);
      if separationB > totalRadius
      then
         return;
      end if;

      declare
         use b2_Common;

         poly1    : access constant b2PolygonShape;     -- Reference polygon.
         poly2    : access constant b2PolygonShape;     -- Incident polygon.

         xf1, xf2 : b2Transform;
         edge1    : Natural;            -- Reference edge.
         flip     : Boolean;
         k_tol    : constant := 0.1 * b2_linearSlop;

         incidentEdge : b2ClipVertex_Pair;
      begin
         if separationB > separationA + k_tol
         then
            poly1 := polygonB'Access;
            poly2 := polygonA'Access;
            xf1   := xfB;
            xf2   := xfA;
            edge1 := edgeB;
            flip  := True;
            manifold.Kind := e_faceB;
         else
            poly1 := polygonA'Access;
            poly2 := polygonB'Access;
            xf1   := xfA;
            xf2   := xfB;
            edge1 := edgeA;
            flip  := False;
            manifold.Kind := e_faceA;
         end if;

         b2findIncidentEdge (incidentEdge, poly1.all, xf1,
                             edge1,
                             poly2.all, xf2);
         declare
            count1    : constant Natural :=      poly1.m_count;
            vertices1 :          b2Vec2s renames poly1.m_vertices;

            iv1 : constant Natural := edge1;
            iv2 : constant Natural := (if edge1 + 1 < count1 then edge1 + 1
                                                             else 0);
            v11 : b2Vec2 := vertices1 (iv1);
            v12 : b2Vec2 := vertices1 (iv2);

            localTangent : b2Vec2 := v12 - v11;

            localNormal : b2Vec2;
            planePoint  : b2Vec2;

            tangent : b2Vec2;
            normal  : b2Vec2;
         begin
            normalize (localTangent);

            localNormal := b2Cross (localTangent, 1.0);
            planePoint  := 0.5 * (v11 + v12);

            tangent     := b2Mul (xf1.q, localTangent);
            normal      := b2Cross (tangent, 1.0);

            v11         := b2Mul (xf1, v11);
            v12         := b2Mul (xf1, v12);

            declare
               -- Face offset.
               frontOffset : constant Real := b2Dot (normal, v11);

               -- Side offsets, extended by polytope skin thickness.
               sideOffset1 : constant Real := -b2Dot (tangent, v11) + totalRadius;
               sideOffset2 : constant Real :=  b2Dot (tangent, v12) + totalRadius;

               -- Clip incident edge against extruded edge1 side edges.
               clipPoints1 : b2ClipVertex_Pair;
               clipPoints2 : b2ClipVertex_Pair;
               np          : Natural;

               pointCount  : Natural := 0;
            begin
               -- Clip to box side 1.
               --
               np := b2clipSegmentToLine (clipPoints1,
                                          incidentEdge,
                                          -tangent,
                                          sideOffset1,
                                          iv1);

               if np < 2
               then
                  return;
               end if;

               -- Clip to negative box side 1.
               --
               np := b2clipSegmentToLine (clipPoints2,
                                          clipPoints1,
                                          tangent,
                                          sideOffset2,
                                          iv2);

               if np < 2
               then
                  return;
               end if;

               -- Now clipPoints2 contains the clipped points.
               --
               manifold.localNormal := localNormal;
               manifold.localPoint  := planePoint;

               for i in 0 .. b2_maxManifoldPoints - 1
               loop
                  declare
                     separation : constant Real :=   b2Dot (normal,
                                                            clipPoints2 (i).v)
                                                   - frontOffset;
                  begin
                     if separation <= totalRadius
                     then
                        declare
                           cp : constant access b2ManifoldPoint := manifold.points (pointCount)'Access;
                           cf :                 b2ContactFeature;
                        begin
                           cp.localPoint := b2MulT (xf2,  clipPoints2 (i).v);
                           cp.id         := clipPoints2 (i).id;

                           if flip
                           then
                              -- Swap features
                              --
                              cf              := cp.id.cf;
                              cp.id.cf.indexA := cf.indexB;
                              cp.id.cf.indexB := cf.indexA;
                              cp.id.cf.typeA  := cf.typeB;
                              cp.id.cf.typeB  := cf.typeA;
                           end if;
                        end;
                        pointCount := pointCount + 1;
                     end if;
                  end;
               end loop;

               manifold.pointCount := pointCount;
            end;
         end;
      end;
   end b2CollidePolygons;



end b2_collide_Polygon;
