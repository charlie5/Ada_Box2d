with
     b2_Collision,
     b2_polygon_Shape,
     b2_Math;


package b2_collide_Polygon
is
   procedure dummy;


   --
--  // Find the max separation between poly1 and poly2 using edge normals from poly1.
--  static float b2FindMaxSeparation(int32* edgeIndex,
--                          const b2PolygonShape* poly1, const b2Transform& xf1,
--                          const b2PolygonShape* poly2, const b2Transform& xf2)
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
--  static void b2FindIncidentEdge(b2ClipVertex c[2],
--                       const b2PolygonShape* poly1, const b2Transform& xf1, int32 edge1,
--                       const b2PolygonShape* poly2, const b2Transform& xf2)
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
--  // Find edge normal of max separation on A - return if separating axis is found
--  // Find edge normal of max separation on B - return if separation axis is found
--  // Choose reference edge as min(minA, minB)
--  // Find incident edge
--  // Clip
--


   use b2_Collision,
       b2_polygon_Shape,
       b2_Math;

   --  Compute the collision manifold between two polygons.
   --
   --  void b2CollidePolygons(b2Manifold* manifold,
   --                         const b2PolygonShape* polygonA, const b2Transform& xfA,
   --                         const b2PolygonShape* polygonB, const b2Transform& xfB);

   procedure b2CollidePolygons (manifold :    out b2Manifold;
                                polygonA : in     b2PolygonShape;   xfA : in b2Transform;
                                polygonB : in     b2PolygonShape;   xfB : in b2Transform);

end b2_collide_Polygon;
