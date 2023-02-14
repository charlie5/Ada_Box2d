with
     b2_Settings,
     b2_Common;


package body b2_collide_Circle
is
   use b2_Settings,
       b2_Common;


   --  void b2CollideCircles
   --    (b2Manifold* manifold,
   --     const b2CircleShape* circleA, const b2Transform& xfA,
   --     const b2CircleShape* circleB, const b2Transform& xfB)
   --  {
   --    manifold->pointCount = 0;
   --
   --    b2Vec2 pA = b2Mul(xfA, circleA->m_p);
   --    b2Vec2 pB = b2Mul(xfB, circleB->m_p);
   --
   --    b2Vec2 d = pB - pA;
   --    float distSqr = b2Dot(d, d);
   --    float rA = circleA->m_radius, rB = circleB->m_radius;
   --    float radius = rA + rB;
   --    if (distSqr > radius * radius)
   --    {
   --       return;
   --    }
   --
   --    manifold->type = b2Manifold::e_circles;
   --    manifold->localPoint = circleA->m_p;
   --    manifold->localNormal.SetZero();
   --    manifold->pointCount = 1;
   --
   --    manifold->points[0].localPoint = circleB->m_p;
   --    manifold->points[0].id.key = 0;
   --  }
   --

   procedure b2CollideCircles (manifold :    out b2Manifold;
                               circleA  : in     b2CircleShape;   xfA : in b2Transform;
                               circleB  : in     b2CircleShape;   xfB : in b2Transform)
   is
      pA : constant b2Vec2 := b2Mul (xfA, circleA.m_p);
      pB : constant b2Vec2 := b2Mul (xfB, circleB.m_p);
      d  : constant b2Vec2 := pB - pA;

      distSqr : constant Real := b2Dot (d, d);
      rA      : constant Real := circleA.m_radius;
      rB      : constant Real := circleB.m_radius;
      radius  : constant Real := rA + rB;

   begin
      manifold.pointCount := 0;

      if distSqr > radius * radius
      then
         return;
      end if;

      manifold.Kind       := e_Circles;
      manifold.localPoint := circleA.m_p;
      manifold.pointCount := 1;
      setZero (manifold.localNormal);

      manifold.points (0).localPoint := circleB.m_p;
      manifold.points (0).id.key     := 0;
   end b2CollideCircles;





   --  void b2CollidePolygonAndCircle
   --    (b2Manifold* manifold,
   --     const b2PolygonShape* polygonA, const b2Transform& xfA,
   --     const b2CircleShape* circleB, const b2Transform& xfB)
   --  {
   --    manifold->pointCount = 0;
   --
   --    // Compute circle position in the frame of the polygon.
   --    b2Vec2 c = b2Mul(xfB, circleB->m_p);
   --    b2Vec2 cLocal = b2MulT(xfA, c);
   --
   --    // Find the min separating edge.
   --    int32 normalIndex = 0;
   --    float separation = -b2_maxFloat;
   --    float radius = polygonA->m_radius + circleB->m_radius;
   --    int32 vertexCount = polygonA->m_count;
   --    const b2Vec2* vertices = polygonA->m_vertices;
   --    const b2Vec2* normals = polygonA->m_normals;
   --
   --    for (int32 i = 0; i < vertexCount; ++i)
   --    {
   --       float s = b2Dot(normals[i], cLocal - vertices[i]);
   --
   --       if (s > radius)
   --       {
   --          // Early out.
   --          return;
   --       }
   --
   --       if (s > separation)
   --       {
   --          separation = s;
   --          normalIndex = i;
   --       }
   --    }
   --
   --    // Vertices that subtend the incident face.
   --    int32 vertIndex1 = normalIndex;
   --    int32 vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
   --    b2Vec2 v1 = vertices[vertIndex1];
   --    b2Vec2 v2 = vertices[vertIndex2];
   --
   --    // If the center is inside the polygon ...
   --    if (separation < b2_epsilon)
   --    {
   --       manifold->pointCount = 1;
   --       manifold->type = b2Manifold::e_faceA;
   --       manifold->localNormal = normals[normalIndex];
   --       manifold->localPoint = 0.5f * (v1 + v2);
   --       manifold->points[0].localPoint = circleB->m_p;
   --       manifold->points[0].id.key = 0;
   --       return;
   --    }
   --
   --    // Compute barycentric coordinates
   --    float u1 = b2Dot(cLocal - v1, v2 - v1);
   --    float u2 = b2Dot(cLocal - v2, v1 - v2);
   --    if (u1 <= 0.0f)
   --    {
   --       if (b2DistanceSquared(cLocal, v1) > radius * radius)
   --       {
   --          return;
   --       }
   --
   --       manifold->pointCount = 1;
   --       manifold->type = b2Manifold::e_faceA;
   --       manifold->localNormal = cLocal - v1;
   --       manifold->localNormal.Normalize();
   --       manifold->localPoint = v1;
   --       manifold->points[0].localPoint = circleB->m_p;
   --       manifold->points[0].id.key = 0;
   --    }
   --    else if (u2 <= 0.0f)
   --    {
   --       if (b2DistanceSquared(cLocal, v2) > radius * radius)
   --       {
   --          return;
   --       }
   --
   --       manifold->pointCount = 1;
   --       manifold->type = b2Manifold::e_faceA;
   --       manifold->localNormal = cLocal - v2;
   --       manifold->localNormal.Normalize();
   --       manifold->localPoint = v2;
   --       manifold->points[0].localPoint = circleB->m_p;
   --       manifold->points[0].id.key = 0;
   --    }
   --    else
   --    {
   --       b2Vec2 faceCenter = 0.5f * (v1 + v2);
   --       float s = b2Dot(cLocal - faceCenter, normals[vertIndex1]);
   --       if (s > radius)
   --       {
   --          return;
   --       }
   --
   --       manifold->pointCount = 1;
   --       manifold->type = b2Manifold::e_faceA;
   --       manifold->localNormal = normals[vertIndex1];
   --       manifold->localPoint = faceCenter;
   --       manifold->points[0].localPoint = circleB->m_p;
   --       manifold->points[0].id.key = 0;
   --    }
   --  }
   --

   procedure b2CollidePolygonAndCircle (manifold :    out b2Manifold;
                                        polygonA : in     b2PolygonShape;   xfA : in b2Transform;
                                        circleB  : in     b2CircleShape;    xfB : in b2Transform)
   is
      -- Compute circle position in the frame of the polygon.
      --
      c      : constant b2Vec2 := b2Mul  (xfB, circleB.m_p);
      cLocal : constant b2Vec2 := b2MulT (xfA, c);

      -- Find the min separating edge.
      --
      normalIndex : Natural := 0;
      separation  : Real    := -b2_maxFloat;
      radius      : constant Real    := polygonA.m_radius + circleB.m_radius;
      vertexCount : constant Natural := polygonA.m_count;

      vertices    : b2Vec2s renames polygonA.m_vertices;
      normals     : b2Vec2s renames polygonA.m_normals;

   begin
      manifold.pointCount := 0;


      for i in 0 .. vertexCount - 1
      loop
         declare
            s : constant Real := b2Dot (normals (i),  cLocal - vertices (i));
         begin
            if s > radius
            then
               return;     -- Early out.
            end if;

            if s > separation
            then
               separation  := s;
               normalIndex := i;
            end if;
         end;
      end loop;


      declare
         -- Vertices that subtend the incident face.
         --
         vertIndex1 : constant Natural := normalIndex;
         vertIndex2 : constant Natural := (if vertIndex1 + 1 < vertexCount then vertIndex1 + 1
                                                                           else 0);
         v1 : constant b2Vec2 := vertices (vertIndex1);
         v2 : constant b2Vec2 := vertices (vertIndex2);
      begin
         -- If the center is inside the polygon ...
         --
         if separation < b2_epsilon
         then
            manifold.pointCount  := 1;
            manifold.Kind        := e_faceA;
            manifold.localNormal := normals (normalIndex);
            manifold.localPoint  := 0.5 * (v1 + v2);

            manifold.points (0).localPoint := circleB.m_p;
            manifold.points (0).id.key     := 0;
            return;
         end if;

         -- Compute barycentric coordinates
         --
         declare
            u1 : constant Real := b2Dot (cLocal - v1,  v2 - v1);
            u2 : constant Real := b2Dot (cLocal - v2,  v1 - v2);
         begin
            if u1 <= 0.0
            then
               if b2DistanceSquared (cLocal, v1)  >  radius * radius
               then
                  return;
               end if;

               manifold.pointCount  := 1;
               manifold.Kind        := e_faceA;
               manifold.localNormal := cLocal - v1;
               manifold.localPoint  := v1;

               manifold.points (0).localPoint := circleB.m_p;
               manifold.points (0).id.key     := 0;

               normalize (manifold.localNormal);

            elsif u2 <= 0.0
            then
               if b2distanceSquared (cLocal, v2)  >  radius * radius
               then
                  return;
               end if;

               manifold.pointCount  := 1;
               manifold.Kind        := e_faceA;
               manifold.localNormal := cLocal - v2;
               manifold.localPoint  := v2;

               manifold.points (0).localPoint := circleB.m_p;
               manifold.points (0).id.key     := 0;

               normalize (manifold.localNormal);

            else
               declare
                  faceCenter : constant b2Vec2 := 0.5 * (v1 + v2);
                  s          : constant Real   := b2Dot (cLocal - faceCenter,
                                                         normals (vertIndex1));
               begin
                  if s > radius
                  then
                     return;
                  end if;

                  manifold.pointCount  := 1;
                  manifold.Kind        := e_faceA;
                  manifold.localNormal := normals (vertIndex1);
                  manifold.localPoint  := faceCenter;

                  manifold.points (0).localPoint := circleB.m_p;
                  manifold.points (0).id.key     := 0;
               end;
            end if;
         end;
      end;
   end b2CollidePolygonAndCircle;


end b2_collide_Circle;
