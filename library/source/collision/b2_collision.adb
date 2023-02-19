with
     b2_Distance;


package body b2_Collision
is

   --  void b2WorldManifold::Initialize (const b2Manifold* manifold,
   --                                    const b2Transform& xfA, float radiusA,
   --                                    const b2Transform& xfB, float radiusB)
   --  {
   --    if (manifold->pointCount == 0)
   --    {
   --       return;
   --    }
   --
   --    switch (manifold->type)
   --    {
   --    case b2Manifold::e_circles:
   --       {
   --          normal.Set(1.0f, 0.0f);
   --          b2Vec2 pointA = b2Mul(xfA, manifold->localPoint);
   --          b2Vec2 pointB = b2Mul(xfB, manifold->points[0].localPoint);
   --          if (b2DistanceSquared(pointA, pointB) > b2_epsilon * b2_epsilon)
   --          {
   --             normal = pointB - pointA;
   --             normal.Normalize();
   --          }
   --
   --          b2Vec2 cA = pointA + radiusA * normal;
   --          b2Vec2 cB = pointB - radiusB * normal;
   --          points[0] = 0.5f * (cA + cB);
   --          separations[0] = b2Dot(cB - cA, normal);
   --       }
   --       break;
   --
   --    case b2Manifold::e_faceA:
   --       {
   --          normal = b2Mul(xfA.q, manifold->localNormal);
   --          b2Vec2 planePoint = b2Mul(xfA, manifold->localPoint);
   --
   --          for (int32 i = 0; i < manifold->pointCount; ++i)
   --          {
   --             b2Vec2 clipPoint = b2Mul(xfB, manifold->points[i].localPoint);
   --             b2Vec2 cA = clipPoint + (radiusA - b2Dot(clipPoint - planePoint, normal)) * normal;
   --             b2Vec2 cB = clipPoint - radiusB * normal;
   --             points[i] = 0.5f * (cA + cB);
   --             separations[i] = b2Dot(cB - cA, normal);
   --          }
   --       }
   --       break;
   --
   --    case b2Manifold::e_faceB:
   --       {
   --          normal = b2Mul(xfB.q, manifold->localNormal);
   --          b2Vec2 planePoint = b2Mul(xfB, manifold->localPoint);
   --
   --          for (int32 i = 0; i < manifold->pointCount; ++i)
   --          {
   --             b2Vec2 clipPoint = b2Mul(xfA, manifold->points[i].localPoint);
   --             b2Vec2 cB = clipPoint + (radiusB - b2Dot(clipPoint - planePoint, normal)) * normal;
   --             b2Vec2 cA = clipPoint - radiusA * normal;
   --             points[i] = 0.5f * (cA + cB);
   --             separations[i] = b2Dot(cA - cB, normal);
   --          }
   --
   --          // Ensure normal points from A to B.
   --          normal = -normal;
   --       }
   --       break;
   --    }
   --  }
   --

   procedure initialize (Self : out b2WorldManifold;   Manifold : in b2Manifold;
                                                       xfA      : in b2Transform;   radiusA : in Real;
                                                       xfB      : in b2Transform;   radiusB : in Real)
   is
   begin
     if manifold.pointCount = 0
     then
        return;
     end if;

     case manifold.Kind
     is
     when e_circles =>
         declare
            pointA : constant b2Vec2 := b2Mul (xfA, manifold.localPoint);
            pointB : constant b2Vec2 := b2Mul (xfB, manifold.points (0).localPoint);
         begin
            set (Self.normal, 1.0, 0.0);

            if b2distanceSquared (pointA, pointB) > b2_epsilon * b2_epsilon
            then
               Self.normal := pointB - pointA;
               normalize (Self.normal);
            end if;

            declare
               cA : constant b2Vec2 := pointA + radiusA * Self.normal;
               cB : constant b2Vec2 := pointB - radiusB * Self.normal;
            begin
               Self.points (0) := 0.5 * (cA + cB);
               Self.separations (0) := b2Dot (cB - cA,  Self.normal);
            end;
         end;

     when e_faceA =>
         declare
            planePoint : constant b2Vec2 := b2Mul (xfA, manifold.localPoint);
         begin
            Self.normal := b2Mul (xfA.q, manifold.localNormal);

            for i in 0 .. manifold.pointCount - 1
            loop
               declare
                  clipPoint : constant b2Vec2 := b2Mul (xfB,  manifold.points (i).localPoint);
                  cA        : constant b2Vec2 := clipPoint + (radiusA - b2Dot (clipPoint - planePoint, Self.normal)) * Self.normal;
                  cB        : constant b2Vec2 := clipPoint -  radiusB * Self.normal;
               begin
                  Self.points (i)      := 0.5 * (cA + cB);
                  Self.separations (i) := b2Dot (cB - cA,  Self.normal);
               end;
           end loop;
         end;

     when e_faceB =>
        declare
            planePoint : constant b2Vec2  := b2Mul (xfB, manifold.localPoint);
         begin
            Self.normal := b2Mul (xfB.q, manifold.localNormal);

            for i in 0 .. manifold.pointCount - 1
            loop
               declare
                  clipPoint : constant b2Vec2 := b2Mul(xfA,  manifold.points (i).localPoint);
                  cB        : constant b2Vec2 := clipPoint + (radiusB - b2Dot(clipPoint - planePoint, Self.normal)) * Self.normal;
                  cA        : constant b2Vec2 := clipPoint -  radiusA * Self.normal;
               begin
                  Self.points (i)      := 0.5 * (cA + cB);
                  Self.separations (i) := b2Dot(cA - cB, Self.normal);
               end;
            end loop;

            -- Ensure normal points from A to B.
            --
            Self.normal := -Self.normal;
         end;
      end case;
   end initialize;




   ---------------
   -- b2PointState
   --

   --  Compute the point states given two manifolds. The states pertain to the transition from manifold1
   --  to manifold2. So state1 is either persist or remove while state2 is either add or persist.
   --
   --  void b2GetPointStates (b2PointState state1[b2_maxManifoldPoints], b2PointState state2[b2_maxManifoldPoints],
   --                         const b2Manifold* manifold1, const b2Manifold* manifold2)
   --  {
   --    for (int32 i = 0; i < b2_maxManifoldPoints; ++i)
   --    {
   --       state1[i] = b2_nullState;
   --       state2[i] = b2_nullState;
   --    }
   --
   --    // Detect persists and removes.
   --    for (int32 i = 0; i < manifold1->pointCount; ++i)
   --    {
   --       b2ContactID id = manifold1->points[i].id;
   --
   --       state1[i] = b2_removeState;
   --
   --       for (int32 j = 0; j < manifold2->pointCount; ++j)
   --       {
   --          if (manifold2->points[j].id.key == id.key)
   --          {
   --             state1[i] = b2_persistState;
   --             break;
   --          }
   --       }
   --    }
   --
   --    // Detect persists and adds.
   --    for (int32 i = 0; i < manifold2->pointCount; ++i)
   --    {
   --       b2ContactID id = manifold2->points[i].id;
   --
   --       state2[i] = b2_addState;
   --
   --       for (int32 j = 0; j < manifold1->pointCount; ++j)
   --       {
   --          if (manifold1->points[j].id.key == id.key)
   --          {
   --             state2[i] = b2_persistState;
   --             break;
   --          }
   --       }
   --    }
   --  }
   --

   procedure b2getPointStates (State1    :    out b2PointStates;
                               State2    :    out b2PointStates;
                               Manifold1 : in     b2Manifold;
                               Manifold2 : in     b2Manifold)
   is
   begin
      for i in 0 .. b2_maxManifoldPoints - 1
      loop
         state1 (i) := b2_nullState;
         state2 (i) := b2_nullState;
      end loop;

      -- Detect persists and removes.
      --
      for i in 0 .. manifold1.pointCount - 1
      loop
         declare
            id : constant b2ContactID := manifold1.points (i).id;
         begin
            state1 (i) := b2_removeState;

            for j in 0 .. manifold2.pointCount - 1
            loop
               if manifold2.points (j).id.key = id.key
               then
                  state1 (i) := b2_persistState;
                  exit;
               end if;
            end loop;
         end;
      end loop;

      -- Detect persists and adds.
      --
      for i in 0 .. manifold2.pointCount - 1
      loop
         declare
            id : constant b2ContactID := manifold2.points (i).id;
         begin
            state2 (i) := b2_addState;

            for j in 0 .. manifold1.pointCount - 1
            loop
               if manifold1.points (j).id.key = id.key
               then
                  state2 (i) := b2_persistState;
                  exit;
               end if;
            end loop;
         end;
      end loop;
   end b2getPointStates;




   ---------
   -- b2AABB
   --

   --    Verify that the bounds are sorted.
   --
   --    bool IsValid() const;
   --
   --  inline bool b2AABB::IsValid() const
   --  {
   --    b2Vec2 d = upperBound - lowerBound;
   --    bool valid = d.x >= 0.0f && d.y >= 0.0f;
   --    valid = valid && lowerBound.IsValid() && upperBound.IsValid();
   --    return valid;
   --  }

   function isValid (Self : in b2AABB) return Boolean
   is
      d     : constant b2Vec2  := Self.upperBound - Self.lowerBound;
      valid :          Boolean :=     d.x >= 0.0
                                  and d.y >= 0.0;
   begin
      valid := Valid and IsValid (Self.lowerBound)
                     and IsValid (Self.upperBound);
      return valid;
   end isValid;




   --    Get the center of the AABB.
   --
   --    b2Vec2 GetCenter() const
   --    {
   --       return 0.5f * (lowerBound + upperBound);
   --    }

   function getCenter (Self : in b2AABB) return b2Vec2
   is
   begin
      return 0.5 * (Self.lowerBound + Self.upperBound);
   end getCenter;



   --    Get the extents of the AABB (half-widths).
   --
   --    b2Vec2 GetExtents() const
   --    {
   --       return 0.5f * (upperBound - lowerBound);
   --    }

   function getExtents (Self : in b2AABB) return b2Vec2
   is
   begin
      return 0.5 * (Self.upperBound - Self.lowerBound);
   end getExtents;



   --    Get the perimeter length.
   --
   --    float GetPerimeter() const
   --    {
   --       float wx = upperBound.x - lowerBound.x;
   --       float wy = upperBound.y - lowerBound.y;
   --       return 2.0f * (wx + wy);
   --    }

   function getPerimeter (Self : in b2AABB) return Real
   is
      wx : constant Real := Self.upperBound.x - Self.lowerBound.x;
      wy : constant Real := Self.upperBound.y - Self.lowerBound.y;
   begin
      return 2.0 * (wx + wy);
   end getPerimeter;



   --    Combine an AABB into this one.
   --
   --    void Combine(const b2AABB& aabb)
   --    {
   --       lowerBound = b2Min(lowerBound, aabb.lowerBound);
   --       upperBound = b2Max(upperBound, aabb.upperBound);
   --    }

   procedure combine (Self : in out b2AABB;   aabb : in b2AABB)
   is
   begin
      Self.lowerBound := b2Min (Self.lowerBound, aabb.lowerBound);
      Self.upperBound := b2Max (Self.upperBound, aabb.upperBound);
   end combine;



   --    Combine two AABBs into this one.
   --
   --    void Combine(const b2AABB& aabb1, const b2AABB& aabb2)
   --    {
   --       lowerBound = b2Min(aabb1.lowerBound, aabb2.lowerBound);
   --       upperBound = b2Max(aabb1.upperBound, aabb2.upperBound);
   --    }

   procedure combine (Self : in out b2AABB;   aabb1 : in b2AABB;
                                              aabb2 : in b2AABB)
   is
   begin
      Self.lowerBound := b2Min (aabb1.lowerBound, aabb2.lowerBound);
      Self.upperBound := b2Max (aabb1.upperBound, aabb2.upperBound);
   end combine;




   --    Does this aabb contain the provided AABB.
   --
   --    bool Contains(const b2AABB& aabb) const
   --    {
   --       bool result = true;
   --       result = result && lowerBound.x <= aabb.lowerBound.x;
   --       result = result && lowerBound.y <= aabb.lowerBound.y;
   --       result = result && aabb.upperBound.x <= upperBound.x;
   --       result = result && aabb.upperBound.y <= upperBound.y;
   --       return result;
   --    }

   function contains (Self : in b2AABB;   aabb : in b2AABB) return Boolean
   is
      result : Boolean := Self.lowerBound.x <= aabb.lowerBound.x;
   begin
      result := result and Self.lowerBound.y <= aabb.lowerBound.y;

      result := result and aabb.upperBound.x <= Self.upperBound.x;
      result := result and aabb.upperBound.y <= Self.upperBound.y;

      return result;
   end contains;





   --  // From Real-time Collision Detection, p179.
   --
   --  bool b2AABB::RayCast(b2RayCastOutput* output, const b2RayCastInput& input) const
   --  {
   --    float tmin = -b2_maxFloat;
   --    float tmax = b2_maxFloat;
   --
   --    b2Vec2 p = input.p1;
   --    b2Vec2 d = input.p2 - input.p1;
   --    b2Vec2 absD = b2Abs(d);
   --
   --    b2Vec2 normal;
   --
   --    for (int32 i = 0; i < 2; ++i)
   --    {
   --       if (absD(i) < b2_epsilon)
   --       {
   --          // Parallel.
   --          if (p(i) < lowerBound(i) || upperBound(i) < p(i))
   --          {
   --             return false;
   --          }
   --       }
   --       else
   --       {
   --          float inv_d = 1.0f / d(i);
   --          float t1 = (lowerBound(i) - p(i)) * inv_d;
   --          float t2 = (upperBound(i) - p(i)) * inv_d;
   --
   --          // Sign of the normal vector.
   --          float s = -1.0f;
   --
   --          if (t1 > t2)
   --          {
   --             b2Swap(t1, t2);
   --             s = 1.0f;
   --          }
   --
   --          // Push the min up
   --          if (t1 > tmin)
   --          {
   --             normal.SetZero();
   --             normal(i) = s;
   --             tmin = t1;
   --          }
   --
   --          // Pull the max down
   --          tmax = b2Min(tmax, t2);
   --
   --          if (tmin > tmax)
   --          {
   --             return false;
   --          }
   --       }
   --    }
   --
   --    // Does the ray start inside the box?
   --    // Does the ray intersect beyond the max fraction?
   --    if (tmin < 0.0f || input.maxFraction < tmin)
   --    {
   --       return false;
   --    }
   --
   --    // Intersection.
   --    output->fraction = tmin;
   --    output->normal = normal;
   --    return true;
   --  }
   --

   function raycast (Self : in b2AABB;   output : out b2RayCastOutput;
                                         input  : in  b2RayCastInput) return Boolean
   is
     tmin : Real   := -b2_maxFloat;
     tmax : Real   :=  b2_maxFloat;

      p    : constant b2Vec2 := input.p1;
      d    : constant b2Vec2 := input.p2 - input.p1;
      absD : constant b2Vec2 := b2Abs (d);

     normal : aliased b2Vec2;

   begin
      for i in 0 .. 1
      loop
         if Element (absD, i)  <  b2_epsilon
         then
            -- Parallel.
            --
            if   Element (p, i)               < Element (Self.lowerBound, i)
              or Element (Self.upperBound, i) < Element (p, i)
            then
               return False;
            end if;

         else
            declare
               inv_d : constant Real := 1.0 / Element (d, i);
               t1    :          Real := (Element (Self.lowerBound, i) - Element (p, i))  *  inv_d;
               t2    :          Real := (Element (Self.upperBound, i) - Element (p, i))  *  inv_d;

               -- Sign of the normal vector.
               --
               s : Real := -1.0;
            begin
               if t1 > t2
               then
                  b2Swap (t1, t2);
                  s := 1.0;
               end if;

               -- Push the min up.
               --
               if t1 > tmin
               then
                  setZero (normal);
                  Element (normal'Access, i).all := s;
                  tmin := t1;
               end if;

               -- Pull the max down.
               --
               tmax := Real'min (tmax, t2);

               if tmin > tmax
               then
                  return false;
               end if;
            end;
         end if;
      end loop;

      -- Does the ray start inside the box?
      -- Does the ray intersect beyond the max fraction?
      --
      if   tmin              < 0.0
        or input.maxFraction < tmin
      then
         return False;
      end if;

      -- Intersection.
      --
      output.fraction := tmin;
      output.normal   := normal;

      return True;
   end raycast;




   --  inline bool b2TestOverlap (const b2AABB& a, const b2AABB& b)
   --  {
   --    b2Vec2 d1, d2;
   --    d1 = b.lowerBound - a.upperBound;
   --    d2 = a.lowerBound - b.upperBound;
   --
   --    if (d1.x > 0.0f || d1.y > 0.0f)
   --       return false;
   --
   --    if (d2.x > 0.0f || d2.y > 0.0f)
   --       return false;
   --
   --    return true;
   --  }

   function b2testOverlap (a, b : in b2AABB) return Boolean
   is
      d1, d2 : b2Vec2;
   begin
      d1 := b.lowerBound - a.upperBound;
      d2 := a.lowerBound - b.upperBound;

      if   d1.x > 0.0
        or d1.y > 0.0
      then
         return False;
      end if;

      if   d2.x > 0.0
        or d2.y > 0.0
      then
         return False;
      end if;

      return True;
   end b2testOverlap;





   --  Clipping for contact manifolds.
   --
   --  // Sutherland-Hodgman clipping.
   --
   --  int32 b2ClipSegmentToLine (b2ClipVertex vOut[2], const b2ClipVertex vIn[2],
   --                             const b2Vec2& normal, float offset, int32 vertexIndexA)
   --  {
   --    // Start with no output points
   --    int32 count = 0;
   --
   --    // Calculate the distance of end points to the line
   --    float distance0 = b2Dot(normal, vIn[0].v) - offset;
   --    float distance1 = b2Dot(normal, vIn[1].v) - offset;
   --
   --    // If the points are behind the plane
   --    if (distance0 <= 0.0f) vOut[count++] = vIn[0];
   --    if (distance1 <= 0.0f) vOut[count++] = vIn[1];
   --
   --    // If the points are on different sides of the plane
   --    if (distance0 * distance1 < 0.0f)
   --    {
   --       // Find intersection point of edge and plane
   --       float interp = distance0 / (distance0 - distance1);
   --       vOut[count].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
   --
   --       // VertexA is hitting edgeB.
   --       vOut[count].id.cf.indexA = static_cast<uint8>(vertexIndexA);
   --       vOut[count].id.cf.indexB = vIn[0].id.cf.indexB;
   --       vOut[count].id.cf.typeA = b2ContactFeature::e_vertex;
   --       vOut[count].id.cf.typeB = b2ContactFeature::e_face;
   --       ++count;
   --
   --       b2Assert(count == 2);
   --    }
   --
   --    return count;
   --  }
   --

   function b2ClipSegmentToLine (vOut         :    out b2ClipVertex_Pair;
                                 vIn          : in     b2ClipVertex_Pair;
                                 Normal       : in     b2Vec2;
                                 Offset       : in     Real;
                                 vertexIndexA : in     Natural) return Natural
   is
      --  Start with no output points.
      --
      count : Natural := 0;

      -- Calculate the distance of end points to the line.
      --
      distance0 : constant Real := b2Dot (normal,  vIn (0).v) - offset;
      distance1 : constant Real := b2Dot (normal,  vIn (1).v) - offset;

      interp : Real;

   begin
      -- If the points are behind the plane.
      --
      if distance0 <= 0.0 then   vOut (count) := vIn (0);   Count := Count + 1;   end if;
      if distance1 <= 0.0 then   vOut (count) := vIn (1);   Count := Count + 1;   end if;

      -- If the points are on different sides of the plane.
      --
      if distance0 * distance1 < 0.0
      then
         -- Find intersection point of edge and plane.
         --
         interp         := distance0  /  (distance0 - distance1);
         vOut (count).v := vIn (0).v  +  interp * (  vIn (1).v
                                                   - vIn (0).v);

         -- VertexA is hitting edgeB.
         --
         vOut (count).id.cf.indexA := vertexIndexA;
         vOut (count).id.cf.indexB := vIn (0).id.cf.indexB;

         vOut (count).id.cf.typeA  := e_vertex;
         vOut (count).id.cf.typeB  := e_face;

         count := count + 1;

         pragma assert (count = 2);
      end if;

      return count;
   end b2ClipSegmentToLine;




   --  bool b2TestOverlap (const b2Shape* shapeA, int32 indexA,
   --                      const b2Shape* shapeB, int32 indexB,
   --                      const b2Transform& xfA, const b2Transform& xfB)
   --  {
   --    b2DistanceInput input;
   --    input.proxyA.Set(shapeA, indexA);
   --    input.proxyB.Set(shapeB, indexB);
   --    input.transformA = xfA;
   --    input.transformB = xfB;
   --    input.useRadii = true;
   --
   --    b2SimplexCache cache;
   --    cache.count = 0;
   --
   --    b2DistanceOutput output;
   --
   --    b2Distance(&output, &cache, &input);
   --
   --    return output.distance < 10.0f * b2_epsilon;
   --  }
   --

   -- Moved to 'b2_collision.overlap' spec.


end b2_Collision;
