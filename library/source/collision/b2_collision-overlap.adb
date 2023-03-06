with
     b2_Distance;


package body b2_Collision.overlap
is

--  bool b2TestOverlap( const b2Shape* shapeA, int32 indexA,
--                const b2Shape* shapeB, int32 indexB,
--                const b2Transform& xfA, const b2Transform& xfB)
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

   function b2TestOverlap (shapeA : access b2Shape;       indexA : in Natural;
                           shapeB : access b2Shape;       indexB : in Natural;
                           xfA    : in     b2Transform;   xfB    : in b2Transform) return Boolean
   is
      use b2_Distance;

      input  : b2DistanceInput;
      cache  : b2SimplexCache;
      output : b2DistanceOutput;
   begin
      set (input.proxyA'Access, shapeA.all'Access, indexA);
      set (input.proxyB'Access, shapeB.all'Access, indexB);

      input.transformA := xfA;
      input.transformB := xfB;
      input.useRadii   := True;

      cache.count := 0;

      b2Distance (output, cache, input);

      return output.distance < 10.0 * b2_epsilon;
   end b2TestOverlap;


end b2_Collision.overlap;
