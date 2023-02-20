with
     b2_Collision,
     b2_circle_Shape,
     b2_polygon_Shape,
     b2_Timer,
     b2_Common,

     ada.Text_IO,
     Interfaces;


package body b2_Time_of_impact
is

   --  float b2_toiTime, b2_toiMaxTime;
   --  int32 b2_toiCalls, b2_toiIters, b2_toiMaxIters;
   --  int32 b2_toiRootIters, b2_toiMaxRootIters;
   --

   b2_toiTime,
   b2_toiMaxTime : Real;

   b2_toiCalls,
   b2_toiIters,
   b2_toiMaxIters : Natural;

   b2_toiRootIters,
   b2_toiMaxRootIters : Natural;



   --    enum Type
   --    {
   --       e_points,
   --       e_faceA,
   --       e_faceB
   --    };
   --

   type Kind is (e_points,
                 e_faceA,
                 e_faceB);



   --  struct b2SeparationFunction
   --  {
   --    -- TODO_ERIN might not need to return the separation
   --
   --    const b2DistanceProxy* m_proxyA;
   --    const b2DistanceProxy* m_proxyB;
   --    b2Sweep m_sweepA, m_sweepB;
   --    Type m_type;
   --    b2Vec2 m_localPoint;
   --    b2Vec2 m_axis;
   --  };

   type b2SeparationFunction is tagged
      record
         m_proxyA,
         m_proxyB  : access constant b2DistanceProxy;

         m_sweepA,
         m_sweepB : b2Sweep;

         m_type       : Kind;
         m_localPoint : b2Vec2;
         m_axis       : b2Vec2;
      end record;



   --    float b2SeparationFunction::Initialize (const b2SimplexCache* cache,
   --                                            const b2DistanceProxy* proxyA, const b2Sweep& sweepA,
   --                                            const b2DistanceProxy* proxyB, const b2Sweep& sweepB,
   --                                            float t1)
   --    {
   --       m_proxyA = proxyA;
   --       m_proxyB = proxyB;
   --       int32 count = cache->count;
   --       b2Assert(0 < count && count < 3);
   --
   --       m_sweepA = sweepA;
   --       m_sweepB = sweepB;
   --
   --       b2Transform xfA, xfB;
   --       m_sweepA.GetTransform(&xfA, t1);
   --       m_sweepB.GetTransform(&xfB, t1);
   --
   --       if (count == 1)
   --       {
   --          m_type = e_points;
   --          b2Vec2 localPointA = m_proxyA->GetVertex(cache->indexA[0]);
   --          b2Vec2 localPointB = m_proxyB->GetVertex(cache->indexB[0]);
   --          b2Vec2 pointA = b2Mul(xfA, localPointA);
   --          b2Vec2 pointB = b2Mul(xfB, localPointB);
   --          m_axis = pointB - pointA;
   --          float s = m_axis.Normalize();
   --          return s;
   --       }
   --       else if (cache->indexA[0] == cache->indexA[1])
   --       {
   --          // Two points on B and one on A.
   --          m_type = e_faceB;
   --          b2Vec2 localPointB1 = proxyB->GetVertex(cache->indexB[0]);
   --          b2Vec2 localPointB2 = proxyB->GetVertex(cache->indexB[1]);
   --
   --          m_axis = b2Cross(localPointB2 - localPointB1, 1.0f);
   --          m_axis.Normalize();
   --          b2Vec2 normal = b2Mul(xfB.q, m_axis);
   --
   --          m_localPoint = 0.5f * (localPointB1 + localPointB2);
   --          b2Vec2 pointB = b2Mul(xfB, m_localPoint);
   --
   --          b2Vec2 localPointA = proxyA->GetVertex(cache->indexA[0]);
   --          b2Vec2 pointA = b2Mul(xfA, localPointA);
   --
   --          float s = b2Dot(pointA - pointB, normal);
   --          if (s < 0.0f)
   --          {
   --             m_axis = -m_axis;
   --             s = -s;
   --          }
   --          return s;
   --       }
   --       else
   --       {
   --          // Two points on A and one or two points on B.
   --          m_type = e_faceA;
   --          b2Vec2 localPointA1 = m_proxyA->GetVertex(cache->indexA[0]);
   --          b2Vec2 localPointA2 = m_proxyA->GetVertex(cache->indexA[1]);
   --
   --          m_axis = b2Cross(localPointA2 - localPointA1, 1.0f);
   --          m_axis.Normalize();
   --          b2Vec2 normal = b2Mul(xfA.q, m_axis);
   --
   --          m_localPoint = 0.5f * (localPointA1 + localPointA2);
   --          b2Vec2 pointA = b2Mul(xfA, m_localPoint);
   --
   --          b2Vec2 localPointB = m_proxyB->GetVertex(cache->indexB[0]);
   --          b2Vec2 pointB = b2Mul(xfB, localPointB);
   --
   --          float s = b2Dot(pointB - pointA, normal);
   --          if (s < 0.0f)
   --          {
   --             m_axis = -m_axis;
   --             s = -s;
   --          }
   --          return s;
   --       }
   --    }
   --

   function Initialize (Self : out b2SeparationFunction;   cache  : access constant b2SimplexCache;
                                                           proxyA : access constant b2DistanceProxy;   sweepA : in b2Sweep;
                                                           proxyB : access constant b2DistanceProxy;   sweepB : in b2Sweep;
                                                           t1     : in              Real) return Real
   is
      count    : Natural;
      xfA, xfB : b2Transform;

   begin
      Self.m_proxyA := proxyA;
      Self.m_proxyB := proxyB;
      count         := cache.count;
      pragma assert (0 < count and count < 3);

      Self.m_sweepA := sweepA;
      Self.m_sweepB := sweepB;

      Self.m_sweepA.getTransform (xfA, t1);
      Self.m_sweepB.getTransform (xfB, t1);

      if count = 1
      then
         declare
            localPointA,
            localPointB : b2Vec2;

            pointA,
            pointB : b2Vec2;

            s : Real;
         begin
            Self.m_type  := e_points;
            localPointA  := Self.m_proxyA.getVertex (cache.indexA (0));
            localPointB  := Self.m_proxyB.getVertex (cache.indexB (0));
            pointA       := b2Mul (xfA, localPointA);
            pointB       := b2Mul (xfB, localPointB);
            Self.m_axis  := pointB - pointA;
            s            := normalize (Self.m_axis);
            return s;
         end;

      elsif cache.indexA (0) = cache.indexA (1)
      then
         declare
            localPointB1,
            localPointB2 : b2Vec2;

            normal : b2Vec2;
            pointB : b2Vec2;

            localPointA : b2Vec2;
            pointA      : b2Vec2;

            s : Real;
         begin
            -- Two points on B and one on A.
            --
            Self.m_type  := e_faceB;
            localPointB1 := proxyB.getVertex (cache.indexB (0));
            localPointB2 := proxyB.getVertex (cache.indexB (1));

            Self.m_axis   := b2Cross (localPointB2 - localPointB1,  1.0);
            normalize (Self.m_axis);
            normal        := b2Mul (xfB.q, Self.m_axis);

            Self.m_localPoint := 0.5 * (localPointB1 + localPointB2);
            pointB            := b2Mul (xfB, Self.m_localPoint);

            localPointA := proxyA.getVertex (cache.indexA (0));
            pointA      := b2Mul (xfA, localPointA);

            s := b2Dot (pointA - pointB,  normal);

            if s < 0.0
            then
               Self.m_axis := -Self.m_axis;
               s := -s;
            end if;

            return s;
         end;

      else
         declare
            localPointA1,
            localPointA2 : b2Vec2;

            normal : b2Vec2;
            pointA : b2Vec2;

            localPointB : b2Vec2;
            pointB      : b2Vec2;

            s : Real;
         begin
            -- Two points on A and one or two points on B.
            --
            Self.m_type  := e_faceA;
            localPointA1 := Self.m_proxyA.getVertex (cache.indexA (0));
            localPointA2 := Self.m_proxyA.getVertex (cache.indexA (1));

            Self.m_axis  := b2Cross (localPointA2 - localPointA1,  1.0);
            normalize (Self.m_axis);

            normal := b2Mul (xfA.q, Self.m_axis);

            Self.m_localPoint := 0.5 * (localPointA1 + localPointA2);
            pointA            := b2Mul (xfA, Self.m_localPoint);

            localPointB := Self.m_proxyB.getVertex (cache.indexB (0));
            pointB      := b2Mul (xfB, localPointB);

            s := b2Dot (pointB - pointA,  normal);

            if s < 0.0
            then
               Self.m_axis := -Self.m_axis;
               s           := -s;
            end if;

            return s;
         end;
      end if;
   end Initialize;





   --    float b2SeparationFunction::FindMinSeparation(int32* indexA, int32* indexB, float t) const
   --    {
   --       b2Transform xfA, xfB;
   --       m_sweepA.GetTransform(&xfA, t);
   --       m_sweepB.GetTransform(&xfB, t);
   --
   --       switch (m_type)
   --       {
   --       case e_points:
   --          {
   --             b2Vec2 axisA = b2MulT(xfA.q,  m_axis);
   --             b2Vec2 axisB = b2MulT(xfB.q, -m_axis);
   --
   --             *indexA = m_proxyA->GetSupport(axisA);
   --             *indexB = m_proxyB->GetSupport(axisB);
   --
   --             b2Vec2 localPointA = m_proxyA->GetVertex(*indexA);
   --             b2Vec2 localPointB = m_proxyB->GetVertex(*indexB);
   --
   --             b2Vec2 pointA = b2Mul(xfA, localPointA);
   --             b2Vec2 pointB = b2Mul(xfB, localPointB);
   --
   --             float separation = b2Dot(pointB - pointA, m_axis);
   --             return separation;
   --          }
   --
   --       case e_faceA:
   --          {
   --             b2Vec2 normal = b2Mul(xfA.q, m_axis);
   --             b2Vec2 pointA = b2Mul(xfA, m_localPoint);
   --
   --             b2Vec2 axisB = b2MulT(xfB.q, -normal);
   --
   --             *indexA = -1;
   --             *indexB = m_proxyB->GetSupport(axisB);
   --
   --             b2Vec2 localPointB = m_proxyB->GetVertex(*indexB);
   --             b2Vec2 pointB = b2Mul(xfB, localPointB);
   --
   --             float separation = b2Dot(pointB - pointA, normal);
   --             return separation;
   --          }
   --
   --       case e_faceB:
   --          {
   --             b2Vec2 normal = b2Mul(xfB.q, m_axis);
   --             b2Vec2 pointB = b2Mul(xfB, m_localPoint);
   --
   --             b2Vec2 axisA = b2MulT(xfA.q, -normal);
   --
   --             *indexB = -1;
   --             *indexA = m_proxyA->GetSupport(axisA);
   --
   --             b2Vec2 localPointA = m_proxyA->GetVertex(*indexA);
   --             b2Vec2 pointA = b2Mul(xfA, localPointA);
   --
   --             float separation = b2Dot(pointA - pointB, normal);
   --             return separation;
   --          }
   --
   --       default:
   --          b2Assert(false);
   --          *indexA = -1;
   --          *indexB = -1;
   --          return 0.0f;
   --       }
   --    }
   --
   --

   function findMinSeparation (Self : in b2SeparationFunction;   indexA,
                                                                 indexB :    out Integer;
                                                                 t      : in     Real) return Real
   is
      xfA, xfB : b2Transform;
   begin
      Self.m_sweepA.getTransform (xfA, t);
      Self.m_sweepB.getTransform (xfB, t);

      case Self.m_type
      is
         when e_points =>
            declare
               axisA : constant b2Vec2 := b2MulT (xfA.q,  Self.m_axis);
               axisB : constant b2Vec2 := b2MulT (xfB.q, -Self.m_axis);

               localPointA,
               localPointB : b2Vec2;

               pointA,
               pointB : b2Vec2;

               separation : Real;
            begin
               indexA := Self.m_proxyA.getSupport (axisA);
               indexB := Self.m_proxyB.getSupport (axisB);

               localPointA := Self.m_proxyA.getVertex (indexA);
               localPointB := Self.m_proxyB.getVertex (indexB);

               pointA := b2Mul (xfA, localPointA);
               pointB := b2Mul (xfB, localPointB);

               separation := b2Dot (pointB - pointA,  Self.m_axis);
               return separation;
            end;

         when e_faceA =>
            declare
               normal : constant b2Vec2 := b2Mul  (xfA.q, Self.m_axis);
               pointA : constant b2Vec2 := b2Mul  (xfA,   Self.m_localPoint);
               axisB  : constant b2Vec2 := b2MulT (xfB.q, -normal);

               localPointB : b2Vec2;
               pointB      : b2Vec2;

               separation  : Real;
            begin
               indexA := -1;
               indexB := Self.m_proxyB.getSupport (axisB);

               localPointB := Self.m_proxyB.getVertex (indexB);
               pointB      := b2Mul (xfB, localPointB);

               separation  := b2Dot (pointB - pointA,  normal);
               return separation;
            end;

         when e_faceB =>
            declare
               normal : constant b2Vec2 := b2Mul (xfB.q, Self.m_axis);
               pointB : constant b2Vec2 := b2Mul (xfB, Self.m_localPoint);
               axisA  : constant b2Vec2 := b2MulT (xfA.q, -normal);

               localPointA : b2Vec2;
               pointA      : b2Vec2;
               separation  : Real;
            begin
               indexB := -1;
               indexA := Self.m_proxyA.getSupport (axisA);

               localPointA := Self.m_proxyA.getVertex (indexA);
               pointA      := b2Mul (xfA, localPointA);

               separation  := b2Dot (pointA - pointB,  normal);
               return separation;
            end;

         --  when others =>
         --     pragma assert (False);
         --     indexA := -1;
         --     indexB := -1;
         --
         --     return 0.0;
      end case;
   end findMinSeparation;





   --    float b2SeparationFunction::Evaluate(int32 indexA, int32 indexB, float t) const
   --    {
   --       b2Transform xfA, xfB;
   --       m_sweepA.GetTransform(&xfA, t);
   --       m_sweepB.GetTransform(&xfB, t);
   --
   --       switch (m_type)
   --       {
   --       case e_points:
   --          {
   --             b2Vec2 localPointA = m_proxyA->GetVertex(indexA);
   --             b2Vec2 localPointB = m_proxyB->GetVertex(indexB);
   --
   --             b2Vec2 pointA = b2Mul(xfA, localPointA);
   --             b2Vec2 pointB = b2Mul(xfB, localPointB);
   --             float separation = b2Dot(pointB - pointA, m_axis);
   --
   --             return separation;
   --          }
   --
   --       case e_faceA:
   --          {
   --             b2Vec2 normal = b2Mul(xfA.q, m_axis);
   --             b2Vec2 pointA = b2Mul(xfA, m_localPoint);
   --
   --             b2Vec2 localPointB = m_proxyB->GetVertex(indexB);
   --             b2Vec2 pointB = b2Mul(xfB, localPointB);
   --
   --             float separation = b2Dot(pointB - pointA, normal);
   --             return separation;
   --          }
   --
   --       case e_faceB:
   --          {
   --             b2Vec2 normal = b2Mul(xfB.q, m_axis);
   --             b2Vec2 pointB = b2Mul(xfB, m_localPoint);
   --
   --             b2Vec2 localPointA = m_proxyA->GetVertex(indexA);
   --             b2Vec2 pointA = b2Mul(xfA, localPointA);
   --
   --             float separation = b2Dot(pointA - pointB, normal);
   --             return separation;
   --          }
   --
   --       default:
   --          b2Assert(false);
   --          return 0.0f;
   --       }
   --    }
   --

   function Evaluate(Self : in b2SeparationFunction;   indexA,
                                                       indexB : in Natural;
                                                       t      : in Real) return Real
   is
      xfA, xfB : b2Transform;
   begin
      Self.m_sweepA.getTransform (xfA, t);
      Self.m_sweepB.getTransform (xfB, t);

      case Self.m_type
      is
         when e_points =>
            declare
               localPointA : constant b2Vec2 := Self.m_proxyA.getVertex (indexA);
               localPointB : constant b2Vec2 := Self.m_proxyB.getVertex (indexB);

               pointA      : constant b2Vec2 := b2Mul (xfA, localPointA);
               pointB      : constant b2Vec2 := b2Mul (xfB, localPointB);

               separation  : constant Real   := b2Dot (pointB - pointA,  Self.m_axis);
            begin
               return separation;
            end;

         when e_faceA =>
            declare
               normal      : constant b2Vec2 := b2Mul(xfA.q, Self.m_axis);
               pointA      : constant b2Vec2 := b2Mul(xfA,   Self.m_localPoint);

               localPointB : constant b2Vec2 := Self.m_proxyB.getVertex (indexB);
               pointB      : constant b2Vec2 := b2Mul (xfB, localPointB);

               separation  : constant Real   := b2Dot (pointB - pointA,  normal);
            begin
               return separation;
            end;

         when e_faceB =>
            declare
               normal      : constant b2Vec2 := b2Mul (xfB.q, Self.m_axis);
               pointB      : constant b2Vec2 := b2Mul (xfB,   Self.m_localPoint);

               localPointA : constant b2Vec2 := Self.m_proxyA.getVertex (indexA);
               pointA      : constant b2Vec2 := b2Mul (xfA, localPointA);

               separation  : constant Real   := b2Dot (pointA - pointB,  normal);
            begin
               return separation;
            end;

         --  when others =>
         --     pragma assert (False);
         --     return 0.0;
      end case;

   end Evaluate;






   --  CCD via the local separating axis method. This seeks progression
   --  by computing the largest time at which separation is maintained.
   --
   --  void b2TimeOfImpact (b2TOIOutput* output, const b2TOIInput* input)
   --  {
   --    b2Timer timer;
   --
   --    ++b2_toiCalls;
   --
   --    output->state = b2TOIOutput::e_unknown;
   --    output->t = input->tMax;
   --
   --    const b2DistanceProxy* proxyA = &input->proxyA;
   --    const b2DistanceProxy* proxyB = &input->proxyB;
   --
   --    b2Sweep sweepA = input->sweepA;
   --    b2Sweep sweepB = input->sweepB;
   --
   --    // Large rotations can make the root finder fail, so we normalize the
   --    // sweep angles.
   --    sweepA.Normalize();
   --    sweepB.Normalize();
   --
   --    float tMax = input->tMax;
   --
   --    float totalRadius = proxyA->m_radius + proxyB->m_radius;
   --    float target = b2Max(b2_linearSlop, totalRadius - 3.0f * b2_linearSlop);
   --    float tolerance = 0.25f * b2_linearSlop;
   --    b2Assert(target > tolerance);
   --
   --    float t1 = 0.0f;
   --    const int32 k_maxIterations = 20;   // TODO_ERIN b2Settings
   --    int32 iter = 0;
   --
   --    // Prepare input for distance query.
   --    b2SimplexCache cache;
   --    cache.count = 0;
   --    b2DistanceInput distanceInput;
   --    distanceInput.proxyA = input->proxyA;
   --    distanceInput.proxyB = input->proxyB;
   --    distanceInput.useRadii = false;
   --
   --    // The outer loop progressively attempts to compute new separating axes.
   --    // This loop terminates when an axis is repeated (no progress is made).
   --    for(;;)
   --    {
   --       b2Transform xfA, xfB;
   --       sweepA.GetTransform(&xfA, t1);
   --       sweepB.GetTransform(&xfB, t1);
   --
   --       // Get the distance between shapes. We can also use the results
   --       // to get a separating axis.
   --       distanceInput.transformA = xfA;
   --       distanceInput.transformB = xfB;
   --       b2DistanceOutput distanceOutput;
   --       b2Distance(&distanceOutput, &cache, &distanceInput);
   --
   --       // If the shapes are overlapped, we give up on continuous collision.
   --       if (distanceOutput.distance <= 0.0f)
   --       {
   --          // Failure!
   --          output->state = b2TOIOutput::e_overlapped;
   --          output->t = 0.0f;
   --          break;
   --       }
   --
   --       if (distanceOutput.distance < target + tolerance)
   --       {
   --          // Victory!
   --          output->state = b2TOIOutput::e_touching;
   --          output->t = t1;
   --          break;
   --       }
   --
   --       // Initialize the separating axis.
   --       b2SeparationFunction fcn;
   --       fcn.Initialize(&cache, proxyA, sweepA, proxyB, sweepB, t1);
   --  #if 0
   --       // Dump the curve seen by the root finder
   --       {
   --          const int32 N = 100;
   --          float dx = 1.0f / N;
   --          float xs[N+1];
   --          float fs[N+1];
   --
   --          float x = 0.0f;
   --
   --          for (int32 i = 0; i <= N; ++i)
   --          {
   --             sweepA.GetTransform(&xfA, x);
   --             sweepB.GetTransform(&xfB, x);
   --             float f = fcn.Evaluate(xfA, xfB) - target;
   --
   --             printf("%g %g\n", x, f);
   --
   --             xs[i] = x;
   --             fs[i] = f;
   --
   --             x += dx;
   --          }
   --       }
   --  #endif
   --
   --       // Compute the TOI on the separating axis. We do this by successively
   --       // resolving the deepest point. This loop is bounded by the number of vertices.
   --       bool done = false;
   --       float t2 = tMax;
   --       int32 pushBackIter = 0;
   --       for (;;)
   --       {
   --          // Find the deepest point at t2. Store the witness point indices.
   --          int32 indexA, indexB;
   --          float s2 = fcn.FindMinSeparation(&indexA, &indexB, t2);
   --
   --          // Is the final configuration separated?
   --          if (s2 > target + tolerance)
   --          {
   --             // Victory!
   --             output->state = b2TOIOutput::e_separated;
   --             output->t = tMax;
   --             done = true;
   --             break;
   --          }
   --
   --          // Has the separation reached tolerance?
   --          if (s2 > target - tolerance)
   --          {
   --             // Advance the sweeps
   --             t1 = t2;
   --             break;
   --          }
   --
   --          // Compute the initial separation of the witness points.
   --          float s1 = fcn.Evaluate(indexA, indexB, t1);
   --
   --          // Check for initial overlap. This might happen if the root finder
   --          // runs out of iterations.
   --          if (s1 < target - tolerance)
   --          {
   --             output->state = b2TOIOutput::e_failed;
   --             output->t = t1;
   --             done = true;
   --             break;
   --          }
   --
   --          // Check for touching
   --          if (s1 <= target + tolerance)
   --          {
   --             // Victory! t1 should hold the TOI (could be 0.0).
   --             output->state = b2TOIOutput::e_touching;
   --             output->t = t1;
   --             done = true;
   --             break;
   --          }
   --
   --          // Compute 1D root of: f(x) - target = 0
   --          int32 rootIterCount = 0;
   --          float a1 = t1, a2 = t2;
   --          for (;;)
   --          {
   --             // Use a mix of the secant rule and bisection.
   --             float t;
   --             if (rootIterCount & 1)
   --             {
   --                // Secant rule to improve convergence.
   --                t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
   --             }
   --             else
   --             {
   --                // Bisection to guarantee progress.
   --                t = 0.5f * (a1 + a2);
   --             }
   --
   --             ++rootIterCount;
   --             ++b2_toiRootIters;
   --
   --             float s = fcn.Evaluate(indexA, indexB, t);
   --
   --             if (b2Abs(s - target) < tolerance)
   --             {
   --                // t2 holds a tentative value for t1
   --                t2 = t;
   --                break;
   --             }
   --
   --             // Ensure we continue to bracket the root.
   --             if (s > target)
   --             {
   --                a1 = t;
   --                s1 = s;
   --             }
   --             else
   --             {
   --                a2 = t;
   --                s2 = s;
   --             }
   --
   --             if (rootIterCount == 50)
   --             {
   --                break;
   --             }
   --          }
   --
   --          b2_toiMaxRootIters = b2Max(b2_toiMaxRootIters, rootIterCount);
   --
   --          ++pushBackIter;
   --
   --          if (pushBackIter == b2_maxPolygonVertices)
   --          {
   --             break;
   --          }
   --       }
   --
   --       ++iter;
   --       ++b2_toiIters;
   --
   --       if (done)
   --       {
   --          break;
   --       }
   --
   --       if (iter == k_maxIterations)
   --       {
   --          // Root finder got stuck. Semi-victory.
   --          output->state = b2TOIOutput::e_failed;
   --          output->t = t1;
   --          break;
   --       }
   --    }
   --
   --    b2_toiMaxIters = b2Max(b2_toiMaxIters, iter);
   --
   --    float time = timer.GetMilliseconds();
   --    b2_toiMaxTime = b2Max(b2_toiMaxTime, time);
   --    b2_toiTime += time;
   --  }
   --

   procedure b2TimeOfImpact (output :    out b2TOIOutput;
                             input  : in     b2TOIInput)
   is
      use b2_Timer,
          b2_Common;

      timer : b2Timer;

      proxyA : constant access constant b2DistanceProxy := input.proxyA'Access;
      proxyB : constant access constant b2DistanceProxy := input.proxyB'Access;

      sweepA : b2Sweep := input.sweepA;
      sweepB : b2Sweep := input.sweepB;

      tMax   : constant Real    := input.tMax;

      totalRadius : constant Real := proxyA.m_radius + proxyB.m_radius;
      target      : constant Real := Real'max (b2_linearSlop,  totalRadius - 3.0 * b2_linearSlop);
      tolerance   : constant Real := 0.25 * b2_linearSlop;
      pragma assert (target > tolerance);

      t1              : Real     :=  0.0;
      k_maxIterations : constant := 20;     -- TODO_ERIN b2Settings
      iter            : Natural  :=  0;

      cache         : aliased b2SimplexCache;
      distanceInput :         b2DistanceInput;

      fcn : b2SeparationFunction;

   begin
      b2_toiCalls  := b2_toiCalls + 1;

      output.state := e_unknown;
      output.t     := input.tMax;

      -- Large rotations can make the root finder fail, so we normalize the sweep angles.
      --
      normalize (sweepA);
      normalize (sweepB);


      -- Prepare input for distance query.
      cache.count            := 0;
      distanceInput.proxyA   := input.proxyA;
      distanceInput.proxyB   := input.proxyB;
      distanceInput.useRadii := False;

      -- The outer loop progressively attempts to compute new separating axes.
      -- This loop terminates when an axis is repeated (no progress is made).
      --
      loop
         declare
            xfA, xfB       : b2Transform;
            distanceOutput : b2DistanceOutput;
            unused         : Real;
         begin
            sweepA.getTransform (xfA, t1);
            sweepB.getTransform (xfB, t1);

            -- Get the distance between shapes. We can also use the results
            -- to get a separating axis.
            --
            distanceInput.transformA := xfA;
            distanceInput.transformB := xfB;

            b2Distance (distanceOutput, cache, distanceInput);

            -- If the shapes are overlapped, we give up on continuous collision.
            --
            if distanceOutput.distance <= 0.0
            then
               -- Failure!
               --
               output.state := e_overlapped;
               output.t     := 0.0;
               exit;
            end if;

            if distanceOutput.distance  <  target + tolerance
            then
               -- Victory!
               --
               output.state := e_touching;
               output.t     := t1;
               exit;
            end if;

            -- Initialize the separating axis.
            --
            unused := fcn.initialize (cache'Access, proxyA, sweepA, proxyB, sweepB, t1);

            --  if False   -- This doesn't compile !
            --  then
            --     -- Dump the curve seen by the root finder.
            --     --
            --     declare
            --        use ada.Text_IO;
            --
            --        N  : constant := 100;
            --        dx : Real     := 1.0 / Real (N);
            --
            --        xs : Reals (0 .. N);
            --        fs : Reals (0 .. N);
            --
            --        x  : Real := 0.0;
            --        f  : Real;
            --     begin
            --        for i in 0 .. N
            --        loop
            --           sweepA.getTransform (xfA, x);
            --           sweepB.getTransform (xfB, x);
            --
            --           f := evaluate (fcn, xfA, xfB)  -  target;
            --
            --           put_Line (x'Image & "   " & f'Image);
            --
            --           xs (i) := x;
            --           fs (i) := f;
            --           x      := x + dx;
            --        end loop;
            --     end;
            --  end if;
         end;


         -- Compute the TOI on the separating axis. We do this by successively
         -- resolving the deepest point. This loop is bounded by the number of vertices.
         --
         declare
            done         : Boolean := False;
            t2           : Real    := tMax;
            pushBackIter : Natural := 0;
         begin
            loop
               -- Find the deepest point at t2. Store the witness point indices.
               --
               declare
                  indexA,
                  indexB : Natural;
                  s1     : Real;
                  s2     : Real   := fcn.findMinSeparation (indexA, indexB, t2);
               begin
                  -- Is the final configuration separated?
                  --
                  if s2 > target + tolerance
                  then
                     -- Victory!
                     output.state := e_separated;
                     output.t     := tMax;
                     done         := True;
                     exit;
                  end if;

                  -- Has the separation reached tolerance?
                  if s2  >  target - tolerance
                  then
                     -- Advance the sweeps
                     --
                     t1 := t2;
                     exit;
                  end if;

                  -- Compute the initial separation of the witness points.
                  --
                  s1 := fcn.evaluate (indexA, indexB, t1);

                  -- Check for initial overlap. This might happen if the root finder
                  -- runs out of iterations.
                  --
                  if s1 < target - tolerance
                  then
                     output.state := e_failed;
                     output.t     := t1;
                     done         := True;
                     exit;
                  end if;

                  -- Check for touching.
                  --
                  if s1  <=  target + tolerance
                  then
                     -- Victory! t1 should hold the TOI (could be 0.0).
                     --
                     output.state := e_touching;
                     output.t     := t1;
                     done         := true;
                     exit;
                  end if;

                  declare
                     use Interfaces;

                     -- Compute 1D root of: f(x) - target = 0.
                     --
                     rootIterCount : Integer := 0;
                     a1            : Real        := t1;
                     a2            : Real        := t2;
                  begin
                     loop
                        declare
                           t, s : Real;
                        begin
                           -- Use a mix of the secant rule and bisection.
                           --
                           if (Unsigned_64 (rootIterCount) and 1) /= 0     -- rootIterCount is odd.
                           then
                              -- Secant rule to improve convergence.
                              --
                              t := a1  +    (target - s1) * (a2 - a1)
                                / (s2 - s1);

                           else                              -- rootIterCount is even.
                              -- Bisection to guarantee progress.
                              --
                              t := 0.5 * (a1 + a2);
                           end if;

                           rootIterCount   := rootIterCount + 1;
                           b2_toiRootIters := b2_toiRootIters + 1;

                           s := fcn.evaluate (indexA, indexB, t);

                           if abs (s - target) < tolerance
                           then
                              -- t2 holds a tentative value for t1.
                              --
                              t2 := t;
                              exit;
                           end if;

                           -- Ensure we continue to bracket the root.
                           --
                           if s > target
                           then
                              a1 := t;
                              s1 := s;
                           else
                              a2 := t;
                              s2 := s;
                           end if;

                           if rootIterCount = 50
                           then
                              exit;
                           end if;
                        end;
                     end loop;

                     b2_toiMaxRootIters := Natural'max (b2_toiMaxRootIters, rootIterCount);
                  end;

                  pushBackIter := pushBackIter + 1;

                  if pushBackIter = b2_maxPolygonVertices
                  then
                     exit;
                  end if;
               end;
            end loop;


            iter        :=  iter + 1;
            b2_toiIters := b2_toiIters + 1;

            if done
            then
               exit;
            end if;

            if iter = k_maxIterations
            then
               -- Root finder got stuck. Semi-victory.
               --
               output.state := e_failed;
               output.t     := t1;
               exit;
            end if;

         end;
      end loop;


      b2_toiMaxIters := Integer'max (b2_toiMaxIters, iter);

      declare
         time : constant Real := timer.getMilliseconds;
      begin
         b2_toiMaxTime := Real'max (b2_toiMaxTime, time);
         b2_toiTime    := b2_toiTime + time;
      end;
   end b2TimeOfImpact;



end b2_Time_of_impact;
