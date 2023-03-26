with
     box2d.b2_contact_Solver,
     box2d.b2_Body,
     box2d.b2_Shape,
     box2d.b2_Fixture,
     box2d.b2_World,

     ada.unchecked_Deallocation;


package body box2d.b2_contact_Solver
is
   use b2_Fixture,
       b2_Body,
       b2_Shape;



   --  // Solver debugging is normally disabled because the block solver sometimes has to deal with a poorly conditioned effective mass matrix.
   --
   --  #define B2_DEBUG_SOLVER 0
   --

   B2_DEBUG_SOLVER : constant Boolean := False;



   --  bool g_blockSolve = true;
   --

   g_blockSolve    : constant Boolean := True;



   --  b2ContactSolver::b2ContactSolver(b2ContactSolverDef* def)
   --  {
   --    m_step = def->step;
   --    m_allocator = def->allocator;
   --    m_count = def->count;
   --    m_positionConstraints = (b2ContactPositionConstraint*)m_allocator->Allocate(m_count * sizeof(b2ContactPositionConstraint));
   --    m_velocityConstraints = (b2ContactVelocityConstraint*)m_allocator->Allocate(m_count * sizeof(b2ContactVelocityConstraint));
   --    m_positions = def->positions;
   --    m_velocities = def->velocities;
   --    m_contacts = def->contacts;
   --
   --    // Initialize position independent portions of the constraints.
   --    for (int32 i = 0; i < m_count; ++i)
   --    {
   --       b2Contact* contact = m_contacts[i];
   --
   --       b2Fixture* fixtureA = contact->m_fixtureA;
   --       b2Fixture* fixtureB = contact->m_fixtureB;
   --       b2Shape* shapeA = fixtureA->GetShape();
   --       b2Shape* shapeB = fixtureB->GetShape();
   --       float radiusA = shapeA->m_radius;
   --       float radiusB = shapeB->m_radius;
   --       b2Body* bodyA = fixtureA->GetBody();
   --       b2Body* bodyB = fixtureB->GetBody();
   --       b2Manifold* manifold = contact->GetManifold();
   --
   --       int32 pointCount = manifold->pointCount;
   --       b2Assert(pointCount > 0);
   --
   --       b2ContactVelocityConstraint* vc = m_velocityConstraints + i;
   --       vc->friction = contact->m_friction;
   --       vc->restitution = contact->m_restitution;
   --       vc->threshold = contact->m_restitutionThreshold;
   --       vc->tangentSpeed = contact->m_tangentSpeed;
   --       vc->indexA = bodyA->m_islandIndex;
   --       vc->indexB = bodyB->m_islandIndex;
   --       vc->invMassA = bodyA->m_invMass;
   --       vc->invMassB = bodyB->m_invMass;
   --       vc->invIA = bodyA->m_invI;
   --       vc->invIB = bodyB->m_invI;
   --       vc->contactIndex = i;
   --       vc->pointCount = pointCount;
   --       vc->K.SetZero();
   --       vc->normalMass.SetZero();
   --
   --       b2ContactPositionConstraint* pc = m_positionConstraints + i;
   --       pc->indexA = bodyA->m_islandIndex;
   --       pc->indexB = bodyB->m_islandIndex;
   --       pc->invMassA = bodyA->m_invMass;
   --       pc->invMassB = bodyB->m_invMass;
   --       pc->localCenterA = bodyA->m_sweep.localCenter;
   --       pc->localCenterB = bodyB->m_sweep.localCenter;
   --       pc->invIA = bodyA->m_invI;
   --       pc->invIB = bodyB->m_invI;
   --       pc->localNormal = manifold->localNormal;
   --       pc->localPoint = manifold->localPoint;
   --       pc->pointCount = pointCount;
   --       pc->radiusA = radiusA;
   --       pc->radiusB = radiusB;
   --       pc->type = manifold->type;
   --
   --       for (int32 j = 0; j < pointCount; ++j)
   --       {
   --          b2ManifoldPoint* cp = manifold->points + j;
   --          b2VelocityConstraintPoint* vcp = vc->points + j;
   --
   --          if (m_step.warmStarting)
   --          {
   --             vcp->normalImpulse = m_step.dtRatio * cp->normalImpulse;
   --             vcp->tangentImpulse = m_step.dtRatio * cp->tangentImpulse;
   --          }
   --          else
   --          {
   --             vcp->normalImpulse = 0.0f;
   --             vcp->tangentImpulse = 0.0f;
   --          }
   --
   --          vcp->rA.SetZero();
   --          vcp->rB.SetZero();
   --          vcp->normalMass = 0.0f;
   --          vcp->tangentMass = 0.0f;
   --          vcp->velocityBias = 0.0f;
   --
   --          pc->localPoints[j] = cp->localPoint;
   --       }
   --    }
   --  }
   --

   function  to_b2ContactSolver (def : access b2ContactSolverDef) return b2ContactSolver
   is
      Self : b2ContactSolver;
   begin
      Self.m_step                := def.step;
      Self.m_count               := def.count;
      Self.m_positionConstraints := new b2ContactPositionConstraints (0 .. Self.m_count - 1);
      Self.m_velocityConstraints := new b2ContactVelocityConstraints (0 .. Self.m_count - 1);
      Self.m_positions           := def.positions;
      Self.m_velocities          := def.velocities;
      Self.m_contacts            := def.contacts;

      -- Initialize position independent portions of the constraints.
      --
      for i in 0 .. Self.m_count - 1
      loop
         declare
            contact    : constant access b2Contact  := Self.m_contacts (i);

            fixtureA   : constant access b2Fixture  := contact.getFixtureA;
            fixtureB   : constant access b2Fixture  := contact.getFixtureB;

            shapeA     : constant access b2Shape    := fixtureA.getShape;
            shapeB     : constant access b2Shape    := fixtureB.getShape;

            radiusA    : constant        Real       := shapeA.m_radius;
            radiusB    : constant        Real       := shapeB.m_radius;

            bodyA      : constant access b2Body     := fixtureA.getBody;
            bodyB      : constant access b2Body     := fixtureB.getBody;

            manifold   : constant access b2Manifold := contact.getManifold;

            pointCount : constant        Natural    := manifold.pointCount;
            pragma assert (pointCount > 0);

            vc : b2ContactVelocityConstraint renames Self.m_velocityConstraints (i);
            pc : b2ContactPositionConstraint renames Self.m_positionConstraints (i);
         begin
            vc.friction     := contact.getFriction;
            vc.restitution  := contact.getRestitution;
            vc.threshold    := contact.getRestitutionThreshold;
            vc.tangentSpeed := contact.getTangentSpeed;
            vc.indexA       := bodyA.m_islandIndex;
            vc.indexB       := bodyB.m_islandIndex;
            vc.invMassA     := bodyA.m_invMass;
            vc.invMassB     := bodyB.m_invMass;
            vc.invIA        := bodyA.m_invI;
            vc.invIB        := bodyB.m_invI;
            vc.contactIndex := i;
            vc.pointCount   := pointCount;
            setZero (vc.K);
            setZero (vc.normalMass);

            pc.indexA       := bodyA.m_islandIndex;
            pc.indexB       := bodyB.m_islandIndex;
            pc.invMassA     := bodyA.m_invMass;
            pc.invMassB     := bodyB.m_invMass;
            pc.localCenterA := bodyA.m_sweep.localCenter;
            pc.localCenterB := bodyB.m_sweep.localCenter;
            pc.invIA        := bodyA.m_invI;
            pc.invIB        := bodyB.m_invI;
            pc.localNormal  := manifold.localNormal;
            pc.localPoint   := manifold.localPoint;
            pc.pointCount   := pointCount;
            pc.radiusA      := radiusA;
            pc.radiusB      := radiusB;
            pc.Kind         := manifold.Kind;

           for j in 0 .. pointCount - 1
            loop
               declare
                  cp  : b2ManifoldPoint           renames manifold.Points (j);
                  vcp : b2VelocityConstraintPoint renames vc.points (j);
               begin
                  if Self.m_step.warmStarting
                  then
                     vcp. normalImpulse := Self.m_step.dtRatio * cp.normalImpulse;
                     vcp.tangentImpulse := Self.m_step.dtRatio * cp.tangentImpulse;
                  else
                     vcp. normalImpulse := 0.0;
                     vcp.tangentImpulse := 0.0;
                  end if;

                  setZero (vcp.rA);
                  setZero (vcp.rB);

                  vcp.normalMass   := 0.0;
                  vcp.tangentMass  := 0.0;
                  vcp.velocityBias := 0.0;

                  pc.localPoints (j) := cp.localPoint;
               end;
            end loop;
         end;
      end loop;

      return Self;
   end to_b2ContactSolver;




   --  b2ContactSolver::~b2ContactSolver()
   --  {
   --    m_allocator->Free(m_velocityConstraints);
   --    m_allocator->Free(m_positionConstraints);
   --  }
   --

   procedure destruct (Self : in out b2ContactSolver)
   is
      procedure free is new ada.unchecked_Deallocation (b2ContactVelocityConstraints,
                                                        b2ContactVelocityConstraints_ptr);
      procedure free is new ada.unchecked_Deallocation (b2ContactPositionConstraints,
                                                        b2ContactPositionConstraints_ptr);
   begin
      free (Self.m_velocityConstraints);
      free (Self.m_positionConstraints);
   end destruct;





   --  // Initialize position dependent portions of the velocity constraints.
   --  void b2ContactSolver::InitializeVelocityConstraints()
   --  {
   --    for (int32 i = 0; i < m_count; ++i)
   --    {
   --       b2ContactVelocityConstraint* vc = m_velocityConstraints + i;
   --       b2ContactPositionConstraint* pc = m_positionConstraints + i;
   --
   --       float radiusA = pc->radiusA;
   --       float radiusB = pc->radiusB;
   --       b2Manifold* manifold = m_contacts[vc->contactIndex]->GetManifold();
   --
   --       int32 indexA = vc->indexA;
   --       int32 indexB = vc->indexB;
   --
   --       float mA = vc->invMassA;
   --       float mB = vc->invMassB;
   --       float iA = vc->invIA;
   --       float iB = vc->invIB;
   --       b2Vec2 localCenterA = pc->localCenterA;
   --       b2Vec2 localCenterB = pc->localCenterB;
   --
   --       b2Vec2 cA = m_positions[indexA].c;
   --       float aA = m_positions[indexA].a;
   --       b2Vec2 vA = m_velocities[indexA].v;
   --       float wA = m_velocities[indexA].w;
   --
   --       b2Vec2 cB = m_positions[indexB].c;
   --       float aB = m_positions[indexB].a;
   --       b2Vec2 vB = m_velocities[indexB].v;
   --       float wB = m_velocities[indexB].w;
   --
   --       b2Assert(manifold->pointCount > 0);
   --
   --       b2Transform xfA, xfB;
   --       xfA.q.Set(aA);
   --       xfB.q.Set(aB);
   --       xfA.p = cA - b2Mul(xfA.q, localCenterA);
   --       xfB.p = cB - b2Mul(xfB.q, localCenterB);
   --
   --       b2WorldManifold worldManifold;
   --       worldManifold.Initialize(manifold, xfA, radiusA, xfB, radiusB);
   --
   --       vc->normal = worldManifold.normal;
   --
   --       int32 pointCount = vc->pointCount;
   --       for (int32 j = 0; j < pointCount; ++j)
   --       {
   --          b2VelocityConstraintPoint* vcp = vc->points + j;
   --
   --          vcp->rA = worldManifold.points[j] - cA;
   --          vcp->rB = worldManifold.points[j] - cB;
   --
   --          float rnA = b2Cross(vcp->rA, vc->normal);
   --          float rnB = b2Cross(vcp->rB, vc->normal);
   --
   --          float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
   --
   --          vcp->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;
   --
   --          b2Vec2 tangent = b2Cross(vc->normal, 1.0f);
   --
   --          float rtA = b2Cross(vcp->rA, tangent);
   --          float rtB = b2Cross(vcp->rB, tangent);
   --
   --          float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
   --
   --          vcp->tangentMass = kTangent > 0.0f ? 1.0f /  kTangent : 0.0f;
   --
   --          // Setup a velocity bias for restitution.
   --          vcp->velocityBias = 0.0f;
   --          float vRel = b2Dot(vc->normal, vB + b2Cross(wB, vcp->rB) - vA - b2Cross(wA, vcp->rA));
   --          if (vRel < -vc->threshold)
   --          {
   --             vcp->velocityBias = -vc->restitution * vRel;
   --          }
   --       }
   --
   --       // If we have two points, then prepare the block solver.
   --       if (vc->pointCount == 2 && g_blockSolve)
   --       {
   --          b2VelocityConstraintPoint* vcp1 = vc->points + 0;
   --          b2VelocityConstraintPoint* vcp2 = vc->points + 1;
   --
   --          float rn1A = b2Cross(vcp1->rA, vc->normal);
   --          float rn1B = b2Cross(vcp1->rB, vc->normal);
   --          float rn2A = b2Cross(vcp2->rA, vc->normal);
   --          float rn2B = b2Cross(vcp2->rB, vc->normal);
   --
   --          float k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
   --          float k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
   --          float k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;
   --
   --          // Ensure a reasonable condition number.
   --          const float k_maxConditionNumber = 1000.0f;
   --          if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
   --          {
   --             // K is safe to invert.
   --             vc->K.ex.Set(k11, k12);
   --             vc->K.ey.Set(k12, k22);
   --             vc->normalMass = vc->K.GetInverse();
   --          }
   --          else
   --          {
   --             // The constraints are redundant, just use one.
   --             // TODO_ERIN use deepest?
   --             vc->pointCount = 1;
   --          }
   --       }
   --    }
   --  }
   --

   procedure initializeVelocityConstraints (Self : in out b2ContactSolver)
   is
   begin
      for i in 0 .. Self.m_count - 1
      loop
         declare
            vc : b2ContactVelocityConstraint renames Self.m_velocityConstraints (i);
            pc : b2ContactPositionConstraint renames Self.m_positionConstraints (i);

            radiusA : constant Real := pc.radiusA;
            radiusB : constant Real := pc.radiusB;

            manifold : constant access b2Manifold := Self.m_contacts (vc.contactIndex).getManifold;

            indexA : constant Natural := vc.indexA;
            indexB : constant Natural := vc.indexB;

            mA : constant Real := vc.invMassA;
            mB : constant Real := vc.invMassB;

            iA : constant Real := vc.invIA;
            iB : constant Real := vc.invIB;

            localCenterA : constant b2Vec2 := pc.localCenterA;
            localCenterB : constant b2Vec2 := pc.localCenterB;

            cA : constant b2Vec2 := Self.m_positions  (indexA).c;
            aA : constant Real   := Self.m_positions  (indexA).a;
            vA : constant b2Vec2 := Self.m_velocities (indexA).v;
            wA : constant Real   := Self.m_velocities (indexA).w;

            cB : constant b2Vec2 := Self.m_positions  (indexB).c;
            aB : constant Real   := Self.m_positions  (indexB).a;
            vB : constant b2Vec2 := Self.m_velocities (indexB).v;
            wB : constant Real   := Self.m_velocities (indexB).w;

            pragma assert (manifold.pointCount > 0);

            xfA,
            xfB : b2Transform;

            worldManifold : b2WorldManifold;
            pointCount    : Natural;

         begin
            set (xfA.q, aA);
            set (xfB.q, aB);

            xfA.p := cA - b2Mul (xfA.q, localCenterA);
            xfB.p := cB - b2Mul (xfB.q, localCenterB);

            initialize (worldManifold, manifold.all, xfA, radiusA,
                                                     xfB, radiusB);
            vc.normal := worldManifold.normal;

            pointCount := vc.pointCount;


            for j in 0 .. pointCount - 1
            loop
               declare
                  vcp      : b2VelocityConstraintPoint renames vc.points (j);
                  rnA, rnB : Real;
                  kNormal  : Real;

                  tangent  : b2Vec2;
                  rtA, rtB : Real;
                  kTangent : Real;
                  vRel     : Real;
               begin
                  vcp.rA := worldManifold.points (j) - cA;
                  vcp.rB := worldManifold.points (j) - cB;

                  rnA := b2Cross (vcp.rA, vc.normal);
                  rnB := b2Cross (vcp.rB, vc.normal);

                  kNormal :=   mA + mB
                             + iA * rnA * rnA
                             + iB * rnB * rnB;

                  vcp.normalMass := (if kNormal > 0.0 then 1.0 / kNormal
                                                      else 0.0);

                  tangent := b2Cross (vc.normal, 1.0);

                  rtA := b2Cross (vcp.rA, tangent);
                  rtB := b2Cross (vcp.rB, tangent);

                  kTangent :=   mA + mB
                              + iA * rtA * rtA
                              + iB * rtB * rtB;

                  vcp.tangentMass := (if kTangent > 0.0 then 1.0 / kTangent
                                                        else 0.0);

                  -- Setup a velocity bias for restitution.
                  --
                  vcp.velocityBias := 0.0;

                  vRel := b2Dot (vc.normal,   vB + b2Cross (wB, vcp.rB)
                                            - vA - b2Cross (wA, vcp.rA));
                  if vRel < -vc.threshold
                  then
                     vcp.velocityBias := -vc.restitution * vRel;
                  end if;
               end;
            end loop;


            -- If we have two points, then prepare the block solver.
            --
            if vc.pointCount = 2 and g_blockSolve
            then
               declare
                  vcp1 : b2VelocityConstraintPoint renames vc.points (0);
                  vcp2 : b2VelocityConstraintPoint renames vc.points (1);

                  rn1A : constant Real := b2Cross (vcp1.rA, vc.normal);
                  rn1B : constant Real := b2Cross (vcp1.rB, vc.normal);
                  rn2A : constant Real := b2Cross (vcp2.rA, vc.normal);
                  rn2B : constant Real := b2Cross (vcp2.rB, vc.normal);

                  k11  : constant Real := mA + mB  +  iA * rn1A * rn1A  +  iB * rn1B * rn1B;
                  k22  : constant Real := mA + mB  +  iA * rn2A * rn2A  +  iB * rn2B * rn2B;
                  k12  : constant Real := mA + mB  +  iA * rn1A * rn2A  +  iB * rn1B * rn2B;

                  -- Ensure a reasonable condition number.
                  --
                  k_maxConditionNumber : constant := 1000.0;
               begin
                  if k11 * k11 <    k_maxConditionNumber
                                 * (k11 * k22  -  k12 * k12)
                  then
                     -- K is safe to invert.
                     set (vc.K.ex,  k11, k12);
                     set (vc.K.ey,  k12, k22);

                     vc.normalMass := getInverse (vc.K);

                  else
                     -- The constraints are redundant, just use one.
                     -- TODO_ERIN use deepest?
                     vc.pointCount := 1;
                  end if;
               end;
            end if;

         end;
      end loop;
   end initializeVelocityConstraints;




   --  void b2ContactSolver::WarmStart()
   --  {
   --    // Warm start.
   --    for (int32 i = 0; i < m_count; ++i)
   --    {
   --       b2ContactVelocityConstraint* vc = m_velocityConstraints + i;
   --
   --       int32 indexA = vc->indexA;
   --       int32 indexB = vc->indexB;
   --       float mA = vc->invMassA;
   --       float iA = vc->invIA;
   --       float mB = vc->invMassB;
   --       float iB = vc->invIB;
   --       int32 pointCount = vc->pointCount;
   --
   --       b2Vec2 vA = m_velocities[indexA].v;
   --       float wA = m_velocities[indexA].w;
   --       b2Vec2 vB = m_velocities[indexB].v;
   --       float wB = m_velocities[indexB].w;
   --
   --       b2Vec2 normal = vc->normal;
   --       b2Vec2 tangent = b2Cross(normal, 1.0f);
   --
   --       for (int32 j = 0; j < pointCount; ++j)
   --       {
   --          b2VelocityConstraintPoint* vcp = vc->points + j;
   --          b2Vec2 P = vcp->normalImpulse * normal + vcp->tangentImpulse * tangent;
   --          wA -= iA * b2Cross(vcp->rA, P);
   --          vA -= mA * P;
   --          wB += iB * b2Cross(vcp->rB, P);
   --          vB += mB * P;
   --       }
   --
   --       m_velocities[indexA].v = vA;
   --       m_velocities[indexA].w = wA;
   --       m_velocities[indexB].v = vB;
   --       m_velocities[indexB].w = wB;
   --    }
   --  }
   --

   procedure warmStart (Self : in out b2ContactSolver)
   is
   begin
      -- Warm start.
      --
      for i in 0 .. Self.m_count - 1
      loop
         declare
            vc : b2ContactVelocityConstraint renames Self.m_velocityConstraints (i);

            indexA : constant Natural := vc.indexA;
            indexB : constant Natural := vc.indexB;

            mA     : constant Real    := vc.invMassA;
            iA     : constant Real    := vc.invIA;
            mB     : constant Real    := vc.invMassB;
            iB     : constant Real    := vc.invIB;

            pointCount : constant Natural := vc.pointCount;

            vA : b2Vec2 := Self.m_velocities (indexA).v;
            wA : Real   := Self.m_velocities (indexA).w;
            vB : b2Vec2 := Self.m_velocities (indexB).v;
            wB : Real   := Self.m_velocities (indexB).w;

            normal  : constant b2Vec2 := vc.normal;
            tangent : constant b2Vec2 := b2Cross (normal, 1.0);
         begin
            for j in 0 .. pointCount - 1
            loop
               declare
                  vcp : b2VelocityConstraintPoint renames vc.points (j);

                  P   : constant b2Vec2 :=   vcp. normalImpulse * normal
                                           + vcp.tangentImpulse * tangent;
               begin
                  wA := wA - iA * b2Cross (vcp.rA, P);
                  vA := vA - mA * P;
                  wB := wB + iB * b2Cross (vcp.rB, P);
                  vB := vB + mB * P;
               end;
            end loop;

            Self.m_velocities (indexA).v := vA;
            Self.m_velocities (indexA).w := wA;
            Self.m_velocities (indexB).v := vB;
            Self.m_velocities (indexB).w := wB;
         end;
      end loop;
   end warmStart;



   --  void b2ContactSolver::SolveVelocityConstraints()
   --  {
   --    for (int32 i = 0; i < m_count; ++i)
   --    {
   --       b2ContactVelocityConstraint* vc = m_velocityConstraints + i;
   --
   --       int32 indexA = vc->indexA;
   --       int32 indexB = vc->indexB;
   --       float mA = vc->invMassA;
   --       float iA = vc->invIA;
   --       float mB = vc->invMassB;
   --       float iB = vc->invIB;
   --       int32 pointCount = vc->pointCount;
   --
   --       b2Vec2 vA = m_velocities[indexA].v;
   --       float wA = m_velocities[indexA].w;
   --       b2Vec2 vB = m_velocities[indexB].v;
   --       float wB = m_velocities[indexB].w;
   --
   --       b2Vec2 normal = vc->normal;
   --       b2Vec2 tangent = b2Cross(normal, 1.0f);
   --       float friction = vc->friction;
   --
   --       b2Assert(pointCount == 1 || pointCount == 2);
   --
   --       // Solve tangent constraints first because non-penetration is more important
   --       // than friction.
   --       for (int32 j = 0; j < pointCount; ++j)
   --       {
   --          b2VelocityConstraintPoint* vcp = vc->points + j;
   --
   --          // Relative velocity at contact
   --          b2Vec2 dv = vB + b2Cross(wB, vcp->rB) - vA - b2Cross(wA, vcp->rA);
   --
   --          // Compute tangent force
   --          float vt = b2Dot(dv, tangent) - vc->tangentSpeed;
   --          float lambda = vcp->tangentMass * (-vt);
   --
   --          // b2Clamp the accumulated force
   --          float maxFriction = friction * vcp->normalImpulse;
   --          float newImpulse = b2Clamp(vcp->tangentImpulse + lambda, -maxFriction, maxFriction);
   --          lambda = newImpulse - vcp->tangentImpulse;
   --          vcp->tangentImpulse = newImpulse;
   --
   --          // Apply contact impulse
   --          b2Vec2 P = lambda * tangent;
   --
   --          vA -= mA * P;
   --          wA -= iA * b2Cross(vcp->rA, P);
   --
   --          vB += mB * P;
   --          wB += iB * b2Cross(vcp->rB, P);
   --       }
   --
   --       // Solve normal constraints
   --       if (pointCount == 1 || g_blockSolve == false)
   --       {
   --          for (int32 j = 0; j < pointCount; ++j)
   --          {
   --             b2VelocityConstraintPoint* vcp = vc->points + j;
   --
   --             // Relative velocity at contact
   --             b2Vec2 dv = vB + b2Cross(wB, vcp->rB) - vA - b2Cross(wA, vcp->rA);
   --
   --             // Compute normal impulse
   --             float vn = b2Dot(dv, normal);
   --             float lambda = -vcp->normalMass * (vn - vcp->velocityBias);
   --
   --             // b2Clamp the accumulated impulse
   --             float newImpulse = b2Max(vcp->normalImpulse + lambda, 0.0f);
   --             lambda = newImpulse - vcp->normalImpulse;
   --             vcp->normalImpulse = newImpulse;
   --
   --             // Apply contact impulse
   --             b2Vec2 P = lambda * normal;
   --             vA -= mA * P;
   --             wA -= iA * b2Cross(vcp->rA, P);
   --
   --             vB += mB * P;
   --             wB += iB * b2Cross(vcp->rB, P);
   --          }
   --       }
   --       else
   --       {
   --          // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
   --          // Build the mini LCP for this contact patch
   --          //
   --          // vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
   --          //
   --          // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
   --          // b = vn0 - velocityBias
   --          //
   --          // The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
   --          // implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
   --          // vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
   --          // solution that satisfies the problem is chosen.
   --          //
   --          // In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
   --          // that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
   --          //
   --          // Substitute:
   --          //
   --          // x = a + d
   --          //
   --          // a := old total impulse
   --          // x := new total impulse
   --          // d := incremental impulse
   --          //
   --          // For the current iteration we extend the formula for the incremental impulse
   --          // to compute the new total impulse:
   --          //
   --          // vn = A * d + b
   --          //    = A * (x - a) + b
   --          //    = A * x + b - A * a
   --          //    = A * x + b'
   --          // b' = b - A * a;
   --
   --          b2VelocityConstraintPoint* cp1 = vc->points + 0;
   --          b2VelocityConstraintPoint* cp2 = vc->points + 1;
   --
   --          b2Vec2 a(cp1->normalImpulse, cp2->normalImpulse);
   --          b2Assert(a.x >= 0.0f && a.y >= 0.0f);
   --
   --          // Relative velocity at contact
   --          b2Vec2 dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);
   --          b2Vec2 dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);
   --
   --          // Compute normal velocity
   --          float vn1 = b2Dot(dv1, normal);
   --          float vn2 = b2Dot(dv2, normal);
   --
   --          b2Vec2 b;
   --          b.x = vn1 - cp1->velocityBias;
   --          b.y = vn2 - cp2->velocityBias;
   --
   --          // Compute b'
   --          b -= b2Mul(vc->K, a);
   --
   --          const float k_errorTol = 1e-3f;
   --          B2_NOT_USED(k_errorTol);
   --
   --          for (;;)
   --          {
   --             //
   --             // Case 1: vn = 0
   --             //
   --             // 0 = A * x + b'
   --             //
   --             // Solve for x:
   --             //
   --             // x = - inv(A) * b'
   --             //
   --             b2Vec2 x = - b2Mul(vc->normalMass, b);
   --
   --             if (x.x >= 0.0f && x.y >= 0.0f)
   --             {
   --                // Get the incremental impulse
   --                b2Vec2 d = x - a;
   --
   --                // Apply incremental impulse
   --                b2Vec2 P1 = d.x * normal;
   --                b2Vec2 P2 = d.y * normal;
   --                vA -= mA * (P1 + P2);
   --                wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));
   --
   --                vB += mB * (P1 + P2);
   --                wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));
   --
   --                // Accumulate
   --                cp1->normalImpulse = x.x;
   --                cp2->normalImpulse = x.y;
   --
   --  #if B2_DEBUG_SOLVER == 1
   --                // Postconditions
   --                dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);
   --                dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);
   --
   --                // Compute normal velocity
   --                vn1 = b2Dot(dv1, normal);
   --                vn2 = b2Dot(dv2, normal);
   --
   --                b2Assert(b2Abs(vn1 - cp1->velocityBias) < k_errorTol);
   --                b2Assert(b2Abs(vn2 - cp2->velocityBias) < k_errorTol);
   --  #endif
   --                break;
   --             }
   --
   --             //
   --             // Case 2: vn1 = 0 and x2 = 0
   --             //
   --             //   0 = a11 * x1 + a12 * 0 + b1'
   --             // vn2 = a21 * x1 + a22 * 0 + b2'
   --             //
   --             x.x = - cp1->normalMass * b.x;
   --             x.y = 0.0f;
   --             vn1 = 0.0f;
   --             vn2 = vc->K.ex.y * x.x + b.y;
   --             if (x.x >= 0.0f && vn2 >= 0.0f)
   --             {
   --                // Get the incremental impulse
   --                b2Vec2 d = x - a;
   --
   --                // Apply incremental impulse
   --                b2Vec2 P1 = d.x * normal;
   --                b2Vec2 P2 = d.y * normal;
   --                vA -= mA * (P1 + P2);
   --                wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));
   --
   --                vB += mB * (P1 + P2);
   --                wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));
   --
   --                // Accumulate
   --                cp1->normalImpulse = x.x;
   --                cp2->normalImpulse = x.y;
   --
   --  #if B2_DEBUG_SOLVER == 1
   --                // Postconditions
   --                dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);
   --
   --                // Compute normal velocity
   --                vn1 = b2Dot(dv1, normal);
   --
   --                b2Assert(b2Abs(vn1 - cp1->velocityBias) < k_errorTol);
   --  #endif
   --                break;
   --             }
   --
   --
   --             //
   --             // Case 3: vn2 = 0 and x1 = 0
   --             //
   --             // vn1 = a11 * 0 + a12 * x2 + b1'
   --             //   0 = a21 * 0 + a22 * x2 + b2'
   --             //
   --             x.x = 0.0f;
   --             x.y = - cp2->normalMass * b.y;
   --             vn1 = vc->K.ey.x * x.y + b.x;
   --             vn2 = 0.0f;
   --
   --             if (x.y >= 0.0f && vn1 >= 0.0f)
   --             {
   --                // Resubstitute for the incremental impulse
   --                b2Vec2 d = x - a;
   --
   --                // Apply incremental impulse
   --                b2Vec2 P1 = d.x * normal;
   --                b2Vec2 P2 = d.y * normal;
   --                vA -= mA * (P1 + P2);
   --                wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));
   --
   --                vB += mB * (P1 + P2);
   --                wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));
   --
   --                // Accumulate
   --                cp1->normalImpulse = x.x;
   --                cp2->normalImpulse = x.y;
   --
   --  #if B2_DEBUG_SOLVER == 1
   --                // Postconditions
   --                dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);
   --
   --                // Compute normal velocity
   --                vn2 = b2Dot(dv2, normal);
   --
   --                b2Assert(b2Abs(vn2 - cp2->velocityBias) < k_errorTol);
   --  #endif
   --                break;
   --             }
   --
   --             //
   --             // Case 4: x1 = 0 and x2 = 0
   --             //
   --             // vn1 = b1
   --             // vn2 = b2;
   --             x.x = 0.0f;
   --             x.y = 0.0f;
   --             vn1 = b.x;
   --             vn2 = b.y;
   --
   --             if (vn1 >= 0.0f && vn2 >= 0.0f )
   --             {
   --                // Resubstitute for the incremental impulse
   --                b2Vec2 d = x - a;
   --
   --                // Apply incremental impulse
   --                b2Vec2 P1 = d.x * normal;
   --                b2Vec2 P2 = d.y * normal;
   --                vA -= mA * (P1 + P2);
   --                wA -= iA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));
   --
   --                vB += mB * (P1 + P2);
   --                wB += iB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));
   --
   --                // Accumulate
   --                cp1->normalImpulse = x.x;
   --                cp2->normalImpulse = x.y;
   --
   --                break;
   --             }
   --
   --             // No solution, give up. This is hit sometimes, but it doesn't seem to matter.
   --             break;
   --          }
   --       }
   --
   --       m_velocities[indexA].v = vA;
   --       m_velocities[indexA].w = wA;
   --       m_velocities[indexB].v = vB;
   --       m_velocities[indexB].w = wB;
   --    }
   --  }
   --

   procedure solveVelocityConstraints (Self : in out b2ContactSolver)
   is
   begin
      for i in 0 .. Self.m_count - 1
      loop
         declare
            vc : b2ContactVelocityConstraint renames Self.m_velocityConstraints (i);

            indexA : constant Natural := vc.indexA;
            indexB : constant Natural := vc.indexB;

            mA : constant Real := vc.invMassA;
            iA : constant Real := vc.invIA;
            mB : constant Real := vc.invMassB;
            iB : constant Real := vc.invIB;

            pointCount : constant Natural := vc.pointCount;

            vA : b2Vec2 := Self.m_velocities (indexA).v;
            wA : Real   := Self.m_velocities (indexA).w;
            vB : b2Vec2 := Self.m_velocities (indexB).v;
            wB : Real   := Self.m_velocities (indexB).w;

            normal   : constant b2Vec2 := vc.normal;
            tangent  : constant b2Vec2 := b2Cross (normal, 1.0);
            friction : constant Real   := vc.friction;

            pragma assert (pointCount = 1 or pointCount = 2);

         begin
            -- Solve tangent constraints first because non-penetration is more important than friction.
            --
            for j in 0 .. pointCount - 1
            loop
               declare
                  vcp : b2VelocityConstraintPoint renames vc.points (j);

                  -- Relative velocity at contact.
                  --
                  dv : constant b2Vec2 :=   vB + b2Cross (wB, vcp.rB)
                                 - vA - b2Cross (wA, vcp.rA);

                  -- Compute tangent force.
                  --
                  vt     : constant Real := b2Dot (dv, tangent) - vc.tangentSpeed;
                   lambda : Real := vcp.tangentMass * (-vt);

                  -- b2Clamp the accumulated force.
                  --
                  maxFriction : constant Real := friction * vcp.normalImpulse;
                  newImpulse  : constant Real := b2Clamp ( vcp.tangentImpulse + lambda,
                                                 -maxFriction,
                                                  maxFriction);
                  P : b2Vec2;
               begin
                  lambda             := newImpulse - vcp.tangentImpulse;
                  vcp.tangentImpulse := newImpulse;

                  -- Apply contact impulse.
                  --
                  P := lambda * tangent;

                  vA := vA - mA * P;
                  wA := wA - iA * b2Cross (vcp.rA, P);

                  vB := vB + mB * P;
                  wB := wB + iB * b2Cross (vcp.rB, P);
               end;
            end loop;


            -- Solve normal constraints.
            --
            if   pointCount   = 1
              or g_blockSolve = False
            then
               for j in 0 .. pointCount - 1
               loop
                  declare
                     vcp : b2VelocityConstraintPoint renames vc.points (j);

                     -- Relative velocity at contact
                     dv  : constant b2Vec2 :=   vB + b2Cross (wB, vcp.rB)
                                     - vA - b2Cross (wA, vcp.rA);

                     -- Compute normal impulse
                     vn     : constant Real := b2Dot (dv, normal);
                     lambda : Real := -vcp.normalMass * (vn - vcp.velocityBias);

                     -- b2Clamp the accumulated impulse
                     newImpulse : constant Real := Real'max (vcp.normalImpulse + lambda, 0.0);

                     P : b2Vec2;
                  begin
                     lambda            := newImpulse - vcp.normalImpulse;
                     vcp.normalImpulse := newImpulse;

                     -- Apply contact impulse
                     P  := lambda * normal;

                     vA := vA - mA * P;
                     wA := wA - iA * b2Cross (vcp.rA, P);

                     vB := vB + mB * P;
                     wB := wB + iB * b2Cross (vcp.rB, P);
                  end;
              end loop;

            else
              -- Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
              -- Build the mini LCP for this contact patch
              --
              -- vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
              --
              -- A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
              -- b = vn0 - velocityBias
              --
              -- The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
              -- implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
              -- vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
              -- solution that satisfies the problem is chosen.
              --
              -- In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
              -- that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
              --
              -- Substitute:
              --
              -- x = a + d
              --
              -- a := old total impulse
              -- x := new total impulse
              -- d := incremental impulse
              --
              -- For the current iteration we extend the formula for the incremental impulse
              -- to compute the new total impulse:
              --
              -- vn = A * d + b
              --    = A * (x - a) + b
              --    = A * x + b - A * a
              --    = A * x + b'
              -- b' = b - A * a;
               declare
                  cp1 : b2VelocityConstraintPoint renames vc.points (0);
                  cp2 : b2VelocityConstraintPoint renames vc.points (1);

                  a : constant b2Vec2 := (cp1.normalImpulse,
                                 cp2.normalImpulse);
                  pragma assert (a.x >= 0.0 and a.y >= 0.0);

                  -- Relative velocity at contact.
                  dv1 : b2Vec2 :=   vB + b2Cross (wB, cp1.rB)
                                  - vA - b2Cross (wA, cp1.rA);

                  dv2 : b2Vec2 :=   vB + b2Cross (wB, cp2.rB)
                                  - vA - b2Cross (wA, cp2.rA);

                  -- Compute normal velocity.
                  vn1 : Real := b2Dot (dv1, normal);
                  vn2 : Real := b2Dot (dv2, normal);

                  b   : b2Vec2;
                  x   : b2Vec2;

                  k_errorTol : constant := 0.001;
               begin
                  b.x := vn1 - cp1.velocityBias;
                  b.y := vn2 - cp2.velocityBias;

                  -- Compute b'
                  b := b - b2Mul (vc.K, a);

                  -- B2_NOT_USED(k_errorTol);

                  loop
                     -- Case 1: vn = 0
                     --
                     -- 0 = A * x + b'
                     --
                     -- Solve for x:
                     --
                     -- x = - inv(A) * b'
                     --
                     x := - b2Mul (vc.normalMass, b);

                     if    x.x >= 0.0
                       and x.y >= 0.0
                     then
                        declare
                           -- Get the incremental impulse
                           d : constant b2Vec2 := x - a;

                           -- Apply incremental impulse
                           P1 : constant b2Vec2 := d.x * normal;
                           P2 : constant b2Vec2 := d.y * normal;
                        begin
                           vA := vA - mA * (P1 + P2);
                           wA := wA - iA * (b2Cross (cp1.rA, P1) + b2Cross (cp2.rA, P2));

                           vB := vB + mB * (P1 + P2);
                           wB := wB + iB * (b2Cross (cp1.rB, P1) + b2Cross (cp2.rB, P2));

                           -- Accumulate
                           cp1.normalImpulse := x.x;
                           cp2.normalImpulse := x.y;

                           if B2_DEBUG_SOLVER
                           then
                              -- Postconditions
                              dv1 :=   vB + b2Cross (wB, cp1.rB)
                                     - vA - b2Cross (wA, cp1.rA);
                              dv2 :=   vB + b2Cross (wB, cp2.rB)
                                     - vA - b2Cross (wA, cp2.rA);

                              -- Compute normal velocity
                              vn1 := b2Dot (dv1, normal);
                              vn2 := b2Dot (dv2, normal);

                              pragma assert (abs (vn1 - cp1.velocityBias)  <  k_errorTol);
                              pragma assert (abs (vn2 - cp2.velocityBias)  <  k_errorTol);
                           end if;

                           exit;
                        end;
                     end if;


                     -- Case 2: vn1 = 0 and x2 = 0
                     --
                     --   0 = a11 * x1 + a12 * 0 + b1'
                     -- vn2 = a21 * x1 + a22 * 0 + b2'
                     --
                     x.x := -cp1.normalMass * b.x;
                     x.y := 0.0;
                     vn1 := 0.0;
                     vn2 := vc.K.ex.y * x.x  +  b.y;

                     if    x.x >= 0.0
                       and vn2 >= 0.0
                     then
                        declare
                           -- Get the incremental impulse
                           d : constant b2Vec2 := x - a;

                           -- Apply incremental impulse
                           P1 : constant b2Vec2 := d.x * normal;
                           P2 : constant b2Vec2 := d.y * normal;
                        begin
                           vA := vA - mA * (P1 + P2);
                           wA := wA - iA * (b2Cross (cp1.rA, P1) + b2Cross (cp2.rA, P2));

                           vB := vB + mB * (P1 + P2);
                           wB := wB + iB * (b2Cross (cp1.rB, P1) + b2Cross (cp2.rB, P2));

                           -- Accumulate
                           cp1.normalImpulse := x.x;
                           cp2.normalImpulse := x.y;

                           if B2_DEBUG_SOLVER
                           then
                              -- Postconditions
                              dv1 :=   vB + b2Cross (wB, cp1.rB)
                                     - vA - b2Cross (wA, cp1.rA);

                              -- Compute normal velocity
                              vn1 := b2Dot (dv1, normal);

                              pragma assert (  abs (vn1 - cp1.velocityBias)
                                             < k_errorTol);
                           end if;

                           exit;
                        end;
                     end if;


                     -- Case 3: vn2 = 0 and x1 = 0
                     --
                     -- vn1 = a11 * 0 + a12 * x2 + b1'
                     --   0 = a21 * 0 + a22 * x2 + b2'
                     --
                     x.x := 0.0;
                     x.y := -cp2.normalMass * b.y;
                     vn1 := vc.K.ey.x * x.y + b.x;
                     vn2 := 0.0;

                     if    x.y >= 0.0
                       and vn1 >= 0.0
                     then
                        declare
                           -- Resubstitute for the incremental impulse.
                           d : constant b2Vec2 := x - a;

                           -- Apply incremental impulse.
                           P1 : constant b2Vec2 := d.x * normal;
                           P2 : constant b2Vec2 := d.y * normal;
                        begin
                           vA := vA - mA * (P1 + P2);
                           wA := wA - iA * (b2Cross (cp1.rA, P1) + b2Cross (cp2.rA, P2));

                           vB := vB + mB * (P1 + P2);
                           wB := wB + iB * (b2Cross (cp1.rB, P1) + b2Cross (cp2.rB, P2));

                           -- Accumulate.
                           cp1.normalImpulse := x.x;
                           cp2.normalImpulse := x.y;

                           if B2_DEBUG_SOLVER
                           then
                              -- Postconditions.
                              dv2 :=   vB + b2Cross (wB, cp2.rB)
                                     - vA - b2Cross (wA, cp2.rA);

                              -- Compute normal velocity.
                              vn2 := b2Dot (dv2, normal);

                              pragma assert (  abs (vn2 - cp2.velocityBias)
                                             < k_errorTol);
                           end if;

                           exit;
                        end;
                     end if;


                     -- Case 4: x1 = 0 and x2 = 0
                     --
                     -- vn1 = b1
                     -- vn2 = b2;
                     x.x := 0.0;
                     x.y := 0.0;
                     vn1 := b.x;
                     vn2 := b.y;

                     if    vn1 >= 0.0
                       and vn2 >= 0.0
                     then
                        declare
                           -- Resubstitute for the incremental impulse.
                           d : constant b2Vec2 := x - a;

                           -- Apply incremental impulse.
                           P1 : constant b2Vec2 := d.x * normal;
                           P2 : constant b2Vec2 := d.y * normal;
                        begin
                           vA := vA - mA * (P1 + P2);
                           wA := wA - iA * (b2Cross (cp1.rA, P1) + b2Cross (cp2.rA, P2));

                           vB := vB + mB * (P1 + P2);
                           wB := wB + iB * (b2Cross (cp1.rB, P1) + b2Cross (cp2.rB, P2));

                           -- Accumulate
                           cp1.normalImpulse := x.x;
                           cp2.normalImpulse := x.y;

                           exit;
                        end;
                     end if;

                     -- No solution, give up. This is hit sometimes, but it doesn't seem to matter.
                     exit;
                  end loop;
               end;
            end if;

            Self.m_velocities (indexA).v := vA;
            Self.m_velocities (indexA).w := wA;
            Self.m_velocities (indexB).v := vB;
            Self.m_velocities (indexB).w := wB;
         end;
      end loop;

   end solveVelocityConstraints;




   --  void b2ContactSolver::StoreImpulses()
   --  {
   --    for (int32 i = 0; i < m_count; ++i)
   --    {
   --       b2ContactVelocityConstraint* vc = m_velocityConstraints + i;
   --       b2Manifold* manifold = m_contacts[vc->contactIndex]->GetManifold();
   --
   --       for (int32 j = 0; j < vc->pointCount; ++j)
   --       {
   --          manifold->points[j].normalImpulse = vc->points[j].normalImpulse;
   --          manifold->points[j].tangentImpulse = vc->points[j].tangentImpulse;
   --       }
   --    }
   --  }
   --

   procedure storeImpulses (Self : in out b2ContactSolver)
   is
   begin
      for i in 0 .. Self.m_count - 1
      loop
         declare
            vc       : b2ContactVelocityConstraint renames Self.m_velocityConstraints (i);
            manifold : constant access b2Manifold       := Self.m_contacts (vc.contactIndex).getManifold;
         begin
           for j in 0 .. vc.pointCount - 1
           loop
              manifold.points (j).normalImpulse  := vc.points (j). normalImpulse;
              manifold.points (j).tangentImpulse := vc.points (j).tangentImpulse;
           end loop;
         end;
      end loop;
   end storeImpulses;






   --  struct b2PositionSolverManifold
   --  {
   --    void Initialize(b2ContactPositionConstraint* pc, const b2Transform& xfA, const b2Transform& xfB, int32 index)
   --    {
   --       b2Assert(pc->pointCount > 0);
   --
   --       switch (pc->type)
   --       {
   --       case b2Manifold::e_circles:
   --          {
   --             b2Vec2 pointA = b2Mul(xfA, pc->localPoint);
   --             b2Vec2 pointB = b2Mul(xfB, pc->localPoints[0]);
   --             normal = pointB - pointA;
   --             normal.Normalize();
   --             point = 0.5f * (pointA + pointB);
   --             separation = b2Dot(pointB - pointA, normal) - pc->radiusA - pc->radiusB;
   --          }
   --          break;
   --
   --       case b2Manifold::e_faceA:
   --          {
   --             normal = b2Mul(xfA.q, pc->localNormal);
   --             b2Vec2 planePoint = b2Mul(xfA, pc->localPoint);
   --
   --             b2Vec2 clipPoint = b2Mul(xfB, pc->localPoints[index]);
   --             separation = b2Dot(clipPoint - planePoint, normal) - pc->radiusA - pc->radiusB;
   --             point = clipPoint;
   --          }
   --          break;
   --
   --       case b2Manifold::e_faceB:
   --          {
   --             normal = b2Mul(xfB.q, pc->localNormal);
   --             b2Vec2 planePoint = b2Mul(xfB, pc->localPoint);
   --
   --             b2Vec2 clipPoint = b2Mul(xfA, pc->localPoints[index]);
   --             separation = b2Dot(clipPoint - planePoint, normal) - pc->radiusA - pc->radiusB;
   --             point = clipPoint;
   --
   --             // Ensure normal points from A to B
   --             normal = -normal;
   --          }
   --          break;
   --       }
   --    }
   --
   --    b2Vec2 normal;
   --    b2Vec2 point;
   --    float separation;
   --  };
   --

   type b2PositionSolverManifold is
      record
         normal     : b2Vec2;
         point      : b2Vec2;
         separation : Real;
      end record;


   procedure initialize (Self : out b2PositionSolverManifold;   pc    : in b2ContactPositionConstraint;
                                                                xfA   : in b2Transform;
                                                                xfB   : in b2Transform;
                                                                index : in Natural)
   is
      pragma assert (pc.pointCount > 0);
   begin
      case pc.Kind is

         when e_circles =>
            declare
               pointA : constant b2Vec2 := b2Mul (xfA, pc.localPoint);
               pointB : constant b2Vec2 := b2Mul (xfB, pc.localPoints (0));
            begin
               Self.normal := pointB - pointA;
               normalize (Self.normal);

               Self.point      := 0.5 * (pointA + pointB);
               Self.separation :=   b2Dot (pointB - pointA, Self.normal)
                                  - pc.radiusA
                                  - pc.radiusB;
           end;

        when e_faceA =>
            Self.normal := b2Mul (xfA.q, pc.localNormal);

            declare
               planePoint : constant b2Vec2 := b2Mul (xfA, pc.localPoint);
               clipPoint  : constant b2Vec2 := b2Mul (xfB, pc.localPoints (index));
            begin
               Self.separation :=   b2Dot (clipPoint - planePoint, Self.normal)
                                  - pc.radiusA
                                  - pc.radiusB;
               Self.point      := clipPoint;
            end;

        when e_faceB =>
            Self.normal := b2Mul (xfB.q, pc.localNormal);

            declare
               planePoint : constant b2Vec2 := b2Mul (xfB, pc.localPoint);
               clipPoint  : constant b2Vec2 := b2Mul (xfA, pc.localPoints (index));
            begin
               Self.separation :=   b2Dot(clipPoint - planePoint, Self.normal)
                                  - pc.radiusA
                                  - pc.radiusB;
              Self.point       := clipPoint;

              -- Ensure normal points from A to B.
              Self.normal := -Self.normal;
            end;
        end case;

   end initialize;




   --  // Sequential solver.
   --  bool b2ContactSolver::SolvePositionConstraints()
   --  {
   --    float minSeparation = 0.0f;
   --
   --    for (int32 i = 0; i < m_count; ++i)
   --    {
   --       b2ContactPositionConstraint* pc = m_positionConstraints + i;
   --
   --       int32 indexA = pc->indexA;
   --       int32 indexB = pc->indexB;
   --       b2Vec2 localCenterA = pc->localCenterA;
   --       float mA = pc->invMassA;
   --       float iA = pc->invIA;
   --       b2Vec2 localCenterB = pc->localCenterB;
   --       float mB = pc->invMassB;
   --       float iB = pc->invIB;
   --       int32 pointCount = pc->pointCount;
   --
   --       b2Vec2 cA = m_positions[indexA].c;
   --       float aA = m_positions[indexA].a;
   --
   --       b2Vec2 cB = m_positions[indexB].c;
   --       float aB = m_positions[indexB].a;
   --
   --       // Solve normal constraints
   --       for (int32 j = 0; j < pointCount; ++j)
   --       {
   --          b2Transform xfA, xfB;
   --          xfA.q.Set(aA);
   --          xfB.q.Set(aB);
   --          xfA.p = cA - b2Mul(xfA.q, localCenterA);
   --          xfB.p = cB - b2Mul(xfB.q, localCenterB);
   --
   --          b2PositionSolverManifold psm;
   --          psm.Initialize(pc, xfA, xfB, j);
   --          b2Vec2 normal = psm.normal;
   --
   --          b2Vec2 point = psm.point;
   --          float separation = psm.separation;
   --
   --          b2Vec2 rA = point - cA;
   --          b2Vec2 rB = point - cB;
   --
   --          // Track max constraint error.
   --          minSeparation = b2Min(minSeparation, separation);
   --
   --          // Prevent large corrections and allow slop.
   --          float C = b2Clamp(b2_baumgarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0f);
   --
   --          // Compute the effective mass.
   --          float rnA = b2Cross(rA, normal);
   --          float rnB = b2Cross(rB, normal);
   --          float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
   --
   --          // Compute normal impulse
   --          float impulse = K > 0.0f ? - C / K : 0.0f;
   --
   --          b2Vec2 P = impulse * normal;
   --
   --          cA -= mA * P;
   --          aA -= iA * b2Cross(rA, P);
   --
   --          cB += mB * P;
   --          aB += iB * b2Cross(rB, P);
   --       }
   --
   --       m_positions[indexA].c = cA;
   --       m_positions[indexA].a = aA;
   --
   --       m_positions[indexB].c = cB;
   --       m_positions[indexB].a = aB;
   --    }
   --
   --    // We can't expect minSpeparation >= -b2_linearSlop because we don't
   --    // push the separation above -b2_linearSlop.
   --    return minSeparation >= -3.0f * b2_linearSlop;
   --  }
   --

   function solvePositionConstraints (Self : in out b2ContactSolver) return Boolean
   is
      minSeparation : Real := 0.0;

   begin
      for i in 0 .. Self.m_count - 1
      loop
       declare
            pc           : b2ContactPositionConstraint renames Self.m_positionConstraints (i);

            indexA       : constant Natural := pc.indexA;
            indexB       : constant Natural := pc.indexB;

            localCenterA : constant b2Vec2  := pc.localCenterA;
            mA           : constant Real    := pc.invMassA;
            iA           : constant Real    := pc.invIA;

            localCenterB : constant b2Vec2  := pc.localCenterB;
            mB           : constant Real    := pc.invMassB;
            iB           : constant Real    := pc.invIB;

            pointCount   : constant Natural := pc.pointCount;

            cA : b2Vec2  := Self.m_positions (indexA).c;
            aA : Real    := Self.m_positions (indexA).a;

            cB : b2Vec2  := Self.m_positions (indexB).c;
            aB : Real    := Self.m_positions (indexB).a;
         begin
            -- Solve normal constraints.
            --
            for j in 0 .. pointCount - 1
            loop
               declare
                  xfA, xfB : b2Transform;
                  psm      : b2PositionSolverManifold;
               begin
                  set (xfA.q, aA);
                  set (xfB.q, aB);

                  xfA.p := cA - b2Mul (xfA.q, localCenterA);
                  xfB.p := cB - b2Mul (xfB.q, localCenterB);

                  initialize (psm, pc, xfA, xfB, j);

                  declare
                     normal     : constant b2Vec2 := psm.normal;

                     point      : constant b2Vec2 := psm.point;
                     separation : constant Real   := psm.separation;

                     rA         : constant b2Vec2 := point - cA;
                     rB         : constant b2Vec2 := point - cB;

                     C          : Real;
                  begin
                     -- Track max constraint error.
                     minSeparation := Real'min (minSeparation, separation);

                     -- Prevent large corrections and allow slop.
                     C := b2clamp ( b2_baumgarte * (separation + b2_linearSlop),
                                   -b2_maxLinearCorrection,
                                    0.0);

                     declare
                        -- Compute the effective mass.
                        rnA : constant Real := b2Cross (rA, normal);
                        rnB : constant Real := b2Cross (rB, normal);
                        K   : constant Real :=   mA + mB
                                               + iA * rnA * rnA
                                               + iB * rnB * rnB;

                        -- Compute normal impulse
                        impulse : constant Real := (if K > 0.0 then -C / K
                                                               else   0.0);
                        P       : constant b2Vec2 := impulse * normal;
                     begin
                        cA := cA - mA * P;
                        aA := aA - iA * b2Cross (rA, P);

                        cB := cB + mB * P;
                        aB := aB + iB * b2Cross (rB, P);
                     end;
                  end;
               end;
            end loop;

            Self.m_positions (indexA).c := cA;
            Self.m_positions (indexA).a := aA;

            Self.m_positions (indexB).c := cB;
            Self.m_positions (indexB).a := aB;
         end;
      end loop;

      -- We can't expect minSpeparation >= -b2_linearSlop because we don't
      -- push the separation above -b2_linearSlop.
      --
      return minSeparation >= -3.0 * b2_linearSlop;
   end solvePositionConstraints;




   --  // Sequential position solver for position constraints.
   --  bool b2ContactSolver::SolveTOIPositionConstraints(int32 toiIndexA, int32 toiIndexB)
   --  {
   --    float minSeparation = 0.0f;
   --
   --    for (int32 i = 0; i < m_count; ++i)
   --    {
   --       b2ContactPositionConstraint* pc = m_positionConstraints + i;
   --
   --       int32 indexA = pc->indexA;
   --       int32 indexB = pc->indexB;
   --       b2Vec2 localCenterA = pc->localCenterA;
   --       b2Vec2 localCenterB = pc->localCenterB;
   --       int32 pointCount = pc->pointCount;
   --
   --       float mA = 0.0f;
   --       float iA = 0.0f;
   --       if (indexA == toiIndexA || indexA == toiIndexB)
   --       {
   --          mA = pc->invMassA;
   --          iA = pc->invIA;
   --       }
   --
   --       float mB = 0.0f;
   --       float iB = 0.;
   --       if (indexB == toiIndexA || indexB == toiIndexB)
   --       {
   --          mB = pc->invMassB;
   --          iB = pc->invIB;
   --       }
   --
   --       b2Vec2 cA = m_positions[indexA].c;
   --       float aA = m_positions[indexA].a;
   --
   --       b2Vec2 cB = m_positions[indexB].c;
   --       float aB = m_positions[indexB].a;
   --
   --       // Solve normal constraints
   --       for (int32 j = 0; j < pointCount; ++j)
   --       {
   --          b2Transform xfA, xfB;
   --          xfA.q.Set(aA);
   --          xfB.q.Set(aB);
   --          xfA.p = cA - b2Mul(xfA.q, localCenterA);
   --          xfB.p = cB - b2Mul(xfB.q, localCenterB);
   --
   --          b2PositionSolverManifold psm;
   --          psm.Initialize(pc, xfA, xfB, j);
   --          b2Vec2 normal = psm.normal;
   --
   --          b2Vec2 point = psm.point;
   --          float separation = psm.separation;
   --
   --          b2Vec2 rA = point - cA;
   --          b2Vec2 rB = point - cB;
   --
   --          // Track max constraint error.
   --          minSeparation = b2Min(minSeparation, separation);
   --
   --          // Prevent large corrections and allow slop.
   --          float C = b2Clamp(b2_toiBaumgarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0f);
   --
   --          // Compute the effective mass.
   --          float rnA = b2Cross(rA, normal);
   --          float rnB = b2Cross(rB, normal);
   --          float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
   --
   --          // Compute normal impulse
   --          float impulse = K > 0.0f ? - C / K : 0.0f;
   --
   --          b2Vec2 P = impulse * normal;
   --
   --          cA -= mA * P;
   --          aA -= iA * b2Cross(rA, P);
   --
   --          cB += mB * P;
   --          aB += iB * b2Cross(rB, P);
   --       }
   --
   --       m_positions[indexA].c = cA;
   --       m_positions[indexA].a = aA;
   --
   --       m_positions[indexB].c = cB;
   --       m_positions[indexB].a = aB;
   --    }
   --
   --    // We can't expect minSpeparation >= -b2_linearSlop because we don't
   --    // push the separation above -b2_linearSlop.
   --    return minSeparation >= -1.5f * b2_linearSlop;
   --  }

   function  SolveTOIPositionConstraints (Self : in out b2ContactSolver;   toiIndexA,
                                                                           toiIndexB : in Natural) return Boolean
   is
      minSeparation : Real := 0.0;

   begin
      for i in 0 .. Self.m_count - 1
      loop
         declare
            pc : b2ContactPositionConstraint renames Self.m_positionConstraints (i);

            indexA       : constant Natural := pc.indexA;
            indexB       : constant Natural := pc.indexB;

            localCenterA : constant b2Vec2  := pc.localCenterA;
            localCenterB : constant b2Vec2  := pc.localCenterB;

            pointCount   : constant Natural := pc.pointCount;

            mA           :          Real    := 0.0;
            iA           :          Real    := 0.0;

            mB, iB       :          Real;
         begin
            if   indexA = toiIndexA
              or indexA = toiIndexB
            then
               mA := pc.invMassA;
               iA := pc.invIA;
            end if;

            mB := 0.0;
            iB := 0.0;

            if   indexB = toiIndexA
              or indexB = toiIndexB
            then
               mB := pc.invMassB;
               iB := pc.invIB;
            end if;

            declare
               cA : b2Vec2 := Self.m_positions (indexA).c;
               aA : Real   := Self.m_positions (indexA).a;

               cB : b2Vec2 := Self.m_positions (indexB).c;
               aB : Real   := Self.m_positions (indexB).a;
            begin
               -- Solve normal constraints
               --
               for j in 0 .. pointCount - 1
               loop
                  declare
                     xfA, xfB : b2Transform;
                     psm      : b2PositionSolverManifold;
                  begin
                     set (xfA.q, aA);
                     set (xfB.q, aB);

                     xfA.p := cA - b2Mul (xfA.q, localCenterA);
                     xfB.p := cB - b2Mul (xfB.q, localCenterB);

                     initialize (psm, pc, xfA, xfB, j);

                     declare
                        normal     : constant b2Vec2 := psm.normal;

                        point      : constant b2Vec2 := psm.point;
                        separation : constant Real   := psm.separation;

                        rA         : constant b2Vec2 := point - cA;
                        rB         : constant b2Vec2 := point - cB;

                        C          :          Real;
                     begin
                        -- Track max constraint error.
                        minSeparation := Real'min (minSeparation, separation);

                        -- Prevent large corrections and allow slop.
                        C := b2Clamp ( b2_toiBaumgarte * (separation + b2_linearSlop),
                                      -b2_maxLinearCorrection,
                                       0.0);

                        declare
                           -- Compute the effective mass.
                           rnA : constant Real := b2Cross(rA, normal);
                           rnB : constant Real := b2Cross(rB, normal);
                           K   : constant Real :=   mA + mB
                                                  + iA * rnA * rnA
                                                  + iB * rnB * rnB;

                           -- Compute normal impulse
                           impulse : constant Real := (if K > 0.0 then - C / K
                                                         else    0.0);

                           P       : constant b2Vec2 := impulse * normal;
                        begin
                           cA := cA - mA * P;
                           aA := aA - iA * b2Cross (rA, P);

                           cB := cB + mB * P;
                           aB := aB + iB * b2Cross (rB, P);
                        end;
                     end;
                  end;
               end loop;

               Self.m_positions (indexA).c := cA;
               Self.m_positions (indexA).a := aA;

               Self.m_positions (indexB).c := cB;
               Self.m_positions (indexB).a := aB;
            end;
         end;
      end loop;


      -- We can't expect minSpeparation >= -b2_linearSlop because we don't
      -- push the separation above -b2_linearSlop.
      --
      return minSeparation >= -1.5 * b2_linearSlop;

   end SolveTOIPositionConstraints;


end box2d.b2_contact_Solver;
