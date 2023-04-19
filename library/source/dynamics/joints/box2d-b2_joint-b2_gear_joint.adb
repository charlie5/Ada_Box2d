with
     box2d.b2_Joint.b2_revolute_Joint,
     box2d.b2_Joint.b2_prismatic_Joint,
     box2d.b2_time_Step,
     box2d.b2_Common;


package body box2d.b2_Joint.b2_gear_Joint
--
--   Gear Joint:
--   C0 = (coordinate1 + ratio * coordinate2)_initial
--   C = (coordinate1 + ratio * coordinate2) - C0 = 0
--   J = [J1 ratio * J2]
--   K = J * invM * JT
--     = J1 * invM1 * J1T + ratio * ratio * J2 * invM2 * J2T
--
--   Revolute:
--   coordinate = rotation
--   Cdot = angularVelocity
--   J = [0 0 1]
--   K = J * invM * JT = invI
--
--   Prismatic:
--   coordinate = dot(p - pg, ug)
--   Cdot = dot(v + cross(w, r), ug)
--   J = [ug cross(r, ug)]
--   K = J * invM * JT = invMass + invI * cross(r, ug)^2
--
is
   -----------------
   -- b2GearJointDef
   --

   --    b2GearJointDef()
   --    {
   --       type = e_gearJoint;
   --       joint1 = nullptr;
   --       joint2 = nullptr;
   --       ratio = 1.0f;
   --    }
   --

   function to_b2GearJointDef return b2GearJointDef
   is
      Self : b2GearJointDef;
   begin
      Self.Kind  := e_gearJoint;
      Self.Ratio := 1.0;

      return Self;
   end to_b2GearJointDef;




   --  procedure initialize (Self : out b2GearJointDef;   Joint1, Joint2 : in b2Joint_ptr;
   --                                                     Ratio          : in Real)
   --  is
   --  begin
   --     return;
   --  end initialize;




   ---------------
   --- b2GearJoint
   --


   --  b2Vec2 b2GearJoint::GetAnchorA() const
   --  {
   --    return m_bodyA->GetWorldPoint(m_localAnchorA);
   --  }
   --

   overriding
   function getAnchorA (Self : in b2GearJoint) return b2Vec2
   is
   begin
      return Self.m_bodyA.getWorldPoint (Self.m_localAnchorA);
   end getAnchorA;





   --  b2Vec2 b2GearJoint::GetAnchorB() const
   --  {
   --    return m_bodyB->GetWorldPoint(m_localAnchorB);
   --  }
   --

   overriding
   function getAnchorB (Self : in b2GearJoint) return b2Vec2
   is
   begin
      return Self.m_bodyB.getWorldPoint (Self.m_localAnchorB);
   end getAnchorB;





   --  b2Vec2 b2GearJoint::GetReactionForce(float inv_dt) const
   --  {
   --    b2Vec2 P = m_impulse * m_JvAC;
   --    return inv_dt * P;
   --  }
   --

   overriding
   function getReactionForce (Self : in b2GearJoint;   inv_dt : in Real) return b2Vec2
   is
      P : constant b2Vec2 := Self.m_impulse * Self.m_JvAC;
   begin
      return inv_dt * P;
   end getReactionForce;





   --  float b2GearJoint::GetReactionTorque(float inv_dt) const
   --  {
   --    float L = m_impulse * m_JwA;
   --    return inv_dt * L;
   --  }
   --

   overriding
   function getReactionTorque (Self : in b2GearJoint;   inv_dt : in Real) return Real
   is
      L : constant Real := Self.m_impulse * Self.m_JwA;
   begin
      return inv_dt * L;
   end getReactionTorque;






   --  void b2GearJoint::SetRatio(float ratio)
   --  {
   --    b2Assert(b2IsValid(ratio));
   --    m_ratio = ratio;
   --  }
   --

   procedure setRatio (Self : in out b2GearJoint;   Ratio : in Real)
   is
       pragma assert (b2IsValid (Ratio));
   begin
      Self.m_Ratio := Ratio;
   end setRatio;






   --  float b2GearJoint::GetRatio() const
   --  {
   --    return m_ratio;
   --  }
   --

   function getRatio (Self : in b2GearJoint) return Real
   is
   begin
      return Self.m_Ratio;
   end getRatio;





   --    Get the first joint.
   --
   --    b2Joint* GetJoint1() { return m_joint1; }
   --

   function getJoint1 (Self : in b2GearJoint) return b2Joint_ptr
   is
   begin
      return Self.m_joint1;
   end getJoint1;



   --    Get the second joint.
   --
   --    b2Joint* GetJoint2() { return m_joint2; }
   --

   function getJoint2 (Self : in b2GearJoint) return b2Joint_ptr
   is
   begin
      return Self.m_joint2;
   end getJoint2;




   --  void b2GearJoint::Dump()
   --  {
   --    int32 indexA = m_bodyA->m_islandIndex;
   --    int32 indexB = m_bodyB->m_islandIndex;
   --
   --    int32 index1 = m_joint1->m_index;
   --    int32 index2 = m_joint2->m_index;
   --
   --    b2Dump("  b2GearJointDef jd;\n");
   --    b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
   --    b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
   --    b2Dump("  jd.collideConnected = bool(%d);\n", m_collideConnected);
   --    b2Dump("  jd.joint1 = joints[%d];\n", index1);
   --    b2Dump("  jd.joint2 = joints[%d];\n", index2);
   --    b2Dump("  jd.ratio = %.9g;\n", m_ratio);
   --    b2Dump("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
   --  }

   --

   overriding
   procedure dump (Self : in b2GearJoint)
   is
      use b2_Common;

      indexA : constant Natural := Self.m_bodyA.m_islandIndex;
      indexB : constant Natural := Self.m_bodyB.m_islandIndex;

      index1 : constant Natural := Self.m_joint1.m_index;
      index2 : constant Natural := Self.m_joint2.m_index;
   begin
      b2Dump ("  b2GearJointDef jd");
      b2Dump ("  jd.bodyA            = bodies[%d]"               & indexA                 'Image);
      b2Dump ("  jd.bodyB            = bodies[%d]"               & indexB                 'Image);
      b2Dump ("  jd.collideConnected = bool(%d)"                 & Self.m_collideConnected'Image);
      b2Dump ("  jd.joint1           = joints[%d]"               & index1                 'Image);
      b2Dump ("  jd.joint2           = joints[%d]"               & index2                 'Image);
      b2Dump ("  jd.ratio            = %.9g"                     & Self.m_ratio           'Image);
      b2Dump ("  joints[%d]          = m_world.createJoint(&jd)" & Self.m_Index           'Image);
   end dump;



   --  overriding
   --  procedure draw (Self : in b2GearJoint;   Draw : access b2Draw'Class)
   --  is
   --  begin
   --     return;
   --  end draw;




   -------------
   -- protected:
   --

   package body Forge
   is

      --  b2GearJoint::b2GearJoint(const b2GearJointDef* def)
      --  : b2Joint(def)
      --  {
      --    m_joint1 = def->joint1;
      --    m_joint2 = def->joint2;
      --
      --    m_typeA = m_joint1->GetType();
      --    m_typeB = m_joint2->GetType();
      --
      --    b2Assert(m_typeA == e_revoluteJoint || m_typeA == e_prismaticJoint);
      --    b2Assert(m_typeB == e_revoluteJoint || m_typeB == e_prismaticJoint);
      --
      --    float coordinateA, coordinateB;
      --
      --    // TODO_ERIN there might be some problem with the joint edges in b2Joint.
      --
      --    m_bodyC = m_joint1->GetBodyA();
      --    m_bodyA = m_joint1->GetBodyB();
      --
      --    // Body B on joint1 must be dynamic
      --    b2Assert(m_bodyA->m_type == b2_dynamicBody);
      --
      --    // Get geometry of joint1
      --    b2Transform xfA = m_bodyA->m_xf;
      --    float aA = m_bodyA->m_sweep.a;
      --    b2Transform xfC = m_bodyC->m_xf;
      --    float aC = m_bodyC->m_sweep.a;
      --
      --    if (m_typeA == e_revoluteJoint)
      --    {
      --       b2RevoluteJoint* revolute = (b2RevoluteJoint*)def->joint1;
      --       m_localAnchorC = revolute->m_localAnchorA;
      --       m_localAnchorA = revolute->m_localAnchorB;
      --       m_referenceAngleA = revolute->m_referenceAngle;
      --       m_localAxisC.SetZero();
      --
      --       coordinateA = aA - aC - m_referenceAngleA;
      --    }
      --    else
      --    {
      --       b2PrismaticJoint* prismatic = (b2PrismaticJoint*)def->joint1;
      --       m_localAnchorC = prismatic->m_localAnchorA;
      --       m_localAnchorA = prismatic->m_localAnchorB;
      --       m_referenceAngleA = prismatic->m_referenceAngle;
      --       m_localAxisC = prismatic->m_localXAxisA;
      --
      --       b2Vec2 pC = m_localAnchorC;
      --       b2Vec2 pA = b2MulT(xfC.q, b2Mul(xfA.q, m_localAnchorA) + (xfA.p - xfC.p));
      --       coordinateA = b2Dot(pA - pC, m_localAxisC);
      --    }
      --
      --    m_bodyD = m_joint2->GetBodyA();
      --    m_bodyB = m_joint2->GetBodyB();
      --
      --    // Body B on joint2 must be dynamic
      --    b2Assert(m_bodyB->m_type == b2_dynamicBody);
      --
      --    // Get geometry of joint2
      --    b2Transform xfB = m_bodyB->m_xf;
      --    float aB = m_bodyB->m_sweep.a;
      --    b2Transform xfD = m_bodyD->m_xf;
      --    float aD = m_bodyD->m_sweep.a;
      --
      --    if (m_typeB == e_revoluteJoint)
      --    {
      --       b2RevoluteJoint* revolute = (b2RevoluteJoint*)def->joint2;
      --       m_localAnchorD = revolute->m_localAnchorA;
      --       m_localAnchorB = revolute->m_localAnchorB;
      --       m_referenceAngleB = revolute->m_referenceAngle;
      --       m_localAxisD.SetZero();
      --
      --       coordinateB = aB - aD - m_referenceAngleB;
      --    }
      --    else
      --    {
      --       b2PrismaticJoint* prismatic = (b2PrismaticJoint*)def->joint2;
      --       m_localAnchorD = prismatic->m_localAnchorA;
      --       m_localAnchorB = prismatic->m_localAnchorB;
      --       m_referenceAngleB = prismatic->m_referenceAngle;
      --       m_localAxisD = prismatic->m_localXAxisA;
      --
      --       b2Vec2 pD = m_localAnchorD;
      --       b2Vec2 pB = b2MulT(xfD.q, b2Mul(xfB.q, m_localAnchorB) + (xfB.p - xfD.p));
      --       coordinateB = b2Dot(pB - pD, m_localAxisD);
      --    }
      --
      --    m_ratio = def->ratio;
      --
      --    m_constant = coordinateA + m_ratio * coordinateB;
      --
      --    m_impulse = 0.0f;
      --  }
      --

      function to_b2GearJoint (Def : in b2GearJointDef) return b2GearJoint
      is
         use b2_Body;

         Self : b2GearJoint;

         coordinateA,
         coordinateB : Real;
      begin
         Self.m_joint1 := def.Joint1;
         Self.m_joint2 := def.Joint2;

         Self.m_typeA  := Self.m_joint1.getType;
         Self.m_typeB  := Self.m_joint2.getType;

         pragma assert (Self.m_typeA = e_revoluteJoint or Self.m_typeA = e_prismaticJoint);
         pragma assert (Self.m_typeB = e_revoluteJoint or Self.m_typeB = e_prismaticJoint);

         -- TODO_ERIN there might be some problem with the joint edges in b2Joint.
         --
         Self.m_bodyC := Self.m_joint1.getBodyA;
         Self.m_bodyA := Self.m_joint1.getBodyB;

         -- Body B on joint1 must be dynamic
         --
         pragma assert (Self.m_bodyA.getType = b2_dynamicBody);

         -- Get geometry of joint1.
         --
         declare
            xfA : constant b2Transform := Self.m_bodyA.m_xf.all;
            xfC : constant b2Transform := Self.m_bodyC.m_xf.all;
            aA  : constant Real        := Self.m_bodyA.m_sweep.a;
            aC  : constant Real        := Self.m_bodyC.m_sweep.a;
         begin
            if Self.m_typeA = e_revoluteJoint
            then
               declare
                  use b2_revolute_Joint;
                  revolute : constant b2RevoluteJoint_ptr := b2RevoluteJoint_ptr (def.joint1);
               begin
                  Self.m_localAnchorC    := revolute.getLocalAnchorA;
                  Self.m_localAnchorA    := revolute.getLocalAnchorB;
                  Self.m_referenceAngleA := revolute.getReferenceAngle;
                  Self.m_localAxisC      := (0.0, 0.0);

                  coordinateA := aA - aC - Self.m_referenceAngleA;
               end;
            else
               declare
                  use b2_prismatic_Joint;
                  prismatic : constant b2PrismaticJoint_ptr := b2PrismaticJoint_ptr (def.joint1);
                  pA, pC    : b2Vec2;
               begin
                  Self.m_localAnchorC    := prismatic.getLocalAnchorA;
                  Self.m_localAnchorA    := prismatic.getLocalAnchorB;
                  Self.m_referenceAngleA := prismatic.getReferenceAngle;
                  Self.m_localAxisC      := prismatic.getLocalAxisA;

                  pC := Self.m_localAnchorC;
                  pA := b2MulT (xfC.q,
                                  b2Mul (xfA.q, Self.m_localAnchorA)
                                + (xfA.p - xfC.p));

                  coordinateA := b2Dot (pA - pC,
                                        Self.m_localAxisC);
               end;
            end if;
         end;

         Self.m_bodyD := Self.m_joint2.getBodyA;
         Self.m_bodyB := Self.m_joint2.getBodyB;

         -- Body B on joint2 must be dynamic
         --
         pragma assert (Self.m_bodyB.getType = b2_dynamicBody);

         -- Get geometry of joint2
         --
         declare
            xfB : constant b2Transform := Self.m_bodyB.m_xf.all;
            xfD : constant b2Transform := Self.m_bodyD.m_xf.all;
            aB  : constant Real        := Self.m_bodyB.m_sweep.a;
            aD  : constant Real        := Self.m_bodyD.m_sweep.a;
         begin
            if Self.m_typeB = e_revoluteJoint
            then
               declare
                  use b2_revolute_Joint;
                  revolute : constant b2RevoluteJoint_ptr := b2RevoluteJoint_ptr (def.joint2);
               begin
                  Self.m_localAnchorD    := revolute.getLocalAnchorA;
                  Self.m_localAnchorB    := revolute.getLocalAnchorB;
                  Self.m_referenceAngleB := revolute.getReferenceAngle;
                  Self.m_localAxisD      := (0.0, 0.0);

                  coordinateB := aB - aD - Self.m_referenceAngleB;
               end;

            else
               declare
                  use b2_prismatic_Joint;
                  prismatic : constant b2PrismaticJoint_ptr := b2PrismaticJoint_ptr (def.joint2);
                  pB, pD    : b2Vec2;
               begin
                  Self.m_localAnchorD    := prismatic.getLocalAnchorA;
                  Self.m_localAnchorB    := prismatic.getLocalAnchorB;
                  Self.m_referenceAngleB := prismatic.getReferenceAngle;
                  Self.m_localAxisD      := prismatic.getLocalAxisA;

                  pD := Self.m_localAnchorD;
                  pB := b2MulT (xfD.q,
                                  b2Mul (xfB.q,
                                         Self.m_localAnchorB)
                                + (xfB.p - xfD.p));

                  coordinateB := b2Dot (pB - pD,
                                        Self.m_localAxisD);
               end;
            end if;
         end;

            Self.m_ratio    := def.ratio;
            Self.m_constant := coordinateA + Self.m_ratio * coordinateB;
            Self.m_impulse  := 0.0;

         return Self;
      end to_b2GearJoint;

   end Forge;





   --  void b2GearJoint::InitVelocityConstraints(const b2SolverData& data)
   --  {
   --    m_indexA = m_bodyA->m_islandIndex;
   --    m_indexB = m_bodyB->m_islandIndex;
   --    m_indexC = m_bodyC->m_islandIndex;
   --    m_indexD = m_bodyD->m_islandIndex;
   --    m_lcA = m_bodyA->m_sweep.localCenter;
   --    m_lcB = m_bodyB->m_sweep.localCenter;
   --    m_lcC = m_bodyC->m_sweep.localCenter;
   --    m_lcD = m_bodyD->m_sweep.localCenter;
   --    m_mA = m_bodyA->m_invMass;
   --    m_mB = m_bodyB->m_invMass;
   --    m_mC = m_bodyC->m_invMass;
   --    m_mD = m_bodyD->m_invMass;
   --    m_iA = m_bodyA->m_invI;
   --    m_iB = m_bodyB->m_invI;
   --    m_iC = m_bodyC->m_invI;
   --    m_iD = m_bodyD->m_invI;
   --
   --    float aA = data.positions[m_indexA].a;
   --    b2Vec2 vA = data.velocities[m_indexA].v;
   --    float wA = data.velocities[m_indexA].w;
   --
   --    float aB = data.positions[m_indexB].a;
   --    b2Vec2 vB = data.velocities[m_indexB].v;
   --    float wB = data.velocities[m_indexB].w;
   --
   --    float aC = data.positions[m_indexC].a;
   --    b2Vec2 vC = data.velocities[m_indexC].v;
   --    float wC = data.velocities[m_indexC].w;
   --
   --    float aD = data.positions[m_indexD].a;
   --    b2Vec2 vD = data.velocities[m_indexD].v;
   --    float wD = data.velocities[m_indexD].w;
   --
   --    b2Rot qA(aA), qB(aB), qC(aC), qD(aD);
   --
   --    m_mass = 0.0f;
   --
   --    if (m_typeA == e_revoluteJoint)
   --    {
   --       m_JvAC.SetZero();
   --       m_JwA = 1.0f;
   --       m_JwC = 1.0f;
   --       m_mass += m_iA + m_iC;
   --    }
   --    else
   --    {
   --       b2Vec2 u = b2Mul(qC, m_localAxisC);
   --       b2Vec2 rC = b2Mul(qC, m_localAnchorC - m_lcC);
   --       b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_lcA);
   --       m_JvAC = u;
   --       m_JwC = b2Cross(rC, u);
   --       m_JwA = b2Cross(rA, u);
   --       m_mass += m_mC + m_mA + m_iC * m_JwC * m_JwC + m_iA * m_JwA * m_JwA;
   --    }
   --
   --    if (m_typeB == e_revoluteJoint)
   --    {
   --       m_JvBD.SetZero();
   --       m_JwB = m_ratio;
   --       m_JwD = m_ratio;
   --       m_mass += m_ratio * m_ratio * (m_iB + m_iD);
   --    }
   --    else
   --    {
   --       b2Vec2 u = b2Mul(qD, m_localAxisD);
   --       b2Vec2 rD = b2Mul(qD, m_localAnchorD - m_lcD);
   --       b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_lcB);
   --       m_JvBD = m_ratio * u;
   --       m_JwD = m_ratio * b2Cross(rD, u);
   --       m_JwB = m_ratio * b2Cross(rB, u);
   --       m_mass += m_ratio * m_ratio * (m_mD + m_mB) + m_iD * m_JwD * m_JwD + m_iB * m_JwB * m_JwB;
   --    }
   --
   --    // Compute effective mass.
   --    m_mass = m_mass > 0.0f ? 1.0f / m_mass : 0.0f;
   --
   --    if (data.step.warmStarting)
   --    {
   --       vA += (m_mA * m_impulse) * m_JvAC;
   --       wA += m_iA * m_impulse * m_JwA;
   --       vB += (m_mB * m_impulse) * m_JvBD;
   --       wB += m_iB * m_impulse * m_JwB;
   --       vC -= (m_mC * m_impulse) * m_JvAC;
   --       wC -= m_iC * m_impulse * m_JwC;
   --       vD -= (m_mD * m_impulse) * m_JvBD;
   --       wD -= m_iD * m_impulse * m_JwD;
   --    }
   --    else
   --    {
   --       m_impulse = 0.0f;
   --    }
   --
   --    data.velocities[m_indexA].v = vA;
   --    data.velocities[m_indexA].w = wA;
   --    data.velocities[m_indexB].v = vB;
   --    data.velocities[m_indexB].w = wB;
   --    data.velocities[m_indexC].v = vC;
   --    data.velocities[m_indexC].w = wC;
   --    data.velocities[m_indexD].v = vD;
   --    data.velocities[m_indexD].w = wD;
   --  }
   --

   overriding
   procedure initVelocityConstraints (Self : in out b2GearJoint;   Data : in b2SolverData)
   is
   begin
      Self.m_indexA := Self.m_bodyA.m_islandIndex;
      Self.m_indexB := Self.m_bodyB.m_islandIndex;
      Self.m_indexC := Self.m_bodyC.m_islandIndex;
      Self.m_indexD := Self.m_bodyD.m_islandIndex;

      Self.m_lcA    := Self.m_bodyA.m_sweep.localCenter;
      Self.m_lcB    := Self.m_bodyB.m_sweep.localCenter;
      Self.m_lcC    := Self.m_bodyC.m_sweep.localCenter;
      Self.m_lcD    := Self.m_bodyD.m_sweep.localCenter;

      Self.m_mA     := Self.m_bodyA.m_invMass;
      Self.m_mB     := Self.m_bodyB.m_invMass;
      Self.m_mC     := Self.m_bodyC.m_invMass;
      Self.m_mD     := Self.m_bodyD.m_invMass;
      Self.m_iA     := Self.m_bodyA.m_invI;
      Self.m_iB     := Self.m_bodyB.m_invI;
      Self.m_iC     := Self.m_bodyC.m_invI;
      Self.m_iD     := Self.m_bodyD.m_invI;

      declare
         aA : constant Real   := data.positions  (Self.m_indexA).a;
         vA :          b2Vec2 := data.velocities (Self.m_indexA).v;
         wA :          Real   := data.velocities (Self.m_indexA).w;

         aB : constant Real   := data.positions  (Self.m_indexB).a;
         vB :          b2Vec2 := data.velocities (Self.m_indexB).v;
         wB :          Real   := data.velocities (Self.m_indexB).w;

         aC : constant Real   := data.positions  (Self.m_indexC).a;
         vC :          b2Vec2 := data.velocities (Self.m_indexC).v;
         wC :          Real   := data.velocities (Self.m_indexC).w;

         aD : constant Real   := data.positions  (Self.m_indexD).a;
         vD :          b2Vec2 := data.velocities (Self.m_indexD).v;
         wD :          Real   := data.velocities (Self.m_indexD).w;

         qA : constant b2Rot  := to_b2Rot (aA);
         qB : constant b2Rot  := to_b2Rot (aB);
         qC : constant b2Rot  := to_b2Rot (aC);
         qD : constant b2Rot  := to_b2Rot (aD);
      begin
        Self.m_mass := 0.0;


        if Self.m_typeA = e_revoluteJoint
        then
           Self.m_JvAC := (0.0, 0.0);
           Self.m_JwA  := 1.0;
           Self.m_JwC  := 1.0;
           Self.m_mass := Self.m_mass + Self.m_iA + Self.m_iC;

         else
            declare
               u  : constant b2Vec2 := b2Mul (qC,  Self.m_localAxisC);
               rC : constant b2Vec2 := b2Mul (qC,  Self.m_localAnchorC - Self.m_lcC);
               rA : constant b2Vec2 := b2Mul (qA,  Self.m_localAnchorA - Self.m_lcA);
            begin
               Self.m_JvAC := u;
               Self.m_JwC  := b2Cross (rC, u);
               Self.m_JwA  := b2Cross (rA, u);
               Self.m_mass :=   Self.m_mass
                              + Self.m_mC
                              + Self.m_mA
                              + Self.m_iC * Self.m_JwC * Self.m_JwC
                              + Self.m_iA * Self.m_JwA * Self.m_JwA;
            end;
         end if;


         if Self.m_typeB = e_revoluteJoint
         then
            Self.m_JvBD := (0.0, 0.0);
            Self.m_JwB  := Self.m_ratio;
            Self.m_JwD  := Self.m_ratio;
            Self.m_mass :=   Self.m_mass
                           + Self.m_ratio * Self.m_ratio * (Self.m_iB + Self.m_iD);

         else
            declare
               u  : constant b2Vec2 := b2Mul (qD, Self.m_localAxisD);
               rD : constant b2Vec2 := b2Mul (qD, Self.m_localAnchorD - Self.m_lcD);
               rB : constant b2Vec2 := b2Mul (qB, Self.m_localAnchorB - Self.m_lcB);
            begin
               Self.m_JvBD := Self.m_ratio * u;
               Self.m_JwD  := Self.m_ratio * b2Cross(rD, u);
               Self.m_JwB  := Self.m_ratio * b2Cross(rB, u);
               Self.m_mass :=   Self.m_mass
                              + Self.m_ratio * Self.m_ratio * (Self.m_mD + Self.m_mB)
                              + Self.m_iD    * Self.m_JwD   * Self.m_JwD
                              + Self.m_iB    * Self.m_JwB   * Self.m_JwB;
            end;
         end if;


         -- Compute effective mass.
         Self.m_mass := (if Self.m_mass > 0.0 then 1.0 / Self.m_mass
                                              else 0.0);

         if data.step.warmStarting
         then
            vA := vA + (Self.m_mA * Self.m_impulse) * Self.m_JvAC;
            wA := wA +  Self.m_iA * Self.m_impulse  * Self.m_JwA;
            vB := vB + (Self.m_mB * Self.m_impulse) * Self.m_JvBD;
            wB := wB +  Self.m_iB * Self.m_impulse  * Self.m_JwB;
            vC := vC - (Self.m_mC * Self.m_impulse) * Self.m_JvAC;
            wC := wC -  Self.m_iC * Self.m_impulse  * Self.m_JwC;
            vD := vD - (Self.m_mD * Self.m_impulse) * Self.m_JvBD;
            wD := wD -  Self.m_iD * Self.m_impulse  * Self.m_JwD;

         else
            Self.m_impulse := 0.0;
         end if;


         data.velocities (Self.m_indexA).v := vA;
         data.velocities (Self.m_indexA).w := wA;
         data.velocities (Self.m_indexB).v := vB;
         data.velocities (Self.m_indexB).w := wB;
         data.velocities (Self.m_indexC).v := vC;
         data.velocities (Self.m_indexC).w := wC;
         data.velocities (Self.m_indexD).v := vD;
         data.velocities (Self.m_indexD).w := wD;
      end;
   end initVelocityConstraints;




   --  void b2GearJoint::SolveVelocityConstraints(const b2SolverData& data)
   --  {
   --    b2Vec2 vA = data.velocities[m_indexA].v;
   --    float wA = data.velocities[m_indexA].w;
   --    b2Vec2 vB = data.velocities[m_indexB].v;
   --    float wB = data.velocities[m_indexB].w;
   --    b2Vec2 vC = data.velocities[m_indexC].v;
   --    float wC = data.velocities[m_indexC].w;
   --    b2Vec2 vD = data.velocities[m_indexD].v;
   --    float wD = data.velocities[m_indexD].w;
   --
   --    float Cdot = b2Dot(m_JvAC, vA - vC) + b2Dot(m_JvBD, vB - vD);
   --    Cdot += (m_JwA * wA - m_JwC * wC) + (m_JwB * wB - m_JwD * wD);
   --
   --    float impulse = -m_mass * Cdot;
   --    m_impulse += impulse;
   --
   --    vA += (m_mA * impulse) * m_JvAC;
   --    wA += m_iA * impulse * m_JwA;
   --    vB += (m_mB * impulse) * m_JvBD;
   --    wB += m_iB * impulse * m_JwB;
   --    vC -= (m_mC * impulse) * m_JvAC;
   --    wC -= m_iC * impulse * m_JwC;
   --    vD -= (m_mD * impulse) * m_JvBD;
   --    wD -= m_iD * impulse * m_JwD;
   --
   --    data.velocities[m_indexA].v = vA;
   --    data.velocities[m_indexA].w = wA;
   --    data.velocities[m_indexB].v = vB;
   --    data.velocities[m_indexB].w = wB;
   --    data.velocities[m_indexC].v = vC;
   --    data.velocities[m_indexC].w = wC;
   --    data.velocities[m_indexD].v = vD;
   --    data.velocities[m_indexD].w = wD;
   --  }
   --

   overriding
   procedure solveVelocityConstraints (Self : in out b2GearJoint;   Data : in b2SolverData)
   is
      vA   : b2Vec2 := data.velocities (Self.m_indexA).v;
      wA   : Real   := data.velocities (Self.m_indexA).w;

      vB   : b2Vec2 := data.velocities (Self.m_indexB).v;
      wB   : Real   := data.velocities (Self.m_indexB).w;

      vC   : b2Vec2 := data.velocities (Self.m_indexC).v;
      wC   : Real   := data.velocities (Self.m_indexC).w;

      vD   : b2Vec2 := data.velocities (Self.m_indexD).v;
      wD   : Real   := data.velocities (Self.m_indexD).w;

      Cdot : Real   :=   b2Dot (Self.m_JvAC,  vA - vC)
                       + b2Dot (Self.m_JvBD,  vB - vD);
   begin
      Cdot :=   Cdot
              + (  Self.m_JwA * wA
                 - Self.m_JwC * wC)
              + (  Self.m_JwB * wB
                 - Self.m_JwD * wD);
      declare
         impulse : constant Real := -Self.m_mass * Cdot;
      begin
         Self.m_impulse := Self.m_impulse + impulse;

         vA := vA + (Self.m_mA * impulse) * Self.m_JvAC;
         wA := wA +  Self.m_iA * impulse  * Self.m_JwA;
         vB := vB + (Self.m_mB * impulse) * Self.m_JvBD;
         wB := wB +  Self.m_iB * impulse  * Self.m_JwB;
         vC := vC - (Self.m_mC * impulse) * Self.m_JvAC;
         wC := wC -  Self.m_iC * impulse  * Self.m_JwC;
         vD := vD - (Self.m_mD * impulse) * Self.m_JvBD;
         wD := wD -  Self.m_iD * impulse  * Self.m_JwD;
      end;
      data.velocities (Self.m_indexA).v := vA;
      data.velocities (Self.m_indexA).w := wA;
      data.velocities (Self.m_indexB).v := vB;
      data.velocities (Self.m_indexB).w := wB;
      data.velocities (Self.m_indexC).v := vC;
      data.velocities (Self.m_indexC).w := wC;
      data.velocities (Self.m_indexD).v := vD;
      data.velocities (Self.m_indexD).w := wD;
   end solveVelocityConstraints;



   --  bool b2GearJoint::SolvePositionConstraints(const b2SolverData& data)
   --  {
   --    b2Vec2 cA = data.positions[m_indexA].c;
   --    float aA = data.positions[m_indexA].a;
   --    b2Vec2 cB = data.positions[m_indexB].c;
   --    float aB = data.positions[m_indexB].a;
   --    b2Vec2 cC = data.positions[m_indexC].c;
   --    float aC = data.positions[m_indexC].a;
   --    b2Vec2 cD = data.positions[m_indexD].c;
   --    float aD = data.positions[m_indexD].a;
   --
   --    b2Rot qA(aA), qB(aB), qC(aC), qD(aD);
   --
   --    float linearError = 0.0f;
   --
   --    float coordinateA, coordinateB;
   --
   --    b2Vec2 JvAC, JvBD;
   --    float JwA, JwB, JwC, JwD;
   --    float mass = 0.0f;
   --
   --    if (m_typeA == e_revoluteJoint)
   --    {
   --       JvAC.SetZero();
   --       JwA = 1.0f;
   --       JwC = 1.0f;
   --       mass += m_iA + m_iC;
   --
   --       coordinateA = aA - aC - m_referenceAngleA;
   --    }
   --    else
   --    {
   --       b2Vec2 u = b2Mul(qC, m_localAxisC);
   --       b2Vec2 rC = b2Mul(qC, m_localAnchorC - m_lcC);
   --       b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_lcA);
   --       JvAC = u;
   --       JwC = b2Cross(rC, u);
   --       JwA = b2Cross(rA, u);
   --       mass += m_mC + m_mA + m_iC * JwC * JwC + m_iA * JwA * JwA;
   --
   --       b2Vec2 pC = m_localAnchorC - m_lcC;
   --       b2Vec2 pA = b2MulT(qC, rA + (cA - cC));
   --       coordinateA = b2Dot(pA - pC, m_localAxisC);
   --    }
   --
   --    if (m_typeB == e_revoluteJoint)
   --    {
   --       JvBD.SetZero();
   --       JwB = m_ratio;
   --       JwD = m_ratio;
   --       mass += m_ratio * m_ratio * (m_iB + m_iD);
   --
   --       coordinateB = aB - aD - m_referenceAngleB;
   --    }
   --    else
   --    {
   --       b2Vec2 u = b2Mul(qD, m_localAxisD);
   --       b2Vec2 rD = b2Mul(qD, m_localAnchorD - m_lcD);
   --       b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_lcB);
   --       JvBD = m_ratio * u;
   --       JwD = m_ratio * b2Cross(rD, u);
   --       JwB = m_ratio * b2Cross(rB, u);
   --       mass += m_ratio * m_ratio * (m_mD + m_mB) + m_iD * JwD * JwD + m_iB * JwB * JwB;
   --
   --       b2Vec2 pD = m_localAnchorD - m_lcD;
   --       b2Vec2 pB = b2MulT(qD, rB + (cB - cD));
   --       coordinateB = b2Dot(pB - pD, m_localAxisD);
   --    }
   --
   --    float C = (coordinateA + m_ratio * coordinateB) - m_constant;
   --
   --    float impulse = 0.0f;
   --    if (mass > 0.0f)
   --    {
   --       impulse = -C / mass;
   --    }
   --
   --    cA += m_mA * impulse * JvAC;
   --    aA += m_iA * impulse * JwA;
   --    cB += m_mB * impulse * JvBD;
   --    aB += m_iB * impulse * JwB;
   --    cC -= m_mC * impulse * JvAC;
   --    aC -= m_iC * impulse * JwC;
   --    cD -= m_mD * impulse * JvBD;
   --    aD -= m_iD * impulse * JwD;
   --
   --    data.positions[m_indexA].c = cA;
   --    data.positions[m_indexA].a = aA;
   --    data.positions[m_indexB].c = cB;
   --    data.positions[m_indexB].a = aB;
   --    data.positions[m_indexC].c = cC;
   --    data.positions[m_indexC].a = aC;
   --    data.positions[m_indexD].c = cD;
   --    data.positions[m_indexD].a = aD;
   --
   --    // TODO_ERIN not implemented
   --    return linearError < b2_linearSlop;
   --  }
   --

   overriding
   function solvePositionConstraints (Self : in out b2GearJoint;   Data : in b2SolverData) return Boolean
   is
      cA : b2Vec2 := data.positions (Self.m_indexA).c;
      aA : Real   := data.positions (Self.m_indexA).a;
      cB : b2Vec2 := data.positions (Self.m_indexB).c;
      aB : Real   := data.positions (Self.m_indexB).a;
      cC : b2Vec2 := data.positions (Self.m_indexC).c;
      aC : Real   := data.positions (Self.m_indexC).a;
      cD : b2Vec2 := data.positions (Self.m_indexD).c;
      aD : Real   := data.positions (Self.m_indexD).a;

      qA : constant b2Rot := to_b2Rot (aA);
      qB : constant b2Rot := to_b2Rot (aB);
      qC : constant b2Rot := to_b2Rot (aC);
      qD : constant b2Rot := to_b2Rot (aD);

      linearError : constant Real := 0.0;

      coordinateA,
      coordinateB : Real;

      JvAC,
      JvBD        : b2Vec2;

      JwA,
      JwB,
      JwC,
      JwD         : Real;

      mass        : Real := 0.0;
   begin
      if Self.m_typeA = e_revoluteJoint
      then
         JvAC := (0.0, 0.0);
         JwA  :=  1.0;
         JwC  :=  1.0;
         mass := mass + Self.m_iA + Self.m_iC;

         coordinateA := aA - aC - Self.m_referenceAngleA;

      else
         declare
            u  : constant b2Vec2 := b2Mul (qC,  Self.m_localAxisC);
            rC : constant b2Vec2 := b2Mul (qC,  Self.m_localAnchorC - Self.m_lcC);
            rA : constant b2Vec2 := b2Mul (qA,  Self.m_localAnchorA - Self.m_lcA);

            pC,
            pA : b2Vec2;
         begin
            JvAC := u;
            JwC  := b2Cross (rC, u);
            JwA  := b2Cross (rA, u);
            mass :=   mass
                    + Self.m_mC + Self.m_mA
                    + Self.m_iC * JwC * JwC
                    + Self.m_iA * JwA * JwA;

            pC := Self.m_localAnchorC - Self.m_lcC;
            pA := b2MulT (qC,  rA + (cA - cC));

            coordinateA := b2Dot (pA - pC,  Self.m_localAxisC);
         end;
      end if;

      if Self.m_typeB = e_revoluteJoint
      then
         JvBD := (0.0, 0.0);
         JwB  := Self.m_ratio;
         JwD  := Self.m_ratio;
         mass :=   mass
                 +  Self.m_ratio
                 *  Self.m_ratio
                 * (Self.m_iB + Self.m_iD);

         coordinateB := aB - aD - Self.m_referenceAngleB;

      else
         declare
            u  : constant b2Vec2 := b2Mul (qD,  Self.m_localAxisD);
            rD : constant b2Vec2 := b2Mul (qD,  Self.m_localAnchorD - Self.m_lcD);
            rB : constant b2Vec2 := b2Mul (qB,  Self.m_localAnchorB - Self.m_lcB);

            pD,
            pB : b2Vec2;
         begin
            JvBD := Self.m_ratio * u;
            JwD  := Self.m_ratio * b2Cross (rD, u);
            JwB  := Self.m_ratio * b2Cross (rB, u);
            mass :=   mass
                    + Self.m_ratio * Self.m_ratio * (Self.m_mD + Self.m_mB)
                    + Self.m_iD * JwD * JwD
                    + Self.m_iB * JwB * JwB;

            pD   := Self.m_localAnchorD - Self.m_lcD;
            pB   := b2MulT (qD,  rB + (cB - cD));

            coordinateB := b2Dot (pB - pD,  Self.m_localAxisD);
         end;
      end if;

      declare
         C : constant Real :=  (  coordinateA
                                + Self.m_ratio * coordinateB)
                             - Self.m_constant;

         impulse : Real := 0.0;
      begin
         if mass > 0.0
         then
            impulse := -C / mass;
         end if;

         cA := cA + Self.m_mA * impulse * JvAC;
         aA := aA + Self.m_iA * impulse * JwA;
         cB := cB + Self.m_mB * impulse * JvBD;
         aB := aB + Self.m_iB * impulse * JwB;
         cC := cC - Self.m_mC * impulse * JvAC;
         aC := aC - Self.m_iC * impulse * JwC;
         cD := cD - Self.m_mD * impulse * JvBD;
         aD := aD - Self.m_iD * impulse * JwD;

         data.positions (Self.m_indexA).c := cA;
         data.positions (Self.m_indexA).a := aA;
         data.positions (Self.m_indexB).c := cB;
         data.positions (Self.m_indexB).a := aB;
         data.positions (Self.m_indexC).c := cC;
         data.positions (Self.m_indexC).a := aC;
         data.positions (Self.m_indexD).c := cD;
         data.positions (Self.m_indexD).a := aD;
      end;

      declare
         use b2_Common;
      begin
         -- TODO_ERIN not implemented
         return linearError < b2_linearSlop;
      end;
   end solvePositionConstraints;


end box2d.b2_Joint.b2_gear_Joint;
