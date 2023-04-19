with
     box2d.b2_time_Step,
     box2d.b2_Common;


package body box2d.b2_Joint.b2_pulley_Joint
is
   --    b2PulleyJointDef()
   --    {
   --       type = e_pulleyJoint;
   --       groundAnchorA.Set(-1.0f, 1.0f);
   --       groundAnchorB.Set(1.0f, 1.0f);
   --       localAnchorA.Set(-1.0f, 0.0f);
   --       localAnchorB.Set(1.0f, 0.0f);
   --       lengthA = 0.0f;
   --       lengthB = 0.0f;
   --       ratio = 1.0f;
   --       collideConnected = true;
   --    }
   --

   function  to_b2PulleyJointDef return b2PulleyJointDef
   is
      Self : b2PulleyJointDef;
   begin
      Self.Kind             := e_pulleyJoint;
      Self.groundAnchorA    := (-1.0, 1.0);
      Self.groundAnchorB    := ( 1.0, 1.0);
      Self.localAnchorA     := (-1.0, 0.0);
      Self.localAnchorB     := ( 1.0, 0.0);
      Self.lengthA          := 0.0;
      Self.lengthB          := 0.0;
      Self.ratio            := 1.0;
      Self.collideConnected := True;

      return Self;
   end to_b2PulleyJointDef;




   --  // Pulley:
   --  // length1 = norm(p1 - s1)
   --  // length2 = norm(p2 - s2)
   --  // C0 = (length1 + ratio * length2)_initial
   --  // C = C0 - (length1 + ratio * length2)
   --  // u1 = (p1 - s1) / norm(p1 - s1)
   --  // u2 = (p2 - s2) / norm(p2 - s2)
   --  // Cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
   --  // J = -[u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
   --  // K = J * invM * JT
   --  //   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)
   --
   --  void b2PulleyJointDef::Initialize(b2Body* bA, b2Body* bB,
   --             const b2Vec2& groundA, const b2Vec2& groundB,
   --             const b2Vec2& anchorA, const b2Vec2& anchorB,
   --             float r)
   --  {
   --    bodyA = bA;
   --    bodyB = bB;
   --    groundAnchorA = groundA;
   --    groundAnchorB = groundB;
   --    localAnchorA = bodyA->GetLocalPoint(anchorA);
   --    localAnchorB = bodyB->GetLocalPoint(anchorB);
   --    b2Vec2 dA = anchorA - groundA;
   --    lengthA = dA.Length();
   --    b2Vec2 dB = anchorB - groundB;
   --    lengthB = dB.Length();
   --    ratio = r;
   --    b2Assert(ratio > b2_epsilon);
   --  }
   --

   procedure initialize (Self : out b2PulleyJointDef;   bodyA,         bodyB         : access b2Body;
                                                        groundAnchorA, groundAnchorB : in     b2Vec2;
                                                        AnchorA,       AnchorB       : in     b2Vec2;
                                                        Ratio                        : in     Real)
   is
      use b2_Common;

      dA : b2Vec2;
      dB : b2Vec2;
   begin
     Self.bodyA         := bodyA;
     Self.bodyB         := bodyB;
     Self.groundAnchorA := groundAnchorA;
     Self.groundAnchorB := groundAnchorB;
     Self.localAnchorA  := bodyA.getLocalPoint (anchorA);
     Self.localAnchorB  := bodyB.getLocalPoint (anchorB);
     dA                 := anchorA - groundAnchorA;
     Self.lengthA       := Length (dA);
     dB                 := anchorB - groundAnchorB;
     Self.lengthB       := Length (dB);
     Self.ratio         := Ratio;
     pragma assert (Self.ratio > b2_epsilon);
   end initialize;





   -----------------
   --- b2PulleyJoint
   --

   --  b2Vec2 b2PulleyJoint::GetGroundAnchorA() const
   --  {
   --    return m_groundAnchorA;
   --  }
   --

   function getGroundAnchorA (Self : in b2PulleyJoint) return b2Vec2
   is
   begin
      return Self.m_groundAnchorA;
   end getGroundAnchorA;



   --  b2Vec2 b2PulleyJoint::GetGroundAnchorB() const
   --  {
   --    return m_groundAnchorB;
   --  }
   --

   function getGroundAnchorB (Self : in b2PulleyJoint) return b2Vec2
   is
   begin
      return Self.m_groundAnchorB;
   end getGroundAnchorB;



   --  b2Vec2 b2PulleyJoint::GetAnchorA() const
   --  {
   --    return m_bodyA->GetWorldPoint(m_localAnchorA);
   --  }
   --

   overriding
   function getAnchorA (Self : in b2PulleyJoint) return b2Vec2
   is
   begin
      return Self.m_bodyA.getWorldPoint (Self.m_localAnchorA);
   end getAnchorA;



   --  b2Vec2 b2PulleyJoint::GetAnchorB() const
   --  {
   --    return m_bodyB->GetWorldPoint(m_localAnchorB);
   --  }
   --

   overriding
   function getAnchorB (Self : in b2PulleyJoint) return b2Vec2
   is
   begin
      return Self.m_bodyB.getWorldPoint (Self.m_localAnchorB);
   end getAnchorB;



   --  b2Vec2 b2PulleyJoint::GetReactionForce(float inv_dt) const
   --  {
   --    b2Vec2 P = m_impulse * m_uB;
   --    return inv_dt * P;
   --  }
   --

   overriding
   function getReactionForce (Self : in b2PulleyJoint;   inv_dt : in Real) return b2Vec2
   is
      P : constant b2Vec2 := Self.m_impulse * Self.m_uB;
   begin
      return inv_dt * P;
   end getReactionForce;



   --  float b2PulleyJoint::GetReactionTorque(float inv_dt) const
   --  {
   --    B2_NOT_USED(inv_dt);
   --    return 0.0f;
   --  }
   --

   overriding
   function getReactionTorque (Self : in b2PulleyJoint;   inv_dt : in Real) return Real
   is
      pragma unreferenced (inv_dt);
   begin
      return 0.0;
   end getReactionTorque;




   --  float b2PulleyJoint::GetLengthA() const
   --  {
   --    return m_lengthA;
   --  }
   --

   function getLengthA (Self : in b2PulleyJoint) return Real
   is
   begin
      return Self.m_lengthA;
   end getLengthA;



   --  float b2PulleyJoint::GetCurrentLengthB() const
   --  {
   --    b2Vec2 p = m_bodyB->GetWorldPoint(m_localAnchorB);
   --    b2Vec2 s = m_groundAnchorB;
   --    b2Vec2 d = p - s;
   --    return d.Length();
   --  }
   --

   function getLengthB (Self : in b2PulleyJoint) return Real
   is
      p : constant b2Vec2 := Self.m_bodyB.getWorldPoint (Self.m_localAnchorB);
      s : constant b2Vec2 := Self.m_groundAnchorB;
      d : constant b2Vec2 := p - s;
   begin
      return Length (d);
   end getLengthB;



   --  float b2PulleyJoint::GetRatio() const
   --  {
   --    return m_ratio;
   --  }
   --

   function getRatio (Self : in out b2PulleyJoint) return Real
   is
   begin
      return Self.m_ratio;
   end getRatio;



   --  float b2PulleyJoint::GetCurrentLengthA() const
   --  {
   --    b2Vec2 p = m_bodyA->GetWorldPoint(m_localAnchorA);
   --    b2Vec2 s = m_groundAnchorA;
   --    b2Vec2 d = p - s;
   --    return d.Length();
   --  }
   --

   function getCurrentLengthA (Self : in b2PulleyJoint) return Real
   is
      p : constant b2Vec2 := Self.m_bodyA.getWorldPoint (Self.m_localAnchorA);
      s : constant b2Vec2 := Self.m_groundAnchorA;
      d : constant b2Vec2 := p - s;
   begin
      return Length (d);
   end getCurrentLengthA;



   --    Get the current length of the segment attached to bodyB.
   --
   --  float b2PulleyJoint::GetCurrentLengthB() const
   --  {
   --    b2Vec2 p = m_bodyB->GetWorldPoint(m_localAnchorB);
   --    b2Vec2 s = m_groundAnchorB;
   --    b2Vec2 d = p - s;
   --    return d.Length();
   --  }

   function getCurrentLengthB (Self : in b2PulleyJoint) return Real
   is
      p : constant b2Vec2 := Self.m_bodyB.getWorldPoint (Self.m_localAnchorB);
      s : constant b2Vec2 := Self.m_groundAnchorB;
      d : constant b2Vec2 := p - s;
   begin
      return Length (d);
   end getCurrentLengthB;



   --  void b2PulleyJoint::ShiftOrigin(const b2Vec2& newOrigin)
   --  {
   --    m_groundAnchorA -= newOrigin;
   --    m_groundAnchorB -= newOrigin;
   --  }

   overriding
   procedure shiftOrigin (Self : in out b2PulleyJoint;   newOrigin : in b2Vec2)
   is
   begin
      Self.m_groundAnchorA := Self.m_groundAnchorA - newOrigin;
      Self.m_groundAnchorB := Self.m_groundAnchorB - newOrigin;
   end shiftOrigin;



   --  void b2PulleyJoint::Dump()
   --  {
   --    int32 indexA = m_bodyA->m_islandIndex;
   --    int32 indexB = m_bodyB->m_islandIndex;
   --
   --    b2Dump("  b2PulleyJointDef jd;\n");
   --    b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
   --    b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
   --    b2Dump("  jd.collideConnected = bool(%d);\n", m_collideConnected);
   --    b2Dump("  jd.groundAnchorA.Set(%.9g, %.9g);\n", m_groundAnchorA.x, m_groundAnchorA.y);
   --    b2Dump("  jd.groundAnchorB.Set(%.9g, %.9g);\n", m_groundAnchorB.x, m_groundAnchorB.y);
   --    b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", m_localAnchorA.x, m_localAnchorA.y);
   --    b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", m_localAnchorB.x, m_localAnchorB.y);
   --    b2Dump("  jd.lengthA = %.9g;\n", m_lengthA);
   --    b2Dump("  jd.lengthB = %.9g;\n", m_lengthB);
   --    b2Dump("  jd.ratio = %.9g;\n", m_ratio);
   --    b2Dump("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
   --  }
   --

   overriding
   procedure dump (Self : in b2PulleyJoint)
   is
      use b2_Common;

      indexA : constant Integer := Self.m_bodyA.m_islandIndex;
      indexB : constant Integer := Self.m_bodyB.m_islandIndex;
   begin
      b2Dump ("  b2PulleyJointDef jd;");
      b2Dump ("  jd.bodyA = bodies[%d];"                  & indexA'Image);
      b2Dump ("  jd.bodyB = bodies[%d];"                  & indexB'Image);
      b2Dump ("  jd.collideConnected = bool(%d);"         & Self.m_collideConnected'Image);
      b2Dump ("  jd.groundAnchorA.Set(%.9g, %.9g);"       & Self.m_groundAnchorA.x 'Image  & Self.m_groundAnchorA.y'Image);
      b2Dump ("  jd.groundAnchorB.Set(%.9g, %.9g);"       & Self.m_groundAnchorB.x 'Image  & Self.m_groundAnchorB.y'Image);
      b2Dump ("  jd.localAnchorA.Set(%.9g, %.9g);"        & Self.m_localAnchorA .x 'Image  & Self.m_localAnchorA .y'Image);
      b2Dump ("  jd.localAnchorB.Set(%.9g, %.9g);"        & Self.m_localAnchorB .x 'Image  & Self.m_localAnchorB .y'Image);
      b2Dump ("  jd.lengthA = %.9g;"                      & Self.m_lengthA'Image);
      b2Dump ("  jd.lengthB = %.9g;"                      & Self.m_lengthB'Image);
      b2Dump ("  jd.ratio = %.9g;"                        & Self.m_ratio  'Image);
      b2Dump ("  joints[%d] = m_world->CreateJoint(&jd);" & Self.m_index  'Image);
   end dump;




   --------------
   --  protected:
   --

   package body Forge
   is

      --  b2PulleyJoint::b2PulleyJoint(const b2PulleyJointDef* def)
      --  : b2Joint(def)
      --  {
      --    m_groundAnchorA = def->groundAnchorA;
      --    m_groundAnchorB = def->groundAnchorB;
      --    m_localAnchorA = def->localAnchorA;
      --    m_localAnchorB = def->localAnchorB;
      --
      --    m_lengthA = def->lengthA;
      --    m_lengthB = def->lengthB;
      --
      --    b2Assert(def->ratio != 0.0f);
      --    m_ratio = def->ratio;
      --
      --    m_constant = def->lengthA + m_ratio * def->lengthB;
      --
      --    m_impulse = 0.0f;
      --  }
      --

      function to_b2PulleyJoint (Def : in b2PulleyJointDef) return b2PulleyJoint
      is
         Self : b2PulleyJoint;
      begin
         b2_Joint.Forge.define (Self, Def);

         Self.m_groundAnchorA := def.groundAnchorA;
         Self.m_groundAnchorB := def.groundAnchorB;
         Self.m_localAnchorA  := def.localAnchorA;
         Self.m_localAnchorB  := def.localAnchorB;

         Self.m_lengthA := def.lengthA;
         Self.m_lengthB := def.lengthB;

         pragma assert (def.ratio /= 0.0);
         Self.m_ratio    := def.ratio;
         Self.m_constant := def.lengthA + Self.m_ratio * def.lengthB;
         Self.m_impulse  := 0.0;

         return Self;
      end to_b2PulleyJoint;

   end Forge;



   --  void b2PulleyJoint::InitVelocityConstraints(const b2SolverData& data)
   --  {
   --    m_indexA = m_bodyA->m_islandIndex;
   --    m_indexB = m_bodyB->m_islandIndex;
   --    m_localCenterA = m_bodyA->m_sweep.localCenter;
   --    m_localCenterB = m_bodyB->m_sweep.localCenter;
   --    m_invMassA = m_bodyA->m_invMass;
   --    m_invMassB = m_bodyB->m_invMass;
   --    m_invIA = m_bodyA->m_invI;
   --    m_invIB = m_bodyB->m_invI;
   --
   --    b2Vec2 cA = data.positions[m_indexA].c;
   --    float aA = data.positions[m_indexA].a;
   --    b2Vec2 vA = data.velocities[m_indexA].v;
   --    float wA = data.velocities[m_indexA].w;
   --
   --    b2Vec2 cB = data.positions[m_indexB].c;
   --    float aB = data.positions[m_indexB].a;
   --    b2Vec2 vB = data.velocities[m_indexB].v;
   --    float wB = data.velocities[m_indexB].w;
   --
   --    b2Rot qA(aA), qB(aB);
   --
   --    m_rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
   --    m_rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
   --
   --    // Get the pulley axes.
   --    m_uA = cA + m_rA - m_groundAnchorA;
   --    m_uB = cB + m_rB - m_groundAnchorB;
   --
   --    float lengthA = m_uA.Length();
   --    float lengthB = m_uB.Length();
   --
   --    if (lengthA > 10.0f * b2_linearSlop)
   --    {
   --       m_uA *= 1.0f / lengthA;
   --    }
   --    else
   --    {
   --       m_uA.SetZero();
   --    }
   --
   --    if (lengthB > 10.0f * b2_linearSlop)
   --    {
   --       m_uB *= 1.0f / lengthB;
   --    }
   --    else
   --    {
   --       m_uB.SetZero();
   --    }
   --
   --    // Compute effective mass.
   --    float ruA = b2Cross(m_rA, m_uA);
   --    float ruB = b2Cross(m_rB, m_uB);
   --
   --    float mA = m_invMassA + m_invIA * ruA * ruA;
   --    float mB = m_invMassB + m_invIB * ruB * ruB;
   --
   --    m_mass = mA + m_ratio * m_ratio * mB;
   --
   --    if (m_mass > 0.0f)
   --    {
   --       m_mass = 1.0f / m_mass;
   --    }
   --
   --    if (data.step.warmStarting)
   --    {
   --       // Scale impulses to support variable time steps.
   --       m_impulse *= data.step.dtRatio;
   --
   --       // Warm starting.
   --       b2Vec2 PA = -(m_impulse) * m_uA;
   --       b2Vec2 PB = (-m_ratio * m_impulse) * m_uB;
   --
   --       vA += m_invMassA * PA;
   --       wA += m_invIA * b2Cross(m_rA, PA);
   --       vB += m_invMassB * PB;
   --       wB += m_invIB * b2Cross(m_rB, PB);
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
   --  }
   --

   overriding
   procedure initVelocityConstraints (Self : in out b2PulleyJoint;   Data : in b2SolverData)
   is
   begin
      Self.m_indexA       := Self.m_bodyA.m_islandIndex;
      Self.m_indexB       := Self.m_bodyB.m_islandIndex;
      Self.m_localCenterA := Self.m_bodyA.m_sweep.localCenter;
      Self.m_localCenterB := Self.m_bodyB.m_sweep.localCenter;
      Self.m_invMassA     := Self.m_bodyA.m_invMass;
      Self.m_invMassB     := Self.m_bodyB.m_invMass;
      Self.m_invIA        := Self.m_bodyA.m_invI;
      Self.m_invIB        := Self.m_bodyB.m_invI;

      declare
         cA : constant b2Vec2 := data.positions  (Self.m_indexA).c;
         aA : constant Real   := data.positions  (Self.m_indexA).a;
         vA :          b2Vec2 := data.velocities (Self.m_indexA).v;
         wA :          Real   := data.velocities (Self.m_indexA).w;

         cB : constant b2Vec2 := data.positions  (Self.m_indexB).c;
         aB : constant Real   := data.positions  (Self.m_indexB).a;
         vB :          b2Vec2 := data.velocities (Self.m_indexB).v;
         wB :          Real   := data.velocities (Self.m_indexB).w;

         qA : constant b2Rot  := to_b2Rot (aA);
         qB : constant b2Rot  := to_b2Rot (aB);

      begin
         Self.m_rA := b2Mul (qA,  Self.m_localAnchorA - Self.m_localCenterA);
         Self.m_rB := b2Mul (qB,  Self.m_localAnchorB - Self.m_localCenterB);

         -- Get the pulley axes.
         Self.m_uA := cA + Self.m_rA - Self.m_groundAnchorA;
         Self.m_uB := cB + Self.m_rB - Self.m_groundAnchorB;

         declare
            use b2_Common;

            lengthA : constant Real := Length (Self.m_uA);
            lengthB : constant Real := Length (Self.m_uB);
         begin
            if lengthA > 10.0 * b2_linearSlop
            then
               multiply (Self.m_uA, 1.0 / lengthA);
            else
               Self.m_uA := (0.0, 0.0);
            end if;

            if lengthB > 10.0 * b2_linearSlop
            then
               multiply (Self.m_uB, 1.0 / lengthB);
            else
               Self.m_uB := (0.0, 0.0);
            end if;
         end;

         -- Compute effective mass.
         declare
            ruA : constant Real := b2Cross (Self.m_rA, Self.m_uA);
            ruB : constant Real := b2Cross (Self.m_rB, Self.m_uB);

            mA  : constant Real := Self.m_invMassA + Self.m_invIA * ruA * ruA;
            mB  : constant Real := Self.m_invMassB + Self.m_invIB * ruB * ruB;
         begin
            Self.m_mass := mA + Self.m_ratio * Self.m_ratio * mB;
         end;


         if Self.m_mass > 0.0
         then
            Self.m_mass := 1.0 / Self.m_mass;
         end if;


         if data.step.warmStarting
         then
            -- Scale impulses to support variable time steps.
            Self.m_impulse := Self.m_impulse * data.step.dtRatio;

            -- Warm starting.
            declare
               pA : constant b2Vec2 := -(Self.m_impulse)                 * Self.m_uA;
               pB : constant b2Vec2 :=  (-Self.m_ratio * Self.m_impulse) * Self.m_uB;
            begin
               vA := vA + Self.m_invMassA * PA;
               wA := wA + Self.m_invIA    * b2Cross (Self.m_rA, PA);
               vB := vB + Self.m_invMassB * PB;
               wB := wB + Self.m_invIB    * b2Cross (Self.m_rB, PB);
            end;
         else
            Self.m_impulse := 0.0;
         end if;


         data.velocities (Self.m_indexA).v := vA;
         data.velocities (Self.m_indexA).w := wA;
         data.velocities (Self.m_indexB).v := vB;
         data.velocities (Self.m_indexB).w := wB;
      end;
   end initVelocityConstraints;



   --  void b2PulleyJoint::SolveVelocityConstraints(const b2SolverData& data)
   --  {
   --    b2Vec2 vA = data.velocities[m_indexA].v;
   --    float wA = data.velocities[m_indexA].w;
   --    b2Vec2 vB = data.velocities[m_indexB].v;
   --    float wB = data.velocities[m_indexB].w;
   --
   --    b2Vec2 vpA = vA + b2Cross(wA, m_rA);
   --    b2Vec2 vpB = vB + b2Cross(wB, m_rB);
   --
   --    float Cdot = -b2Dot(m_uA, vpA) - m_ratio * b2Dot(m_uB, vpB);
   --    float impulse = -m_mass * Cdot;
   --    m_impulse += impulse;
   --
   --    b2Vec2 PA = -impulse * m_uA;
   --    b2Vec2 PB = -m_ratio * impulse * m_uB;
   --    vA += m_invMassA * PA;
   --    wA += m_invIA * b2Cross(m_rA, PA);
   --    vB += m_invMassB * PB;
   --    wB += m_invIB * b2Cross(m_rB, PB);
   --
   --    data.velocities[m_indexA].v = vA;
   --    data.velocities[m_indexA].w = wA;
   --    data.velocities[m_indexB].v = vB;
   --    data.velocities[m_indexB].w = wB;
   --  }
   --

   overriding
   procedure solveVelocityConstraints (Self : in out b2PulleyJoint;   Data : in b2SolverData)
   is
      vA      :          b2Vec2 := data.velocities (Self.m_indexA).v;
      wA      :          Real   := data.velocities (Self.m_indexA).w;
      vB      :          b2Vec2 := data.velocities (Self.m_indexB).v;
      wB      :          Real   := data.velocities (Self.m_indexB).w;

      vpA     : constant b2Vec2 := vA + b2Cross (wA, Self.m_rA);
      vpB     : constant b2Vec2 := vB + b2Cross (wB, Self.m_rB);

      Cdot    : constant Real   :=                 -b2Dot (Self.m_uA, vpA)
                          - Self.m_ratio * b2Dot (Self.m_uB, vpB);
      impulse : constant Real   := -Self.m_mass * Cdot;

      PA      : constant b2Vec2 := -impulse * Self.m_uA;
      PB      : constant b2Vec2 := -Self.m_ratio * impulse * Self.m_uB;
   begin
      Self.m_impulse := Self.m_impulse + impulse;

      vA := vA + Self.m_invMassA * PA;
      wA := wA + Self.m_invIA    * b2Cross (Self.m_rA, PA);
      vB := vB + Self.m_invMassB * PB;
      wB := wB + Self.m_invIB    * b2Cross (Self.m_rB, PB);

      data.velocities (Self.m_indexA).v := vA;
      data.velocities (Self.m_indexA).w := wA;
      data.velocities (Self.m_indexB).v := vB;
      data.velocities (Self.m_indexB).w := wB;
   end solveVelocityConstraints;



   --  bool b2PulleyJoint::SolvePositionConstraints(const b2SolverData& data)
   --  {
   --    b2Vec2 cA = data.positions[m_indexA].c;
   --    float aA = data.positions[m_indexA].a;
   --    b2Vec2 cB = data.positions[m_indexB].c;
   --    float aB = data.positions[m_indexB].a;
   --
   --    b2Rot qA(aA), qB(aB);
   --
   --    b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
   --    b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
   --
   --    // Get the pulley axes.
   --    b2Vec2 uA = cA + rA - m_groundAnchorA;
   --    b2Vec2 uB = cB + rB - m_groundAnchorB;
   --
   --    float lengthA = uA.Length();
   --    float lengthB = uB.Length();
   --
   --    if (lengthA > 10.0f * b2_linearSlop)
   --    {
   --       uA *= 1.0f / lengthA;
   --    }
   --    else
   --    {
   --       uA.SetZero();
   --    }
   --
   --    if (lengthB > 10.0f * b2_linearSlop)
   --    {
   --       uB *= 1.0f / lengthB;
   --    }
   --    else
   --    {
   --       uB.SetZero();
   --    }
   --
   --    // Compute effective mass.
   --    float ruA = b2Cross(rA, uA);
   --    float ruB = b2Cross(rB, uB);
   --
   --    float mA = m_invMassA + m_invIA * ruA * ruA;
   --    float mB = m_invMassB + m_invIB * ruB * ruB;
   --
   --    float mass = mA + m_ratio * m_ratio * mB;
   --
   --    if (mass > 0.0f)
   --    {
   --       mass = 1.0f / mass;
   --    }
   --
   --    float C = m_constant - lengthA - m_ratio * lengthB;
   --    float linearError = b2Abs(C);
   --
   --    float impulse = -mass * C;
   --
   --    b2Vec2 PA = -impulse * uA;
   --    b2Vec2 PB = -m_ratio * impulse * uB;
   --
   --    cA += m_invMassA * PA;
   --    aA += m_invIA * b2Cross(rA, PA);
   --    cB += m_invMassB * PB;
   --    aB += m_invIB * b2Cross(rB, PB);
   --
   --    data.positions[m_indexA].c = cA;
   --    data.positions[m_indexA].a = aA;
   --    data.positions[m_indexB].c = cB;
   --    data.positions[m_indexB].a = aB;
   --
   --    return linearError < b2_linearSlop;
   --  }
   --

   overriding
   function solvePositionConstraints (Self : in out b2PulleyJoint;   Data : in b2SolverData) return Boolean
   is
      use b2_Common;

      cA :          b2Vec2 := data.positions (Self.m_indexA).c;
      aA :          Real   := data.positions (Self.m_indexA).a;
      cB :          b2Vec2 := data.positions (Self.m_indexB).c;
      aB :          Real   := data.positions (Self.m_indexB).a;

      qA : constant b2Rot  := to_b2Rot (aA);
      qB : constant b2Rot  := to_b2Rot (aB);

      rA : constant b2Vec2 := b2Mul (qA, Self.m_localAnchorA - Self.m_localCenterA);
      rB : constant b2Vec2 := b2Mul (qB, Self.m_localAnchorB - Self.m_localCenterB);

      -- Get the pulley axes.
      uA :          b2Vec2 := cA + rA - Self.m_groundAnchorA;
      uB :          b2Vec2 := cB + rB - Self.m_groundAnchorB;

      lengthA : constant Real := Length (uA);
      lengthB : constant Real := Length (uB);

   begin
      if lengthA > 10.0 * b2_linearSlop
      then
         multiply (uA, 1.0 / lengthA);
      else
         uA := (0.0, 0.0);
      end if;

      if lengthB > 10.0 * b2_linearSlop
      then
         multiply (uB, 1.0 / lengthB);
      else
         uB := (0.0, 0.0);
      end if;

      -- Compute effective mass.
      declare
         ruA  : constant Real := b2Cross (rA, uA);
         ruB  : constant Real := b2Cross (rB, uB);

         mA   : constant Real := Self.m_invMassA  +  Self.m_invIA * ruA * ruA;
         mB   : constant Real := Self.m_invMassB  +  Self.m_invIB * ruB * ruB;

         mass :          Real := mA  +  Self.m_ratio * Self.m_ratio * mB;
      begin
         if mass > 0.0
         then
            mass := 1.0 / mass;
         end if;

         declare
            C           : constant Real := Self.m_constant - lengthA - Self.m_ratio * lengthB;
            linearError : constant Real := abs C;

            impulse     : constant Real := -mass * C;

            PA          : constant b2Vec2 := -impulse * uA;
            PB          : constant b2Vec2 := -Self.m_ratio * impulse * uB;
         begin
            cA := cA + Self.m_invMassA * PA;
            aA := aA + Self.m_invIA    * b2Cross (rA, PA);
            cB := cB + Self.m_invMassB * PB;
            aB := aB + Self.m_invIB    * b2Cross (rB, PB);

            data.positions (Self.m_indexA).c := cA;
            data.positions (Self.m_indexA).a := aA;
            data.positions (Self.m_indexB).c := cB;
            data.positions (Self.m_indexB).a := aB;

            return linearError < b2_linearSlop;
         end;
      end;
   end solvePositionConstraints;



end box2d.b2_Joint.b2_pulley_Joint;
