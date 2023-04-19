with
     box2d.b2_time_Step,
     box2d.b2_Common;


package body box2d.b2_Joint.b2_motor_Joint
is


   --    b2MotorJointDef()
   --    {
   --       type = e_motorJoint;
   --       linearOffset.SetZero();
   --       angularOffset = 0.0f;
   --       maxForce = 1.0f;
   --       maxTorque = 1.0f;
   --       correctionFactor = 0.3f;
   --    }
   --

   function  to_b2MotorJointDef return b2MotorJointDef
   is
      Self : b2MotorJointDef;
   begin
      Self.Kind             := e_motorJoint;
      Self.linearOffset     := (0.0, 0.0);
      Self.angularOffset    := 0.0;
      Self.maxForce         := 1.0;
      Self.maxTorque        := 1.0;
      Self.correctionFactor := 0.3;

      return Self;
   end to_b2MotorJointDef;



   --  // Point-to-point constraint
   --  // Cdot = v2 - v1
   --  //      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
   --  // J = [-I -r1_skew I r2_skew ]
   --  // Identity used:
   --  // w k % (rx i + ry j) = w * (-ry i + rx j)
   --  //
   --  // r1 = offset - c1
   --  // r2 = -c2
   --
   --  // Angle constraint
   --  // Cdot = w2 - w1
   --  // J = [0 0 -1 0 0 1]
   --  // K = invI1 + invI2
   --

   --  void b2MotorJointDef::Initialize(b2Body* bA, b2Body* bB)
   --  {
   --    bodyA = bA;
   --    bodyB = bB;
   --    b2Vec2 xB = bodyB->GetPosition();
   --    linearOffset = bodyA->GetLocalPoint(xB);
   --
   --    float angleA = bodyA->GetAngle();
   --    float angleB = bodyB->GetAngle();
   --    angularOffset = angleB - angleA;
   --  }
   --

   procedure initialize (Self : out b2MotorJointDef;   BodyA, BodyB : in b2Body_ptr)
   is
      xB : constant b2Vec2 := BodyB.getPosition;
   begin
      Self.bodyA := BodyA;
      Self.bodyB := BodyB;

      Self.linearOffset := BodyA.getLocalPoint (xB);

      declare
         angleA : constant Real := BodyA.getAngle;
         angleB : constant Real := BodyB.getAngle;
      begin
         Self.angularOffset := angleB - angleA;
      end;
   end initialize;



   ----------------
   --- b2MotorJoint
   --


   --  b2Vec2 b2MotorJoint::GetAnchorA() const
   --  {
   --    return m_bodyA->GetPosition();
   --  }
   --

   overriding
   function getAnchorA (Self : in b2MotorJoint) return b2Vec2
   is
   begin
      return Self.m_bodyA.getPosition;
   end getAnchorA;



   --  b2Vec2 b2MotorJoint::GetAnchorB() const
   --  {
   --    return m_bodyB->GetPosition();
   --  }
   --

   overriding
   function getAnchorB (Self : in b2MotorJoint) return b2Vec2
   is
   begin
      return Self.m_bodyB.getPosition;
   end getAnchorB;



   --  b2Vec2 b2MotorJoint::GetReactionForce(float inv_dt) const
   --  {
   --    return inv_dt * m_linearImpulse;
   --  }
   --

   overriding
   function getReactionForce (Self : in b2MotorJoint;   inv_dt : in Real) return b2Vec2
   is
   begin
      return inv_dt * Self.m_linearImpulse;
   end getReactionForce;



   --  float b2MotorJoint::GetReactionTorque(float inv_dt) const
   --  {
   --    return inv_dt * m_angularImpulse;
   --  }
   --

   overriding
   function getReactionTorque (Self : in b2MotorJoint;   inv_dt : in Real) return Real
   is
   begin
      return inv_dt * Self.m_angularImpulse;
   end getReactionTorque;



   --  void b2MotorJoint::SetLinearOffset(const b2Vec2& linearOffset)
   --  {
   --    if (linearOffset.x != m_linearOffset.x || linearOffset.y != m_linearOffset.y)
   --    {
   --       m_bodyA->SetAwake(true);
   --       m_bodyB->SetAwake(true);
   --       m_linearOffset = linearOffset;
   --    }
   --  }
   --

   procedure setLinearOffset (Self : in out b2MotorJoint;   linearOffset : in b2Vec2)
   is
   begin
      if   linearOffset.x /= Self.m_linearOffset.x
        or linearOffset.y /= Self.m_linearOffset.y
     then
         Self.m_bodyA.setAwake (True);
         Self.m_bodyB.setAwake (True);

         Self.m_linearOffset := linearOffset;
     end if;
   end setLinearOffset;



   --  const b2Vec2& b2MotorJoint::GetLinearOffset() const
   --  {
   --    return m_linearOffset;
   --  }
   --

   function getLinearOffset (Self : in b2MotorJoint) return b2Vec2
   is
   begin
      return Self.m_linearOffset;
   end getLinearOffset;




   --  void b2MotorJoint::SetAngularOffset(float angularOffset)
   --  {
   --    if (angularOffset != m_angularOffset)
   --    {
   --       m_bodyA->SetAwake(true);
   --       m_bodyB->SetAwake(true);
   --       m_angularOffset = angularOffset;
   --    }
   --  }
   --

   procedure setAngularOffset (Self : in out b2MotorJoint;   angularOffset : in Real)
   is
   begin
     if angularOffset /= Self.m_angularOffset
     then
         Self.m_bodyA.setAwake (True);
         Self.m_bodyB.setAwake (True);

         Self.m_angularOffset := angularOffset;
     end if;
   end setAngularOffset;



   --  float b2MotorJoint::GetAngularOffset() const
   --  {
   --    return m_angularOffset;
   --  }
   --

   function getAngularOffset (Self : in b2MotorJoint) return Real
   is
   begin
      return Self.m_angularOffset;
   end getAngularOffset;



   --  void b2MotorJoint::SetMaxForce(float force)
   --  {
   --    b2Assert(b2IsValid(force) && force >= 0.0f);
   --    m_maxForce = force;
   --  }
   --

   procedure setMaxForce (Self : in out b2MotorJoint;   Force : in Real)
   is
      pragma assert (    b2IsValid (Force)
                     and Force >= 0.0);
   begin
      Self.m_maxForce := Force;
   end setMaxForce;



   --  float b2MotorJoint::GetMaxForce() const
   --  {
   --    return m_maxForce;
   --  }
   --

   function getMaxForce (Self : in b2MotorJoint) return Real
   is
   begin
      return Self.m_maxForce;
   end getMaxForce;



   --  void b2MotorJoint::SetMaxTorque(float torque)
   --  {
   --    b2Assert(b2IsValid(torque) && torque >= 0.0f);
   --    m_maxTorque = torque;
   --  }
   --

   procedure setMaxTorque (Self : in out b2MotorJoint;   Torque : in Real)
   is
      pragma assert (    b2IsValid (Torque)
                     and Torque >= 0.0);
   begin
      Self.m_maxTorque := Torque;
   end setMaxTorque;




   --  float b2MotorJoint::GetMaxTorque() const
   --  {
   --    return m_maxTorque;
   --  }
   --

   function getMaxTorque (Self : in b2MotorJoint) return Real
   is
   begin
      return Self.m_maxTorque;
   end getMaxTorque;



   --  void b2MotorJoint::SetCorrectionFactor(float factor)
   --  {
   --    b2Assert(b2IsValid(factor) && 0.0f <= factor && factor <= 1.0f);
   --    m_correctionFactor = factor;
   --  }
   --

   procedure setCorrectionFactor (Self : in out b2MotorJoint;   Factor : in Real)
   is
      pragma assert (    b2IsValid (Factor)
                     and 0.0    <= Factor
                     and Factor <= 1.0);
   begin
      Self.m_correctionFactor := Factor;
   end setCorrectionFactor;



   --  float b2MotorJoint::GetCorrectionFactor() const
   --  {
   --    return m_correctionFactor;
   --  }
   --

   function getCorrectionFactor (Self : in b2MotorJoint) return Real
   is
   begin
      return Self.m_correctionFactor;
   end getCorrectionFactor;



   --  void b2MotorJoint::Dump()
   --  {
   --    int32 indexA = m_bodyA->m_islandIndex;
   --    int32 indexB = m_bodyB->m_islandIndex;
   --
   --    b2Dump("  b2MotorJointDef jd;\n");
   --    b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
   --    b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
   --    b2Dump("  jd.collideConnected = bool(%d);\n", m_collideConnected);
   --    b2Dump("  jd.linearOffset.Set(%.9g, %.9g);\n", m_linearOffset.x, m_linearOffset.y);
   --    b2Dump("  jd.angularOffset = %.9g;\n", m_angularOffset);
   --    b2Dump("  jd.maxForce = %.9g;\n", m_maxForce);
   --    b2Dump("  jd.maxTorque = %.9g;\n", m_maxTorque);
   --    b2Dump("  jd.correctionFactor = %.9g;\n", m_correctionFactor);
   --    b2Dump("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
   --  }

   overriding
   procedure dump (Self : in b2MotorJoint)
   is
      use b2_Common;

      indexA : constant Integer := Self.m_bodyA.m_islandIndex;
      indexB : constant Integer := Self.m_bodyB.m_islandIndex;
   begin
      b2Dump ("  b2MotorJointDef jd;");
      b2Dump ("  jd.bodyA = bodies[%d];" &                  indexA'Image);
      b2Dump ("  jd.bodyB = bodies[%d];" &                  indexB'Image);
      b2Dump ("  jd.collideConnected = bool(%d);" &         Self.m_collideConnected'Image);
      b2Dump ("  jd.linearOffset.Set(%.9g, %.9g);" &        Self.m_linearOffset.x'Image
                                                   &        Self.m_linearOffset.y'Image);
      b2Dump ("  jd.angularOffset = %.9g;" &                Self.m_angularOffset'Image);
      b2Dump ("  jd.maxForce = %.9g;" &                     Self.m_maxForce'Image);
      b2Dump ("  jd.maxTorque = %.9g;" &                    Self.m_maxTorque'Image);
      b2Dump ("  jd.correctionFactor = %.9g;" &             Self.m_correctionFactor'Image);
      b2Dump ("  joints[%d] = m_world->CreateJoint(&jd);" & Self.m_index'Image);
   end dump;




   -------------
   -- protected:
   --

   package body Forge
   is

      --  b2MotorJoint::b2MotorJoint(const b2MotorJointDef* def)
      --  : b2Joint(def)
      --  {
      --    m_linearOffset = def->linearOffset;
      --    m_angularOffset = def->angularOffset;
      --
      --    m_linearImpulse.SetZero();
      --    m_angularImpulse = 0.0f;
      --
      --    m_maxForce = def->maxForce;
      --    m_maxTorque = def->maxTorque;
      --    m_correctionFactor = def->correctionFactor;
      --  }
      --

      function to_b2MotorJoint (Def : in b2MotorJointDef) return b2MotorJoint
      is
         Self : b2MotorJoint; --  := (to_b2Joint (Def) with others => <>);
      begin
         b2_Joint.Forge.define (Self, Def);     -- TODO: Make sure all child joints do this !

         Self.m_linearOffset     := def.linearOffset;
         Self.m_angularOffset    := def.angularOffset;

         Self.m_linearImpulse    := (0.0, 0.0);
         Self.m_angularImpulse   := 0.0;

         Self.m_maxForce         := def.maxForce;
         Self.m_maxTorque        := def.maxTorque;
         Self.m_correctionFactor := def.correctionFactor;

         return Self;
      end to_b2MotorJoint;

   end Forge;




   --  void b2MotorJoint::InitVelocityConstraints(const b2SolverData& data)
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
   --    // Compute the effective mass matrix.
   --    m_rA = b2Mul(qA, m_linearOffset - m_localCenterA);
   --    m_rB = b2Mul(qB, -m_localCenterB);
   --
   --    // J = [-I -r1_skew I r2_skew]
   --    // r_skew = [-ry; rx]
   --
   --    // Matlab
   --    // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
   --    //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
   --    //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]
   --
   --    float mA = m_invMassA, mB = m_invMassB;
   --    float iA = m_invIA, iB = m_invIB;
   --
   --    // Upper 2 by 2 of K for point to point
   --    b2Mat22 K;
   --    K.ex.x = mA + mB + iA * m_rA.y * m_rA.y + iB * m_rB.y * m_rB.y;
   --    K.ex.y = -iA * m_rA.x * m_rA.y - iB * m_rB.x * m_rB.y;
   --    K.ey.x = K.ex.y;
   --    K.ey.y = mA + mB + iA * m_rA.x * m_rA.x + iB * m_rB.x * m_rB.x;
   --
   --    m_linearMass = K.GetInverse();
   --
   --    m_angularMass = iA + iB;
   --    if (m_angularMass > 0.0f)
   --    {
   --       m_angularMass = 1.0f / m_angularMass;
   --    }
   --
   --    m_linearError = cB + m_rB - cA - m_rA;
   --    m_angularError = aB - aA - m_angularOffset;
   --
   --    if (data.step.warmStarting)
   --    {
   --       // Scale impulses to support a variable time step.
   --       m_linearImpulse *= data.step.dtRatio;
   --       m_angularImpulse *= data.step.dtRatio;
   --
   --       b2Vec2 P(m_linearImpulse.x, m_linearImpulse.y);
   --       vA -= mA * P;
   --       wA -= iA * (b2Cross(m_rA, P) + m_angularImpulse);
   --       vB += mB * P;
   --       wB += iB * (b2Cross(m_rB, P) + m_angularImpulse);
   --    }
   --    else
   --    {
   --       m_linearImpulse.SetZero();
   --       m_angularImpulse = 0.0f;
   --    }
   --
   --    data.velocities[m_indexA].v = vA;
   --    data.velocities[m_indexA].w = wA;
   --    data.velocities[m_indexB].v = vB;
   --    data.velocities[m_indexB].w = wB;
   --  }
   --

   overriding
   procedure initVelocityConstraints (Self : in out b2MotorJoint;   Data : in b2SolverData)
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

         mA : Real;
         mB : Real;
         iA : Real;
         iB : Real;

      begin
         -- Compute the effective mass matrix.
         --
         Self.m_rA := b2Mul (qA, Self.m_linearOffset - Self.m_localCenterA);
         Self.m_rB := b2Mul (qB,                     - Self.m_localCenterB);

         -- J = (-I -r1_skew I r2_skew)
         -- r_skew = (-ry; rx)
         --

         -- Matlab
         -- K = ( mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB)
         --     (  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB)
         --     (          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB)
         --

         mA := Self.m_invMassA;
         mB := Self.m_invMassB;
         iA := Self.m_invIA;
         iB := Self.m_invIB;

         -- Upper 2 by 2 of K for point to point
         declare
            K  : b2Mat22;
         begin
            K.ex.x :=   mA + mB
                      + iA * Self.m_rA.y * Self.m_rA.y
                      + iB * Self.m_rB.y * Self.m_rB.y;

            K.ex.y :=  -iA * Self.m_rA.x * Self.m_rA.y
                      - iB * Self.m_rB.x * Self.m_rB.y;

            K.ey.x := K.ex.y;
            K.ey.y :=   mA + mB
                      + iA * Self.m_rA.x * Self.m_rA.x
                      + iB * Self.m_rB.x * Self.m_rB.x;

            Self.m_linearMass := getInverse (K);
         end;

         Self.m_angularMass := iA + iB;

         if Self.m_angularMass > 0.0
         then
            Self.m_angularMass := 1.0 / Self.m_angularMass;
         end if;

         Self.m_linearError  := cB + Self.m_rB - cA - Self.m_rA;
         Self.m_angularError := aB - aA - Self.m_angularOffset;

         if data.step.warmStarting
         then
            -- Scale impulses to support a variable time step.
            Self.m_linearImpulse  := Self.m_linearImpulse  * data.step.dtRatio;
            Self.m_angularImpulse := Self.m_angularImpulse * data.step.dtRatio;

            declare
               P : constant b2Vec2 := (Self.m_linearImpulse.x,
                                       Self.m_linearImpulse.y);
            begin
               vA := vA  - mA * P;
               wA := wA  -  iA * (b2Cross (Self.m_rA, P)  +  Self.m_angularImpulse);
               vB := vB  +  mB * P;
               wB := wB  +  iB * (b2Cross (Self.m_rB, P)  +  Self.m_angularImpulse);
            end;

         else
            Self.m_linearImpulse  := (0.0, 0.0);
            Self.m_angularImpulse := 0.0;
         end if;

         data.velocities (Self.m_indexA).v := vA;
         data.velocities (Self.m_indexA).w := wA;
         data.velocities (Self.m_indexB).v := vB;
         data.velocities (Self.m_indexB).w := wB;
      end;
end initVelocityConstraints;



   --  void b2MotorJoint::SolveVelocityConstraints(const b2SolverData& data)
   --  {
   --    b2Vec2 vA = data.velocities[m_indexA].v;
   --    float wA = data.velocities[m_indexA].w;
   --    b2Vec2 vB = data.velocities[m_indexB].v;
   --    float wB = data.velocities[m_indexB].w;
   --
   --    float mA = m_invMassA, mB = m_invMassB;
   --    float iA = m_invIA, iB = m_invIB;
   --
   --    float h = data.step.dt;
   --    float inv_h = data.step.inv_dt;
   --
   --    // Solve angular friction
   --    {
   --       float Cdot = wB - wA + inv_h * m_correctionFactor * m_angularError;
   --       float impulse = -m_angularMass * Cdot;
   --
   --       float oldImpulse = m_angularImpulse;
   --       float maxImpulse = h * m_maxTorque;
   --       m_angularImpulse = b2Clamp(m_angularImpulse + impulse, -maxImpulse, maxImpulse);
   --       impulse = m_angularImpulse - oldImpulse;
   --
   --       wA -= iA * impulse;
   --       wB += iB * impulse;
   --    }
   --
   --    // Solve linear friction
   --    {
   --       b2Vec2 Cdot = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA) + inv_h * m_correctionFactor * m_linearError;
   --
   --       b2Vec2 impulse = -b2Mul(m_linearMass, Cdot);
   --       b2Vec2 oldImpulse = m_linearImpulse;
   --       m_linearImpulse += impulse;
   --
   --       float maxImpulse = h * m_maxForce;
   --
   --       if (m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse)
   --       {
   --          m_linearImpulse.Normalize();
   --          m_linearImpulse *= maxImpulse;
   --       }
   --
   --       impulse = m_linearImpulse - oldImpulse;
   --
   --       vA -= mA * impulse;
   --       wA -= iA * b2Cross(m_rA, impulse);
   --
   --       vB += mB * impulse;
   --       wB += iB * b2Cross(m_rB, impulse);
   --    }
   --
   --    data.velocities[m_indexA].v = vA;
   --    data.velocities[m_indexA].w = wA;
   --    data.velocities[m_indexB].v = vB;
   --    data.velocities[m_indexB].w = wB;
   --  }
   --

   overriding
   procedure solveVelocityConstraints (Self : in out b2MotorJoint;   Data : in b2SolverData)
   is
      vA    :          b2Vec2 := data.velocities (Self.m_indexA).v;
      wA    :          Real   := data.velocities (Self.m_indexA).w;
      vB    :          b2Vec2 := data.velocities (Self.m_indexB).v;
      wB    :          Real   := data.velocities (Self.m_indexB).w;

      mA    : constant Real   := Self.m_invMassA;
      mB    : constant Real   := Self.m_invMassB;
      iA    : constant Real   := Self.m_invIA;
      iB    : constant Real   := Self.m_invIB;

      h     : constant Real   := data.step.dt;
      inv_h : constant Real   := data.step.inv_dt;

   begin
      -- Solve angular friction.
      declare
         Cdot       : constant Real :=   wB - wA
                                       + inv_h * Self.m_correctionFactor * Self.m_angularError;
         impulse    :          Real := -Self.m_angularMass * Cdot;

         oldImpulse : constant Real := Self.m_angularImpulse;
         maxImpulse : constant Real := h * Self.m_maxTorque;
      begin
         Self.m_angularImpulse := b2Clamp ( Self.m_angularImpulse + impulse,
                                           -maxImpulse,
                                            maxImpulse);
         impulse := Self.m_angularImpulse - oldImpulse;

         wA := wA - iA * impulse;
         wB := wB + iB * impulse;
      end;

     -- Solve linear friction.
     declare
         Cdot       : constant b2Vec2 :=   vB + b2Cross (wB, Self.m_rB)
                                         - vA - b2Cross (wA, Self.m_rA)
                                         + inv_h * Self.m_correctionFactor * Self.m_linearError;

         impulse    :          b2Vec2 := -b2Mul (Self.m_linearMass, Cdot);
         oldImpulse : constant b2Vec2 := Self.m_linearImpulse;

         maxImpulse : constant Real   := h * Self.m_maxForce;
      begin
         Self.m_linearImpulse := Self.m_linearImpulse + impulse;

         if LengthSquared (Self.m_linearImpulse) > maxImpulse * maxImpulse
         then
            normalize (Self.m_linearImpulse);
            Self.m_linearImpulse := Self.m_linearImpulse * maxImpulse;
         end if;

         impulse := Self.m_linearImpulse - oldImpulse;

         vA := vA - mA * impulse;
         wA := wA - iA * b2Cross (Self.m_rA, impulse);

         vB := vB + mB * impulse;
         wB := wB + iB * b2Cross (Self.m_rB, impulse);
      end;

      data.velocities (Self.m_indexA).v := vA;
      data.velocities (Self.m_indexA).w := wA;
      data.velocities (Self.m_indexB).v := vB;
      data.velocities (Self.m_indexB).w := wB;
   end solveVelocityConstraints;



   --  bool b2MotorJoint::SolvePositionConstraints(const b2SolverData& data)
   --  {
   --    B2_NOT_USED(data);
   --
   --    return true;
   --  }
   --

   overriding
   function solvePositionConstraints (Self : in out b2MotorJoint;   Data : in b2SolverData) return Boolean
   is
   begin
      return True;
   end solvePositionConstraints;


end box2d.b2_Joint.b2_motor_Joint;
