with
     box2d.b2_Draw,
     box2d.b2_time_Step,
     box2d.b2_Common;


package body box2d.b2_Joint.b2_revolute_Joint
is
   --    b2RevoluteJointDef()
   --    {
   --       type = e_revoluteJoint;
   --       localAnchorA.Set(0.0f, 0.0f);
   --       localAnchorB.Set(0.0f, 0.0f);
   --       referenceAngle = 0.0f;
   --       lowerAngle = 0.0f;
   --       upperAngle = 0.0f;
   --       maxMotorTorque = 0.0f;
   --       motorSpeed = 0.0f;
   --       enableLimit = false;
   --       enableMotor = false;
   --    }
   --

   function  to_b2RevoluteJointDef return b2RevoluteJointDef
   is
      Self : b2RevoluteJointDef;
   begin
      Self.Kind           := e_revoluteJoint;
      Self.localAnchorA   := (0.0, 0.0);
      Self.localAnchorB   := (0.0, 0.0);
      Self.referenceAngle := 0.0;
      Self.lowerAngle     := 0.0;
      Self.upperAngle     := 0.0;
      Self.maxMotorTorque := 0.0;
      Self.motorSpeed     := 0.0;
      Self.enableLimit    := False;
      Self.enableMotor    := False;

      return Self;
   end to_b2RevoluteJointDef;



   --  // Point-to-point constraint
   --  // C = p2 - p1
   --  // Cdot = v2 - v1
   --  //      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
   --  // J = [-I -r1_skew I r2_skew ]
   --  // Identity used:
   --  // w k % (rx i + ry j) = w * (-ry i + rx j)
   --
   --  // Motor constraint
   --  // Cdot = w2 - w1
   --  // J = [0 0 -1 0 0 1]
   --  // K = invI1 + invI2
   --
   --  void b2RevoluteJointDef::Initialize(b2Body* bA, b2Body* bB, const b2Vec2& anchor)
   --  {
   --    bodyA = bA;
   --    bodyB = bB;
   --    localAnchorA = bodyA->GetLocalPoint(anchor);
   --    localAnchorB = bodyB->GetLocalPoint(anchor);
   --    referenceAngle = bodyB->GetAngle() - bodyA->GetAngle();
   --  }
   --

   procedure initialize (Self : out b2RevoluteJointDef;   bodyA, bodyB   : access b2Body;
                                                          Anchor         : in     b2Vec2)
   is
   begin
      Self.bodyA          := bodyA;
      Self.bodyB          := bodyB;
      Self.localAnchorA   := bodyA.getLocalPoint (Anchor);
      Self.localAnchorB   := bodyB.getLocalPoint (Anchor);
      Self.referenceAngle := bodyB.getAngle - bodyA.getAngle;
   end initialize;






   -------------------
   --- b2RevoluteJoint
   --


   --  b2Vec2 b2RevoluteJoint::GetAnchorA() const
   --  {
   --    return m_bodyA->GetWorldPoint(m_localAnchorA);
   --  }
   --

   overriding
   function getAnchorA (Self : in b2RevoluteJoint) return b2Vec2
   is
   begin
      return Self.m_bodyA.getWorldPoint (Self.m_localAnchorA);
   end getAnchorA;



   --  b2Vec2 b2RevoluteJoint::GetAnchorB() const
   --  {
   --    return m_bodyB->GetWorldPoint(m_localAnchorB);
   --  }
   --

   overriding
   function getAnchorB (Self : in b2RevoluteJoint) return b2Vec2
   is
   begin
      return Self.m_bodyB.getWorldPoint (Self.m_localAnchorB);
   end getAnchorB;



   --  b2Vec2 b2RevoluteJoint::GetReactionForce(float inv_dt) const
   --  {
   --    b2Vec2 P(m_impulse.x, m_impulse.y);
   --    return inv_dt * P;
   --  }
   --

   overriding
   function getReactionForce (Self : in b2RevoluteJoint;   inv_dt : in Real) return b2Vec2
   is
      P : constant b2Vec2 := (Self.m_impulse.x,
                              Self.m_impulse.y);
   begin
      return inv_dt * P;
   end getReactionForce;



   --  float b2RevoluteJoint::GetReactionTorque(float inv_dt) const
   --  {
   --    return inv_dt * (m_motorImpulse + m_lowerImpulse - m_upperImpulse);
   --  }
   --

   overriding
   function getReactionTorque (Self : in b2RevoluteJoint;   inv_dt : in Real) return Real
   is
   begin
      return inv_dt * (Self.m_motorImpulse + Self.m_lowerImpulse - Self.m_upperImpulse);
   end getReactionTorque;



   --    The local anchor point relative to bodyA's origin.
   --    const b2Vec2& GetLocalAnchorA() const { return m_localAnchorA; }
   --

   function getLocalAnchorA (Self : in b2RevoluteJoint) return b2Vec2
   is
   begin
      return Self.m_localAnchorA;
   end getLocalAnchorA;


   --    The local anchor point relative to bodyB's origin.
   --    const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }
   --

   function getLocalAnchorB (Self : in b2RevoluteJoint) return b2Vec2
   is
   begin
      return Self.m_localAnchorB;
   end getLocalAnchorB;



   --    Get the reference angle.
   --
   --    float GetReferenceAngle() const { return m_referenceAngle; }
   --

   function getReferenceAngle (Self : in b2RevoluteJoint) return Real
   is
   begin
      return Self.m_referenceAngle;
   end getReferenceAngle;



   --  float b2RevoluteJoint::GetJointAngle() const
   --  {
   --    b2Body* bA = m_bodyA;
   --    b2Body* bB = m_bodyB;
   --    return bB->m_sweep.a - bA->m_sweep.a - m_referenceAngle;
   --  }
   --

   function getJointAngle (Self : in b2RevoluteJoint) return Real
   is
      bA : constant access b2Body := Self.m_bodyA;
      bB : constant access b2Body := Self.m_bodyB;
   begin
      return bB.m_sweep.a - bA.m_sweep.a - Self.m_referenceAngle;
   end getJointAngle;



   --  float b2RevoluteJoint::GetJointSpeed() const
   --  {
   --    b2Body* bA = m_bodyA;
   --    b2Body* bB = m_bodyB;
   --    return bB->m_angularVelocity - bA->m_angularVelocity;
   --  }
   --

   function getJointSpeed (Self : in b2RevoluteJoint) return Real
   is
      bA : constant access b2Body := Self.m_bodyA;
      bB : constant access b2Body := Self.m_bodyB;
   begin
      return bB.getAngularVelocity - bA.getAngularVelocity;
   end getJointSpeed;



   --  bool b2RevoluteJoint::IsLimitEnabled() const
   --  {
   --    return m_enableLimit;
   --  }
   --

   function isLimitEnabled (Self : in b2RevoluteJoint) return Boolean
   is
   begin
      return Self.m_enableLimit;
   end isLimitEnabled;



   --  void b2RevoluteJoint::EnableLimit(bool flag)
   --  {
   --    if (flag != m_enableLimit)
   --    {
   --       m_bodyA->SetAwake(true);
   --       m_bodyB->SetAwake(true);
   --       m_enableLimit = flag;
   --       m_lowerImpulse = 0.0f;
   --       m_upperImpulse = 0.0f;
   --    }
   --  }
   --

   procedure enableLimit (Self : in out b2RevoluteJoint;   Flag : in Boolean)
   is
   begin
      if Flag /= Self.m_enableLimit
      then
         Self.m_bodyA.setAwake (True);
         Self.m_bodyB.setAwake (True);
         Self.m_enableLimit  := Flag;
         Self.m_lowerImpulse := 0.0;
         Self.m_upperImpulse := 0.0;
      end if;
   end enableLimit;



   --  float b2RevoluteJoint::GetLowerLimit() const
   --  {
   --    return m_lowerAngle;
   --  }
   --

   function getLowerLimit (Self : in b2RevoluteJoint) return Real
   is
   begin
      return Self.m_lowerAngle;
   end getLowerLimit;



   --  float b2RevoluteJoint::GetUpperLimit() const
   --  {
   --    return m_upperAngle;
   --  }
   --

   function getUpperLimit (Self : in b2RevoluteJoint) return Real
   is
   begin
      return Self.m_upperAngle;
   end getUpperLimit;



   --  void b2RevoluteJoint::SetLimits(float lower, float upper)
   --  {
   --    b2Assert(lower <= upper);
   --
   --    if (lower != m_lowerAngle || upper != m_upperAngle)
   --    {
   --       m_bodyA->SetAwake(true);
   --       m_bodyB->SetAwake(true);
   --       m_lowerImpulse = 0.0f;
   --       m_upperImpulse = 0.0f;
   --       m_lowerAngle = lower;
   --       m_upperAngle = upper;
   --    }
   --  }
   --

   procedure setLimits (Self : in out b2RevoluteJoint;   Lower, Upper : in Real)
   is
     pragma assert (Lower <= Upper);
   begin
      if   Lower /= Self.m_lowerAngle
        or Upper /= Self.m_upperAngle
      then
         Self.m_bodyA.setAwake (True);
         Self.m_bodyB.setAwake (True);
         Self.m_lowerImpulse   := 0.0;
         Self.m_upperImpulse   := 0.0;
         Self.m_lowerAngle     := Lower;
         Self.m_upperAngle     := Upper;
      end if;
   end setLimits;



   --  bool b2RevoluteJoint::IsMotorEnabled() const
   --  {
   --    return m_enableMotor;
   --  }
   --

   function isMotorEnabled (Self : in b2RevoluteJoint) return Boolean
   is
   begin
      return Self.m_enableMotor;
   end isMotorEnabled;



   --  void b2RevoluteJoint::EnableMotor(bool flag)
   --  {
   --    if (flag != m_enableMotor)
   --    {
   --       m_bodyA->SetAwake(true);
   --       m_bodyB->SetAwake(true);
   --       m_enableMotor = flag;
   --    }
   --  }
   --

   procedure enableMotor (Self : in out b2RevoluteJoint;   Flag : in Boolean)
   is
   begin
      if Flag /= Self.m_enableMotor
      then
         Self.m_bodyA.setAwake (True);
         Self.m_bodyB.setAwake (True);

         Self.m_enableMotor := Flag;
      end if;
   end enableMotor;



   --  void b2RevoluteJoint::SetMotorSpeed(float speed)
   --  {
   --    if (speed != m_motorSpeed)
   --    {
   --       m_bodyA->SetAwake(true);
   --       m_bodyB->SetAwake(true);
   --       m_motorSpeed = speed;
   --    }
   --  }
   --

   procedure setMotorSpeed (Self : in out b2RevoluteJoint;   Speed : in Real)
   is
   begin
     if Speed /= Self.m_motorSpeed
     then
         Self.m_bodyA.setAwake (True);
         Self.m_bodyB.setAwake (True);

         Self.m_motorSpeed := Speed;
      end if;
   end setMotorSpeed;



   --    Get the motor speed in radians per second.
   --
   --    inline float b2RevoluteJoint::GetMotorSpeed() const
   --    {
   --      return m_motorSpeed;
   --    }
   --

   function getMotorSpeed (Self : in b2RevoluteJoint) return Real
   is
   begin
      return Self.m_motorSpeed;
   end getMotorSpeed;



   --  void b2RevoluteJoint::SetMaxMotorTorque(float torque)
   --  {
   --    if (torque != m_maxMotorTorque)
   --    {
   --       m_bodyA->SetAwake(true);
   --       m_bodyB->SetAwake(true);
   --       m_maxMotorTorque = torque;
   --    }
   --  }
   --

   procedure setMaxMotorTorque (Self : in out b2RevoluteJoint;   Torque : in Real)
   is
   begin
      if Torque /= Self.m_maxMotorTorque
      then
         Self.m_bodyA.setAwake (True);
         Self.m_bodyB.setAwake (True);

         Self.m_maxMotorTorque := torque;
      end if;
   end setMaxMotorTorque;



   --    float GetMaxMotorTorque() const { return m_maxMotorTorque; }
   --

   function getMaxMotorTorque (Self : in b2RevoluteJoint) return Real
   is
   begin
      return Self.m_maxMotorTorque;
   end getMaxMotorTorque;



   --  float b2RevoluteJoint::GetMotorTorque(float inv_dt) const
   --  {
   --    return inv_dt * m_motorImpulse;
   --  }
   --

   function getMotorTorque (Self : in b2RevoluteJoint;   inv_dt : Real) return Real
   is
   begin
      return inv_dt * Self.m_motorImpulse;
   end getMotorTorque;



   --  void b2RevoluteJoint::Draw(b2Draw* draw) const
   --  {
   --    const b2Transform& xfA = m_bodyA->GetTransform();
   --    const b2Transform& xfB = m_bodyB->GetTransform();
   --    b2Vec2 pA = b2Mul(xfA, m_localAnchorA);
   --    b2Vec2 pB = b2Mul(xfB, m_localAnchorB);
   --
   --    b2Color c1(0.7f, 0.7f, 0.7f);
   --    b2Color c2(0.3f, 0.9f, 0.3f);
   --    b2Color c3(0.9f, 0.3f, 0.3f);
   --    b2Color c4(0.3f, 0.3f, 0.9f);
   --    b2Color c5(0.4f, 0.4f, 0.4f);
   --
   --    draw->DrawPoint(pA, 5.0f, c4);
   --    draw->DrawPoint(pB, 5.0f, c5);
   --
   --    float aA = m_bodyA->GetAngle();
   --    float aB = m_bodyB->GetAngle();
   --    float angle = aB - aA - m_referenceAngle;
   --
   --    const float L = 0.5f;
   --
   --    b2Vec2 r = L * b2Vec2(cosf(angle), sinf(angle));
   --    draw->DrawSegment(pB, pB + r, c1);
   --    draw->DrawCircle(pB, L, c1);
   --
   --    if (m_enableLimit)
   --    {
   --       b2Vec2 rlo = L * b2Vec2(cosf(m_lowerAngle), sinf(m_lowerAngle));
   --       b2Vec2 rhi = L * b2Vec2(cosf(m_upperAngle), sinf(m_upperAngle));
   --
   --       draw->DrawSegment(pB, pB + rlo, c2);
   --       draw->DrawSegment(pB, pB + rhi, c3);
   --    }
   --
   --    b2Color color(0.5f, 0.8f, 0.8f);
   --    draw->DrawSegment(xfA.p, pA, color);
   --    draw->DrawSegment(pA, pB, color);
   --    draw->DrawSegment(xfB.p, pB, color);
   --  }

   overriding
   procedure draw (Self : in b2RevoluteJoint;   Draw : access b2Draw'Class)
   is
      use b2_Math.Functions;

      xfA : constant b2Transform := Self.m_bodyA.getTransform;
      xfB : constant b2Transform := Self.m_bodyB.getTransform;

      pA : constant b2Vec2  := b2Mul (xfA, Self.m_localAnchorA);
      pB : constant b2Vec2  := b2Mul (xfB, Self.m_localAnchorB);

      c1 : constant b2Color := to_b2Color (0.7, 0.7, 0.7);
      c2 : constant b2Color := to_b2Color (0.3, 0.9, 0.3);
      c3 : constant b2Color := to_b2Color (0.9, 0.3, 0.3);
      c4 : constant b2Color := to_b2Color (0.3, 0.3, 0.9);
      c5 : constant b2Color := to_b2Color (0.4, 0.4, 0.4);

      L  : constant Real   := 0.5;

   begin
      draw_any (Self, Draw);

      draw.drawPoint (pA, 5.0, c4);
      draw.drawPoint (pB, 5.0, c5);

      declare
         aA    : constant Real   := Self.m_bodyA.getAngle;
         aB    : constant Real   := Self.m_bodyB.getAngle;
         angle : constant Real   := aB - aA - Self.m_referenceAngle;

         r     : constant b2Vec2 := L * b2Vec2' (cos (angle),
                                                 sin (angle));
      begin
         draw.drawSegment (pB,  pB + r,  c1);
         draw.drawCircle  (pB,  L,       c1);
      end;

      if Self.m_enableLimit
      then
         declare
            rlo : constant b2Vec2 := L * b2Vec2' (cos (Self.m_lowerAngle),
                                         sin (Self.m_lowerAngle));
            rhi : constant b2Vec2 := L * b2Vec2' (cos (Self.m_upperAngle),
                                         sin (Self.m_upperAngle));
         begin
            draw.drawSegment (pB,  pB + rlo,  c2);
            draw.drawSegment (pB,  pB + rhi,  c3);
         end;
      end if;

      declare
         color : constant b2Color := to_b2Color (0.5, 0.8, 0.8);
      begin
         draw.drawSegment (xfA.p, pA, color);
         draw.drawSegment (pA,    pB, color);
         draw.drawSegment (xfB.p, pB, color);
         end;
   end draw;



   --  void b2RevoluteJoint::Dump()
   --  {
   --    int32 indexA = m_bodyA->m_islandIndex;
   --    int32 indexB = m_bodyB->m_islandIndex;
   --
   --    b2Dump("  b2RevoluteJointDef jd;\n");
   --    b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
   --    b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
   --    b2Dump("  jd.collideConnected = bool(%d);\n", m_collideConnected);
   --    b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", m_localAnchorA.x, m_localAnchorA.y);
   --    b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", m_localAnchorB.x, m_localAnchorB.y);
   --    b2Dump("  jd.referenceAngle = %.9g;\n", m_referenceAngle);
   --    b2Dump("  jd.enableLimit = bool(%d);\n", m_enableLimit);
   --    b2Dump("  jd.lowerAngle = %.9g;\n", m_lowerAngle);
   --    b2Dump("  jd.upperAngle = %.9g;\n", m_upperAngle);
   --    b2Dump("  jd.enableMotor = bool(%d);\n", m_enableMotor);
   --    b2Dump("  jd.motorSpeed = %.9g;\n", m_motorSpeed);
   --    b2Dump("  jd.maxMotorTorque = %.9g;\n", m_maxMotorTorque);
   --    b2Dump("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
   --  }
   --

   overriding
   procedure dump (Self : in b2RevoluteJoint)
   is
      use b2_Common;

      indexA : constant Integer := Self.m_bodyA.m_islandIndex;
      indexB : constant Integer := Self.m_bodyB.m_islandIndex;
   begin
      b2Dump ("  b2RevoluteJointDef jd;");
      b2Dump ("  jd.bodyA = bodies[%d];"                  & indexA'Image);
      b2Dump ("  jd.bodyB = bodies[%d];"                  & indexB'Image);
      b2Dump ("  jd.collideConnected = bool(%d);"         & Self.m_collideConnected'Image);
      b2Dump ("  jd.localAnchorA.Set(%.9g, %.9g);"        & Self.m_localAnchorA.x'Image & Self.m_localAnchorA.y'Image);
      b2Dump ("  jd.localAnchorB.Set(%.9g, %.9g);"        & Self.m_localAnchorB.x'Image & Self.m_localAnchorB.y'Image);
      b2Dump ("  jd.referenceAngle = %.9g;"               & Self.m_referenceAngle'Image);
      b2Dump ("  jd.enableLimit = bool(%d);"              & Self.m_enableLimit'Image);
      b2Dump ("  jd.lowerAngle = %.9g;"                   & Self.m_lowerAngle'Image);
      b2Dump ("  jd.upperAngle = %.9g;"                   & Self.m_upperAngle'Image);
      b2Dump ("  jd.enableMotor = bool(%d);"              & Self.m_enableMotor'Image);
      b2Dump ("  jd.motorSpeed = %.9g;"                   & Self.m_motorSpeed'Image);
      b2Dump ("  jd.maxMotorTorque = %.9g;"               & Self.m_maxMotorTorque'Image);
      b2Dump ("  joints[%d] = m_world->CreateJoint(&jd);" & Self.m_index'Image);
   end dump;




   --------------
   --  protected:
   --
   --    friend class b2Joint;
   --    friend class b2GearJoint;
   --
   --    b2RevoluteJoint(const b2RevoluteJointDef* def);
   --

   package body Forge
   is

      --  b2RevoluteJoint::b2RevoluteJoint(const b2RevoluteJointDef* def)
      --  : b2Joint(def)
      --  {
      --    m_localAnchorA = def->localAnchorA;
      --    m_localAnchorB = def->localAnchorB;
      --    m_referenceAngle = def->referenceAngle;
      --
      --    m_impulse.SetZero();
      --    m_axialMass = 0.0f;
      --    m_motorImpulse = 0.0f;
      --    m_lowerImpulse = 0.0f;
      --    m_upperImpulse = 0.0f;
      --
      --    m_lowerAngle = def->lowerAngle;
      --    m_upperAngle = def->upperAngle;
      --    m_maxMotorTorque = def->maxMotorTorque;
      --    m_motorSpeed = def->motorSpeed;
      --    m_enableLimit = def->enableLimit;
      --    m_enableMotor = def->enableMotor;
      --
      --    m_angle = 0.0f;
      --  }
      --

      function to_b2RevoluteJoint (Def : in b2RevoluteJointDef) return b2RevoluteJoint
      is
         Self : b2RevoluteJoint;
      begin

         b2_Joint.Forge.define (Self, Def);

         Self.m_localAnchorA   := def.localAnchorA;
         Self.m_localAnchorB   := def.localAnchorB;
         Self.m_referenceAngle := def.referenceAngle;

         Self.m_impulse        := (0.0, 0.0);
         Self.m_axialMass      := 0.0;
         Self.m_motorImpulse   := 0.0;
         Self.m_lowerImpulse   := 0.0;
         Self.m_upperImpulse   := 0.0;

         Self.m_lowerAngle     := def.lowerAngle;
         Self.m_upperAngle     := def.upperAngle;
         Self.m_maxMotorTorque := def.maxMotorTorque;
         Self.m_motorSpeed     := def.motorSpeed;
         Self.m_enableLimit    := def.enableLimit;
         Self.m_enableMotor    := def.enableMotor;

         Self.m_angle := 0.0;

         return Self;
      end to_b2RevoluteJoint;

   end Forge;




   --  void b2RevoluteJoint::InitVelocityConstraints(const b2SolverData& data)
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
   --    float aA = data.positions[m_indexA].a;
   --    b2Vec2 vA = data.velocities[m_indexA].v;
   --    float wA = data.velocities[m_indexA].w;
   --
   --    float aB = data.positions[m_indexB].a;
   --    b2Vec2 vB = data.velocities[m_indexB].v;
   --    float wB = data.velocities[m_indexB].w;
   --
   --    b2Rot qA(aA), qB(aB);
   --
   --    m_rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
   --    m_rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
   --
   --    // J = [-I -r1_skew I r2_skew]
   --    // r_skew = [-ry; rx]
   --
   --    // Matlab
   --    // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x]
   --    //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB]
   --
   --    float mA = m_invMassA, mB = m_invMassB;
   --    float iA = m_invIA, iB = m_invIB;
   --
   --    m_K.ex.x = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB;
   --    m_K.ey.x = -m_rA.y * m_rA.x * iA - m_rB.y * m_rB.x * iB;
   --    m_K.ex.y = m_K.ey.x;
   --    m_K.ey.y = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB;
   --
   --    m_axialMass = iA + iB;
   --    bool fixedRotation;
   --    if (m_axialMass > 0.0f)
   --    {
   --       m_axialMass = 1.0f / m_axialMass;
   --       fixedRotation = false;
   --    }
   --    else
   --    {
   --       fixedRotation = true;
   --    }
   --
   --    m_angle = aB - aA - m_referenceAngle;
   --    if (m_enableLimit == false || fixedRotation)
   --    {
   --       m_lowerImpulse = 0.0f;
   --       m_upperImpulse = 0.0f;
   --    }
   --
   --    if (m_enableMotor == false || fixedRotation)
   --    {
   --       m_motorImpulse = 0.0f;
   --    }
   --
   --    if (data.step.warmStarting)
   --    {
   --       // Scale impulses to support a variable time step.
   --       m_impulse *= data.step.dtRatio;
   --       m_motorImpulse *= data.step.dtRatio;
   --       m_lowerImpulse *= data.step.dtRatio;
   --       m_upperImpulse *= data.step.dtRatio;
   --
   --       float axialImpulse = m_motorImpulse + m_lowerImpulse - m_upperImpulse;
   --       b2Vec2 P(m_impulse.x, m_impulse.y);
   --
   --       vA -= mA * P;
   --       wA -= iA * (b2Cross(m_rA, P) + axialImpulse);
   --
   --       vB += mB * P;
   --       wB += iB * (b2Cross(m_rB, P) + axialImpulse);
   --    }
   --    else
   --    {
   --       m_impulse.SetZero();
   --       m_motorImpulse = 0.0f;
   --       m_lowerImpulse = 0.0f;
   --       m_upperImpulse = 0.0f;
   --    }
   --
   --    data.velocities[m_indexA].v = vA;
   --    data.velocities[m_indexA].w = wA;
   --    data.velocities[m_indexB].v = vB;
   --    data.velocities[m_indexB].w = wB;
   --  }
   --

   overriding
   procedure initVelocityConstraints (Self : in out b2RevoluteJoint;   Data : in b2SolverData)
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
         aA : constant Real   := data.positions  (Self.m_indexA).a;
         vA :          b2Vec2 := data.velocities (Self.m_indexA).v;
         wA :          Real   := data.velocities (Self.m_indexA).w;

         aB : constant Real   := data.positions  (Self.m_indexB).a;
         vB :          b2Vec2 := data.velocities (Self.m_indexB).v;
         wB :          Real   := data.velocities (Self.m_indexB).w;

         qA : constant b2Rot  := to_b2Rot (aA);
         qB : constant b2Rot  := to_b2Rot (aB);

         mA, mB : Real;
         iA, iB : Real;

         fixedRotation : Boolean;
      begin
         Self.m_rA := b2Mul (qA,  Self.m_localAnchorA - Self.m_localCenterA);
         Self.m_rB := b2Mul (qB,  Self.m_localAnchorB - Self.m_localCenterB);

         -- J      = [-I -r1_skew I r2_skew]
         -- r_skew = [-ry; rx]
         --

         -- Matlab
         -- K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x]
         --     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB]
         --

         mA := Self.m_invMassA;
         mB := Self.m_invMassB;
         iA := Self.m_invIA;
         iB := Self.m_invIB;

         Self.m_K.ex.x :=   mA + mB
                          + Self.m_rA.y * Self.m_rA.y * iA
                          + Self.m_rB.y * Self.m_rB.y * iB;

         Self.m_K.ey.x :=   -Self.m_rA.y * Self.m_rA.x * iA
                          -  Self.m_rB.y * Self.m_rB.x * iB;

         Self.m_K.ex.y := Self.m_K.ey.x;

         Self.m_K.ey.y :=   mA + mB
                          + Self.m_rA.x * Self.m_rA.x * iA
                          + Self.m_rB.x * Self.m_rB.x * iB;


         Self.m_axialMass := iA + iB;

         if Self.m_axialMass > 0.0
         then
            Self.m_axialMass := 1.0 / Self.m_axialMass;
            fixedRotation    := False;
         else
            fixedRotation    := True;
         end if;


         Self.m_angle := aB - aA - Self.m_referenceAngle;

         if  (Self.m_enableLimit = False)
           or fixedRotation
         then
            Self.m_lowerImpulse := 0.0;
            Self.m_upperImpulse := 0.0;
         end if;


         if  (Self.m_enableMotor = False)
           or fixedRotation
         then
            Self.m_motorImpulse := 0.0;
         end if;


         if data.step.warmStarting
         then
            -- Scale impulses to support a variable time step.
            --
            Self.m_impulse      := Self.m_impulse      * data.step.dtRatio;
            Self.m_motorImpulse := Self.m_motorImpulse * data.step.dtRatio;
            Self.m_lowerImpulse := Self.m_lowerImpulse * data.step.dtRatio;
            Self.m_upperImpulse := Self.m_upperImpulse * data.step.dtRatio;

            declare
               axialImpulse : constant Real   := Self.m_motorImpulse + Self.m_lowerImpulse - Self.m_upperImpulse;
               P            : constant b2Vec2 := (Self.m_impulse.x,
                                                  Self.m_impulse.y);
            begin
               vA := vA - mA * P;
               wA := wA - iA * (b2Cross (Self.m_rA, P) + axialImpulse);

               vB := vB + mB * P;
               wB := wB + iB * (b2Cross (Self.m_rB, P) + axialImpulse);
            end;
         else
            Self.m_impulse      := (0.0, 0.0);
            Self.m_motorImpulse :=  0.0;
            Self.m_lowerImpulse :=  0.0;
            Self.m_upperImpulse :=  0.0;
         end if;

         data.velocities (Self.m_indexA).v := vA;
         data.velocities (Self.m_indexA).w := wA;
         data.velocities (Self.m_indexB).v := vB;
         data.velocities (Self.m_indexB).w := wB;
      end;
   end initVelocityConstraints;



   --  void b2RevoluteJoint::SolveVelocityConstraints(const b2SolverData& data)
   --  {
   --    b2Vec2 vA = data.velocities[m_indexA].v;
   --    float wA = data.velocities[m_indexA].w;
   --    b2Vec2 vB = data.velocities[m_indexB].v;
   --    float wB = data.velocities[m_indexB].w;
   --
   --    float mA = m_invMassA, mB = m_invMassB;
   --    float iA = m_invIA, iB = m_invIB;
   --
   --    bool fixedRotation = (iA + iB == 0.0f);
   --
   --    // Solve motor constraint.
   --    if (m_enableMotor && fixedRotation == false)
   --    {
   --       float Cdot = wB - wA - m_motorSpeed;
   --       float impulse = -m_axialMass * Cdot;
   --       float oldImpulse = m_motorImpulse;
   --       float maxImpulse = data.step.dt * m_maxMotorTorque;
   --       m_motorImpulse = b2Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
   --       impulse = m_motorImpulse - oldImpulse;
   --
   --       wA -= iA * impulse;
   --       wB += iB * impulse;
   --    }
   --
   --    if (m_enableLimit && fixedRotation == false)
   --    {
   --       // Lower limit
   --       {
   --          float C = m_angle - m_lowerAngle;
   --          float Cdot = wB - wA;
   --          float impulse = -m_axialMass * (Cdot + b2Max(C, 0.0f) * data.step.inv_dt);
   --          float oldImpulse = m_lowerImpulse;
   --          m_lowerImpulse = b2Max(m_lowerImpulse + impulse, 0.0f);
   --          impulse = m_lowerImpulse - oldImpulse;
   --
   --          wA -= iA * impulse;
   --          wB += iB * impulse;
   --       }
   --
   --       // Upper limit
   --       // Note: signs are flipped to keep C positive when the constraint is satisfied.
   --       // This also keeps the impulse positive when the limit is active.
   --       {
   --          float C = m_upperAngle - m_angle;
   --          float Cdot = wA - wB;
   --          float impulse = -m_axialMass * (Cdot + b2Max(C, 0.0f) * data.step.inv_dt);
   --          float oldImpulse = m_upperImpulse;
   --          m_upperImpulse = b2Max(m_upperImpulse + impulse, 0.0f);
   --          impulse = m_upperImpulse - oldImpulse;
   --
   --          wA += iA * impulse;
   --          wB -= iB * impulse;
   --       }
   --    }
   --
   --    // Solve point-to-point constraint
   --    {
   --       b2Vec2 Cdot = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA);
   --       b2Vec2 impulse = m_K.Solve(-Cdot);
   --
   --       m_impulse.x += impulse.x;
   --       m_impulse.y += impulse.y;
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
   procedure solveVelocityConstraints (Self : in out b2RevoluteJoint;   Data : in b2SolverData)
   is
      vA : b2Vec2 := data.velocities (Self.m_indexA).v;
      wA : Real   := data.velocities (Self.m_indexA).w;
      vB : b2Vec2 := data.velocities (Self.m_indexB).v;
      wB : Real   := data.velocities (Self.m_indexB).w;

      mA : constant Real := Self.m_invMassA;
      mB : constant Real := Self.m_invMassB;

      iA : constant Real := Self.m_invIA;
      iB : constant Real := Self.m_invIB;

      fixedRotation : constant Boolean := iA + iB = 0.0;

   begin
      -- Solve motor constraint.
      --
      if     Self.m_enableMotor
        and (fixedRotation = False)
      then
         declare
            Cdot       : constant Real :=  wB - wA - Self.m_motorSpeed;
            impulse    :          Real := -Self.m_axialMass * Cdot;
            oldImpulse : constant Real :=  Self.m_motorImpulse;
            maxImpulse : constant Real :=  data.step.dt * Self.m_maxMotorTorque;
         begin
            Self.m_motorImpulse := b2Clamp ( Self.m_motorImpulse + impulse,
                                            -maxImpulse,
                                             maxImpulse);
            impulse := Self.m_motorImpulse - oldImpulse;
            wA      := wA - iA * impulse;
            wB      := wB + iB * impulse;
         end;
      end if;


      if     Self.m_enableLimit
        and (fixedRotation = False)
      then
         -- Lower limit.
         --
         declare
            C          : constant Real :=  Self.m_angle - Self.m_lowerAngle;
            Cdot       : constant Real :=  wB - wA;
            impulse    :          Real := -Self.m_axialMass * (Cdot + Real'max (C, 0.0) * data.step.inv_dt);
            oldImpulse : constant Real :=  Self.m_lowerImpulse;
         begin
            Self.m_lowerImpulse := Real'max (Self.m_lowerImpulse + impulse,  0.0);
            impulse             := Self.m_lowerImpulse - oldImpulse;

            wA := wA - iA * impulse;
            wB := wB + iB * impulse;
         end;

         -- Upper limit.
         --
         -- Note: signs are flipped to keep C positive when the constraint is satisfied.
         -- This also keeps the impulse positive when the limit is active.
         --
        declare
            C          : constant Real :=  Self.m_upperAngle - Self.m_angle;
            Cdot       : constant Real :=  wA - wB;
            impulse    :          Real := -Self.m_axialMass * (Cdot + Real'max (C, 0.0) * data.step.inv_dt);
            oldImpulse : constant Real :=  Self.m_upperImpulse;
         begin
           Self.m_upperImpulse := Real'max (Self.m_upperImpulse + impulse,  0.0);
           impulse             := Self.m_upperImpulse - oldImpulse;

           wA := wA + iA * impulse;
           wB := wB - iB * impulse;
        end;
     end if;


      -- Solve point-to-point constraint.
      --
      declare
         Cdot    : constant b2Vec2 :=   vB + b2Cross (wB, Self.m_rB)
                                      - vA - b2Cross (wA, Self.m_rA);
         impulse : constant b2Vec2 := solve (Self.m_K, -Cdot);
      begin
         Self.m_impulse.x := Self.m_impulse.x + impulse.x;
         Self.m_impulse.y := Self.m_impulse.y + impulse.y;

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



   --  bool b2RevoluteJoint::SolvePositionConstraints(const b2SolverData& data)
   --  {
   --    b2Vec2 cA = data.positions[m_indexA].c;
   --    float aA = data.positions[m_indexA].a;
   --    b2Vec2 cB = data.positions[m_indexB].c;
   --    float aB = data.positions[m_indexB].a;
   --
   --    b2Rot qA(aA), qB(aB);
   --
   --    float angularError = 0.0f;
   --    float positionError = 0.0f;
   --
   --    bool fixedRotation = (m_invIA + m_invIB == 0.0f);
   --
   --    // Solve angular limit constraint
   --    if (m_enableLimit && fixedRotation == false)
   --    {
   --       float angle = aB - aA - m_referenceAngle;
   --       float C = 0.0f;
   --
   --       if (b2Abs(m_upperAngle - m_lowerAngle) < 2.0f * b2_angularSlop)
   --       {
   --          // Prevent large angular corrections
   --          C = b2Clamp(angle - m_lowerAngle, -b2_maxAngularCorrection, b2_maxAngularCorrection);
   --       }
   --       else if (angle <= m_lowerAngle)
   --       {
   --          // Prevent large angular corrections and allow some slop.
   --          C = b2Clamp(angle - m_lowerAngle + b2_angularSlop, -b2_maxAngularCorrection, 0.0f);
   --       }
   --       else if (angle >= m_upperAngle)
   --       {
   --          // Prevent large angular corrections and allow some slop.
   --          C = b2Clamp(angle - m_upperAngle - b2_angularSlop, 0.0f, b2_maxAngularCorrection);
   --       }
   --
   --       float limitImpulse = -m_axialMass * C;
   --       aA -= m_invIA * limitImpulse;
   --       aB += m_invIB * limitImpulse;
   --       angularError = b2Abs(C);
   --    }
   --
   --    // Solve point-to-point constraint.
   --    {
   --       qA.Set(aA);
   --       qB.Set(aB);
   --       b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
   --       b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
   --
   --       b2Vec2 C = cB + rB - cA - rA;
   --       positionError = C.Length();
   --
   --       float mA = m_invMassA, mB = m_invMassB;
   --       float iA = m_invIA, iB = m_invIB;
   --
   --       b2Mat22 K;
   --       K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
   --       K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
   --       K.ey.x = K.ex.y;
   --       K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;
   --
   --       b2Vec2 impulse = -K.Solve(C);
   --
   --       cA -= mA * impulse;
   --       aA -= iA * b2Cross(rA, impulse);
   --
   --       cB += mB * impulse;
   --       aB += iB * b2Cross(rB, impulse);
   --    }
   --
   --    data.positions[m_indexA].c = cA;
   --    data.positions[m_indexA].a = aA;
   --    data.positions[m_indexB].c = cB;
   --    data.positions[m_indexB].a = aB;
   --
   --    return positionError <= b2_linearSlop && angularError <= b2_angularSlop;
   --  }
   --

   overriding
   function solvePositionConstraints (Self : in out b2RevoluteJoint;   Data : in b2SolverData) return Boolean
   is
      use b2_Common;

      cA : b2Vec2 := data.positions (Self.m_indexA).c;
      aA : Real   := data.positions (Self.m_indexA).a;
      cB : b2Vec2 := data.positions (Self.m_indexB).c;
      aB : Real   := data.positions (Self.m_indexB).a;

      qA : b2Rot := to_b2Rot (aA);
      qB : b2Rot := to_b2Rot (aB);

      angularError  : Real := 0.0;
      positionError : Real := 0.0;

      fixedRotation : constant Boolean := (Self.m_invIA + Self.m_invIB = 0.0);
   begin
      -- Solve angular limit constraint.
      --
      if     Self.m_enableLimit
        and (fixedRotation = False)
      then
         declare
            angle        : constant Real := aB - aA - Self.m_referenceAngle;
            C            :          Real := 0.0;
            limitImpulse :          Real;
         begin
            if  abs (Self.m_upperAngle - Self.m_lowerAngle)
              < 2.0 * b2_angularSlop
            then
               -- Prevent large angular corrections.
               C := b2Clamp ( angle - Self.m_lowerAngle,
                             -b2_maxAngularCorrection,
                              b2_maxAngularCorrection);

            elsif angle <= Self.m_lowerAngle
            then
               -- Prevent large angular corrections and allow some slop.
               C := b2Clamp ( angle - Self.m_lowerAngle + b2_angularSlop,
                             -b2_maxAngularCorrection,
                              0.0);

            elsif angle >= Self.m_upperAngle
            then
               -- Prevent large angular corrections and allow some slop.
               C := b2Clamp (angle - Self.m_upperAngle - b2_angularSlop,
                             0.0,
                             b2_maxAngularCorrection);
            end if;

            limitImpulse := -Self.m_axialMass * C;
            aA           :=  aA - Self.m_invIA * limitImpulse;
            aB           :=  aB + Self.m_invIB * limitImpulse;
            angularError :=  abs C;
         end;
      end if;

      -- Solve point-to-point constraint.
      --
      set (qA, aA);
      set (qB, aB);

      declare
         rA : constant b2Vec2 := b2Mul (qA,  Self.m_localAnchorA - Self.m_localCenterA);
         rB : constant b2Vec2 := b2Mul (qB,  Self.m_localAnchorB - Self.m_localCenterB);

         C  : constant b2Vec2 := cB + rB - cA - rA;

         mA : constant Real   := Self.m_invMassA;
         mB : constant Real   := Self.m_invMassB;

         iA : constant Real   := Self.m_invIA;
         iB : constant Real   := Self.m_invIB;

         K  : b2Mat22;
      begin
         positionError := Length (C);

         K.ex.x :=   mA + mB
                   + iA * rA.y * rA.y
                   + iB * rB.y * rB.y;

         K.ex.y :=   -iA * rA.x * rA.y
                   -  iB * rB.x * rB.y;
         K.ey.x := K.ex.y;

         K.ey.y :=   mA + mB
                   + iA * rA.x * rA.x
                   + iB * rB.x * rB.x;
         declare
            impulse : constant b2Vec2 := -solve (K, C);
         begin
            cA := cA - mA * impulse;
            aA := aA - iA * b2Cross (rA, impulse);

            cB := cB + mB * impulse;
            aB := aB + iB * b2Cross (rB, impulse);
         end;
      end;

      data.positions (Self.m_indexA).c := cA;
      data.positions (Self.m_indexA).a := aA;
      data.positions (Self.m_indexB).c := cB;
      data.positions (Self.m_indexB).a := aB;

      return positionError <= b2_linearSlop
        and  angularError  <= b2_angularSlop;
   end solvePositionConstraints;



end box2d.b2_Joint.b2_revolute_Joint;
