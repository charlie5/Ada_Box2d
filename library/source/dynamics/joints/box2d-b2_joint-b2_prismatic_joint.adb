with
     box2d.b2_Draw,
     box2d.b2_time_Step,
     box2d.b2_Common;


package body box2d.b2_Joint.b2_prismatic_Joint
--
--  Linear constraint (point-to-line)
--  d = p2 - p1 = x2 + r2 - x1 - r1
--  C = dot(perp, d)
--  Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
--       = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
--  J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
--
--  Angular constraint
--  C = a2 - a1 + a_initial
--  Cdot = w2 - w1
--  J = [0 0 -1 0 0 1]
--
--  K = J * invM * JT
--
--  J = [-a -s1 a s2]
--      [0  -1  0  1]
--  a = perp
--  s1 = cross(d + r1, a) = cross(p2 - x1, a)
--  s2 = cross(r2, a) = cross(p2 - x2, a)
--
--  Motor/Limit linear constraint
--  C = dot(ax1, d)
--  Cdot = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
--  J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]
--
--  Predictive limit is applied even when the limit is not active.
--  Prevents a constraint speed that can lead to a constraint error in one time step.
--  Want C2 = C1 + h * Cdot >= 0
--  Or:
--  Cdot + C1/h >= 0
--  I do not apply a negative constraint error because that is handled in position correction.
--  So:
--  Cdot + max(C1, 0)/h >= 0
--
--  Block Solver
--  We develop a block solver that includes the angular and linear constraints. This makes the limit stiffer.
--
--  The Jacobian has 2 rows:
--  J = [-uT -s1 uT s2] // linear
--      [0   -1   0  1] // angular
--
--  u = perp
--  s1 = cross(d + r1, u), s2 = cross(r2, u)
--  a1 = cross(d + r1, v), a2 = cross(r2, v)
--
is

   -----------------------
   --- b2PrismaticJointDef
   --

   --    b2PrismaticJointDef()
   --    {
   --       type = e_prismaticJoint;
   --       localAnchorA.SetZero();
   --       localAnchorB.SetZero();
   --       localAxisA.Set(1.0f, 0.0f);
   --       referenceAngle = 0.0f;
   --       enableLimit = false;
   --       lowerTranslation = 0.0f;
   --       upperTranslation = 0.0f;
   --       enableMotor = false;
   --       maxMotorForce = 0.0f;
   --       motorSpeed = 0.0f;
   --    }
   --

   function to_b2PrismaticJointDef return b2PrismaticJointDef
   is
      Self : b2PrismaticJointDef;
   begin
      Self.Kind             := e_prismaticJoint;
      Self.localAnchorA     := ((0.0, 0.0));
      Self.localAnchorB     := ((0.0, 0.0));
      Self.localAxisA       := ((1.0, 1.0));
      Self.referenceAngle   := 0.0;
      Self.enableLimit      := False;
      Self.lowerTranslation := 0.0;
      Self.upperTranslation := 0.0;
      Self.enableMotor      := False;
      Self.maxMotorForce    := 0.0;
      Self.motorSpeed       := 0.0;

      return Self;
   end to_b2PrismaticJointDef;



   --    Initialize the bodies, anchors, axis, and reference angle using the world
   --    anchor and unit world axis.
   --    void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor, const b2Vec2& axis);
   --
   --  void b2PrismaticJointDef::Initialize(b2Body* bA, b2Body* bB, const b2Vec2& anchor, const b2Vec2& axis)
   --  {
   --    bodyA = bA;
   --    bodyB = bB;
   --    localAnchorA = bodyA->GetLocalPoint(anchor);
   --    localAnchorB = bodyB->GetLocalPoint(anchor);
   --    localAxisA = bodyA->GetLocalVector(axis);
   --    referenceAngle = bodyB->GetAngle() - bodyA->GetAngle();
   --  }
   --

   procedure initialize (Self : out b2PrismaticJointDef;   BodyA, BodyB : access b2Body;
                                                           Anchor       : in     b2Vec2;
                                                           Axis         : in     b2Vec2)
   is
   begin
     Self.bodyA          := BodyA;
     Self.bodyB          := BodyB;
     Self.localAnchorA   := BodyA.getLocalPoint  (anchor);
     Self.localAnchorB   := BodyB.getLocalPoint  (anchor);
     Self.localAxisA     := BodyA.getLocalVector (axis);
     Self.referenceAngle := BodyB.getAngle - BodyA.getAngle;
   end initialize;




   --------------------
   --- b2PrismaticJoint
   --


   --  b2Vec2 b2PrismaticJoint::GetAnchorA() const
   --  {
   --    return m_bodyA->GetWorldPoint(m_localAnchorA);
   --  }
   --

   overriding
   function getAnchorA (Self : in b2PrismaticJoint) return b2Vec2
   is
   begin
      return Self.m_bodyA.getWorldPoint (Self.m_localAnchorA);
   end getAnchorA;



   --  b2Vec2 b2PrismaticJoint::GetAnchorB() const
   --  {
   --    return m_bodyB->GetWorldPoint(m_localAnchorB);
   --  }
   --

   overriding
   function getAnchorB (Self : in b2PrismaticJoint) return b2Vec2
   is
   begin
      return Self.m_bodyB.getWorldPoint (Self.m_localAnchorB);
   end getAnchorB;



   --    The local anchor point relative to bodyA's origin.
   --
   --    const b2Vec2& GetLocalAnchorA() const  { return m_localAnchorA; }
   --

   function getLocalAnchorA (Self : in b2PrismaticJoint) return b2Vec2
   is
   begin
      return Self.m_localAnchorA;
   end getLocalAnchorA;


   --    The local anchor point relative to bodyB's origin.
   --
   --    const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }
   --

   function getLocalAnchorB (Self : in b2PrismaticJoint) return b2Vec2
   is
   begin
      return Self.m_localAnchorB;
   end getLocalAnchorB;



   --    b2Vec2 GetReactionForce(float inv_dt) const override;
   --
   --  b2Vec2 b2PrismaticJoint::GetReactionForce(float inv_dt) const
   --  {
   --    return inv_dt * (m_impulse.x * m_perp + (m_motorImpulse + m_lowerImpulse - m_upperImpulse) * m_axis);
   --  }
   --

   overriding
   function getReactionForce (Self : in b2PrismaticJoint;   inv_dt : in Real) return b2Vec2
   is
   begin
      return   inv_dt
             * (  Self.m_impulse.x * Self.m_perp
                + (    Self.m_motorImpulse
                     + Self.m_lowerImpulse
                     - Self.m_upperImpulse)
                   * Self.m_axis);
   end getReactionForce;



   --    float GetReactionTorque(float inv_dt) const override;
   --
   --  float b2PrismaticJoint::GetReactionTorque(float inv_dt) const
   --  {
   --    return inv_dt * m_impulse.y;
   --  }
   --

   overriding
   function getReactionTorque (Self : in b2PrismaticJoint;   inv_dt : in Real) return Real
   is
   begin
      return inv_dt * Self.m_impulse.y;
   end getReactionTorque;





   --    The local joint axis relative to bodyA.
   --
   --    const b2Vec2& GetLocalAxisA() const { return m_localXAxisA; }
   --

   function GetLocalAxisA (Self : in b2PrismaticJoint) return b2Vec2
   is
   begin
      return Self.m_localXAxisA;
   end GetLocalAxisA;





   --    Get the reference angle.
   --
   --    float GetReferenceAngle() const { return m_referenceAngle; }
   --

   function GetReferenceAngle (Self : in b2PrismaticJoint) return Real
   is
   begin
      return Self.m_referenceAngle;
   end GetReferenceAngle;




   --    Get the current joint translation, usually in meters.
   --
   --  float b2PrismaticJoint::GetJointTranslation() const
   --  {
   --    b2Vec2 pA = m_bodyA->GetWorldPoint(m_localAnchorA);
   --    b2Vec2 pB = m_bodyB->GetWorldPoint(m_localAnchorB);
   --    b2Vec2 d = pB - pA;
   --    b2Vec2 axis = m_bodyA->GetWorldVector(m_localXAxisA);
   --
   --    float translation = b2Dot(d, axis);
   --    return translation;
   --  }
   --

   function GetJointTranslation (Self : in b2PrismaticJoint) return Real
   is
      pA   : constant b2Vec2 := Self.m_bodyA.getWorldPoint (Self.m_localAnchorA);
      pB   : constant b2Vec2 := Self.m_bodyB.getWorldPoint (Self.m_localAnchorB);
      d    : constant b2Vec2 := pB - pA;
      axis : constant b2Vec2 := Self.m_bodyA.getWorldVector (Self.m_localXAxisA);

      Translation : constant Real := b2Dot(d, axis);
   begin
      return Translation;
   end GetJointTranslation;




   --    Get the current joint translation speed, usually in meters per second.
   --
   --  float b2PrismaticJoint::GetJointSpeed() const
   --  {
   --    b2Body* bA = m_bodyA;
   --    b2Body* bB = m_bodyB;
   --
   --    b2Vec2 rA = b2Mul(bA->m_xf.q, m_localAnchorA - bA->m_sweep.localCenter);
   --    b2Vec2 rB = b2Mul(bB->m_xf.q, m_localAnchorB - bB->m_sweep.localCenter);
   --    b2Vec2 p1 = bA->m_sweep.c + rA;
   --    b2Vec2 p2 = bB->m_sweep.c + rB;
   --    b2Vec2 d = p2 - p1;
   --    b2Vec2 axis = b2Mul(bA->m_xf.q, m_localXAxisA);
   --
   --    b2Vec2 vA = bA->m_linearVelocity;
   --    b2Vec2 vB = bB->m_linearVelocity;
   --    float wA = bA->m_angularVelocity;
   --    float wB = bB->m_angularVelocity;
   --
   --    float speed = b2Dot(d, b2Cross(wA, axis)) + b2Dot(axis, vB + b2Cross(wB, rB) - vA - b2Cross(wA, rA));
   --    return speed;
   --  }
   --

   function GetJointSpeed (Self : in b2PrismaticJoint) return Real
   is
      bA    : constant access b2Body := Self.m_bodyA;
      bB    : constant access b2Body := Self.m_bodyB;

      rA    : constant b2Vec2 := b2Mul (bA.m_xf.q, Self.m_localAnchorA - bA.m_sweep.localCenter);
      rB    : constant b2Vec2 := b2Mul (bB.m_xf.q, Self.m_localAnchorB - bB.m_sweep.localCenter);

      p1    : constant b2Vec2 := bA.m_sweep.c + rA;
      p2    : constant b2Vec2 := bB.m_sweep.c + rB;

      d     : constant b2Vec2 := p2 - p1;
      axis  : constant b2Vec2 := b2Mul (bA.m_xf.q, Self.m_localXAxisA);

      vA    : constant b2Vec2 := bA.getLinearVelocity;
      vB    : constant b2Vec2 := bB.getLinearVelocity;

      wA    : constant Real := bA.getAngularVelocity;
      wB    : constant Real := bB.getAngularVelocity;

      Speed : constant Real :=   b2Dot (d, b2Cross (wA, axis))
                               + b2Dot (axis,   vB
                                              + b2Cross (wB, rB)
                                              - vA
                                              - b2Cross (wA, rA));
   begin
      return Speed;
   end GetJointSpeed;




   --    Is the joint limit enabled?
   --
   --  bool b2PrismaticJoint::IsLimitEnabled() const
   --  {
   --    return m_enableLimit;
   --  }
   --

   function IsLimitEnabled (Self : in b2PrismaticJoint) return Boolean
   is
   begin
      return Self.m_enableLimit;
   end IsLimitEnabled;




   --    Enable/disable the joint limit.
   --
   --  void b2PrismaticJoint::EnableLimit(bool flag)
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

   procedure EnableLimit (Self : in out b2PrismaticJoint;   Flag : in Boolean)
   is
   begin
      if Flag /= Self.m_enableLimit
      then
         Self.m_bodyA.setAwake (true);
         Self.m_bodyB.setAwake (true);

         Self.m_enableLimit  := Flag;

         Self.m_lowerImpulse := 0.0;
         Self.m_upperImpulse := 0.0;
      end if;
   end EnableLimit;



   --    Get the lower joint limit, usually in meters.
   --
   --  float b2PrismaticJoint::GetLowerLimit() const
   --  {
   --    return m_lowerTranslation;
   --  }
   --

   function GetLowerLimit (Self : in b2PrismaticJoint) return Real
   is
   begin
      return Self.m_lowerTranslation;
   end GetLowerLimit;




   --    Get the upper joint limit, usually in meters.
   --
   --  float b2PrismaticJoint::GetUpperLimit() const
   --  {
   --    return m_upperTranslation;
   --  }
   --

   function GetUpperLimit (Self : in b2PrismaticJoint) return Real
   is
   begin
      return Self.m_upperTranslation;
   end GetUpperLimit;




   --    Set the joint limits, usually in meters.
   --
   --  void b2PrismaticJoint::SetLimits(float lower, float upper)
   --  {
   --    b2Assert(lower <= upper);
   --    if (lower != m_lowerTranslation || upper != m_upperTranslation)
   --    {
   --       m_bodyA->SetAwake(true);
   --       m_bodyB->SetAwake(true);
   --       m_lowerTranslation = lower;
   --       m_upperTranslation = upper;
   --       m_lowerImpulse = 0.0f;
   --       m_upperImpulse = 0.0f;
   --    }
   --  }
   --

   procedure SetLimits (Self : in out b2PrismaticJoint;   Lower, Upper : in Real)
   is
      pragma assert (lower <= upper);
   begin
      if   lower /= Self.m_lowerTranslation
        or upper /= Self.m_upperTranslation
      then
         Self.m_bodyA.setAwake (True);
         Self.m_bodyB.setAwake (True);

         Self.m_lowerTranslation := Lower;
         Self.m_upperTranslation := Upper;

         Self.m_lowerImpulse := 0.0;
         Self.m_upperImpulse := 0.0;
     end if;
   end SetLimits;





   --    Is the joint motor enabled?
   --
   --  bool b2PrismaticJoint::IsMotorEnabled() const
   --  {
   --    return m_enableMotor;
   --  }
   --

   function IsMotorEnabled (Self : in b2PrismaticJoint) return Boolean
   is
   begin
      return Self.m_enableMotor;
   end IsMotorEnabled;




   --    Enable/disable the joint motor.
   --
   --  void b2PrismaticJoint::EnableMotor(bool flag)
   --  {
   --    if (flag != m_enableMotor)
   --    {
   --       m_bodyA->SetAwake(true);
   --       m_bodyB->SetAwake(true);
   --       m_enableMotor = flag;
   --    }
   --  }
   --

   procedure EnableMotor (Self : in out b2PrismaticJoint;   Flag : in Boolean)
   is
   begin
     if Flag /= Self.m_enableMotor
     then
         Self.m_bodyA.setAwake (True);
         Self.m_bodyB.setAwake (True);

         Self.m_enableMotor := Flag;
     end if;
   end EnableMotor;




   --    Set the motor speed, usually in meters per second.
   --
   --  void b2PrismaticJoint::SetMotorSpeed(float speed)
   --  {
   --    if (speed != m_motorSpeed)
   --    {
   --       m_bodyA->SetAwake(true);
   --       m_bodyB->SetAwake(true);
   --       m_motorSpeed = speed;
   --    }
   --  }
   --

   procedure SetMotorSpeed (Self : in out b2PrismaticJoint;   Speed : in Real)
   is
   begin
      if Speed /= Self.m_motorSpeed
      then
         Self.m_bodyA.setAwake (True);
         Self.m_bodyB.setAwake (True);

         Self.m_motorSpeed := Speed;
      end if;
   end SetMotorSpeed;





   --  Get the motor speed, usually in meters per second.
   --
   --  inline float b2PrismaticJoint::GetMotorSpeed() const
   --  {
   --    return m_motorSpeed;
   --  }
   --

   function GetMotorSpeed (Self : in b2PrismaticJoint) return Real
   is
   begin
      return Self.m_motorSpeed;
   end GetMotorSpeed;




   --    Set the maximum motor force, usually in N.
   --
   --  void b2PrismaticJoint::SetMaxMotorForce(float force)
   --  {
   --    if (force != m_maxMotorForce)
   --    {
   --       m_bodyA->SetAwake(true);
   --       m_bodyB->SetAwake(true);
   --       m_maxMotorForce = force;
   --    }
   --  }
   --

   procedure SetMaxMotorForce (Self : in out b2PrismaticJoint;   Force : in Real)
   is
   begin
      if Force /= Self.m_maxMotorForce
      then
         Self.m_bodyA.setAwake (True);
         Self.m_bodyB.setAwake (True);

         Self.m_maxMotorForce := Force;
      end if;
   end SetMaxMotorForce;




   --    float GetMaxMotorForce() const { return m_maxMotorForce; }
   --

   function GetMaxMotorForce (Self : in b2PrismaticJoint) return Real
   is
   begin
      return Self.m_maxMotorForce;
   end GetMaxMotorForce;



   --    Get the current motor force given the inverse time step, usually in N.
   --
   --  float b2PrismaticJoint::GetMotorForce(float inv_dt) const
   --  {
   --    return inv_dt * m_motorImpulse;
   --  }
   --

   function GetMotorForce (Self : in b2PrismaticJoint;   inv_dt : in Real) return Real
   is
   begin
      return inv_dt * Self.m_motorImpulse;
   end GetMotorForce;




   --    Dump to b2Log
   --
   --  void b2PrismaticJoint::Dump()
   --  {
   --    // FLT_DECIMAL_DIG == 9
   --
   --    int32 indexA = m_bodyA->m_islandIndex;
   --    int32 indexB = m_bodyB->m_islandIndex;
   --
   --    b2Dump("  b2PrismaticJointDef jd;\n");
   --    b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
   --    b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
   --    b2Dump("  jd.collideConnected = bool(%d);\n", m_collideConnected);
   --    b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", m_localAnchorA.x, m_localAnchorA.y);
   --    b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", m_localAnchorB.x, m_localAnchorB.y);
   --    b2Dump("  jd.localAxisA.Set(%.9g, %.9g);\n", m_localXAxisA.x, m_localXAxisA.y);
   --    b2Dump("  jd.referenceAngle = %.9g;\n", m_referenceAngle);
   --    b2Dump("  jd.enableLimit = bool(%d);\n", m_enableLimit);
   --    b2Dump("  jd.lowerTranslation = %.9g;\n", m_lowerTranslation);
   --    b2Dump("  jd.upperTranslation = %.9g;\n", m_upperTranslation);
   --    b2Dump("  jd.enableMotor = bool(%d);\n", m_enableMotor);
   --    b2Dump("  jd.motorSpeed = %.9g;\n", m_motorSpeed);
   --    b2Dump("  jd.maxMotorForce = %.9g;\n", m_maxMotorForce);
   --    b2Dump("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
   --  }
   --

   overriding
   procedure dump (Self : in b2PrismaticJoint)
   is
      use b2_Common;
      indexA : constant Natural := Self.m_bodyA.m_islandIndex;
      indexB : constant Natural := Self.m_bodyB.m_islandIndex;
   begin
     b2Dump ("  b2PrismaticJointDef jd;");
     b2Dump ("  jd.bodyA = bodies[%d];"                  & indexA'Image);
     b2Dump ("  jd.bodyB = bodies[%d];"                  & indexB'Image);
     b2Dump ("  jd.collideConnected = bool(%d);"         & Self.m_collideConnected'Image);
     b2Dump ("  jd.localAnchorA.Set(%.9g, %.9g);"        & Self.m_localAnchorA.x  'Image & Self.m_localAnchorA.y'Image);
     b2Dump ("  jd.localAnchorB.Set(%.9g, %.9g);"        & Self.m_localAnchorB.x  'Image & Self.m_localAnchorB.y'Image);
     b2Dump ("  jd.localAxisA.Set(%.9g, %.9g);"          & Self.m_localXAxisA.x   'Image & Self.m_localXAxisA .y'Image);
     b2Dump ("  jd.referenceAngle = %.9g;"               & Self.m_referenceAngle  'Image);
     b2Dump ("  jd.enableLimit = bool(%d);"              & Self.m_enableLimit     'Image);
     b2Dump ("  jd.lowerTranslation = %.9g;"             & Self.m_lowerTranslation'Image);
     b2Dump ("  jd.upperTranslation = %.9g;"             & Self.m_upperTranslation'Image);
     b2Dump ("  jd.enableMotor = bool(%d);"              & Self.m_enableMotor     'Image);
     b2Dump ("  jd.motorSpeed = %.9g;"                   & Self.m_motorSpeed      'Image);
     b2Dump ("  jd.maxMotorForce = %.9g;"                & Self.m_maxMotorForce   'Image);
     b2Dump ("  joints[%d] = m_world->CreateJoint(&jd);" & Self.m_index'Image);
   end dump;




   --  void b2PrismaticJoint::Draw(b2Draw* draw) const
   --  {
   --    const b2Transform& xfA = m_bodyA->GetTransform();
   --    const b2Transform& xfB = m_bodyB->GetTransform();
   --    b2Vec2 pA = b2Mul(xfA, m_localAnchorA);
   --    b2Vec2 pB = b2Mul(xfB, m_localAnchorB);
   --
   --    b2Vec2 axis = b2Mul(xfA.q, m_localXAxisA);
   --
   --    b2Color c1(0.7f, 0.7f, 0.7f);
   --    b2Color c2(0.3f, 0.9f, 0.3f);
   --    b2Color c3(0.9f, 0.3f, 0.3f);
   --    b2Color c4(0.3f, 0.3f, 0.9f);
   --    b2Color c5(0.4f, 0.4f, 0.4f);
   --
   --    draw->DrawSegment(pA, pB, c5);
   --
   --    if (m_enableLimit)
   --    {
   --       b2Vec2 lower = pA + m_lowerTranslation * axis;
   --       b2Vec2 upper = pA + m_upperTranslation * axis;
   --       b2Vec2 perp = b2Mul(xfA.q, m_localYAxisA);
   --       draw->DrawSegment(lower, upper, c1);
   --       draw->DrawSegment(lower - 0.5f * perp, lower + 0.5f * perp, c2);
   --       draw->DrawSegment(upper - 0.5f * perp, upper + 0.5f * perp, c3);
   --    }
   --    else
   --    {
   --       draw->DrawSegment(pA - 1.0f * axis, pA + 1.0f * axis, c1);
   --    }
   --
   --    draw->DrawPoint(pA, 5.0f, c1);
   --    draw->DrawPoint(pB, 5.0f, c4);
   --  }
   --

   overriding
   procedure draw (Self : in b2PrismaticJoint;   Draw : access b2Draw'Class)
   is
      xfA  : constant b2Transform := Self.m_bodyA.getTransform;
      xfB  : constant b2Transform := Self.m_bodyB.getTransform;

      pA   : constant b2Vec2  := b2Mul (xfA,   Self.m_localAnchorA);
      pB   : constant b2Vec2  := b2Mul (xfB,   Self.m_localAnchorB);
      axis : constant b2Vec2  := b2Mul (xfA.q, Self.m_localXAxisA);

      c1   : constant b2Color := to_b2Color (0.7, 0.7, 0.7);
      c2   : constant b2Color := to_b2Color (0.3, 0.9, 0.3);
      c3   : constant b2Color := to_b2Color (0.9, 0.3, 0.3);
      c4   : constant b2Color := to_b2Color (0.3, 0.3, 0.9);
      c5   : constant b2Color := to_b2Color (0.4, 0.4, 0.4);

   begin
      draw_any (Self, Draw);
      draw.drawSegment (pA, pB, c5);

      if Self.m_enableLimit
      then
         declare
            lower : constant b2Vec2 := pA + Self.m_lowerTranslation * axis;
            upper : constant b2Vec2 := pA + Self.m_upperTranslation * axis;

            perp  : constant b2Vec2 := b2Mul (xfA.q, Self.m_localYAxisA);
         begin
            draw.drawSegment (lower, upper, c1);
            draw.drawSegment (lower - 0.5 * perp,  lower + 0.5 * perp,  c2);
            draw.drawSegment (upper - 0.5 * perp,  upper + 0.5 * perp,  c3);
         end;

      else
         draw.drawSegment (pA - 1.0 * axis,  pA + 1.0 * axis,  c1);
      end if;

      draw.drawPoint(pA, 5.0, c1);
      draw.drawPoint(pB, 5.0, c4);
   end draw;






   --------------
   --  protected:
   --

   --    friend class b2Joint;
   --    friend class b2GearJoint;
   --



   --  b2PrismaticJoint::b2PrismaticJoint(const b2PrismaticJointDef* def)
   --  : b2Joint(def)
   --  {
   --    m_localAnchorA = def->localAnchorA;
   --    m_localAnchorB = def->localAnchorB;
   --    m_localXAxisA = def->localAxisA;
   --    m_localXAxisA.Normalize();
   --    m_localYAxisA = b2Cross(1.0f, m_localXAxisA);
   --    m_referenceAngle = def->referenceAngle;
   --
   --    m_impulse.SetZero();
   --    m_axialMass = 0.0f;
   --    m_motorImpulse = 0.0f;
   --    m_lowerImpulse = 0.0f;
   --    m_upperImpulse = 0.0f;
   --
   --    m_lowerTranslation = def->lowerTranslation;
   --    m_upperTranslation = def->upperTranslation;
   --
   --    b2Assert(m_lowerTranslation <= m_upperTranslation);
   --
   --    m_maxMotorForce = def->maxMotorForce;
   --    m_motorSpeed = def->motorSpeed;
   --    m_enableLimit = def->enableLimit;
   --    m_enableMotor = def->enableMotor;
   --
   --    m_translation = 0.0f;
   --    m_axis.SetZero();
   --    m_perp.SetZero();
   --  }
   --

   package body Forge
   is
      --    b2DistanceJoint(const b2DistanceJointDef* data);
      --
      function to_b2PrismaticJoint (Def : in b2PrismaticJointDef) return b2PrismaticJoint
      is
         Self : b2PrismaticJoint;
      begin
         Self.m_localAnchorA   := def.localAnchorA;
         Self.m_localAnchorB   := def.localAnchorB;
         Self.m_localXAxisA    := def.localAxisA;

         Normalize (Self.m_localXAxisA);

         Self.m_localYAxisA    := b2Cross (1.0, Self.m_localXAxisA);
         Self.m_referenceAngle := def.referenceAngle;

         Self.m_impulse        := ((0.0, 0.0));
         Self.m_axialMass      := 0.0;
         Self.m_motorImpulse   := 0.0;
         Self.m_lowerImpulse   := 0.0;
         Self.m_upperImpulse   := 0.0;

         Self.m_lowerTranslation := def.lowerTranslation;
         Self.m_upperTranslation := def.upperTranslation;

         pragma assert (Self.m_lowerTranslation <= Self.m_upperTranslation);

         Self.m_maxMotorForce  := def.maxMotorForce;
         Self.m_motorSpeed     := def.motorSpeed;
         Self.m_enableLimit    := def.enableLimit;
         Self.m_enableMotor    := def.enableMotor;

         Self.m_translation    := 0.0;
         Self.m_axis           := ((0.0, 0.0));
         Self.m_perp           := ((0.0, 0.0));

         return Self;
      end to_b2PrismaticJoint;

   end Forge;






   --    friend class b2Joint;
   --


   --  void b2PrismaticJoint::InitVelocityConstraints(const b2SolverData& data)
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
   --    // Compute the effective masses.
   --    b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
   --    b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
   --    b2Vec2 d = (cB - cA) + rB - rA;
   --
   --    float mA = m_invMassA, mB = m_invMassB;
   --    float iA = m_invIA, iB = m_invIB;
   --
   --    // Compute motor Jacobian and effective mass.
   --    {
   --       m_axis = b2Mul(qA, m_localXAxisA);
   --       m_a1 = b2Cross(d + rA, m_axis);
   --       m_a2 = b2Cross(rB, m_axis);
   --
   --       m_axialMass = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;
   --       if (m_axialMass > 0.0f)
   --       {
   --          m_axialMass = 1.0f / m_axialMass;
   --       }
   --    }
   --
   --    // Prismatic constraint.
   --    {
   --       m_perp = b2Mul(qA, m_localYAxisA);
   --
   --       m_s1 = b2Cross(d + rA, m_perp);
   --       m_s2 = b2Cross(rB, m_perp);
   --
   --       float k11 = mA + mB + iA * m_s1 * m_s1 + iB * m_s2 * m_s2;
   --       float k12 = iA * m_s1 + iB * m_s2;
   --       float k22 = iA + iB;
   --       if (k22 == 0.0f)
   --       {
   --          // For bodies with fixed rotation.
   --          k22 = 1.0f;
   --       }
   --
   --       m_K.ex.Set(k11, k12);
   --       m_K.ey.Set(k12, k22);
   --    }
   --
   --    if (m_enableLimit)
   --    {
   --       m_translation = b2Dot(m_axis, d);
   --    }
   --    else
   --    {
   --       m_lowerImpulse = 0.0f;
   --       m_upperImpulse = 0.0f;
   --    }
   --
   --    if (m_enableMotor == false)
   --    {
   --       m_motorImpulse = 0.0f;
   --    }
   --
   --    if (data.step.warmStarting)
   --    {
   --       // Account for variable time step.
   --       m_impulse *= data.step.dtRatio;
   --       m_motorImpulse *= data.step.dtRatio;
   --       m_lowerImpulse *= data.step.dtRatio;
   --       m_upperImpulse *= data.step.dtRatio;
   --
   --       float axialImpulse = m_motorImpulse + m_lowerImpulse - m_upperImpulse;
   --       b2Vec2 P = m_impulse.x * m_perp + axialImpulse * m_axis;
   --       float LA = m_impulse.x * m_s1 + m_impulse.y + axialImpulse * m_a1;
   --       float LB = m_impulse.x * m_s2 + m_impulse.y + axialImpulse * m_a2;
   --
   --       vA -= mA * P;
   --       wA -= iA * LA;
   --
   --       vB += mB * P;
   --       wB += iB * LB;
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
   --

   overriding
   procedure initVelocityConstraints (Self : in out b2PrismaticJoint;   Data : in b2SolverData)
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

         -- Compute the effective masses.
         rA : constant b2Vec2 := b2Mul (qA,  Self.m_localAnchorA - Self.m_localCenterA);
         rB : constant b2Vec2 := b2Mul (qB,  Self.m_localAnchorB - Self.m_localCenterB);
         d  : constant b2Vec2 := (cB - cA) + rB - rA;

         mA : constant Real   := Self.m_invMassA;
         mB : constant Real   := Self.m_invMassB;
         iA : constant Real   := Self.m_invIA;
         iB : constant Real   := Self.m_invIB;

      begin
         -- Compute motor Jacobian and effective mass.
         begin
            Self.m_axis := b2Mul   (qA,     Self.m_localXAxisA);
            Self.m_a1   := b2Cross (d + rA, Self.m_axis);
            Self.m_a2   := b2Cross (rB,     Self.m_axis);

            Self.m_axialMass :=   mA + mB
                                + iA * Self.m_a1 * Self.m_a1
                                + iB * Self.m_a2 * Self.m_a2;

            if Self.m_axialMass > 0.0
            then
               Self.m_axialMass := 1.0 / Self.m_axialMass;
            end if;
         end;

        -- Prismatic constraint.
        begin
            Self.m_perp := b2Mul (qA, Self.m_localYAxisA);

            Self.m_s1   := b2Cross (d + rA, Self.m_perp);
            Self.m_s2   := b2Cross (rB,     Self.m_perp);

            declare
               k11 : constant Real :=   mA + mB
                                      + iA * Self.m_s1 * Self.m_s1
                                      + iB * Self.m_s2 * Self.m_s2;

               k12 : constant Real :=   iA * Self.m_s1
                                      + iB * Self.m_s2;

               k22 : Real := iA + iB;
            begin
               if k22 = 0.0
               then
                  -- For bodies with fixed rotation.
                  k22 := 1.0;
               end if;

               Self.m_K.ex := (k11, k12);
               Self.m_K.ey := (k12, k22);
            end;
         end;


         if Self.m_enableLimit
         then
            Self.m_translation := b2Dot (Self.m_axis, d);
         else
            Self.m_lowerImpulse := 0.0;
            Self.m_upperImpulse := 0.0;
         end if;


         if Self.m_enableMotor = False
         then
            Self.m_motorImpulse := 0.0;
         end if;


         if data.step.warmStarting
         then
            -- Account for variable time step.
            Self.m_impulse      := Self.m_impulse      * data.step.dtRatio;
            Self.m_motorImpulse := Self.m_motorImpulse * data.step.dtRatio;
            Self.m_lowerImpulse := Self.m_lowerImpulse * data.step.dtRatio;
            Self.m_upperImpulse := Self.m_upperImpulse * data.step.dtRatio;

            declare
               axialImpulse : constant Real   := Self.m_motorImpulse + Self.m_lowerImpulse - Self.m_upperImpulse;
               P            : constant b2Vec2 := Self.m_impulse.x * Self.m_perp                  + axialImpulse * Self.m_axis;
               LA           : constant Real   := Self.m_impulse.x * Self.m_s1 + Self.m_impulse.y + axialImpulse * Self.m_a1;
               LB           : constant Real   := Self.m_impulse.x * Self.m_s2 + Self.m_impulse.y + axialImpulse * Self.m_a2;
            begin
               vA := vA - mA * P;
               wA := wA - iA * LA;

               vB := vB + mB * P;
               wB := wB + iB * LB;
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





   --  void b2PrismaticJoint::SolveVelocityConstraints(const b2SolverData& data)
   --  {
   --    b2Vec2 vA = data.velocities[m_indexA].v;
   --    float wA = data.velocities[m_indexA].w;
   --    b2Vec2 vB = data.velocities[m_indexB].v;
   --    float wB = data.velocities[m_indexB].w;
   --
   --    float mA = m_invMassA, mB = m_invMassB;
   --    float iA = m_invIA, iB = m_invIB;
   --
   --    // Solve linear motor constraint
   --    if (m_enableMotor)
   --    {
   --       float Cdot = b2Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
   --       float impulse = m_axialMass * (m_motorSpeed - Cdot);
   --       float oldImpulse = m_motorImpulse;
   --       float maxImpulse = data.step.dt * m_maxMotorForce;
   --       m_motorImpulse = b2Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
   --       impulse = m_motorImpulse - oldImpulse;
   --
   --       b2Vec2 P = impulse * m_axis;
   --       float LA = impulse * m_a1;
   --       float LB = impulse * m_a2;
   --
   --       vA -= mA * P;
   --       wA -= iA * LA;
   --       vB += mB * P;
   --       wB += iB * LB;
   --    }
   --
   --    if (m_enableLimit)
   --    {
   --       // Lower limit
   --       {
   --          float C = m_translation - m_lowerTranslation;
   --          float Cdot = b2Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
   --          float impulse = -m_axialMass * (Cdot + b2Max(C, 0.0f) * data.step.inv_dt);
   --          float oldImpulse = m_lowerImpulse;
   --          m_lowerImpulse = b2Max(m_lowerImpulse + impulse, 0.0f);
   --          impulse = m_lowerImpulse - oldImpulse;
   --
   --          b2Vec2 P = impulse * m_axis;
   --          float LA = impulse * m_a1;
   --          float LB = impulse * m_a2;
   --
   --          vA -= mA * P;
   --          wA -= iA * LA;
   --          vB += mB * P;
   --          wB += iB * LB;
   --       }
   --
   --       // Upper limit
   --       // Note: signs are flipped to keep C positive when the constraint is satisfied.
   --       // This also keeps the impulse positive when the limit is active.
   --       {
   --          float C = m_upperTranslation - m_translation;
   --          float Cdot = b2Dot(m_axis, vA - vB) + m_a1 * wA - m_a2 * wB;
   --          float impulse = -m_axialMass * (Cdot + b2Max(C, 0.0f) * data.step.inv_dt);
   --          float oldImpulse = m_upperImpulse;
   --          m_upperImpulse = b2Max(m_upperImpulse + impulse, 0.0f);
   --          impulse = m_upperImpulse - oldImpulse;
   --
   --          b2Vec2 P = impulse * m_axis;
   --          float LA = impulse * m_a1;
   --          float LB = impulse * m_a2;
   --
   --          vA += mA * P;
   --          wA += iA * LA;
   --          vB -= mB * P;
   --          wB -= iB * LB;
   --       }
   --    }
   --
   --    // Solve the prismatic constraint in block form.
   --    {
   --       b2Vec2 Cdot;
   --       Cdot.x = b2Dot(m_perp, vB - vA) + m_s2 * wB - m_s1 * wA;
   --       Cdot.y = wB - wA;
   --
   --       b2Vec2 df = m_K.Solve(-Cdot);
   --       m_impulse += df;
   --
   --       b2Vec2 P = df.x * m_perp;
   --       float LA = df.x * m_s1 + df.y;
   --       float LB = df.x * m_s2 + df.y;
   --
   --       vA -= mA * P;
   --       wA -= iA * LA;
   --
   --       vB += mB * P;
   --       wB += iB * LB;
   --    }
   --
   --    data.velocities[m_indexA].v = vA;
   --    data.velocities[m_indexA].w = wA;
   --    data.velocities[m_indexB].v = vB;
   --    data.velocities[m_indexB].w = wB;
   --  }
   --

   overriding
   procedure solveVelocityConstraints (Self : in out b2PrismaticJoint;   Data : in b2SolverData)
   is
      vA : b2Vec2 := data.velocities (Self.m_indexA).v;
      wA : Real   := data.velocities (Self.m_indexA).w;
      vB : b2Vec2 := data.velocities (Self.m_indexB).v;
      wB : Real   := data.velocities (Self.m_indexB).w;

      mA : constant Real   := Self.m_invMassA;
      mB : constant Real   := Self.m_invMassB;
      iA : constant Real   := Self.m_invIA;
      iB : constant Real   := Self.m_invIB;

   begin
      -- Solve linear motor constraint
      if Self.m_enableMotor
      then
         declare
            Cdot       : constant Real :=   b2Dot (Self.m_axis, vB - vA)
                                          + Self.m_a2 * wB
                                          - Self.m_a1 * wA;

            impulse    :          Real := Self.m_axialMass * (Self.m_motorSpeed - Cdot);
            oldImpulse : constant Real := Self.m_motorImpulse;
            maxImpulse : constant Real := data.step.dt * Self.m_maxMotorForce;
         begin
            Self.m_motorImpulse := b2Clamp (Self.m_motorImpulse + impulse,
                                            -maxImpulse,
                                             maxImpulse);
            impulse := Self.m_motorImpulse - oldImpulse;

            declare
               P  : constant b2Vec2 := impulse * Self.m_axis;
               LA : constant Real   := impulse * Self.m_a1;
               LB : constant Real   := impulse * Self.m_a2;
            begin
               vA := vA - mA * P;
               wA := wA - iA * LA;
               vB := vB + mB * P;
               wB := wB + iB * LB;
            end;
         end;
      end if;


      if Self.m_enableLimit
     then
        -- Lower limit
        declare
            C          : constant Real := Self.m_translation - Self.m_lowerTranslation;
            Cdot       : constant Real :=   b2Dot (Self.m_axis, vB - vA)
                                          + Self.m_a2 * wB
                                          - Self.m_a1 * wA;

            impulse    :          Real := -Self.m_axialMass  *  (Cdot + Real'max (C, 0.0) * data.step.inv_dt);
            oldImpulse : constant Real := Self.m_lowerImpulse;
         begin
            Self.m_lowerImpulse := Real'max (Self.m_lowerImpulse + impulse, 0.0);
            impulse             := Self.m_lowerImpulse - oldImpulse;

            declare
               P  : constant b2Vec2 := impulse * Self.m_axis;
               LA : constant Real   := impulse * Self.m_a1;
               LB : constant Real   := impulse * Self.m_a2;
            begin
               vA := vA - mA * P;
               wA := wA - iA * LA;
               vB := vB + mB * P;
               wB := wB + iB * LB;
            end;
         end;

         -- Upper limit
         -- Note: signs are flipped to keep C positive when the constraint is satisfied.
         -- This also keeps the impulse positive when the limit is active.
         declare
            C          : constant Real := Self.m_upperTranslation - Self.m_translation;
            Cdot       : constant Real :=   b2Dot (Self.m_axis, vA - vB)
                                          + Self.m_a1 * wA
                                          - Self.m_a2 * wB;

            impulse    :          Real := -Self.m_axialMass  *  (Cdot + Real'max (C, 0.0) * data.step.inv_dt);
            oldImpulse : constant Real := Self.m_upperImpulse;
         begin
            Self.m_upperImpulse := Real'max (Self.m_upperImpulse + impulse, 0.0);
            impulse             := Self.m_upperImpulse - oldImpulse;

            declare
               P  : constant b2Vec2 := impulse * Self.m_axis;
               LA : constant Real   := impulse * Self.m_a1;
               LB : constant Real   := impulse * Self.m_a2;
            begin
               vA := vA + mA * P;
               wA := wA + iA * LA;
               vB := vB - mB * P;
               wB := wB - iB * LB;
            end;
         end;
      end if;


      -- Solve the prismatic constraint in block form.
      declare
         Cdot : b2Vec2;
         df   : b2Vec2;
      begin
         Cdot.x :=   b2Dot (Self.m_perp, vB - vA)
                   + Self.m_s2 * wB
                   - Self.m_s1 * wA;
        Cdot.y  := wB - wA;

        df             := solve (Self.m_K, -Cdot);
        Self.m_impulse := Self.m_impulse + df;

         declare
            P  : constant b2Vec2 := df.x * Self.m_perp;
            LA : constant Real   := df.x * Self.m_s1 + df.y;
            LB : constant Real   := df.x * Self.m_s2 + df.y;
         begin
            vA := vA - mA * P;
            wA := wA - iA * LA;

            vB := vB + mB * P;
            wB := wB + iB * LB;
         end;
      end;

      data.velocities (Self.m_indexA).v := vA;
      data.velocities (Self.m_indexA).w := wA;
      data.velocities (Self.m_indexB).v := vB;
      data.velocities (Self.m_indexB).w := wB;
   end solveVelocityConstraints;






   --  // A velocity based solver computes reaction forces(impulses) using the velocity constraint solver.Under this context,
   --  // the position solver is not there to resolve forces.It is only there to cope with integration error.
   --  //
   --  // Therefore, the pseudo impulses in the position solver do not have any physical meaning.Thus it is okay if they suck.
   --  //
   --  // We could take the active state from the velocity solver.However, the joint might push past the limit when the velocity
   --  // solver indicates the limit is inactive.
   --
   --  bool b2PrismaticJoint::SolvePositionConstraints(const b2SolverData& data)
   --  {
   --    b2Vec2 cA = data.positions[m_indexA].c;
   --    float aA = data.positions[m_indexA].a;
   --    b2Vec2 cB = data.positions[m_indexB].c;
   --    float aB = data.positions[m_indexB].a;
   --
   --    b2Rot qA(aA), qB(aB);
   --
   --    float mA = m_invMassA, mB = m_invMassB;
   --    float iA = m_invIA, iB = m_invIB;
   --
   --    // Compute fresh Jacobians
   --    b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
   --    b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
   --    b2Vec2 d = cB + rB - cA - rA;
   --
   --    b2Vec2 axis = b2Mul(qA, m_localXAxisA);
   --    float a1 = b2Cross(d + rA, axis);
   --    float a2 = b2Cross(rB, axis);
   --    b2Vec2 perp = b2Mul(qA, m_localYAxisA);
   --
   --    float s1 = b2Cross(d + rA, perp);
   --    float s2 = b2Cross(rB, perp);
   --
   --    b2Vec3 impulse;
   --    b2Vec2 C1;
   --    C1.x = b2Dot(perp, d);
   --    C1.y = aB - aA - m_referenceAngle;
   --
   --    float linearError = b2Abs(C1.x);
   --    float angularError = b2Abs(C1.y);
   --
   --    bool active = false;
   --    float C2 = 0.0f;
   --    if (m_enableLimit)
   --    {
   --       float translation = b2Dot(axis, d);
   --       if (b2Abs(m_upperTranslation - m_lowerTranslation) < 2.0f * b2_linearSlop)
   --       {
   --          C2 = translation;
   --          linearError = b2Max(linearError, b2Abs(translation));
   --          active = true;
   --       }
   --       else if (translation <= m_lowerTranslation)
   --       {
   --          C2 = b2Min(translation - m_lowerTranslation, 0.0f);
   --          linearError = b2Max(linearError, m_lowerTranslation - translation);
   --          active = true;
   --       }
   --       else if (translation >= m_upperTranslation)
   --       {
   --          C2 = b2Max(translation - m_upperTranslation, 0.0f);
   --          linearError = b2Max(linearError, translation - m_upperTranslation);
   --          active = true;
   --       }
   --    }
   --
   --    if (active)
   --    {
   --       float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
   --       float k12 = iA * s1 + iB * s2;
   --       float k13 = iA * s1 * a1 + iB * s2 * a2;
   --       float k22 = iA + iB;
   --       if (k22 == 0.0f)
   --       {
   --          // For fixed rotation
   --          k22 = 1.0f;
   --       }
   --       float k23 = iA * a1 + iB * a2;
   --       float k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;
   --
   --       b2Mat33 K;
   --       K.ex.Set(k11, k12, k13);
   --       K.ey.Set(k12, k22, k23);
   --       K.ez.Set(k13, k23, k33);
   --
   --       b2Vec3 C;
   --       C.x = C1.x;
   --       C.y = C1.y;
   --       C.z = C2;
   --
   --       impulse = K.Solve33(-C);
   --    }
   --    else
   --    {
   --       float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
   --       float k12 = iA * s1 + iB * s2;
   --       float k22 = iA + iB;
   --       if (k22 == 0.0f)
   --       {
   --          k22 = 1.0f;
   --       }
   --
   --       b2Mat22 K;
   --       K.ex.Set(k11, k12);
   --       K.ey.Set(k12, k22);
   --
   --       b2Vec2 impulse1 = K.Solve(-C1);
   --       impulse.x = impulse1.x;
   --       impulse.y = impulse1.y;
   --       impulse.z = 0.0f;
   --    }
   --
   --    b2Vec2 P = impulse.x * perp + impulse.z * axis;
   --    float LA = impulse.x * s1 + impulse.y + impulse.z * a1;
   --    float LB = impulse.x * s2 + impulse.y + impulse.z * a2;
   --
   --    cA -= mA * P;
   --    aA -= iA * LA;
   --    cB += mB * P;
   --    aB += iB * LB;
   --
   --    data.positions[m_indexA].c = cA;
   --    data.positions[m_indexA].a = aA;
   --    data.positions[m_indexB].c = cB;
   --    data.positions[m_indexB].a = aB;
   --
   --    return linearError <= b2_linearSlop && angularError <= b2_angularSlop;
   --  }
   --

   overriding
   function solvePositionConstraints (Self : in out b2PrismaticJoint;   Data : in b2SolverData) return Boolean
   is
      cA :          b2Vec2 := data.positions (Self.m_indexA).c;
      aA :          Real   := data.positions (Self.m_indexA).a;
      cB :          b2Vec2 := data.positions (Self.m_indexB).c;
      aB :          Real   := data.positions (Self.m_indexB).a;

      qA : constant b2Rot  := to_b2Rot (aA);
      qB : constant b2Rot  := to_b2Rot (aB);

      mA : constant Real   := Self.m_invMassA;
      mB : constant Real   := Self.m_invMassB;
      iA : constant Real   := Self.m_invIA;
      iB : constant Real   := Self.m_invIB;

      -- Compute fresh Jacobians.
      --
      rA   : constant b2Vec2 := b2Mul (qA,  Self.m_localAnchorA - Self.m_localCenterA);
      rB   : constant b2Vec2 := b2Mul (qB,  Self.m_localAnchorB - Self.m_localCenterB);
      d    : constant b2Vec2 := cB + rB - cA - rA;

      axis : constant b2Vec2 := b2Mul (qA, Self.m_localXAxisA);
      a1   : constant Real   := b2Cross (d + rA, axis);
      a2   : constant Real   := b2Cross (rB, axis);
      perp : constant b2Vec2 := b2Mul (qA, Self.m_localYAxisA);

      s1   : constant Real   := b2Cross (d + rA, perp);
      s2   : constant Real   := b2Cross (rB,     perp);

      impulse : b2Vec3;
      C1      : b2Vec2;

   begin
      C1.x := b2Dot (perp, d);
      C1.y := aB - aA - Self.m_referenceAngle;


      declare
         use b2_Common;

         linearError  :          Real    := abs (C1.x);
         angularError : constant Real    := abs (C1.y);

         active : Boolean := False;
         C2     : Real    := 0.0;
      begin
         if Self.m_enableLimit
         then
            declare
               translation : constant Real := b2Dot (axis, d);
            begin
               if abs (Self.m_upperTranslation - Self.m_lowerTranslation) < 2.0 * b2_linearSlop
               then
                  C2          := translation;
                  linearError := Real'max (linearError, abs translation);
                  active      := True;

               elsif translation <= Self.m_lowerTranslation
               then
                  C2          := Real'min (translation - Self.m_lowerTranslation, 0.0);
                  linearError := Real'max (linearError, Self.m_lowerTranslation - translation);
                  active      := True;

               elsif translation >= Self.m_upperTranslation
               then
                  C2          := Real'max (translation - Self.m_upperTranslation, 0.0);
                  linearError := Real'max (linearError, translation - Self.m_upperTranslation);
                  active      := True;
               end if;
            end;
         end if;


         if active
         then
            declare
               k11 : constant Real := mA + mB      +  iA * s1 * s1  +  iB * s2 * s2;
               k12 : constant Real := iA * s1      +  iB * s2;
               k13 : constant Real := iA * s1 * a1 +  iB * s2 * a2;
               k22 :          Real := iA + iB;
            begin
               if k22 = 0.0
               then
                  -- For fixed rotation
                  k22 := 1.0;
               end if;

               declare
                  k23 : constant Real :=   iA * a1 + iB * a2;
                  k33 : constant Real :=   mA + mB
                                         + iA * a1 * a1
                                         + iB * a2 * a2;

                  K   : b2Mat33;
                  C   : b2Vec3;
               begin
                  K.ex := (k11, k12, k13);
                  K.ey := (k12, k22, k23);
                  K.ez := (k13, k23, k33);

                  C.x := C1.x;
                  C.y := C1.y;
                  C.z := C2;

                  impulse := solve33 (K, -C);
               end;
            end;
         else
            declare
               k11 : constant Real :=   mA + mB
                                      + iA * s1 * s1
                                      + iB * s2 * s2;
               k12 : constant Real := iA * s1  +  iB * s2;
               k22 :          Real := iA + iB;
            begin
               if k22 = 0.0
               then
                  k22 := 1.0;
               end if;

               declare
                  K        : b2Mat22;
                  impulse1 : b2Vec2;
               begin
                  K.ex := (k11, k12);
                  K.ey := (k12, k22);

                  impulse1  := solve (K, -C1);
                  impulse.x := impulse1.x;
                  impulse.y := impulse1.y;
                  impulse.z := 0.0;
               end;
            end;
         end if;


         declare
            P  : constant b2Vec2 := impulse.x * perp  +  impulse.z             * axis;
            LA : constant Real   := impulse.x * s1    +  impulse.y + impulse.z * a1;
            LB : constant Real   := impulse.x * s2    +  impulse.y + impulse.z * a2;
         begin
            cA := cA - mA * P;
            aA := aA - iA * LA;
            cB := cB + mB * P;
            aB := aB + iB * LB;

            data.positions (Self.m_indexA).c := cA;
            data.positions (Self.m_indexA).a := aA;
            data.positions (Self.m_indexB).c := cB;
            data.positions (Self.m_indexB).a := aB;
         end;


         return  linearError <= b2_linearSlop
           and  angularError <= b2_angularSlop;
      end;
   end solvePositionConstraints;



end box2d.b2_Joint.b2_prismatic_Joint;
