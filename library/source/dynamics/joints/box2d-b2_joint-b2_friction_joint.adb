with
     box2d.b2_Body,
     box2d.b2_time_Step,
     box2d.b2_Common;


package body box2d.b2_Joint.b2_friction_Joint
is

   --    b2FrictionJointDef()
   --    {
   --       type = e_frictionJoint;
   --       localAnchorA.SetZero();
   --       localAnchorB.SetZero();
   --       maxForce = 0.0f;
   --       maxTorque = 0.0f;
   --    }
   --

   function to_b2FrictionJointDef return b2FrictionJointDef
   is
      Self : b2FrictionJointDef;
   begin
      Self.Kind := e_frictionJoint;
      setZero (Self.localAnchorA);
      setZero (Self.localAnchorB);

      Self.maxForce  := 0.0;
      Self.maxTorque := 0.0;

      return Self;
   end to_b2FrictionJointDef;



   --  // Point-to-point constraint
   --  // Cdot = v2 - v1
   --  //      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
   --  // J = [-I -r1_skew I r2_skew ]
   --  // Identity used:
   --  // w k % (rx i + ry j) = w * (-ry i + rx j)
   --
   --  // Angle constraint
   --  // Cdot = w2 - w1
   --  // J = [0 0 -1 0 0 1]
   --  // K = invI1 + invI2
   --




   --  void b2FrictionJointDef::Initialize(b2Body* bA, b2Body* bB, const b2Vec2& anchor)
   --  {
   --    bodyA = bA;
   --    bodyB = bB;
   --    localAnchorA = bodyA->GetLocalPoint(anchor);
   --    localAnchorB = bodyB->GetLocalPoint(anchor);
   --  }
   --

   procedure initialize (Self : out b2FrictionJointDef;   bodyA, bodyB : access b2_Body.b2Body;
                                                          anchor       : in     b2Vec2)
   is
   begin
      Self.bodyA := bodyA;
      Self.bodyB := bodyB;

      Self.localAnchorA := bodyA.getLocalPoint (anchor);
      Self.localAnchorB := bodyB.getLocalPoint (anchor);
   end initialize;




   -------------------
   --- b2FrictionJoint
   --

   --  public:
   --

   --  b2Vec2 b2FrictionJoint::GetAnchorA() const
   --  {
   --    return m_bodyA->GetWorldPoint(m_localAnchorA);
   --  }
   --

   overriding
   function getAnchorA (Self : in b2FrictionJoint) return b2Vec2
   is
   begin
      return Self.m_bodyA.getWorldPoint (Self.m_localAnchorA);
   end getAnchorA;



   --  b2Vec2 b2FrictionJoint::GetAnchorB() const
   --  {
   --    return m_bodyB->GetWorldPoint(m_localAnchorB);
   --  }
   --

   overriding
   function getAnchorB (Self : in b2FrictionJoint) return b2Vec2
   is
   begin
      return Self.m_bodyA.getWorldPoint (Self.m_localAnchorB);
   end getAnchorB;



   --  b2Vec2 b2FrictionJoint::GetReactionForce(float inv_dt) const
   --  {
   --    return inv_dt * m_linearImpulse;
   --  }
   --

   overriding
   function getReactionForce (Self : in b2FrictionJoint;   inv_dt : in Real) return b2Vec2
   is
   begin
      return inv_dt * Self.m_linearImpulse;
   end getReactionForce;



   --  float b2FrictionJoint::GetReactionTorque(float inv_dt) const
   --  {
   --    return inv_dt * m_angularImpulse;
   --  }
   --

   overriding
   function getReactionTorque (Self : in b2FrictionJoint;   inv_dt : in Real) return Real
   is
   begin
      return inv_dt * Self.m_angularImpulse;
   end getReactionTorque;



   --    The local anchor point relative to bodyA's origin.
   --    const b2Vec2& GetLocalAnchorA() const { return m_localAnchorA; }
   --

   function getLocalAnchorA (Self : in b2FrictionJoint) return b2Vec2
   is
   begin
      return Self.m_localAnchorA;
   end getLocalAnchorA;


   --    The local anchor point relative to bodyB's origin.
   --    const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }
   --

   function getLocalAnchorB (Self : in b2FrictionJoint) return b2Vec2
   is
   begin
      return Self.m_localAnchorB;
   end getLocalAnchorB;





   --  void b2FrictionJoint::SetMaxForce(float force)
   --  {
   --    b2Assert(b2IsValid(force) && force >= 0.0f);
   --    m_maxForce = force;
   --  }
   --

   procedure setMaxForce (Self : in out b2FrictionJoint;   Force : in Real)
   is
      pragma assert (    b2IsValid (force)
                     and force >= 0.0);
   begin
      Self.m_maxForce := Force;
   end setMaxForce;




   --  float b2FrictionJoint::GetMaxForce() const
   --  {
   --    return m_maxForce;
   --  }
   --

   function getMaxForce (Self : in b2FrictionJoint) return Real
   is
   begin
      return Self.m_maxForce;
   end getMaxForce;




   --  void b2FrictionJoint::SetMaxTorque(float torque)
   --  {
   --    b2Assert(b2IsValid(torque) && torque >= 0.0f);
   --    m_maxTorque = torque;
   --  }
   --

   procedure SetMaxTorque (Self : in out b2FrictionJoint;   Torque : in Real)
   is
      pragma assert (    b2IsValid (torque)
                     and torque >= 0.0);
   begin
      Self.m_maxTorque := torque;
   end SetMaxTorque;




   --  float b2FrictionJoint::GetMaxTorque() const
   --  {
   --    return m_maxTorque;
   --  }
   --

   function getMaxTorque (Self : in b2FrictionJoint) return Real
   is
   begin
      return Self.m_maxTorque;
   end getMaxTorque;





   -- protected:
   --

   package body Forge
   is
      --  b2FrictionJoint::b2FrictionJoint(const b2FrictionJointDef* def)
      --  : b2Joint(def)
      --  {
      --    m_localAnchorA = def->localAnchorA;
      --    m_localAnchorB = def->localAnchorB;
      --
      --    m_linearImpulse.SetZero();
      --    m_angularImpulse = 0.0f;
      --
      --    m_maxForce = def->maxForce;
      --    m_maxTorque = def->maxTorque;
      --  }
      --

      function to_b2FrictionJoint (Def : in b2FrictionJointDef) return b2FrictionJoint
      is
         Self : b2FrictionJoint;
      begin
         b2_Joint.Forge.define (b2Joint    (Self),
                                b2JointDef (Def));

         Self.m_localAnchorA := def.localAnchorA;
         Self.m_localAnchorB := def.localAnchorB;

         setZero (Self.m_linearImpulse);
         Self.m_angularImpulse := 0.0;

         Self.m_maxForce  := def.maxForce;
         Self.m_maxTorque := def.maxTorque;

         return Self;
      end to_b2FrictionJoint;

   end Forge;




   --  void b2FrictionJoint::InitVelocityConstraints(const b2SolverData& data)
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
   --    // Compute the effective mass matrix.
   --    m_rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
   --    m_rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
   --
   --    // J = [-I -r1_skew I r2_skew]
   --    //     [ 0       -1 0       1]
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
   procedure initVelocityConstraints (Self : in out b2FrictionJoint;   Data : in b2SolverData)
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

      begin
         -- Compute the effective mass matrix.
         --
         Self.m_rA := b2Mul (qA, Self.m_localAnchorA - Self.m_localCenterA);
         Self.m_rB := b2Mul (qB, Self.m_localAnchorB - Self.m_localCenterB);

         -- J = [-I -r1_skew I r2_skew]
         --     [ 0       -1 0       1]
         -- r_skew = [-ry; rx]

         -- Matlab
         -- K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
         --     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
         --     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

         declare
            mA : constant Real := Self.m_invMassA;
            mB : constant Real := Self.m_invMassB;

            iA : constant Real := Self.m_invIA;
            iB : constant Real := Self.m_invIB;

            K  : b2Mat22;
         begin
            K.ex.x :=   mA + mB
                      + iA * Self.m_rA.y * Self.m_rA.y
                      + iB * Self.m_rB.y * Self.m_rB.y;

            K.ex.y :=  -iA * Self.m_rA.x * Self.m_rA.y
                      - iB * Self.m_rB.x * Self.m_rB.y;

            K.ey.x := K.ex.y;
            K.ey.y := mA + mB + iA * Self.m_rA.x * Self.m_rA.x + iB * Self.m_rB.x * Self.m_rB.x;

            Self.m_linearMass  := getInverse (K);
            Self.m_angularMass := iA + iB;

            if Self.m_angularMass > 0.0
            then
               Self.m_angularMass := 1.0 / Self.m_angularMass;
            end if;

            if data.step.warmStarting
            then
               -- Scale impulses to support a variable time step.
               --
               Self.m_linearImpulse  := Self. m_linearImpulse * data.step.dtRatio;
               Self.m_angularImpulse := Self.m_angularImpulse * data.step.dtRatio;

               declare
                  P : constant b2Vec2 := (Self.m_linearImpulse.x,
                                          Self.m_linearImpulse.y);
               begin
                  vA := vA - mA * P;
                  wA := wA - iA * (b2Cross (Self.m_rA, P) + Self.m_angularImpulse);
                  vB := vB + mB * P;
                  wB := wB + iB * (b2Cross (Self.m_rB, P) + Self.m_angularImpulse);
               end;

            else
               setZero (Self.m_linearImpulse);
               Self.m_angularImpulse := 0.0;
            end if;
         end;

         data.velocities (Self.m_indexA).v := vA;
         data.velocities (Self.m_indexA).w := wA;
         data.velocities (Self.m_indexB).v := vB;
         data.velocities (Self.m_indexB).w := wB;
      end;
   end initVelocityConstraints;





   --  void b2FrictionJoint::SolveVelocityConstraints(const b2SolverData& data)
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
   --
   --    // Solve angular friction
   --    {
   --       float Cdot = wB - wA;
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
   --       b2Vec2 Cdot = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA);
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
   procedure solveVelocityConstraints (Self : in out b2FrictionJoint;   Data : in b2SolverData)
   is
      vA : b2Vec2 := data.velocities (Self.m_indexA).v;
      wA : Real   := data.velocities (Self.m_indexA).w;
      vB : b2Vec2 := data.velocities (Self.m_indexB).v;
      wB : Real   := data.velocities (Self.m_indexB).w;

      mA : constant Real := Self.m_invMassA;
      mB : constant Real := Self.m_invMassB;

      iA : constant Real := Self.m_invIA;
      iB : constant Real := Self.m_invIB;

      h  : constant Real := data.step.dt;

   begin
      -- Solve angular friction.
      --
     declare
         Cdot       : constant Real := wB - wA;
         impulse    :          Real := -Self.m_angularMass * Cdot;

         oldImpulse : constant Real := Self.m_angularImpulse;
         maxImpulse : constant Real := h * Self.m_maxTorque;
      begin
         Self.m_angularImpulse := b2Clamp (Self.m_angularImpulse + impulse,
                                           -maxImpulse,
                                            maxImpulse);
         impulse := Self.m_angularImpulse - oldImpulse;

         wA := wA - iA * impulse;
         wB := wB + iB * impulse;
      end;

      -- Solve linear friction.
      --
      declare
         Cdot       : constant b2Vec2 :=   vB + b2Cross (wB, Self.m_rB)
                                         - vA - b2Cross (wA, Self.m_rA);

         impulse    :          b2Vec2 := -b2Mul (Self.m_linearMass, Cdot);
         oldImpulse : constant b2Vec2 :=  Self.m_linearImpulse;

         maxImpulse : constant Real   := h * Self.m_maxForce;
      begin
         Self.m_linearImpulse := Self.m_linearImpulse + impulse;

         if lengthSquared (Self.m_linearImpulse) > maxImpulse * maxImpulse
         then
            normalize (Self.m_linearImpulse);
            Self.m_linearImpulse := Self.m_linearImpulse *maxImpulse;
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




   --  bool b2FrictionJoint::SolvePositionConstraints(const b2SolverData& data)
   --  {
   --    B2_NOT_USED(data);
   --
   --    return true;
   --  }
   --

   overriding
   function solvePositionConstraints (Self : in out b2FrictionJoint;   Data : in b2SolverData) return Boolean
   is
   begin
      return True;
   end solvePositionConstraints;




   --  void b2FrictionJoint::Dump()
   --  {
   --    int32 indexA = m_bodyA->m_islandIndex;
   --    int32 indexB = m_bodyB->m_islandIndex;
   --
   --    b2Dump("  b2FrictionJointDef jd;\n");
   --    b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
   --    b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
   --    b2Dump("  jd.collideConnected = bool(%d);\n", m_collideConnected);
   --    b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", m_localAnchorA.x, m_localAnchorA.y);
   --    b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", m_localAnchorB.x, m_localAnchorB.y);
   --    b2Dump("  jd.maxForce = %.9g;\n", m_maxForce);
   --    b2Dump("  jd.maxTorque = %.9g;\n", m_maxTorque);
   --    b2Dump("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
   --  }
   --

   overriding
   procedure dump (Self : in b2FrictionJoint)
   is
      use b2_Common;

      indexA : constant Natural := Self.m_bodyA.m_islandIndex;
      indexB : constant Natural := Self.m_bodyB.m_islandIndex;
   begin
     b2Dump ("  b2FrictionJointDef   jd;");
     b2Dump ("  jd.bodyA = bodies[%d];"                  & indexA                 'Image);
     b2Dump ("  jd.bodyB = bodies[%d];"                  & indexB                 'Image);
     b2Dump ("  jd.collideConnected = bool(%d);"         & Self.m_collideConnected'Image);
     b2Dump ("  jd.localAnchorA.Set(%.9g, %.9g);"        & Self.m_localAnchorA.x  'Image & Self.m_localAnchorA.y'Image);
     b2Dump ("  jd.localAnchorB.Set(%.9g, %.9g);"        & Self.m_localAnchorB.x  'Image & Self.m_localAnchorB.y'Image);
     b2Dump ("  jd.maxForce = %.9g;"                     & Self.m_maxForce        'Image);
     b2Dump ("  jd.maxTorque = %.9g;"                    & Self.m_maxTorque       'Image);
     b2Dump ("  joints[%d] = m_world->CreateJoint(&jd);" & Self.m_index           'Image);
   end dump;


end box2d.b2_Joint.b2_friction_Joint;
