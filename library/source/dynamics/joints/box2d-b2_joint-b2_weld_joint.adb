with
     box2d.b2_time_Step,
     box2d.b2_Common;


package body box2d.b2_Joint.b2_weld_Joint
is
   --    b2WeldJointDef()
   --    {
   --       type = e_weldJoint;
   --       localAnchorA.Set(0.0f, 0.0f);
   --       localAnchorB.Set(0.0f, 0.0f);
   --       referenceAngle = 0.0f;
   --       stiffness = 0.0f;
   --       damping = 0.0f;
   --    }
   --

   function  to_b2WeldJointDef return b2WeldJointDef
   is
      Self : b2WeldJointDef;
   begin
      Self.Kind           := e_weldJoint;
      Self.localAnchorA   := (0.0, 0.0);
      Self.localAnchorB   := (0.0, 0.0);
      Self.referenceAngle := 0.0;
      Self.stiffness      := 0.0;
      Self.damping        := 0.0;

      return Self;
   end to_b2WeldJointDef;




   --  // Point-to-point constraint
   --  // C = p2 - p1
   --  // Cdot = v2 - v1
   --  //      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
   --  // J = [-I -r1_skew I r2_skew ]
   --  // Identity used:
   --  // w k % (rx i + ry j) = w * (-ry i + rx j)
   --
   --  // Angle constraint
   --  // C = angle2 - angle1 - referenceAngle
   --  // Cdot = w2 - w1
   --  // J = [0 0 -1 0 0 1]
   --  // K = invI1 + invI2
   --
   --  void b2WeldJointDef::Initialize(b2Body* bA, b2Body* bB, const b2Vec2& anchor)
   --  {
   --    bodyA = bA;
   --    bodyB = bB;
   --    localAnchorA = bodyA->GetLocalPoint(anchor);
   --    localAnchorB = bodyB->GetLocalPoint(anchor);
   --    referenceAngle = bodyB->GetAngle() - bodyA->GetAngle();
   --  }
   --

   procedure initialize (Self : out b2WeldJointDef;   bodyA, bodyB : access b2Body;
                                                      Anchor       : in     b2Vec2)
   is
   begin
     Self.bodyA          := bodyA;
     Self.bodyB          := bodyB;
     Self.localAnchorA   := bodyA.getLocalPoint (Anchor);
     Self.localAnchorB   := bodyB.getLocalPoint (Anchor);
     Self.referenceAngle := bodyB.getAngle - bodyA.getAngle;
   end initialize;






   ---------------
   --- b2WeldJoint
   --

   --  b2Vec2 b2WeldJoint::GetAnchorA() const
   --  {
   --    return m_bodyA->GetWorldPoint(m_localAnchorA);
   --  }
   --

   overriding
   function getAnchorA (Self : in b2WeldJoint) return b2Vec2
   is
   begin
      return Self.m_bodyA.getWorldPoint (Self.m_localAnchorA);
   end getAnchorA;



   --  b2Vec2 b2WeldJoint::GetAnchorB() const
   --  {
   --    return m_bodyB->GetWorldPoint(m_localAnchorB);
   --  }
   --

   overriding
   function getAnchorB (Self : in b2WeldJoint) return b2Vec2
   is
   begin
      return Self.m_bodyB.getWorldPoint (Self.m_localAnchorB);
   end getAnchorB;



   --  b2Vec2 b2WeldJoint::GetReactionForce(float inv_dt) const
   --  {
   --    b2Vec2 P(m_impulse.x, m_impulse.y);
   --    return inv_dt * P;
   --  }
   --

   overriding
   function getReactionForce (Self : in b2WeldJoint;   inv_dt : in Real) return b2Vec2
   is
      P : constant b2Vec2 := (Self.m_impulse.x,
                              Self.m_impulse.y);
   begin
      return inv_dt * P;
   end getReactionForce;



   --  float b2WeldJoint::GetReactionTorque(float inv_dt) const
   --  {
   --    return inv_dt * m_impulse.z;
   --  }
   --

   overriding
   function getReactionTorque (Self : in b2WeldJoint;   inv_dt : in Real) return Real
   is
   begin
      return inv_dt * Self.m_impulse.z;
   end getReactionTorque;



   --    The local anchor point relative to bodyA's origin.
   --    const b2Vec2& GetLocalAnchorA() const { return m_localAnchorA; }
   --

   function getLocalAnchorA (Self : in b2WeldJoint) return b2Vec2
   is
   begin
      return Self.m_localAnchorA;
   end getLocalAnchorA;


   --    The local anchor point relative to bodyB's origin.
   --    const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }
   --

   function getLocalAnchorB (Self : in b2WeldJoint) return b2Vec2
   is
   begin
      return Self.m_localAnchorB;
   end getLocalAnchorB;



   --    Get the reference angle.
   --    float GetReferenceAngle() const { return m_referenceAngle; }
   --

   function getReferenceAngle (Self : in b2WeldJoint) return Real
   is
   begin
      return Self.m_referenceAngle;
   end getReferenceAngle;



   --    Set/get the linear stiffness in N/m
   --
   --    void SetStiffness(float stiffness) { m_stiffness = stiffness; }
   --

   procedure setStiffness (Self : in out b2WeldJoint;   stiffness : in Real)
   is
   begin
      Self.m_stiffness := stiffness;
   end setStiffness;



   --    float GetStiffness() const { return m_stiffness; }
   --

   function getStiffness (Self : in b2WeldJoint) return Real
   is
   begin
      return Self.m_stiffness;
   end getStiffness;



   --    Set/get linear damping in N*s/m
   --
   --    void SetDamping(float damping) { m_damping = damping; }
   --

   procedure setDamping (Self : in out b2WeldJoint;   damping : in Real)
   is
   begin
      Self.m_damping := damping;
   end setDamping;



   --    float GetDamping() const { return m_damping; }
   --

   function getDamping (Self : in b2WeldJoint) return Real
   is
   begin
      return Self.m_damping;
   end getDamping;



   --  void b2WeldJoint::Dump()
   --  {
   --    int32 indexA = m_bodyA->m_islandIndex;
   --    int32 indexB = m_bodyB->m_islandIndex;
   --
   --    b2Dump("  b2WeldJointDef jd;\n");
   --    b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
   --    b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
   --    b2Dump("  jd.collideConnected = bool(%d);\n", m_collideConnected);
   --    b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", m_localAnchorA.x, m_localAnchorA.y);
   --    b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", m_localAnchorB.x, m_localAnchorB.y);
   --    b2Dump("  jd.referenceAngle = %.9g;\n", m_referenceAngle);
   --    b2Dump("  jd.stiffness = %.9g;\n", m_stiffness);
   --    b2Dump("  jd.damping = %.9g;\n", m_damping);
   --    b2Dump("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
   --  }

   overriding
   procedure dump (Self : in b2WeldJoint)
   is
      use b2_Common;

      indexA : constant Integer := Self.m_bodyA.m_islandIndex;
      indexB : constant Integer := Self.m_bodyB.m_islandIndex;
   begin
      b2Dump ("  b2WeldJointDef jd;");
      b2Dump ("  jd.bodyA = bodies[%d];"                  & indexA                 'Image);
      b2Dump ("  jd.bodyB = bodies[%d];"                  & indexB                 'Image);
      b2Dump ("  jd.collideConnected = bool(%d);"         & Self.m_collideConnected'Image);
      b2Dump ("  jd.localAnchorA.Set(%.9g, %.9g);"        & Self.m_localAnchorA.x  'Image & Self.m_localAnchorA.y'Image);
      b2Dump ("  jd.localAnchorB.Set(%.9g, %.9g);"        & Self.m_localAnchorB.x  'Image & Self.m_localAnchorB.y'Image);
      b2Dump ("  jd.referenceAngle = %.9g;"               & Self.m_referenceAngle  'Image);
      b2Dump ("  jd.stiffness = %.9g;"                    & Self.m_stiffness       'Image);
      b2Dump ("  jd.damping = %.9g;"                      & Self.m_damping         'Image);
      b2Dump ("  joints[%d] = m_world->CreateJoint(&jd);" & Self.m_index           'Image);
   end dump;






   --------------
   --  protected:
   --
   --    friend class b2Joint;
   --

   --    b2WeldJoint(const b2WeldJointDef* def);
   --

   package body Forge
   is

      --  b2WeldJoint::b2WeldJoint(const b2WeldJointDef* def)
      --  : b2Joint(def)
      --  {
      --    m_localAnchorA = def->localAnchorA;
      --    m_localAnchorB = def->localAnchorB;
      --    m_referenceAngle = def->referenceAngle;
      --    m_stiffness = def->stiffness;
      --    m_damping = def->damping;
      --
      --    m_impulse.SetZero();
      --  }
      --

      function to_b2WeldJoint (Def : in b2WeldJointDef) return b2WeldJoint
      is
         Self : b2WeldJoint;
      begin
         Self.m_localAnchorA   := def.localAnchorA;
         Self.m_localAnchorB   := def.localAnchorB;
         Self.m_referenceAngle := def.referenceAngle;
         Self.m_stiffness      := def.stiffness;
         Self.m_damping        := def.damping;
         Self.m_impulse        := (0.0, 0.0, 0.0);

         return Self;
      end to_b2WeldJoint;

   end Forge;





   --  void b2WeldJoint::InitVelocityConstraints(const b2SolverData& data)
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
   --    b2Mat33 K;
   --    K.ex.x = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB;
   --    K.ey.x = -m_rA.y * m_rA.x * iA - m_rB.y * m_rB.x * iB;
   --    K.ez.x = -m_rA.y * iA - m_rB.y * iB;
   --    K.ex.y = K.ey.x;
   --    K.ey.y = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB;
   --    K.ez.y = m_rA.x * iA + m_rB.x * iB;
   --    K.ex.z = K.ez.x;
   --    K.ey.z = K.ez.y;
   --    K.ez.z = iA + iB;
   --
   --    if (m_stiffness > 0.0f)
   --    {
   --       K.GetInverse22(&m_mass);
   --
   --       float invM = iA + iB;
   --
   --       float C = aB - aA - m_referenceAngle;
   --
   --       // Damping coefficient
   --       float d = m_damping;
   --
   --       // Spring stiffness
   --       float k = m_stiffness;
   --
   --       // magic formulas
   --       float h = data.step.dt;
   --       m_gamma = h * (d + h * k);
   --       m_gamma = m_gamma != 0.0f ? 1.0f / m_gamma : 0.0f;
   --       m_bias = C * h * k * m_gamma;
   --
   --       invM += m_gamma;
   --       m_mass.ez.z = invM != 0.0f ? 1.0f / invM : 0.0f;
   --    }
   --    else if (K.ez.z == 0.0f)
   --    {
   --       K.GetInverse22(&m_mass);
   --       m_gamma = 0.0f;
   --       m_bias = 0.0f;
   --    }
   --    else
   --    {
   --       K.GetSymInverse33(&m_mass);
   --       m_gamma = 0.0f;
   --       m_bias = 0.0f;
   --    }
   --
   --    if (data.step.warmStarting)
   --    {
   --       // Scale impulses to support a variable time step.
   --       m_impulse *= data.step.dtRatio;
   --
   --       b2Vec2 P(m_impulse.x, m_impulse.y);
   --
   --       vA -= mA * P;
   --       wA -= iA * (b2Cross(m_rA, P) + m_impulse.z);
   --
   --       vB += mB * P;
   --       wB += iB * (b2Cross(m_rB, P) + m_impulse.z);
   --    }
   --    else
   --    {
   --       m_impulse.SetZero();
   --    }
   --
   --    data.velocities[m_indexA].v = vA;
   --    data.velocities[m_indexA].w = wA;
   --    data.velocities[m_indexB].v = vB;
   --    data.velocities[m_indexB].w = wB;
   --  }
   --

   overriding
   procedure initVelocityConstraints (Self : in out b2WeldJoint;   Data : in b2SolverData)
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

         mA :          Real;
         mB :          Real;
         iA :          Real;
         iB :          Real;

         K  :          b2Mat33;

      begin
         Self.m_rA := b2Mul (qA,  Self.m_localAnchorA - Self.m_localCenterA);
         Self.m_rB := b2Mul (qB,  Self.m_localAnchorB - Self.m_localCenterB);

         -- J =  (-I -r1_skew I r2_skew)
         --      ( 0       -1 0       1)
         -- r_skew =  (-ry; rx)

         -- Matlab
         -- K =  ( mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB)
         --      (  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB)
         --      (          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB)

         mA := Self.m_invMassA;
         mB := Self.m_invMassB;
         iA := Self.m_invIA;
         iB := Self.m_invIB;

         K.ex.x :=   mA + mB
                   + Self.m_rA.y * Self.m_rA.y * iA
                   + Self.m_rB.y * Self.m_rB.y * iB;

         K.ey.x :=  -Self.m_rA.y * Self.m_rA.x * iA
                   - Self.m_rB.y * Self.m_rB.x * iB;

         K.ez.x :=  -Self.m_rA.y * iA
                   - Self.m_rB.y * iB;

         K.ex.y := K.ey.x;

         K.ey.y :=   mA + mB
                   + Self.m_rA.x * Self.m_rA.x * iA
                   + Self.m_rB.x * Self.m_rB.x * iB;

         K.ez.y :=   Self.m_rA.x * iA
                   + Self.m_rB.x * iB;

         K.ex.z := K.ez.x;
         K.ey.z := K.ez.y;
         K.ez.z := iA + iB;


         if Self.m_stiffness > 0.0
         then
            getInverse22 (K, Self.m_mass);

            declare
               invM :          Real := iA + iB;
               C    : constant Real := aB - aA - Self.m_referenceAngle;

               -- Damping coefficient.
               d    : constant Real := Self.m_damping;

               -- Spring stiffness.
               k    : constant Real := Self.m_stiffness;

               h    : constant Real := data.step.dt;
            begin
               -- Magic formulas.
               Self.m_gamma     := h * (d + h * k);
               Self.m_gamma     := (if Self.m_gamma /= 0.0 then 1.0 / Self.m_gamma
                                                           else 0.0);
               Self.m_bias      := C * h * k * Self.m_gamma;

               invM             := invM + Self.m_gamma;
               Self.m_mass.ez.z := (if invM /= 0.0 then 1.0 / invM
                                                   else 0.0);
            end;

         elsif K.ez.z = 0.0
         then
            getInverse22 (K, Self.m_mass);
            Self.m_gamma := 0.0;
            Self.m_bias  := 0.0;

         else
            getSymInverse33 (K, Self.m_mass);
            Self.m_gamma := 0.0;
            Self.m_bias  := 0.0;
         end if;


         if data.step.warmStarting
         then
            -- Scale impulses to support a variable time step.
            multiply (Self.m_impulse, By => data.step.dtRatio);

            declare
               P : constant b2Vec2 := (Self.m_impulse.x,
                                       Self.m_impulse.y);
            begin
               vA := vA  -  mA * P;
               wA := wA  -  iA * (b2Cross (Self.m_rA, P)  +  Self.m_impulse.z);

               vB := vB  +  mB * P;
               wB := wB  +  iB * (b2Cross (Self.m_rB, P)  +  Self.m_impulse.z);
            end;
         else
            Self.m_impulse := (0.0, 0.0, 0.0);
         end if;

         data.velocities (Self.m_indexA).v := vA;
         data.velocities (Self.m_indexA).w := wA;
         data.velocities (Self.m_indexB).v := vB;
         data.velocities (Self.m_indexB).w := wB;
      end;
   end initVelocityConstraints;



   --  void b2WeldJoint::SolveVelocityConstraints(const b2SolverData& data)
   --  {
   --    b2Vec2 vA = data.velocities[m_indexA].v;
   --    float wA = data.velocities[m_indexA].w;
   --    b2Vec2 vB = data.velocities[m_indexB].v;
   --    float wB = data.velocities[m_indexB].w;
   --
   --    float mA = m_invMassA, mB = m_invMassB;
   --    float iA = m_invIA, iB = m_invIB;
   --
   --    if (m_stiffness > 0.0f)
   --    {
   --       float Cdot2 = wB - wA;
   --
   --       float impulse2 = -m_mass.ez.z * (Cdot2 + m_bias + m_gamma * m_impulse.z);
   --       m_impulse.z += impulse2;
   --
   --       wA -= iA * impulse2;
   --       wB += iB * impulse2;
   --
   --       b2Vec2 Cdot1 = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA);
   --
   --       b2Vec2 impulse1 = -b2Mul22(m_mass, Cdot1);
   --       m_impulse.x += impulse1.x;
   --       m_impulse.y += impulse1.y;
   --
   --       b2Vec2 P = impulse1;
   --
   --       vA -= mA * P;
   --       wA -= iA * b2Cross(m_rA, P);
   --
   --       vB += mB * P;
   --       wB += iB * b2Cross(m_rB, P);
   --    }
   --    else
   --    {
   --       b2Vec2 Cdot1 = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA);
   --       float Cdot2 = wB - wA;
   --       b2Vec3 Cdot(Cdot1.x, Cdot1.y, Cdot2);
   --
   --       b2Vec3 impulse = -b2Mul(m_mass, Cdot);
   --       m_impulse += impulse;
   --
   --       b2Vec2 P(impulse.x, impulse.y);
   --
   --       vA -= mA * P;
   --       wA -= iA * (b2Cross(m_rA, P) + impulse.z);
   --
   --       vB += mB * P;
   --       wB += iB * (b2Cross(m_rB, P) + impulse.z);
   --    }
   --
   --    data.velocities[m_indexA].v = vA;
   --    data.velocities[m_indexA].w = wA;
   --    data.velocities[m_indexB].v = vB;
   --    data.velocities[m_indexB].w = wB;
   --  }
   --

   overriding
   procedure solveVelocityConstraints (Self : in out b2WeldJoint;   Data : in b2SolverData)
   is
      vA :          b2Vec2 := data.velocities (Self.m_indexA).v;
      wA :          Real   := data.velocities (Self.m_indexA).w;
      vB :          b2Vec2 := data.velocities (Self.m_indexB).v;
      wB :          Real   := data.velocities (Self.m_indexB).w;

      mA : constant Real   := Self.m_invMassA;
      mB : constant Real   := Self.m_invMassB;
      iA : constant Real   := Self.m_invIA;
      iB : constant Real   := Self.m_invIB;
   begin
      if Self.m_stiffness > 0.0
      then
         declare
            Cdot2    : constant Real := wB - wA;
            impulse2 : constant Real := -Self.m_mass.ez.z * (  Cdot2
                                                             + Self.m_bias
                                                             + Self.m_gamma * Self.m_impulse.z);
            Cdot1    : b2Vec2;
            impulse1 : b2Vec2;
            P        : b2Vec2;
         begin
            Self.m_impulse.z := Self.m_impulse.z + impulse2;

            wA    := wA - iA * impulse2;
            wB    := wB + iB * impulse2;

            Cdot1 :=   vB + b2Cross (wB, Self.m_rB)
                     - vA - b2Cross (wA, Self.m_rA);

            impulse1         := -b2Mul22 (Self.m_mass, Cdot1);

            Self.m_impulse.x := Self.m_impulse.x + impulse1.x;
            Self.m_impulse.y := Self.m_impulse.y + impulse1.y;

            P  := impulse1;

            vA := vA - mA * P;
            wA := wA - iA * b2Cross (Self.m_rA, P);

            vB := vB + mB * P;
            wB := wB + iB * b2Cross (Self.m_rB, P);
         end;

      else
         declare
            Cdot1   : constant b2Vec2 :=   vB + b2Cross (wB, Self.m_rB)
                                         - vA - b2Cross (wA, Self.m_rA);
            Cdot2   : constant Real   := wB - wA;
            Cdot    : constant b2Vec3 := (Cdot1.x, Cdot1.y, Cdot2);

            impulse : constant b2Vec3 := -b2Mul (Self.m_mass, Cdot);

            P       : constant b2Vec2 := (impulse.x, impulse.y);
         begin
            Self.m_impulse := Self.m_impulse + impulse;

            vA := vA - mA * P;
            wA := wA - iA * (b2Cross (Self.m_rA, P) + impulse.z);

            vB := vB + mB * P;
            wB := wB + iB * (b2Cross (Self.m_rB, P) + impulse.z);
         end;
      end if;

      data.velocities (Self.m_indexA).v := vA;
      data.velocities (Self.m_indexA).w := wA;
      data.velocities (Self.m_indexB).v := vB;
      data.velocities (Self.m_indexB).w := wB;
   end solveVelocityConstraints;



   --  bool b2WeldJoint::SolvePositionConstraints(const b2SolverData& data)
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
   --    b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
   --    b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
   --
   --    float positionError, angularError;
   --
   --    b2Mat33 K;
   --    K.ex.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
   --    K.ey.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
   --    K.ez.x = -rA.y * iA - rB.y * iB;
   --    K.ex.y = K.ey.x;
   --    K.ey.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
   --    K.ez.y = rA.x * iA + rB.x * iB;
   --    K.ex.z = K.ez.x;
   --    K.ey.z = K.ez.y;
   --    K.ez.z = iA + iB;
   --
   --    if (m_stiffness > 0.0f)
   --    {
   --       b2Vec2 C1 =  cB + rB - cA - rA;
   --
   --       positionError = C1.Length();
   --       angularError = 0.0f;
   --
   --       b2Vec2 P = -K.Solve22(C1);
   --
   --       cA -= mA * P;
   --       aA -= iA * b2Cross(rA, P);
   --
   --       cB += mB * P;
   --       aB += iB * b2Cross(rB, P);
   --    }
   --    else
   --    {
   --       b2Vec2 C1 =  cB + rB - cA - rA;
   --       float C2 = aB - aA - m_referenceAngle;
   --
   --       positionError = C1.Length();
   --       angularError = b2Abs(C2);
   --
   --       b2Vec3 C(C1.x, C1.y, C2);
   --
   --       b2Vec3 impulse;
   --       if (K.ez.z > 0.0f)
   --       {
   --          impulse = -K.Solve33(C);
   --       }
   --       else
   --       {
   --          b2Vec2 impulse2 = -K.Solve22(C1);
   --          impulse.Set(impulse2.x, impulse2.y, 0.0f);
   --       }
   --
   --       b2Vec2 P(impulse.x, impulse.y);
   --
   --       cA -= mA * P;
   --       aA -= iA * (b2Cross(rA, P) + impulse.z);
   --
   --       cB += mB * P;
   --       aB += iB * (b2Cross(rB, P) + impulse.z);
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
   function solvePositionConstraints (Self : in out b2WeldJoint;   Data : in b2SolverData) return Boolean
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

      rA : constant b2Vec2 := b2Mul (qA, Self.m_localAnchorA - Self.m_localCenterA);
      rB : constant b2Vec2 := b2Mul (qB, Self.m_localAnchorB - Self.m_localCenterB);

      K  : b2Mat33;

      positionError,
      angularError : Real;

   begin
      K.ex.x :=   mA + mB
                + rA.y * rA.y * iA
                + rB.y * rB.y * iB;

      K.ey.x :=  -rA.y * rA.x * iA
                - rB.y * rB.x * iB;

      K.ez.x :=  -rA.y * iA
                - rB.y * iB;

      K.ex.y := K.ey.x;

      K.ey.y :=   mA + mB
                + rA.x * rA.x * iA
                + rB.x * rB.x * iB;

      K.ez.y :=   rA.x * iA
                + rB.x * iB;

      K.ex.z := K.ez.x;
      K.ey.z := K.ez.y;
      K.ez.z := iA + iB;


      if Self.m_stiffness > 0.0
      then
         declare
            C1 : constant b2Vec2 :=  cB + rB - cA - rA;
            P  :          b2Vec2;
         begin
            positionError := Length (C1);
            angularError  := 0.0;

            P  := -solve22 (K, C1);

            cA := cA - mA * P;
            aA := aA - iA * b2Cross (rA, P);

            cB := cB + mB * P;
            aB := aB + iB * b2Cross (rB, P);
         end;

      else
         declare
            C1       : constant b2Vec2 := cB + rB - cA - rA;
            C2       : constant Real   := aB - aA - Self.m_referenceAngle;
            C        :          b2Vec3;
            impulse  :          b2Vec3;
            impulse2 :          b2Vec2;
            P        :          b2Vec2;
         begin
            positionError := Length (C1);
            angularError  := abs     C2;

            C := (C1.x, C1.y, C2);

            if K.ez.z > 0.0
            then
               impulse := -solve33 (K, C);
            else
               impulse2 := -solve22 (K, C1);
               impulse  := (impulse2.x, impulse2.y, 0.0);
            end if;

            P := (impulse.x, impulse.y);

            cA := cA - mA * P;
            aA := aA - iA * (b2Cross (rA, P) + impulse.z);

            cB := cB + mB * P;
            aB := aB + iB * (b2Cross (rB, P) + impulse.z);
         end;
      end if;


      data.positions (Self.m_indexA).c := cA;
      data.positions (Self.m_indexA).a := aA;
      data.positions (Self.m_indexB).c := cB;
      data.positions (Self.m_indexB).a := aB;

      return positionError <= b2_Common.b2_linearSlop
        and  angularError  <= b2_Common.b2_angularSlop;
   end solvePositionConstraints;



end box2d.b2_Joint.b2_weld_Joint;
