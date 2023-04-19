with
     box2d.b2_Body,
     box2d.b2_time_Step,
     box2d.b2_Common;


package body box2d.b2_Joint.b2_mouse_Joint
is

   --    b2MouseJointDef()
   --    {
   --       type = e_mouseJoint;
   --       target.Set(0.0f, 0.0f);
   --       maxForce = 0.0f;
   --       stiffness = 0.0f;
   --       damping = 0.0f;
   --    }
   --

   function to_b2MouseJointDef return b2MouseJointDef
   is
      Self : b2MouseJointDef;
   begin
      Self.Kind      := e_mouseJoint;
      Self.target    := (0.0, 0.0);
      Self.maxForce  := 0.0;
      Self.stiffness := 0.0;
      Self.damping   := 0.0;

      return Self;
   end to_b2MouseJointDef;





   ----------------
   --- b2MouseJoint
   --


   --  b2Vec2 b2MouseJoint::GetAnchorA() const
   --  {
   --    return m_targetA;
   --  }
   --

   overriding
   function getAnchorA (Self : in b2MouseJoint) return b2Vec2
   is
   begin
      return Self.m_targetA;
   end getAnchorA;



   --  b2Vec2 b2MouseJoint::GetAnchorB() const
   --  {
   --    return m_bodyB->GetWorldPoint(m_localAnchorB);
   --  }
   --

   overriding
   function getAnchorB (Self : in b2MouseJoint) return b2Vec2
   is
   begin
      return Self.m_bodyB.getWorldPoint (Self.m_localAnchorB);
   end getAnchorB;



   --  b2Vec2 b2MouseJoint::GetReactionForce(float inv_dt) const
   --  {
   --    return inv_dt * m_impulse;
   --  }
   --

   overriding
   function getReactionForce (Self : in b2MouseJoint;   inv_DT : in Real) return b2Vec2
   is
   begin
      return inv_dt * Self.m_impulse;
   end getReactionForce;



   --  float b2MouseJoint::GetReactionTorque(float inv_dt) const
   --  {
   --    return inv_dt * 0.0f;
   --  }
   --

   overriding
   function getReactionTorque (Self : in b2MouseJoint;   inv_DT : in Real) return Real
   is
   begin
      return 0.0;
   end getReactionTorque;



   --  void b2MouseJoint::SetTarget(const b2Vec2& target)
   --  {
   --    if (target != m_targetA)
   --    {
   --       m_bodyB->SetAwake(true);
   --       m_targetA = target;
   --    }
   --  }
   --

   procedure setTarget (Self : in out b2MouseJoint;   Target : in b2Vec2)
   is
   begin
      if Target /= Self.m_targetA
      then
         Self.m_bodyB.setAwake (True);
         Self.m_targetA := Target;
      end if;
   end setTarget;



   --  const b2Vec2& b2MouseJoint::GetTarget() const
   --  {
   --    return m_targetA;
   --  }
   --

   function  getTarget (Self : in     b2MouseJoint)         return b2Vec2
   is
   begin
      return Self.m_targetA;
   end getTarget;



   --  void b2MouseJoint::SetMaxForce(float force)
   --  {
   --    m_maxForce = force;
   --  }
   --

   procedure setMaxForce (Self : in out b2MouseJoint;   Force : in Real)
   is
   begin
      Self.m_maxForce := Force;
   end setMaxForce;



   --  float b2MouseJoint::GetMaxForce() const
   --  {
   --    return m_maxForce;
   --  }
   --

   function  getMaxForce (Self : in     b2MouseJoint)       return Real
   is
   begin
      return Self.m_maxForce;
   end getMaxForce;



   --    Set/get the linear stiffness in N/m
   --
   --    void SetStiffness(float stiffness) { m_stiffness = stiffness; }
   --

   procedure setStiffness (Self : in out b2MouseJoint;   Stiffness : in Real)
   is
   begin
      Self.m_stiffness := Stiffness;
   end setStiffness;


   --    float GetStiffness() const { return m_stiffness; }
   --

   function getStiffness (Self : in b2MouseJoint) return Real
   is
   begin
      return Self.m_Stiffness;
   end getStiffness;





   --    Set/get linear damping in N*s/m
   --
   --    void SetDamping(float damping) { m_damping = damping; }
   --

   procedure setDamping (Self : in out b2MouseJoint;   damping : in Real)
   is
   begin
      Self.m_damping := damping;
   end setDamping;


   --    float GetDamping() const { return m_damping; }
   --

   function getDamping (Self : in b2MouseJoint) return Real
   is
   begin
      return Self.m_damping;
   end getDamping;





   --  void b2MouseJoint::ShiftOrigin(const b2Vec2& newOrigin)
   --  {
   --    m_targetA -= newOrigin;
   --  }

   overriding
   procedure shiftOrigin (Self : in out b2MouseJoint;   newOrigin : in b2Vec2)
   is
   begin
        Self.m_targetA := Self.m_targetA - newOrigin;
   end shiftOrigin;



   --    The mouse joint does not support dumping.
   --
   --    void Dump() override { b2Log("Mouse joint dumping is not supported.\n"); }
   --

   overriding
   procedure dump (Self : in b2MouseJoint)
   is
      use b2_Common;
   begin
      b2Log ("Mouse joint dumping is not supported.");
   end dump;



   --    void Draw(b2Draw* draw) const override;
   --

   overriding
   procedure draw (Self : in b2MouseJoint;   Draw : access b2Draw'Class)
   is
   begin
      draw_any (Self, Draw);
   end draw;




   -------------
   -- protected:
   --

   package body Forge
   is

      --  // p = attached point, m = mouse point
      --  // C = p - m
      --  // Cdot = v
      --  //      = v + cross(w, r)
      --  // J = [I r_skew]
      --  // Identity used:
      --  // w k % (rx i + ry j) = w * (-ry i + rx j)
      --
      --  b2MouseJoint::b2MouseJoint(const b2MouseJointDef* def)
      --  : b2Joint(def)
      --  {
      --    m_targetA = def->target;
      --    m_localAnchorB = b2MulT(m_bodyB->GetTransform(), m_targetA);
      --    m_maxForce = def->maxForce;
      --    m_stiffness = def->stiffness;
      --    m_damping = def->damping;
      --
      --    m_impulse.SetZero();
      --    m_beta = 0.0f;
      --    m_gamma = 0.0f;
      --  }
      --

      function to_b2MouseJoint (Def : in b2MouseJointDef) return b2MouseJoint
      is
         Self : b2MouseJoint;
      begin
         b2_Joint.Forge.define (Self, Def);

         Self.m_targetA      := def.target;
         Self.m_localAnchorB := b2MulT (Self.m_bodyB.getTransform,
                                        Self.m_targetA);
         Self.m_maxForce     := def.maxForce;
         Self.m_stiffness    := def.stiffness;
         Self.m_damping      := def.damping;

         Self.m_impulse      := (0.0, 0.0);
         Self.m_beta         := 0.0;
         Self.m_gamma        := 0.0;

         return Self;
      end to_b2MouseJoint;

   end Forge;





   --  void b2MouseJoint::InitVelocityConstraints(const b2SolverData& data)
   --  {
   --    m_indexB = m_bodyB->m_islandIndex;
   --    m_localCenterB = m_bodyB->m_sweep.localCenter;
   --    m_invMassB = m_bodyB->m_invMass;
   --    m_invIB = m_bodyB->m_invI;
   --
   --    b2Vec2 cB = data.positions[m_indexB].c;
   --    float aB = data.positions[m_indexB].a;
   --    b2Vec2 vB = data.velocities[m_indexB].v;
   --    float wB = data.velocities[m_indexB].w;
   --
   --    b2Rot qB(aB);
   --
   --    float mass = m_bodyB->GetMass();
   --
   --    float d = m_damping;
   --    float k = m_stiffness;
   --
   --    // magic formulas
   --    // gamma has units of inverse mass.
   --    // beta has units of inverse time.
   --    float h = data.step.dt;
   --    m_gamma = h * (d + h * k);
   --    if (m_gamma != 0.0f)
   --    {
   --       m_gamma = 1.0f / m_gamma;
   --    }
   --    m_beta = h * k * m_gamma;
   --
   --    // Compute the effective mass matrix.
   --    m_rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
   --
   --    // K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
   --    //      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
   --    //        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
   --    b2Mat22 K;
   --    K.ex.x = m_invMassB + m_invIB * m_rB.y * m_rB.y + m_gamma;
   --    K.ex.y = -m_invIB * m_rB.x * m_rB.y;
   --    K.ey.x = K.ex.y;
   --    K.ey.y = m_invMassB + m_invIB * m_rB.x * m_rB.x + m_gamma;
   --
   --    m_mass = K.GetInverse();
   --
   --    m_C = cB + m_rB - m_targetA;
   --    m_C *= m_beta;
   --
   --    // Cheat with some damping
   --    wB *= 0.98f;
   --
   --    if (data.step.warmStarting)
   --    {
   --       m_impulse *= data.step.dtRatio;
   --       vB += m_invMassB * m_impulse;
   --       wB += m_invIB * b2Cross(m_rB, m_impulse);
   --    }
   --    else
   --    {
   --       m_impulse.SetZero();
   --    }
   --
   --    data.velocities[m_indexB].v = vB;
   --    data.velocities[m_indexB].w = wB;
   --  }
   --

   overriding
   procedure initVelocityConstraints (Self : in out b2MouseJoint;   Data : in b2SolverData)
   is
   begin
     Self.m_indexB       := Self.m_bodyB.m_islandIndex;
     Self.m_localCenterB := Self.m_bodyB.m_sweep.localCenter;
     Self.m_invMassB     := Self.m_bodyB.m_invMass;
     Self.m_invIB        := Self.m_bodyB.m_invI;

      declare
         cB   : constant b2Vec2 := data.positions  (Self.m_indexB).c;
         aB   : constant Real   := data.positions  (Self.m_indexB).a;
         vB   :          b2Vec2 := data.velocities (Self.m_indexB).v;
         wB   :          Real   := data.velocities (Self.m_indexB).w;

         qB   : constant b2Rot  := to_b2Rot (aB);
         mass :          Real   := Self.m_bodyB.getMass;

         d    : constant Real   := Self.m_damping;
         k    : constant Real   := Self.m_stiffness;
         h    : constant Real   := data.step.dt;
      begin
         -- Magic formulas.
         --
         -- gamma has units of inverse mass.
         -- beta has units of inverse time.
         Self.m_gamma := h * (d + h * k);

         if Self.m_gamma /= 0.0
         then
            Self.m_gamma := 1.0 / Self.m_gamma;
         end if;

         Self.m_beta := h * k * Self.m_gamma;

         -- Compute the effective mass matrix.
         Self.m_rB := b2Mul (qB,  Self.m_localAnchorB - Self.m_localCenterB);

         -- K    = ((1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2))
         --      = (1/m1+1/m2     0    ) + invI1 * (r1.y*r1.y -r1.x*r1.y) + invI2 * (r1.y*r1.y -r1.x*r1.y)
         --        (    0     1/m1+1/m2)           (-r1.x*r1.y r1.x*r1.x)           (-r1.x*r1.y r1.x*r1.x)
         declare
            K : b2Mat22;
         begin
            K.ex.x :=   Self.m_invMassB
              + Self.m_invIB * Self.m_rB.y * Self.m_rB.y
              + Self.m_gamma;

            K.ex.y := -Self.m_invIB * Self.m_rB.x * Self.m_rB.y;
            K.ey.x :=  K.ex.y;
            K.ey.y :=   Self.m_invMassB
              + Self.m_invIB * Self.m_rB.x * Self.m_rB.x
              + Self.m_gamma;

            Self.m_mass := getInverse (K);
         end;

         Self.m_C := cB + Self.m_rB - Self.m_targetA;
         Self.m_C := Self.m_C * Self.m_beta;

         -- Cheat with some damping.
         wB := wB * 0.98;

         if data.step.warmStarting
         then
            Self.m_impulse := Self.m_impulse       * data.step.dtRatio;
            vB             := vB + Self.m_invMassB * Self.m_impulse;
            wB             := wB + Self.m_invIB    * b2Cross (Self.m_rB, Self.m_impulse);
         else
            Self.m_impulse := (0.0, 0.0);
         end if;

         data.velocities (Self.m_indexB).v := vB;
         data.velocities (Self.m_indexB).w := wB;
      end;

   end initVelocityConstraints;



   --  void b2MouseJoint::SolveVelocityConstraints(const b2SolverData& data)
   --  {
   --    b2Vec2 vB = data.velocities[m_indexB].v;
   --    float wB = data.velocities[m_indexB].w;
   --
   --    // Cdot = v + cross(w, r)
   --    b2Vec2 Cdot = vB + b2Cross(wB, m_rB);
   --    b2Vec2 impulse = b2Mul(m_mass, -(Cdot + m_C + m_gamma * m_impulse));
   --
   --    b2Vec2 oldImpulse = m_impulse;
   --    m_impulse += impulse;
   --    float maxImpulse = data.step.dt * m_maxForce;
   --    if (m_impulse.LengthSquared() > maxImpulse * maxImpulse)
   --    {
   --       m_impulse *= maxImpulse / m_impulse.Length();
   --    }
   --    impulse = m_impulse - oldImpulse;
   --
   --    vB += m_invMassB * impulse;
   --    wB += m_invIB * b2Cross(m_rB, impulse);
   --
   --    data.velocities[m_indexB].v = vB;
   --    data.velocities[m_indexB].w = wB;
   --  }
   --

   overriding
   procedure solveVelocityConstraints (Self : in out b2MouseJoint;   Data : in b2SolverData)
   is
      vB : b2Vec2 := data.velocities (Self.m_indexB).v;
      wB : Real   := data.velocities (Self.m_indexB).w;

      -- Cdot := v + cross(w, r)
      Cdot       : constant b2Vec2 := vB + b2Cross (wB, Self.m_rB);
      impulse    :          b2Vec2 := b2Mul (Self.m_mass,
                                             -(Cdot + Self.m_C + Self.m_gamma * Self.m_impulse));

      oldImpulse : constant b2Vec2 := Self.m_impulse;
      maxImpulse :          Real;

   begin
     Self.m_impulse := Self.m_impulse + impulse;
     maxImpulse     := data.step.dt * Self.m_maxForce;

      if LengthSquared (Self.m_impulse) > maxImpulse * maxImpulse
      then
         Self.m_impulse := Self.m_impulse * (maxImpulse / Length (Self.m_impulse));
      end if;

      impulse := Self.m_impulse - oldImpulse;

      vB := vB + Self.m_invMassB * impulse;
      wB := wB + Self.m_invIB    * b2Cross (Self.m_rB, impulse);

      data.velocities (Self.m_indexB).v := vB;
      data.velocities (Self.m_indexB).w := wB;
   end solveVelocityConstraints;



   --  bool b2MouseJoint::SolvePositionConstraints(const b2SolverData& data)
   --  {
   --    B2_NOT_USED(data);
   --    return true;
   --  }
   --

   overriding
   function solvePositionConstraints (Self : in out b2MouseJoint;   Data : in b2SolverData) return Boolean
   is
   begin
      return True;
   end solvePositionConstraints;


end box2d.b2_Joint.b2_mouse_Joint;
