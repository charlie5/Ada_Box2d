with
     b2_Draw,
     b2_Common;


package body b2_Joint.b2_Distance_Joint
is
   use b2_Joint,
       b2_Common;



   --  // 1-D constrained system
   --  // m (v2 - v1) = lambda
   --  // v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
   --  // x2 = x1 + h * v2
   --
   --  // 1-D mass-damper-spring system
   --  // m (v2 - v1) + h * d * v2 + h * k *
   --
   --  // C = norm(p2 - p1) - L
   --  // u = (p2 - p1) / norm(p2 - p1)
   --  // Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
   --  // J = [-u -cross(r1, u) u cross(r2, u)]
   --  // K = J * invM * JT
   --  //   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2
   --
   --





   --  void b2DistanceJointDef::Initialize (b2Body* b1,            b2Body* b2,
   --                                       const b2Vec2& anchor1, const b2Vec2& anchor2)
   --  {
   --    bodyA = b1;
   --    bodyB = b2;
   --    localAnchorA = bodyA->GetLocalPoint(anchor1);
   --    localAnchorB = bodyB->GetLocalPoint(anchor2);
   --    b2Vec2 d = anchor2 - anchor1;
   --    length = b2Max(d.Length(), b2_linearSlop);
   --    minLength = length;
   --    maxLength = length;
   --  }
   --

   procedure initialize (Self : out b2DistanceJointDef;   bodyA,   bodyB   : access b2Body;
                                                          anchorA, anchorB : in     b2Vec2)
   is
      d : b2Vec2;
   begin
      Self.bodyA := bodyA;
      Self.bodyB := bodyB;

      Self.localAnchorA := bodyA.getLocalPoint (anchorA);
      Self.localAnchorB := bodyB.getLocalPoint (anchorB);

      d           := anchorB - anchorA;
      Self.length := Real'Max (Length (d), b2_linearSlop);

      Self.minLength := Self.length;
      Self.maxLength := Self.length;
   end initialize;




   --    b2DistanceJointDef()
   --    {
   --       type = e_distanceJoint;
   --       localAnchorA.Set(0.0f, 0.0f);
   --       localAnchorB.Set(0.0f, 0.0f);
   --       length = 1.0f;
   --       minLength = 0.0f;
   --       maxLength = FLT_MAX;
   --       stiffness = 0.0f;
   --       damping = 0.0f;
   --    }
   --

   function  to_b2DistanceJointDef return b2DistanceJointDef
   is
      Self : b2DistanceJointDef;
   begin
      Self.Kind := e_distanceJoint;

      Self.localAnchorA := (0.0, 0.0);
      Self.localAnchorB := (0.0, 0.0);

      Self.length    := 1.0;
      Self.minLength := 0.0;
      Self.maxLength := Real'Last;
      Self.stiffness := 0.0;
      Self.damping   := 0.0;

      return Self;
   end to_b2DistanceJointDef;





   package body Forge
   is

      --  b2DistanceJoint::b2DistanceJoint(const b2DistanceJointDef* def)
      --  : b2Joint(def)
      --  {
      --    m_localAnchorA = def->localAnchorA;
      --    m_localAnchorB = def->localAnchorB;
      --    m_length = b2Max(def->length, b2_linearSlop);
      --    m_minLength = b2Max(def->minLength, b2_linearSlop);
      --    m_maxLength = b2Max(def->maxLength, m_minLength);
      --    m_stiffness = def->stiffness;
      --    m_damping = def->damping;
      --
      --    m_gamma = 0.0f;
      --    m_bias = 0.0f;
      --    m_impulse = 0.0f;
      --    m_lowerImpulse = 0.0f;
      --    m_upperImpulse = 0.0f;
      --    m_currentLength = 0.0f;
      --  }
      --

      function to_b2DistanceJoint (Def : in b2DistanceJointDef) return b2DistanceJoint
      is

         Self : b2DistanceJoint;
      begin
         b2_Joint.Forge.define (b2Joint    (Self),
                                b2JointDef (Def));

         Self.m_localAnchorA := def.localAnchorA;
         Self.m_localAnchorB := def.localAnchorB;

         Self.m_length    := Real'Max (def.length,    b2_linearSlop);
         Self.m_minLength := Real'Max (def.minLength, b2_linearSlop);
         Self.m_maxLength := Real'Max (def.maxLength, Self.m_minLength);

         Self.m_stiffness := def.stiffness;
         Self.m_damping   := def.damping;

         Self.m_gamma         := 0.0;
         Self.m_bias          := 0.0;
         Self.m_impulse       := 0.0;
         Self.m_lowerImpulse  := 0.0;
         Self.m_upperImpulse  := 0.0;
         Self.m_currentLength := 0.0;

         return Self;
      end to_b2DistanceJoint;

   end Forge;





   --  void b2DistanceJoint::InitVelocityConstraints(const b2SolverData& data)
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
   --    m_u = cB + m_rB - cA - m_rA;
   --
   --    // Handle singularity.
   --    m_currentLength = m_u.Length();
   --    if (m_currentLength > b2_linearSlop)
   --    {
   --       m_u *= 1.0f / m_currentLength;
   --    }
   --    else
   --    {
   --       m_u.Set(0.0f, 0.0f);
   --       m_mass = 0.0f;
   --       m_impulse = 0.0f;
   --       m_lowerImpulse = 0.0f;
   --       m_upperImpulse = 0.0f;
   --    }
   --
   --    float crAu = b2Cross(m_rA, m_u);
   --    float crBu = b2Cross(m_rB, m_u);
   --    float invMass = m_invMassA + m_invIA * crAu * crAu + m_invMassB + m_invIB * crBu * crBu;
   --    m_mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;
   --
   --    if (m_stiffness > 0.0f && m_minLength < m_maxLength)
   --    {
   --       // soft
   --       float C = m_currentLength - m_length;
   --
   --       float d = m_damping;
   --       float k = m_stiffness;
   --
   --       // magic formulas
   --       float h = data.step.dt;
   --
   --       // gamma = 1 / (h * (d + h * k))
   --       // the extra factor of h in the denominator is since the lambda is an impulse, not a force
   --       m_gamma = h * (d + h * k);
   --       m_gamma = m_gamma != 0.0f ? 1.0f / m_gamma : 0.0f;
   --       m_bias = C * h * k * m_gamma;
   --
   --       invMass += m_gamma;
   --       m_softMass = invMass != 0.0f ? 1.0f / invMass : 0.0f;
   --    }
   --    else
   --    {
   --       // rigid
   --       m_gamma = 0.0f;
   --       m_bias = 0.0f;
   --       m_softMass = m_mass;
   --    }
   --
   --    if (data.step.warmStarting)
   --    {
   --       // Scale the impulse to support a variable time step.
   --       m_impulse *= data.step.dtRatio;
   --       m_lowerImpulse *= data.step.dtRatio;
   --       m_upperImpulse *= data.step.dtRatio;
   --
   --       b2Vec2 P = (m_impulse + m_lowerImpulse - m_upperImpulse) * m_u;
   --       vA -= m_invMassA * P;
   --       wA -= m_invIA * b2Cross(m_rA, P);
   --       vB += m_invMassB * P;
   --       wB += m_invIB * b2Cross(m_rB, P);
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
   procedure initVelocityConstraints (Self : in out b2DistanceJoint;   Data : in b2SolverData)
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

         qA : constant b2Rot  := to_b2Rot (Angle => aA);
         qB : constant b2Rot  := to_b2Rot (Angle => aB);

      begin
         Self.m_rA := b2Mul (qA, Self.m_localAnchorA - Self.m_localCenterA);
         Self.m_rB := b2Mul (qB, Self.m_localAnchorB - Self.m_localCenterB);
         Self.m_u  :=   cB + Self.m_rB
                      - cA - Self.m_rA;

         -- Handle singularity.
         --
         Self.m_currentLength := Length(Self.m_u);

         if Self.m_currentLength > b2_linearSlop
         then
            Self.m_u := Self.m_u * (1.0 / Self.m_currentLength);

         else
            Self.m_u            := (0.0, 0.0);
            Self.m_mass         :=  0.0;
            Self.m_impulse      :=  0.0;
            Self.m_lowerImpulse :=  0.0;
            Self.m_upperImpulse :=  0.0;
         end if;


         declare
            crAu    : constant Real := b2Cross (Self.m_rA, Self.m_u);
            crBu    : constant Real := b2Cross (Self.m_rB, Self.m_u);

            invMass :          Real :=   Self.m_invMassA + Self.m_invIA * crAu * crAu
                                       + Self.m_invMassB + Self.m_invIB * crBu * crBu;
         begin
            Self.m_mass := (if invMass /= 0.0 then 1.0 / invMass
                                              else 0.0);

            if    Self.m_stiffness > 0.0
              and Self.m_minLength < Self.m_maxLength
            then
               declare
                  -- soft
                  C : constant Real := Self.m_currentLength - Self.m_length;

                  d : constant Real := Self.m_damping;
                  k : constant Real := Self.m_stiffness;

                  -- magic formulas
                  h : constant Real := data.step.dt;
               begin
                  -- gamma = 1 / (h * (d + h * k))
                  -- the extra factor of h in the denominator is since the lambda is an impulse, not a force
                  Self.m_gamma := h * (d + h * k);
                  Self.m_gamma := (if Self.m_gamma /= 0.0 then 1.0 / Self.m_gamma
                                                          else 0.0);
                  Self.m_bias  := C * h * k * Self.m_gamma;

                  invMass         := invMass + Self.m_gamma;
                  Self.m_softMass := (if invMass /= 0.0 then 1.0 / invMass
                                                        else 0.0);
               end;
            else
               -- rigid
               Self.m_gamma    := 0.0;
               Self.m_bias     := 0.0;
               Self.m_softMass := Self.m_mass;
            end if;


            if data.step.warmStarting
            then
               -- Scale the impulse to support a variable time step.
               --
               Self.m_impulse      := Self.m_impulse      * data.step.dtRatio;
               Self.m_lowerImpulse := Self.m_lowerImpulse * data.step.dtRatio;
               Self.m_upperImpulse := Self.m_upperImpulse * data.step.dtRatio;

               declare
                  P : constant b2Vec2 :=  (Self.m_impulse + Self.m_lowerImpulse - Self.m_upperImpulse)
                                         * Self.m_u;
               begin
                  vA := vA - Self.m_invMassA * P;
                  wA := wA - Self.m_invIA    * b2Cross (Self.m_rA, P);
                  vB := vB + Self.m_invMassB * P;
                  wB := wB + Self.m_invIB    * b2Cross (Self.m_rB, P);
               end;
            else
               Self.m_impulse := 0.0;
            end if;

         end;

         data.velocities (Self.m_indexA).v := vA;
         data.velocities (Self.m_indexA).w := wA;
         data.velocities (Self.m_indexB).v := vB;
         data.velocities (Self.m_indexB).w := wB;
      end;
   end initVelocityConstraints;





   --  void b2DistanceJoint::SolveVelocityConstraints(const b2SolverData& data)
   --  {
   --    b2Vec2 vA = data.velocities[m_indexA].v;
   --    float wA = data.velocities[m_indexA].w;
   --    b2Vec2 vB = data.velocities[m_indexB].v;
   --    float wB = data.velocities[m_indexB].w;
   --
   --    if (m_minLength < m_maxLength)
   --    {
   --       if (m_stiffness > 0.0f)
   --       {
   --          // Cdot = dot(u, v + cross(w, r))
   --          b2Vec2 vpA = vA + b2Cross(wA, m_rA);
   --          b2Vec2 vpB = vB + b2Cross(wB, m_rB);
   --          float Cdot = b2Dot(m_u, vpB - vpA);
   --
   --          float impulse = -m_softMass * (Cdot + m_bias + m_gamma * m_impulse);
   --          m_impulse += impulse;
   --
   --          b2Vec2 P = impulse * m_u;
   --          vA -= m_invMassA * P;
   --          wA -= m_invIA * b2Cross(m_rA, P);
   --          vB += m_invMassB * P;
   --          wB += m_invIB * b2Cross(m_rB, P);
   --       }
   --
   --       // lower
   --       {
   --          float C = m_currentLength - m_minLength;
   --          float bias = b2Max(0.0f, C) * data.step.inv_dt;
   --
   --          b2Vec2 vpA = vA + b2Cross(wA, m_rA);
   --          b2Vec2 vpB = vB + b2Cross(wB, m_rB);
   --          float Cdot = b2Dot(m_u, vpB - vpA);
   --
   --          float impulse = -m_mass * (Cdot + bias);
   --          float oldImpulse = m_lowerImpulse;
   --          m_lowerImpulse = b2Max(0.0f, m_lowerImpulse + impulse);
   --          impulse = m_lowerImpulse - oldImpulse;
   --          b2Vec2 P = impulse * m_u;
   --
   --          vA -= m_invMassA * P;
   --          wA -= m_invIA * b2Cross(m_rA, P);
   --          vB += m_invMassB * P;
   --          wB += m_invIB * b2Cross(m_rB, P);
   --       }
   --
   --       // upper
   --       {
   --          float C = m_maxLength - m_currentLength;
   --          float bias = b2Max(0.0f, C) * data.step.inv_dt;
   --
   --          b2Vec2 vpA = vA + b2Cross(wA, m_rA);
   --          b2Vec2 vpB = vB + b2Cross(wB, m_rB);
   --          float Cdot = b2Dot(m_u, vpA - vpB);
   --
   --          float impulse = -m_mass * (Cdot + bias);
   --          float oldImpulse = m_upperImpulse;
   --          m_upperImpulse = b2Max(0.0f, m_upperImpulse + impulse);
   --          impulse = m_upperImpulse - oldImpulse;
   --          b2Vec2 P = -impulse * m_u;
   --
   --          vA -= m_invMassA * P;
   --          wA -= m_invIA * b2Cross(m_rA, P);
   --          vB += m_invMassB * P;
   --          wB += m_invIB * b2Cross(m_rB, P);
   --       }
   --    }
   --    else
   --    {
   --       // Equal limits
   --
   --       // Cdot = dot(u, v + cross(w, r))
   --       b2Vec2 vpA = vA + b2Cross(wA, m_rA);
   --       b2Vec2 vpB = vB + b2Cross(wB, m_rB);
   --       float Cdot = b2Dot(m_u, vpB - vpA);
   --
   --       float impulse = -m_mass * Cdot;
   --       m_impulse += impulse;
   --
   --       b2Vec2 P = impulse * m_u;
   --       vA -= m_invMassA * P;
   --       wA -= m_invIA * b2Cross(m_rA, P);
   --       vB += m_invMassB * P;
   --       wB += m_invIB * b2Cross(m_rB, P);
   --    }
   --
   --    data.velocities[m_indexA].v = vA;
   --    data.velocities[m_indexA].w = wA;
   --    data.velocities[m_indexB].v = vB;
   --    data.velocities[m_indexB].w = wB;
   --  }
   --

   overriding
   procedure solveVelocityConstraints (Self : in out b2DistanceJoint;   Data : in b2SolverData)
   is
      vA : b2Vec2 := data.velocities (Self.m_indexA).v;
      wA : Real   := data.velocities (Self.m_indexA).w;
      vB : b2Vec2 := data.velocities (Self.m_indexB).v;
      wB : Real   := data.velocities (Self.m_indexB).w;

   begin
      if Self.m_minLength < Self.m_maxLength
      then
        if Self.m_stiffness > 0.0
         then
            declare
               -- Cdot = dot(u, v + cross(w, r))
               vpA     : constant b2Vec2 := vA + b2Cross(wA, Self.m_rA);
               vpB     : constant b2Vec2 := vB + b2Cross(wB, Self.m_rB);
               Cdot    : constant Real   := b2Dot (Self.m_u,  vpB - vpA);
               impulse : constant Real   :=   -Self.m_softMass
                                   * (Cdot + Self.m_bias + Self.m_gamma * Self.m_impulse);
               P       : constant b2Vec2 := impulse * Self.m_u;
            begin
               Self.m_impulse := Self.m_impulse + impulse;

               vA := vA - Self.m_invMassA * P;
               wA := wA - Self.m_invIA * b2Cross (Self.m_rA, P);
               vB := vB + Self.m_invMassB * P;
               wB := wB + Self.m_invIB * b2Cross (Self.m_rB, P);
            end;
        end if;

        -- lower
        declare
            C    : constant Real := Self.m_currentLength - Self.m_minLength;
            bias : constant Real := Real'Max (0.0, C) * data.step.inv_dt;

            vpA  : constant b2Vec2 := vA + b2Cross (wA, Self.m_rA);
            vpB  : constant b2Vec2 := vB + b2Cross (wB, Self.m_rB);
            Cdot : constant Real   := b2Dot (Self.m_u,  vpB - vpA);

            impulse    : Real := -Self.m_mass * (Cdot + bias);
            oldImpulse : constant Real :=  Self.m_lowerImpulse;

            P : b2Vec2;
         begin
            Self.m_lowerImpulse := Real'Max (0.0,  Self.m_lowerImpulse + impulse);
            impulse             := Self.m_lowerImpulse - oldImpulse;

            P  := impulse * Self.m_u;

            vA := vA - Self.m_invMassA * P;
            wA := wA - Self.m_invIA * b2Cross (Self.m_rA, P);
            vB := vB + Self.m_invMassB * P;
            wB := wB + Self.m_invIB * b2Cross (Self.m_rB, P);
        end;

        -- upper
        declare
            C    : constant Real   := Self.m_maxLength - Self.m_currentLength;
            bias : constant Real   := Real'Max (0.0, C) * data.step.inv_dt;

            vpA  : constant b2Vec2 := vA + b2Cross(wA, Self.m_rA);
            vpB  : constant b2Vec2 := vB + b2Cross(wB, Self.m_rB);
            Cdot : constant Real   := b2Dot (Self.m_u, vpA - vpB);

            impulse    : Real := -Self.m_mass * (Cdot + bias);
            oldImpulse : constant Real :=  Self.m_upperImpulse;

            P    : b2Vec2;
         begin
            Self.m_upperImpulse := Real'Max (0.0,  Self.m_upperImpulse + impulse);
            impulse             := Self.m_upperImpulse - oldImpulse;

            P := -impulse * Self.m_u;

            vA := vA - Self.m_invMassA * P;
            wA := wA - Self.m_invIA * b2Cross (Self.m_rA, P);
            vB := vB + Self.m_invMassB * P;
            wB := wB + Self.m_invIB * b2Cross (Self.m_rB, P);
        end;

     else     -- Equal limits
         declare
            -- Cdot = dot(u, v + cross(w, r))
            vpA     : constant b2Vec2 := vA + b2Cross (wA, Self.m_rA);
            vpB     : constant b2Vec2 := vB + b2Cross (wB, Self.m_rB);
            Cdot    : constant Real   := b2Dot (Self.m_u, vpB - vpA);
            impulse : constant Real   := -Self.m_mass * Cdot;

            P : b2Vec2;
         begin
            Self.m_impulse := Self.m_impulse + impulse;

            P  := impulse * Self.m_u;

            vA := vA - Self.m_invMassA * P;
            wA := wA - Self.m_invIA * b2Cross (Self.m_rA, P);
            vB := vB + Self.m_invMassB * P;
            wB := wB + Self.m_invIB * b2Cross (Self.m_rB, P);
         end;
      end if;

      data.velocities (Self.m_indexA).v := vA;
      data.velocities (Self.m_indexA).w := wA;
      data.velocities (Self.m_indexB).v := vB;
      data.velocities (Self.m_indexB).w := wB;
   end solveVelocityConstraints;





   --  bool b2DistanceJoint::SolvePositionConstraints(const b2SolverData& data)
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
   --    b2Vec2 u = cB + rB - cA - rA;
   --
   --    float length = u.Normalize();
   --    float C;
   --    if (m_minLength == m_maxLength)
   --    {
   --       C = length - m_minLength;
   --    }
   --    else if (length < m_minLength)
   --    {
   --       C = length - m_minLength;
   --    }
   --    else if (m_maxLength < length)
   --    {
   --       C = length - m_maxLength;
   --    }
   --    else
   --    {
   --       return true;
   --    }
   --
   --    float impulse = -m_mass * C;
   --    b2Vec2 P = impulse * u;
   --
   --    cA -= m_invMassA * P;
   --    aA -= m_invIA * b2Cross(rA, P);
   --    cB += m_invMassB * P;
   --    aB += m_invIB * b2Cross(rB, P);
   --
   --    data.positions[m_indexA].c = cA;
   --    data.positions[m_indexA].a = aA;
   --    data.positions[m_indexB].c = cB;
   --    data.positions[m_indexB].a = aB;
   --
   --    return b2Abs(C) < b2_linearSlop;
   --  }
   --

   overriding
   function solvePositionConstraints (Self : in out b2DistanceJoint;   Data : in b2SolverData) return Boolean
   is
      cA : b2Vec2 := data.positions (Self.m_indexA).c;
      aA : Real   := data.positions (Self.m_indexA).a;
      cB : b2Vec2 := data.positions (Self.m_indexB).c;
      aB : Real   := data.positions (Self.m_indexB).a;

      qA : constant b2Rot  := to_b2Rot (aA);
      qB : constant b2Rot  := to_b2Rot (aB);

      rA : constant b2Vec2 := b2Mul (qA, Self.m_localAnchorA - Self.m_localCenterA);
      rB : constant b2Vec2 := b2Mul (qB, Self.m_localAnchorB - Self.m_localCenterB);
      u  :          b2Vec2 :=   cB + rB
                              - cA - rA;

      length : constant Real := normalize (u);
      C      :          Real;

   begin
      if Self.m_minLength = Self.m_maxLength
      then
         C := length - Self.m_minLength;

      elsif length < Self.m_minLength
      then
         C := length - Self.m_minLength;

      elsif Self.m_maxLength < length
      then
         C := length - Self.m_maxLength;

      else
         return true;
      end if;

      declare
         impulse : constant Real   := -Self.m_mass * C;
         P       : constant b2Vec2 :=  impulse * u;
      begin
         cA := cA - Self.m_invMassA * P;
         aA := aA - Self.m_invIA * b2Cross (rA, P);
         cB := cB + Self.m_invMassB * P;
         aB := aB + Self.m_invIB * b2Cross (rB, P);
      end;

      data.positions (Self.m_indexA).c := cA;
      data.positions (Self.m_indexA).a := aA;
      data.positions (Self.m_indexB).c := cB;
      data.positions (Self.m_indexB).a := aB;

      return abs (C) < b2_linearSlop;
   end solvePositionConstraints;




   --  b2Vec2 b2DistanceJoint::GetAnchorA() const
   --  {
   --    return m_bodyA->GetWorldPoint(m_localAnchorA);
   --  }
   --

   overriding
   function getAnchorA (Self : in b2DistanceJoint) return b2Vec2
   is
   begin
      return Self.m_bodyA.getWorldPoint (Self.m_localAnchorA);
   end getAnchorA;



   --  b2Vec2 b2DistanceJoint::GetAnchorB() const
   --  {
   --    return m_bodyB->GetWorldPoint(m_localAnchorB);
   --  }
   --

   overriding
   function getAnchorB (Self : in b2DistanceJoint) return b2Vec2
   is
   begin
      return Self.m_bodyB.getWorldPoint (Self.m_localAnchorB);
   end getAnchorB;



   --  b2Vec2 b2DistanceJoint::GetReactionForce(float inv_dt) const
   --  {
   --    b2Vec2 F = inv_dt * (m_impulse + m_lowerImpulse - m_upperImpulse) * m_u;
   --    return F;
   --  }
   --

   overriding
   function getReactionForce (Self : in b2DistanceJoint;   inv_dt : in Real) return b2Vec2
   is
      F : constant b2Vec2 := inv_dt * (Self.m_impulse + Self.m_lowerImpulse - Self.m_upperImpulse) * Self.m_u;
   begin
      return F;
   end getReactionForce;




   --  float b2DistanceJoint::GetReactionTorque(float inv_dt) const
   --  {
   --    B2_NOT_USED(inv_dt);
   --    return 0.0f;
   --  }
   --

   overriding
   function getReactionTorque (Self : in b2DistanceJoint;   inv_dt : in Real) return Real
   is
   begin
      return 0.0;
   end getReactionTorque;




   --  float b2DistanceJoint::SetLength(float length)
   --  {
   --    m_impulse = 0.0f;
   --    m_length = b2Max(b2_linearSlop, length);
   --    return m_length;
   --  }
   --

   function setLength (Self : in out b2DistanceJoint;   Length : in Real) return Real
   is
   begin
      Self.m_impulse := 0.0;
      Self.m_length  := Real'max (b2_linearSlop, length);

      return Self.m_length;
   end setLength;




   --  float b2DistanceJoint::SetMinLength(float minLength)
   --  {
   --    m_lowerImpulse = 0.0f;
   --    m_minLength = b2Clamp(minLength, b2_linearSlop, m_maxLength);
   --    return m_minLength;
   --  }
   --

   function setMinLength (Self : in out b2DistanceJoint;   minLength : in Real) return Real
   is
   begin
      Self.m_lowerImpulse := 0.0;
      Self.m_minLength    := b2Clamp (minLength, b2_linearSlop, Self.m_maxLength);

      return Self.m_minLength;
   end setMinLength;




   --  float b2DistanceJoint::SetMaxLength(float maxLength)
   --  {
   --    m_upperImpulse = 0.0f;
   --    m_maxLength = b2Max(maxLength, m_minLength);
   --    return m_maxLength;
   --  }
   --

   function setMaxLength (Self : in out b2DistanceJoint;   maxLength : in Real) return Real
   is
   begin
      Self.m_upperImpulse := 0.0;
      Self.m_maxLength    := Real'max (maxLength, Self.m_minLength);

      return Self.m_maxLength;
   end setMaxLength;




   --  float b2DistanceJoint::GetCurrentLength() const
   --  {
   --    b2Vec2 pA = m_bodyA->GetWorldPoint(m_localAnchorA);
   --    b2Vec2 pB = m_bodyB->GetWorldPoint(m_localAnchorB);
   --    b2Vec2 d = pB - pA;
   --    float length = d.Length();
   --    return length;
   --  }
   --

   function getCurrentLength (Self : in b2DistanceJoint) return Real
   is
      pA : constant b2Vec2 := Self.m_bodyA.getWorldPoint (Self.m_localAnchorA);
      pB : constant b2Vec2 := Self.m_bodyB.getWorldPoint (Self.m_localAnchorB);
      d  : constant b2Vec2 := pB - pA;

      length : constant Real := b2_Math.Length (d);
   begin
      return length;
   end getCurrentLength;





   --    The local anchor point relative to bodyA's origin.
   --    const b2Vec2& GetLocalAnchorA() const { return m_localAnchorA; }
   --

   function getLocalAnchorA (Self : in b2DistanceJoint) return b2Vec2
   is
   begin
      return Self.m_localAnchorA;
   end getLocalAnchorA;


   --    The local anchor point relative to bodyB's origin.
   --    const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }
   --

   function getLocalAnchorB (Self : in b2DistanceJoint) return b2Vec2
   is
   begin
      return Self.m_localAnchorB;
   end getLocalAnchorB;



     --    Get the rest length
     --    float GetLength() const { return m_length; }
     --

   function getLength (Self : in b2DistanceJoint) return Real
   is
   begin
      return Self.m_length;
   end getLength;




   --    Get the minimum length
   --    float GetMinLength() const { return m_minLength; }
   --

   function getMinLength (Self : in b2DistanceJoint) return Real
   is
   begin
      return Self.m_minLength;
   end getMinLength;



   --    Get the maximum length
   --    float GetMaxLength() const { return m_maxLength; }
   --
   function getMaxLength (Self : in b2DistanceJoint) return Real
   is
   begin
      return Self.m_maxLength;
   end getMaxLength;




     --    Set/get the linear stiffness in N/m
     --
     --    void SetStiffness(float stiffness) { m_stiffness = stiffness; }
     --

   procedure setStiffness (Self : in out b2DistanceJoint;   stiffness : in Real)
   is
   begin
      Self.m_stiffness := stiffness;
   end setStiffness;


   --    float GetStiffness() const { return m_stiffness; }
   --

   function getStiffness (Self : in b2DistanceJoint) return Real
   is
   begin
      return Self.m_stiffness;
   end getStiffness;


   --    Set/get linear damping in N*s/m
   --
   --    void SetDamping(float damping) { m_damping = damping; }
   --

   procedure setDamping (Self : in out b2DistanceJoint;   damping : in Real)
   is
   begin
      Self.m_damping := damping;
   end setDamping;


   --    float GetDamping() const { return m_damping; }
   --

   function getDamping (Self : in b2DistanceJoint) return Real
   is
   begin
      return Self.m_damping;
   end getDamping;





   --  void b2DistanceJoint::Dump()
   --  {
   --    int32 indexA = m_bodyA->m_islandIndex;
   --    int32 indexB = m_bodyB->m_islandIndex;
   --
   --    b2Dump("  b2DistanceJointDef jd;\n");
   --    b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
   --    b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
   --    b2Dump("  jd.collideConnected = bool(%d);\n", m_collideConnected);
   --    b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", m_localAnchorA.x, m_localAnchorA.y);
   --    b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", m_localAnchorB.x, m_localAnchorB.y);
   --    b2Dump("  jd.length = %.9g;\n", m_length);
   --    b2Dump("  jd.minLength = %.9g;\n", m_minLength);
   --    b2Dump("  jd.maxLength = %.9g;\n", m_maxLength);
   --    b2Dump("  jd.stiffness = %.9g;\n", m_stiffness);
   --    b2Dump("  jd.damping = %.9g;\n", m_damping);
   --    b2Dump("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
   --  }
   --

   overriding
   procedure dump (Self : in b2DistanceJoint)
   is
   begin
      raise Program_Error with "TODO";
   end dump;





   --  void b2DistanceJoint::Draw(b2Draw* draw) const
   --  {
   --    const b2Transform& xfA = m_bodyA->GetTransform();
   --    const b2Transform& xfB = m_bodyB->GetTransform();
   --    b2Vec2 pA = b2Mul(xfA, m_localAnchorA);
   --    b2Vec2 pB = b2Mul(xfB, m_localAnchorB);
   --
   --    b2Vec2 axis = pB - pA;
   --    float length = axis.Normalize();
   --
   --    b2Color c1(0.7f, 0.7f, 0.7f);
   --    b2Color c2(0.3f, 0.9f, 0.3f);
   --    b2Color c3(0.9f, 0.3f, 0.3f);
   --    b2Color c4(0.4f, 0.4f, 0.4f);
   --
   --    draw->DrawSegment(pA, pB, c4);
   --
   --    b2Vec2 pRest = pA + m_length * axis;
   --    draw->DrawPoint(pRest, 8.0f, c1);
   --
   --    if (m_minLength != m_maxLength)
   --    {
   --       if (m_minLength > b2_linearSlop)
   --       {
   --          b2Vec2 pMin = pA + m_minLength * axis;
   --          draw->DrawPoint(pMin, 4.0f, c2);
   --       }
   --
   --       if (m_maxLength < FLT_MAX)
   --       {
   --          b2Vec2 pMax = pA + m_maxLength * axis;
   --          draw->DrawPoint(pMax, 4.0f, c3);
   --       }
   --    }
   --  }

   overriding
   procedure draw (Self : in b2DistanceJoint)
   is
   begin
      raise Program_Error with "TODO";
   end draw;




end b2_Joint.b2_Distance_Joint;
