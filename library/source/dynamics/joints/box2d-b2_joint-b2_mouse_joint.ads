package box2d.b2_Joint.b2_mouse_Joint
--
--  A mouse joint is used to make a point on a body track a
--  specified world point. This a soft constraint with a maximum
--  force. This allows the constraint to stretch and without
--  applying huge forces.
--
--  NOTE: this joint is not documented in the manual because it was
--  developed to be used in the testbed. If you want to learn how to
--  use the mouse joint, look at the testbed.
--
is
   -------------------
   --- b2MouseJointDef
   --

   --  Mouse joint definition. This requires a world target point,
   --  tuning parameters, and the time step.
   --

   --  struct b2MouseJointDef : public b2JointDef
   --  {
   --    The initial world target point. This is assumed
   --    to coincide with the body anchor initially.
   --    b2Vec2 target;
   --
   --    The maximum constraint force that can be exerted
   --    to move the candidate body. Usually you will express
   --    as some multiple of the weight (multiplier * mass * gravity).
   --    float maxForce;
   --
   --    The linear stiffness in N/m
   --    float stiffness;
   --
   --    The linear damping in N*s/m
   --    float damping;
   --  };
   --

   type b2MouseJointDef is new b2_Joint.b2JointDef with
      record
         Target    : b2Vec2;     -- The initial world target point. This is assumed
                                 -- to coincide with the body anchor initially.

         maxForce  : Real;       -- The maximum constraint force that can be exerted
                                 -- to move the candidate body. Usually you will express
                                 -- as some multiple of the weight (multiplier * mass * gravity).

         Stiffness : Real;       -- The linear stiffness in N/m.
         Damping   : Real;       -- The linear damping in N*s/m.
      end record;



   --    b2MouseJointDef()
   --    {
   --       type = e_mouseJoint;
   --       target.Set(0.0f, 0.0f);
   --       maxForce = 0.0f;
   --       stiffness = 0.0f;
   --       damping = 0.0f;
   --    }
   --

   function to_b2MouseJointDef return b2MouseJointDef;





   ----------------
   --- b2MouseJoint
   --

   --  A distance joint constrains two points on two bodies to remain at a fixed
   --  distance from each other. You can view this as a massless, rigid rod.
   --

   type b2MouseJoint     is new b2_Joint.b2Joint with private;
   type b2MouseJoint_ptr is access all b2MouseJoint'Class;

   --  class b2DistanceJoint : public b2Joint
   --  {
   --  public:
   --
   --    b2Vec2 GetAnchorA() const override;
   --    b2Vec2 GetAnchorB() const override;
   --

   overriding
   function getAnchorA (Self : in b2MouseJoint) return b2Vec2;

   overriding
   function getAnchorB (Self : in b2MouseJoint) return b2Vec2;



   --    Get the reaction force given the inverse time step.
   --    Unit is N.
   --    b2Vec2 GetReactionForce(float inv_dt) const override;
   --

   overriding
   function getReactionForce (Self : in b2MouseJoint;   inv_DT : in Real) return b2Vec2;



   --    Get the reaction torque given the inverse time step.
   --    Unit is N*m. This is always zero for a distance joint.
   --
   --    float GetReactionTorque(float inv_dt) const override;
   --

   overriding
   function getReactionTorque (Self : in b2MouseJoint;   inv_DT : in Real) return Real;



   --    Use this to update the target point.
   --
   --    void SetTarget(const b2Vec2& target);
   --    const b2Vec2& GetTarget() const;
   --

   procedure setTarget (Self : in out b2MouseJoint;   Target : in b2Vec2);
   function  getTarget (Self : in     b2MouseJoint)        return b2Vec2;



   --    Set/get the maximum force in Newtons.
   --
   --    void SetMaxForce(float force);
   --    float GetMaxForce() const;
   --

   procedure setMaxForce (Self : in out b2MouseJoint;   Force : in Real);
   function  getMaxForce (Self : in     b2MouseJoint)       return Real;



   --    Set/get the linear stiffness in N/m
   --
   --    void SetStiffness(float stiffness) { m_stiffness = stiffness; }
   --

   procedure setStiffness (Self : in out b2MouseJoint;   stiffness : in Real)
     with inline;


   --    float GetStiffness() const { return m_stiffness; }
   --

   function getStiffness (Self : in b2MouseJoint) return Real
     with inline;





   --    Set/get linear damping in N*s/m
   --
   --    void SetDamping(float damping) { m_damping = damping; }
   --

   procedure setDamping (Self : in out b2MouseJoint;   damping : in Real)
     with inline;


   --    float GetDamping() const { return m_damping; }
   --

   function getDamping (Self : in b2MouseJoint) return Real
     with inline;





   --    Implement b2Joint::ShiftOrigin
   --    void ShiftOrigin(const b2Vec2& newOrigin) override;
   --

   overriding
   procedure shiftOrigin (Self : in out b2MouseJoint;   newOrigin : in b2Vec2);



   --    The mouse joint does not support dumping.
   --
   --    void Dump() override { b2Log("Mouse joint dumping is not supported.\n"); }
   --

   overriding
   procedure dump (Self : in b2MouseJoint);



   --    void Draw(b2Draw* draw) const override;
   --

   overriding
   procedure draw (Self : in b2MouseJoint;   Draw : access b2Draw'Class);




   -- protected:
   --

   package Forge
   is

      --    b2MouseJoint(const b2MouseJointDef* def);
      --

      function to_b2MouseJoint (Def : in b2MouseJointDef) return b2MouseJoint;

   end Forge;




private

   --    friend class b2Joint;
   --


   --    void InitVelocityConstraints(const b2SolverData& data) override;
   --

   overriding
   procedure initVelocityConstraints (Self : in out b2MouseJoint;   Data : in b2SolverData);



   --    void SolveVelocityConstraints(const b2SolverData& data) override;
   --

   overriding
   procedure solveVelocityConstraints (Self : in out b2MouseJoint;   Data : in b2SolverData);



   --    bool SolvePositionConstraints(const b2SolverData& data) override;
   --

   overriding
   function solvePositionConstraints (Self : in out b2MouseJoint;   Data : in b2SolverData) return Boolean;




   --  class b2MouseJoint : public b2Joint
   --  {
   --  public:
   --
   --  protected:
   --    friend class b2Joint;
   --
   --
   --    b2Vec2 m_localAnchorB;
   --    b2Vec2 m_targetA;
   --    float m_stiffness;
   --    float m_damping;
   --    float m_beta;
   --
   --    // Solver shared
   --    b2Vec2 m_impulse;
   --    float m_maxForce;
   --    float m_gamma;
   --
   --    // Solver temp
   --    int32 m_indexA;
   --    int32 m_indexB;
   --    b2Vec2 m_rB;
   --    b2Vec2 m_localCenterB;
   --    float m_invMassB;
   --    float m_invIB;
   --    b2Mat22 m_mass;
   --    b2Vec2 m_C;
   --  };
   --

   type b2MouseJoint is new b2_Joint.b2Joint with
      record
         m_localAnchorB : b2Vec2;
         m_targetA      : b2Vec2;

         m_stiffness,
         m_damping,
         m_beta         : Real;

         -- Solver shared.
         --
         m_Impulse      : b2Vec2;
         m_maxForce,
         m_Gamma        : Real;

         -- Solver temp.
         --
         m_indexA,
         m_indexB       : Natural;

         m_rB           : b2Vec2;
         m_localCenterB : b2Vec2;

         m_invMassB,
         m_invIB        : Real;
         m_mass         : b2Mat22;
         m_C            : b2Vec2;
      end record;

end box2d.b2_Joint.b2_mouse_Joint;
