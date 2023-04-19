with
     box2d.b2_Body;


package box2d.b2_Joint.b2_motor_Joint
--
--  A motor joint is used to control the relative motion
--  between two bodies. A typical usage is to control the movement
--  of a dynamic body with respect to the ground.
--
is
   use b2_Body;



   -----------------------
   --- b2MotorJointDef ---
   -----------------------


   --  Motor joint definition.

   --  struct b2MotorJointDef : public b2JointDef
   --  {
   --    Position of bodyB minus the position of bodyA, in bodyA's frame, in meters.
   --    b2Vec2 linearOffset;
   --

   --    The bodyB angle minus bodyA angle in radians.
   --    float angularOffset;
   --

   --    The maximum motor force in N.
   --    float maxForce;
   --

   --    The maximum motor torque in N-m.
   --    float maxTorque;
   --

   --    Position correction factor in the range [0,1].
   --    float correctionFactor;
   --  };
   --


   type b2MotorJointDef is new b2_Joint.b2JointDef with
      record
         linearOffset     : b2Vec2;     -- Position of bodyB minus the position of bodyA, in bodyA's frame, in meters.
         angularOffset    : Real;       -- The bodyB angle minus bodyA angle in radians.

         maxForce         : Real;       -- The maximum motor force  in N.
         maxTorque        : Real;       -- The maximum motor torque in N-m.

         correctionFactor : Real;       -- Position correction factor in the range [0,1].
      end record;


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

   function  to_b2MotorJointDef return b2MotorJointDef;



   --    Initialize the bodies and offsets using the current transforms.
   --    void Initialize(b2Body* bodyA, b2Body* bodyB);
   --

   procedure initialize (Self : out b2MotorJointDef;   BodyA, BodyB : in b2Body_ptr);





   --------------------
   --- b2MotorJoint ---
   --------------------

   type b2MotorJoint     is new b2_Joint.b2Joint with private;
   type b2MotorJoint_ptr is access all b2MotorJoint'Class;

   --    b2Vec2 GetAnchorA() const override;
   --    b2Vec2 GetAnchorB() const override;
   --

   overriding
   function getAnchorA (Self : in b2MotorJoint) return b2Vec2;

   overriding
   function getAnchorB (Self : in b2MotorJoint) return b2Vec2;



   --    Get the reaction force given the inverse time step.
   --    Unit is N.
   --
   --    b2Vec2 GetReactionForce(float inv_dt) const override;
   --

   overriding
   function getReactionForce (Self : in b2MotorJoint;   inv_dt : in Real) return b2Vec2;



   --    Get the reaction torque given the inverse time step.
   --    Unit is N*m. This is always zero for a distance joint.
   --
   --    float GetReactionTorque(float inv_dt) const override;
   --

   overriding
   function getReactionTorque (Self : in b2MotorJoint;   inv_dt : in Real) return Real;



   --    Set the target linear offset, in frame A, in meters.
   --
   --    void SetLinearOffset(const b2Vec2& linearOffset);
   --

   procedure setLinearOffset (Self : in out b2MotorJoint;   linearOffset : in b2Vec2);



   --    Set/get the target linear offset, in frame A, in meters.
   --
   --    const b2Vec2& GetLinearOffset() const;
   --

   function getLinearOffset (Self : in b2MotorJoint) return b2Vec2;




   --    Set the target angular offset, in radians.
   --
   --    void SetAngularOffset(float angularOffset);
   --

   procedure setAngularOffset (Self : in out b2MotorJoint;   angularOffset : in Real);



   --    Get the target angular offset, in radians.
   --
   --    float GetAngularOffset() const;
   --

   function getAngularOffset (Self : in b2MotorJoint) return Real;



   --    Set the maximum friction force in N.
   --
   --    void SetMaxForce(float force);
   --

   procedure setMaxForce (Self : in out b2MotorJoint;   Force : in Real);



   --    Get the maximum friction force in N.
   --
   --    float GetMaxForce() const;
   --

   function getMaxForce (Self : in b2MotorJoint) return Real;



   --    Set the maximum friction torque in N*m.
   --
   --    void SetMaxTorque(float torque);
   --

   procedure setMaxTorque (Self : in out b2MotorJoint;   Torque : in Real);




   --    Get the maximum friction torque in N*m.
   --
   --    float GetMaxTorque() const;
   --

   function getMaxTorque (Self : in b2MotorJoint) return Real;



   --    Set the position correction factor in the range [0,1].
   --
   --    void SetCorrectionFactor(float factor);
   --

   procedure setCorrectionFactor (Self : in out b2MotorJoint;   Factor : in Real);



   --    Get the position correction factor in the range [0,1].
   --
   --    float GetCorrectionFactor() const;
   --

   function getCorrectionFactor (Self : in b2MotorJoint) return Real;



   --    Dump to b2Log
   --    void Dump() override;
   --

   overriding
   procedure dump (Self : in b2MotorJoint);




   -------------
   -- protected:
   --

   package Forge
   is

      --    b2MotorJoint(const b2MotorJointDef* def);
      --

      function to_b2MotorJoint (Def : in b2MotorJointDef) return b2MotorJoint;

   end Forge;




private

   --    friend class b2Joint;
   --


   --    void InitVelocityConstraints(const b2SolverData& data) override;
   --

   overriding
   procedure initVelocityConstraints (Self : in out b2MotorJoint;   Data : in b2SolverData);



   --    void SolveVelocityConstraints(const b2SolverData& data) override;
   --

   overriding
   procedure solveVelocityConstraints (Self : in out b2MotorJoint;   Data : in b2SolverData);



   --    bool SolvePositionConstraints(const b2SolverData& data) override;
   --

   overriding
   function solvePositionConstraints (Self : in out b2MotorJoint;   Data : in b2SolverData) return Boolean;




   --  class b2MotorJoint : public b2Joint
   --  {
   --  public:
   --  protected:
   --
   --    friend class b2Joint;
   --
   --
   --    // Solver shared
   --    b2Vec2 m_linearOffset;
   --    float m_angularOffset;
   --    b2Vec2 m_linearImpulse;
   --    float m_angularImpulse;
   --    float m_maxForce;
   --    float m_maxTorque;
   --    float m_correctionFactor;
   --
   --    // Solver temp
   --    int32 m_indexA;
   --    int32 m_indexB;
   --    b2Vec2 m_rA;
   --    b2Vec2 m_rB;
   --    b2Vec2 m_localCenterA;
   --    b2Vec2 m_localCenterB;
   --    b2Vec2 m_linearError;
   --    float m_angularError;
   --    float m_invMassA;
   --    float m_invMassB;
   --    float m_invIA;
   --    float m_invIB;
   --    b2Mat22 m_linearMass;
   --    float m_angularMass;
   --  };
   --

   type b2MotorJoint is new b2_Joint.b2Joint with
      record
         -- Solver shared.
         --
         m_linearOffset,
         m_linearImpulse    : b2Vec2;

         m_angularOffset,
         m_angularImpulse,
         m_maxForce,
         m_maxTorque,
         m_correctionFactor : Real;

         -- Solver temp.
         --
         m_indexA,
         m_indexB        : Natural;

         m_rA,
         m_rB            : b2Vec2;

         m_localCenterA,
         m_localCenterB  : b2Vec2;

         m_linearError   : b2Vec2;
         m_angularError  : Real;

         m_invMassA,
         m_invMassB      : Real;

         m_invIA,
         m_invIB         : Real;

         m_linearMass    : b2Mat22;
         m_angularMass   : Real;
      end record;


end box2d.b2_Joint.b2_motor_Joint;
