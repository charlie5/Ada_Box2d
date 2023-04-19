private
with
     box2d.b2_Body;


package box2d.b2_Joint.b2_gear_Joint
  --
  --  A gear joint is used to connect two joints together. Either joint
  --  can be a revolute or prismatic joint. You specify a gear ratio
  --  to bind the motions together:
  --  coordinate1 + ratio * coordinate2 = constant
  --  The ratio can be negative or positive. If one joint is a revolute joint
  --  and the other joint is a prismatic joint, then the ratio will have units
  --  of length or units of 1/length.
  --  @warning You have to manually destroy the gear joint if joint1 or joint2
  --  is destroyed.
  --
is
   -----------------
   -- b2GearJointDef
   --

   --  Gear joint definition. This definition requires two existing
   --  revolute or prismatic joints (any combination will work).
   --  @warning bodyB on the input joints must both be dynamic
   --  struct b2GearJointDef : public b2JointDef
   --  {
   --    b2GearJointDef()
   --    {
   --       type = e_gearJoint;
   --       joint1 = nullptr;
   --       joint2 = nullptr;
   --       ratio = 1.0f;
   --    }
   --
   --    The first revolute/prismatic joint attached to the gear joint.
   --    b2Joint* joint1;
   --
   --    The second revolute/prismatic joint attached to the gear joint.
   --    b2Joint* joint2;
   --
   --    The gear ratio.
   --    @see b2GearJoint for explanation.
   --    float ratio;
   --  };
   --

   type b2GearJointDef is new b2_Joint.b2JointDef with
      record
         Joint1,                   -- The first  revolute/prismatic joint attached to the gear joint.
         Joint2 : b2Joint_ptr;     -- The second revolute/prismatic joint attached to the gear joint.

         Ratio  : Real;            -- See b2GearJoint for explanation.
      end record;


   function  to_b2GearJointDef return b2GearJointDef;

   --  procedure initialize (Self : out b2GearJointDef;   Joint1, Joint2 : in b2Joint_ptr;
   --                                                     Ratio          : in Real);



   ---------------
   --- b2GearJoint
   --

   --  class b2GearJoint : public b2Joint
   --

   type b2GearJoint     is new b2_Joint.b2Joint with private;
   type b2GearJoint_ptr is access all b2GearJoint'Class;


   --  public:
   --


   --    b2Vec2 GetAnchorA() const override;
   --    b2Vec2 GetAnchorB() const override;
   --

   overriding
   function getAnchorA (Self : in b2GearJoint) return b2Vec2;

   overriding
   function getAnchorB (Self : in b2GearJoint) return b2Vec2;



   --    Get the reaction force given the inverse time step.
   --    Unit is N.
   --
   --    b2Vec2 GetReactionForce(float inv_dt) const override;
   --

   overriding
   function getReactionForce (Self : in b2GearJoint;   inv_dt : in Real) return b2Vec2;



   --    Get the reaction torque given the inverse time step.
   --    Unit is N*m. This is always zero for a distance joint.
   --
   --    float GetReactionTorque(float inv_dt) const override;
   --

   overriding
   function getReactionTorque (Self : in b2GearJoint;   inv_dt : in Real) return Real;




   --    Get the first joint.
   --
   --    b2Joint* GetJoint1() { return m_joint1; }
   --

   function getJoint1 (Self : in b2GearJoint) return b2Joint_ptr
     with inline;


   --    Get the second joint.
   --
   --    b2Joint* GetJoint2() { return m_joint2; }
   --

   function getJoint2 (Self : in b2GearJoint) return b2Joint_ptr
     with inline;



   --    Set the gear ratio.
   --
   --    void SetRatio(float ratio);
   --

   procedure setRatio (Self : in out b2GearJoint;   Ratio : in Real);



   --    Get the gear ratio.
   --
   --    float GetRatio() const;
   --

   function getRatio (Self : in b2GearJoint) return Real;



   --    Dump joint to dmLog.
   --
   --    void Dump() override;
   --

   overriding
   procedure dump (Self : in b2GearJoint);



   --  --    void Draw(b2Draw* draw) const override;
   --  --
   --
   --  overriding
   --  procedure draw (Self : in b2GearJoint;   Draw : access b2Draw'Class);




   -------------
   -- protected:
   --

   package Forge
   is

      --    b2GearJoint(const b2DistanceJointDef* data);
      --

      function to_b2GearJoint (Def : in b2GearJointDef) return b2GearJoint;

   end Forge;




private

   --    friend class b2Joint;
   --


   --    void InitVelocityConstraints(const b2SolverData& data) override;
   --

   overriding
   procedure initVelocityConstraints (Self : in out b2GearJoint;   Data : in b2SolverData);



   --    void SolveVelocityConstraints(const b2SolverData& data) override;
   --

   overriding
   procedure solveVelocityConstraints (Self : in out b2GearJoint;   Data : in b2SolverData);



   --    bool SolvePositionConstraints(const b2SolverData& data) override;
   --

   overriding
   function solvePositionConstraints (Self : in out b2GearJoint;   Data : in b2SolverData) return Boolean;




   --    b2Joint* m_joint1;
   --    b2Joint* m_joint2;
   --
   --    b2JointType m_typeA;
   --    b2JointType m_typeB;
   --
   --    // Body A is connected to body C
   --    // Body B is connected to body D
   --    b2Body* m_bodyC;
   --    b2Body* m_bodyD;
   --
   --    // Solver shared
   --    b2Vec2 m_localAnchorA;
   --    b2Vec2 m_localAnchorB;
   --    b2Vec2 m_localAnchorC;
   --    b2Vec2 m_localAnchorD;
   --
   --    b2Vec2 m_localAxisC;
   --    b2Vec2 m_localAxisD;
   --
   --    float m_referenceAngleA;
   --    float m_referenceAngleB;
   --
   --    float m_constant;
   --    float m_ratio;
   --
   --    float m_impulse;
   --
   --    // Solver temp
   --    int32 m_indexA, m_indexB, m_indexC, m_indexD;
   --    b2Vec2 m_lcA, m_lcB, m_lcC, m_lcD;
   --    float m_mA, m_mB, m_mC, m_mD;
   --    float m_iA, m_iB, m_iC, m_iD;
   --    b2Vec2 m_JvAC, m_JvBD;
   --    float m_JwA, m_JwB, m_JwC, m_JwD;
   --    float m_mass;

   type b2GearJoint is new b2_Joint.b2Joint with
      record
         m_joint1,
         m_joint2 : b2Joint_ptr;

         m_TypeA,
         m_TypeB  : b2JointType;

         m_bodyC,                           -- Body A is connected to body C.
         m_bodyD  : b2_Body.b2Body_ptr;     -- Body B is connected to body D.

         -- Solver shared.
         --
         m_localAnchorA,
         m_localAnchorB,
         m_localAnchorC,
         m_localAnchorD    : b2Vec2;
         m_gamma,

         m_localAxisC,
         m_localAxisD      : b2Vec2;

         m_referenceAngleA,
         m_referenceAngleB : Real;

         m_Constant        : Real;
         m_Ratio           : Real;

         m_Impulse         : Real;

         -- Solver temp.
         --
         m_indexA,
         m_indexB,
         m_indexC,
         m_indexD : Natural;

         m_lcA,
         m_lcB,
         m_lcC,
         m_lcD    : b2Vec2;

         m_mA,
         m_mB,
         m_mC,
         m_mD     : Real;

         m_iA,
         m_iB,
         m_iC,
         m_iD     : Real;

         m_JvAC,
         m_JvBD   : b2Vec2;

         m_JwA,
         m_JwB,
         m_JwC,
         m_JwD    : Real;

         m_mass   : Real;
      end record;


end box2d.b2_Joint.b2_gear_Joint;
