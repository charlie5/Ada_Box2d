with
     box2d.b2_Collision,
     box2d.b2_Contact,
     box2d.b2_Math,
     box2d.b2_time_Step,
     box2d.b2_Settings,
     box2d.b2_Common;


package box2d.b2_contact_Solver
is
   use b2_time_Step,
       b2_Collision,
       b2_Contact,
       b2_Math,
       b2_Settings,
       b2_Common;


   --
   --  class b2Contact;
   --  class b2Body;
   --  class b2StackAllocator;
   --  struct b2ContactPositionConstraint;
   --

   type b2ContactPositionConstraint      is private;
   type b2ContactPositionConstraints     is array (Natural range <>) of b2ContactPositionConstraint;
   type b2ContactPositionConstraints_ptr is access b2ContactPositionConstraints;

   --  struct b2VelocityConstraintPoint
   --  {
   --    b2Vec2 rA;
   --    b2Vec2 rB;
   --    float normalImpulse;
   --    float tangentImpulse;
   --    float normalMass;
   --    float tangentMass;
   --    float velocityBias;
   --  };
   --

   type b2VelocityConstraintPoint is
      record
         rA : b2Vec2;
         rB : b2Vec2;

         normalImpulse  : Real;
         tangentImpulse : Real;
         normalMass     : Real;
         tangentMass    : Real;
         velocityBias   : Real;
      end record;




   --  struct b2ContactVelocityConstraint
   --  {
   --    b2VelocityConstraintPoint points[b2_maxManifoldPoints];
   --    b2Vec2 normal;
   --    b2Mat22 normalMass;
   --    b2Mat22 K;
   --    int32 indexA;
   --    int32 indexB;
   --    float invMassA, invMassB;
   --    float invIA, invIB;
   --    float friction;
   --    float restitution;
   --    float threshold;
   --    float tangentSpeed;
   --    int32 pointCount;
   --    int32 contactIndex;
   --  };
   --

   type b2VelocityConstraintPoints is array (0 .. b2_maxManifoldPoints - 1) of b2VelocityConstraintPoint;


   type b2ContactVelocityConstraint is
      record
         points       : b2VelocityConstraintPoints;
         normal       : b2Vec2;
         normalMass   : b2Mat22;
         K            : b2Mat22;

         indexA       : Natural;
         indexB       : Natural;

         invMassA,
         invMassB     : Real;

         invIA,
         invIB        : Real;

         friction     : Real;
         restitution  : Real;
         threshold    : Real;
         tangentSpeed : Real;

         pointCount   : Natural;
         contactIndex : Natural;
      end record;


   type b2ContactVelocityConstraints     is array (Natural range <>) of b2ContactVelocityConstraint;
   type b2ContactVelocityConstraints_ptr is access b2ContactVelocityConstraints;




   --  struct b2ContactSolverDef
   --  {
   --    b2TimeStep step;
   --    b2Contact** contacts;
   --    int32 count;
   --    b2Position* positions;
   --    b2Velocity* velocities;
   --    b2StackAllocator* allocator;
   --  };
   --

   type b2Contacts is array (Natural range <>) of access b2Contact;

   type b2ContactSolverDef is
      record
         step       :        b2TimeStep;
         contacts   : access b2Contacts;
         count      :        Natural;
         positions  : access b2Positions;
         velocities : access b2Velocities;
      end record;





   --  class b2ContactSolver
   --  {
   --  public:
   --    b2ContactSolver(b2ContactSolverDef* def);
   --    ~b2ContactSolver();
   --
   --    void InitializeVelocityConstraints();
   --
   --    void WarmStart();
   --    void SolveVelocityConstraints();
   --    void StoreImpulses();
   --
   --    bool SolvePositionConstraints();
   --    bool SolveTOIPositionConstraints(int32 toiIndexA, int32 toiIndexB);
   --
   --    b2TimeStep m_step;
   --    b2Position* m_positions;
   --    b2Velocity* m_velocities;
   --    b2StackAllocator* m_allocator;
   --    b2ContactPositionConstraint* m_positionConstraints;
   --    b2ContactVelocityConstraint* m_velocityConstraints;
   --    b2Contact** m_contacts;
   --    int m_count;
   --  };
   --

   type b2ContactSolver is tagged
      record
         m_step                :        b2TimeStep;
         m_positions           : access b2Positions;
         m_velocities          : access b2Velocities;
         m_positionConstraints :        b2ContactPositionConstraints_ptr;
         m_velocityConstraints :        b2ContactVelocityConstraints_ptr;
         m_contacts            : access b2Contacts;
         m_count               :        Natural;
      end record;


   function  to_b2ContactSolver (def : access b2ContactSolverDef) return b2ContactSolver;

   procedure destruct (Self : in out b2ContactSolver);

   procedure initializeVelocityConstraints (Self : in out b2ContactSolver);

   procedure warmStart                (Self : in out b2ContactSolver);
   procedure solveVelocityConstraints (Self : in out b2ContactSolver);
   procedure storeImpulses            (Self : in out b2ContactSolver);

   function  solvePositionConstraints    (Self : in out b2ContactSolver) return Boolean;
   function  solveTOIPositionConstraints (Self : in out b2ContactSolver;   toiIndexA,
                                          toiIndexB : in Natural) return Boolean;



private


   --  struct b2ContactPositionConstraint
   --  {
   --    b2Vec2 localPoints[b2_maxManifoldPoints];
   --    b2Vec2 localNormal;
   --    b2Vec2 localPoint;
   --    int32 indexA;
   --    int32 indexB;
   --    float invMassA, invMassB;
   --    b2Vec2 localCenterA, localCenterB;
   --    float invIA, invIB;
   --    b2Manifold::Type type;
   --    float radiusA, radiusB;
   --    int32 pointCount;
   --  };
   --

   type b2ContactPositionConstraint is
      record
         localPoints  : b2Vec2s (0 .. b2_maxManifoldPoints - 1);

         localNormal  : b2Vec2;
         localPoint   : b2Vec2;

         indexA       : Natural;
         indexB       : Natural;

         invMassA,
         invMassB     : Real;

         localCenterA,
         localCenterB : b2Vec2;

         invIA,
         invIB        : Real;

         Kind         : b2manifold_Type;

         radiusA,
         radiusB      : Real;

         pointCount   : Natural;
      end record;



end box2d.b2_contact_Solver;
