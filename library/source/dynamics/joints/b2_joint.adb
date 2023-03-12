with
     b2_Body,
     b2_Joint.b2_Distance_Joint,
     b2_Draw,
     b2_Joint.b2_friction_Joint,
     b2_Joint.b2_gear_Joint,
     b2_Joint.b2_motor_Joint,
     b2_Joint.b2_mouse_Joint,
     b2_Joint.b2_prismatic_Joint,
     b2_Joint.b2_pulley_Joint,
     b2_Joint.b2_revolute_Joint,
     b2_Joint.b2_weld_Joint,
     b2_Joint.b2_wheel_Joint,
     b2_World,
     b2_Common;


package body b2_Joint
is
   use b2_Distance_Joint,
       b2_friction_Joint,
       b2_gear_Joint,
       b2_motor_Joint,
       b2_mouse_Joint,
       b2_prismatic_Joint,
       b2_pulley_Joint,
       b2_revolute_Joint,
       b2_weld_Joint,
       b2_wheel_Joint,
       b2_Common;


   --  void b2LinearStiffness (float& stiffness, float& damping,
   --                          float frequencyHertz, float dampingRatio,
   --                          const b2Body* bodyA, const b2Body* bodyB)
   --  {
   --    float massA = bodyA->GetMass();
   --    float massB = bodyB->GetMass();
   --    float mass;
   --    if (massA > 0.0f && massB > 0.0f)
   --    {
   --       mass = massA * massB / (massA + massB);
   --    }
   --    else if (massA > 0.0f)
   --    {
   --       mass = massA;
   --    }
   --    else
   --    {
   --       mass = massB;
   --    }
   --
   --    float omega = 2.0f * b2_pi * frequencyHertz;
   --    stiffness = mass * omega * omega;
   --    damping = 2.0f * mass * dampingRatio * omega;
   --  }
   --

   procedure b2LinearStiffness (stiffness      :    out Real;
                                damping        :    out Real;
                                frequencyHertz : in     Real;
                                dampingRatio   : in     Real;
                                bodyA,
                                bodyB          : in     b2_Body.b2Body)
   is
      massA : constant Real := bodyA.getMass;
      massB : constant Real := bodyB.getMass;
      mass  :          Real;
      omega :          Real;
   begin

      if    massA > 0.0
        and massB > 0.0
      then
         mass :=    massA * massB
                 / (massA + massB);

      elsif massA > 0.0
      then
         mass := massA;

      else
         mass := massB;
      end if;

      omega     := 2.0  * b2_pi * frequencyHertz;
      stiffness := mass * omega * omega;
      damping   := 2.0  * mass  * dampingRatio * omega;
   end b2LinearStiffness;





   --  void b2AngularStiffness (float& stiffness, float& damping,
   --                           float frequencyHertz, float dampingRatio,
   --                           const b2Body* bodyA, const b2Body* bodyB)
   --  {
   --    float IA = bodyA->GetInertia();
   --    float IB = bodyB->GetInertia();
   --    float I;
   --    if (IA > 0.0f && IB > 0.0f)
   --    {
   --       I = IA * IB / (IA + IB);
   --    }
   --    else if (IA > 0.0f)
   --    {
   --       I = IA;
   --    }
   --    else
   --    {
   --       I = IB;
   --    }
   --
   --    float omega = 2.0f * b2_pi * frequencyHertz;
   --    stiffness = I * omega * omega;
   --    damping = 2.0f * I * dampingRatio * omega;
   --  }
   --

   procedure b2AngularStiffness (stiffness      :    out Real;
                                 damping        :    out Real;
                                 frequencyHertz : in     Real;
                                 dampingRatio   : in     Real;
                                 bodyA,
                                 bodyB          : in     b2_Body.b2Body)
   is
      IA    : constant Real := bodyA.getInertia;
      IB    : constant Real := bodyB.getInertia;
      I     :          Real;
      Omega :          Real;
   begin
      if    IA > 0.0
        and IB > 0.0
      then
         I :=    IA * IB
              / (IA + IB);

      elsif IA > 0.0
      then
         I := IA;

      else
         I := IB;
      end if;

     omega     := 2.0 * b2_pi * frequencyHertz;
     stiffness := I   * omega * omega;
     damping   := 2.0 * I     * dampingRatio * omega;
   end b2AngularStiffness;





   --  inline b2JointType b2Joint::GetType() const
   --  {
   --    return m_type;
   --  }
   --

   function getType (Self : in b2Joint) return b2JointType
   is
   begin
      return Self.m_type;
   end getType;





   --  inline b2Body* b2Joint::GetBodyA()
   --  {
   --    return m_bodyA;
   --  }
   --

   function getBodyA (Self : in b2Joint) return access b2_Body.b2Body
   is
   begin
      return Self.m_bodyA;
   end getBodyA;




   --  inline b2Body* b2Joint::GetBodyB()
   --  {
   --    return m_bodyB;
   --  }
   --
   --

   function getBodyB (Self : in b2Joint) return access b2_Body.b2Body
   is
   begin
      return Self.m_bodyB;
   end getBodyB;




   --  inline b2Joint* b2Joint::GetNext()
   --  {
   --    return m_next;
   --  }
   --
   --  inline const b2Joint* b2Joint::GetNext() const
   --  {
   --    return m_next;
   --  }
   --

   function getNext (Self : in b2Joint) return access b2Joint'Class
   is
   begin
      return Self.m_next;
   end getNext;




   --  inline b2JointUserData& b2Joint::GetUserData()
   --  {
   --    return m_userData;
   --  }
   --

   function getUserData (Self : in b2Joint) return b2JointUserData
   is
   begin
      return Self.m_userData;
   end getUserData;






   --    Short-cut function to determine if either body is enabled.    -- TODO: This checks if *both* bodies are enabled ! Ask Erin about it.
   --
   --  bool b2Joint::IsEnabled() const
   --  {
   --    return m_bodyA->IsEnabled() && m_bodyB->IsEnabled();
   --  }
   --

   function isEnabled (Self : in b2Joint) return Boolean
   is
   begin
      return     Self.m_bodyA.isEnabled
             and Self.m_bodyB.isEnabled;
   end isEnabled;



   --  inline bool b2Joint::GetCollideConnected() const
   --  {
   --    return m_collideConnected;
   --  }
   --

   function getCollideConnected (Self : in b2Joint) return Boolean
   is
   begin
      return Self.m_collideConnected;
   end getCollideConnected;






   --------------
   --  protected:



   --  b2Joint* b2Joint::Create(const b2JointDef* def, b2BlockAllocator* allocator)
   --  {
   --    b2Joint* joint = nullptr;
   --
   --    switch (def->type)
   --    {
   --    case e_distanceJoint:
   --       {
   --          void* mem = allocator->Allocate(sizeof(b2DistanceJoint));
   --          joint = new (mem) b2DistanceJoint(static_cast<const b2DistanceJointDef*>(def));
   --       }
   --       break;
   --
   --    case e_mouseJoint:
   --       {
   --          void* mem = allocator->Allocate(sizeof(b2MouseJoint));
   --          joint = new (mem) b2MouseJoint(static_cast<const b2MouseJointDef*>(def));
   --       }
   --       break;
   --
   --    case e_prismaticJoint:
   --       {
   --          void* mem = allocator->Allocate(sizeof(b2PrismaticJoint));
   --          joint = new (mem) b2PrismaticJoint(static_cast<const b2PrismaticJointDef*>(def));
   --       }
   --       break;
   --
   --    case e_revoluteJoint:
   --       {
   --          void* mem = allocator->Allocate(sizeof(b2RevoluteJoint));
   --          joint = new (mem) b2RevoluteJoint(static_cast<const b2RevoluteJointDef*>(def));
   --       }
   --       break;
   --
   --    case e_pulleyJoint:
   --       {
   --          void* mem = allocator->Allocate(sizeof(b2PulleyJoint));
   --          joint = new (mem) b2PulleyJoint(static_cast<const b2PulleyJointDef*>(def));
   --       }
   --       break;
   --
   --    case e_gearJoint:
   --       {
   --          void* mem = allocator->Allocate(sizeof(b2GearJoint));
   --          joint = new (mem) b2GearJoint(static_cast<const b2GearJointDef*>(def));
   --       }
   --       break;
   --
   --    case e_wheelJoint:
   --       {
   --          void* mem = allocator->Allocate(sizeof(b2WheelJoint));
   --          joint = new (mem) b2WheelJoint(static_cast<const b2WheelJointDef*>(def));
   --       }
   --       break;
   --
   --    case e_weldJoint:
   --       {
   --          void* mem = allocator->Allocate(sizeof(b2WeldJoint));
   --          joint = new (mem) b2WeldJoint(static_cast<const b2WeldJointDef*>(def));
   --       }
   --       break;
   --
   --    case e_frictionJoint:
   --       {
   --          void* mem = allocator->Allocate(sizeof(b2FrictionJoint));
   --          joint = new (mem) b2FrictionJoint(static_cast<const b2FrictionJointDef*>(def));
   --       }
   --       break;
   --
   --    case e_motorJoint:
   --       {
   --          void* mem = allocator->Allocate(sizeof(b2MotorJoint));
   --          joint = new (mem) b2MotorJoint(static_cast<const b2MotorJointDef*>(def));
   --       }
   --       break;
   --
   --    default:
   --       b2Assert(false);
   --       break;
   --    }
   --
   --    return joint;
   --  }
   --


   type b2DistanceJoint_ptr is access all b2DistanceJoint'Class;



   function create (def : in b2JointDef'Class) return access b2Joint'Class
   is
      joint : access b2Joint'Class;
   begin
      case def.Kind
      is
         when e_distanceJoint =>
            declare
               Result : constant b2DistanceJoint_ptr := new b2DistanceJoint' (b2_Distance_Joint.Forge.to_b2DistanceJoint (b2DistanceJointDef (def)));
            begin
               joint := Result.all'Access; -- b2Joint_ptr' (new b2DistanceJoint' (to_b2DistanceJoint (b2DistanceJointDef (def))));
            end;

        when e_mouseJoint =>
              --  void* mem = allocator->Allocate(sizeof(b2MouseJoint));
              joint := new b2MouseJoint (b2MouseJointDef (def));

        when e_prismaticJoint =>
              --  void* mem = allocator->Allocate(sizeof(b2PrismaticJoint));
              joint := new b2PrismaticJoint (b2PrismaticJointDef (def));

        when e_revoluteJoint =>
              --  void* mem = allocator->Allocate(sizeof(b2RevoluteJoint));
              joint := new b2RevoluteJoint (b2RevoluteJointDef (def));

        when e_pulleyJoint =>
              --  void* mem = allocator->Allocate(sizeof(b2PulleyJoint));
              joint := new b2PulleyJoint (b2PulleyJointDef (def));

        when e_gearJoint =>
              --  void* mem = allocator->Allocate(sizeof(b2GearJoint));
              joint := new b2GearJoint (b2GearJointDef (def));

        when e_wheelJoint =>
              --  void* mem = allocator->Allocate(sizeof(b2WheelJoint));
              joint := new b2WheelJoint (b2WheelJointDef (def));

        when e_weldJoint =>
              --  void* mem = allocator->Allocate(sizeof(b2WeldJoint));
              joint := new b2WeldJoint (b2WeldJointDef (def));

        when e_frictionJoint =>
              --  void* mem = allocator->Allocate(sizeof(b2FrictionJoint));
              joint := new b2FrictionJoint (b2FrictionJointDef (def));

        when e_motorJoint =>
              --  void* mem = allocator->Allocate(sizeof(b2MotorJoint));
              joint := new b2MotorJoint (b2MotorJointDef (def));

        when others =>
           pragma assert (False);
        end case;

        return joint;
   end create;




   --  void b2Joint::Destroy(b2Joint* joint, b2BlockAllocator* allocator)
   --  {
   --    joint->~b2Joint();
   --    switch (joint->m_type)
   --    {
   --    case e_distanceJoint:
   --       allocator->Free(joint, sizeof(b2DistanceJoint));
   --       break;
   --
   --    case e_mouseJoint:
   --       allocator->Free(joint, sizeof(b2MouseJoint));
   --       break;
   --
   --    case e_prismaticJoint:
   --       allocator->Free(joint, sizeof(b2PrismaticJoint));
   --       break;
   --
   --    case e_revoluteJoint:
   --       allocator->Free(joint, sizeof(b2RevoluteJoint));
   --       break;
   --
   --    case e_pulleyJoint:
   --       allocator->Free(joint, sizeof(b2PulleyJoint));
   --       break;
   --
   --    case e_gearJoint:
   --       allocator->Free(joint, sizeof(b2GearJoint));
   --       break;
   --
   --    case e_wheelJoint:
   --       allocator->Free(joint, sizeof(b2WheelJoint));
   --       break;
   --
   --    case e_weldJoint:
   --       allocator->Free(joint, sizeof(b2WeldJoint));
   --       break;
   --
   --    case e_frictionJoint:
   --       allocator->Free(joint, sizeof(b2FrictionJoint));
   --       break;
   --
   --    case e_motorJoint:
   --       allocator->Free(joint, sizeof(b2MotorJoint));
   --       break;
   --
   --    default:
   --       b2Assert(false);
   --       break;
   --    }
   --  }
   --

   procedure destroy (joint : access b2Joint'Class)
   is
   begin
      return;
      --    joint->~b2Joint();
      --    switch (joint->m_type)
      --    {
      --    case e_distanceJoint:
      --       allocator->Free(joint, sizeof(b2DistanceJoint));
      --       break;
      --
      --    case e_mouseJoint:
      --       allocator->Free(joint, sizeof(b2MouseJoint));
      --       break;
      --
      --    case e_prismaticJoint:
      --       allocator->Free(joint, sizeof(b2PrismaticJoint));
      --       break;
      --
      --    case e_revoluteJoint:
      --       allocator->Free(joint, sizeof(b2RevoluteJoint));
      --       break;
      --
      --    case e_pulleyJoint:
      --       allocator->Free(joint, sizeof(b2PulleyJoint));
      --       break;
      --
      --    case e_gearJoint:
      --       allocator->Free(joint, sizeof(b2GearJoint));
      --       break;
      --
      --    case e_wheelJoint:
      --       allocator->Free(joint, sizeof(b2WheelJoint));
      --       break;
      --
      --    case e_weldJoint:
      --       allocator->Free(joint, sizeof(b2WeldJoint));
      --       break;
      --
      --    case e_frictionJoint:
      --       allocator->Free(joint, sizeof(b2FrictionJoint));
      --       break;
      --
      --    case e_motorJoint:
      --       allocator->Free(joint, sizeof(b2MotorJoint));
      --       break;
      --
      --    default:
      --       b2Assert(false);
      --       break;
      --    }
   end destroy;




   package body Forge
   is

      --  b2Joint::b2Joint(const b2JointDef* def)
      --  {
      --    b2Assert(def->bodyA != def->bodyB);
      --
      --    m_type = def->type;
      --    m_prev = nullptr;
      --    m_next = nullptr;
      --    m_bodyA = def->bodyA;
      --    m_bodyB = def->bodyB;
      --    m_index = 0;
      --    m_collideConnected = def->collideConnected;
      --    m_islandFlag = false;
      --    m_userData = def->userData;
      --
      --    m_edgeA.joint = nullptr;
      --    m_edgeA.other = nullptr;
      --    m_edgeA.prev = nullptr;
      --    m_edgeA.next = nullptr;
      --
      --    m_edgeB.joint = nullptr;
      --    m_edgeB.other = nullptr;
      --    m_edgeB.prev = nullptr;
      --    m_edgeB.next = nullptr;
      --  }
      --

      procedure define (Self : out b2Joint;   def : in b2JointDef)
      is
         --    b2Assert(def->bodyA != def->bodyB);
         --  Self : b2Joint;
      begin
         null;
         --    m_type = def->type;
         --    m_prev = nullptr;
         --    m_next = nullptr;
         --    m_bodyA = def->bodyA;
         --    m_bodyB = def->bodyB;
         --    m_index = 0;
         --    m_collideConnected = def->collideConnected;
         --    m_islandFlag = false;
         --    m_userData = def->userData;
         --
         --    m_edgeA.joint = nullptr;
         --    m_edgeA.other = nullptr;
         --    m_edgeA.prev = nullptr;
         --    m_edgeA.next = nullptr;
         --
         --    m_edgeB.joint = nullptr;
         --    m_edgeB.other = nullptr;
         --    m_edgeB.prev = nullptr;
         --    m_edgeB.next = nullptr;


         --  return Self;
      end define;

   end Forge;


   --  void b2Joint::Draw (b2Draw* draw) const
   --  {
   --    const b2Transform& xf1 = m_bodyA->GetTransform();
   --    const b2Transform& xf2 = m_bodyB->GetTransform();
   --    b2Vec2 x1 = xf1.p;
   --    b2Vec2 x2 = xf2.p;
   --    b2Vec2 p1 = GetAnchorA();
   --    b2Vec2 p2 = GetAnchorB();
   --
   --    b2Color color(0.5f, 0.8f, 0.8f);
   --
   --    switch (m_type)
   --    {
   --    case e_distanceJoint:
   --       draw->DrawSegment(p1, p2, color);
   --       break;
   --
   --    case e_pulleyJoint:
   --    {
   --       b2PulleyJoint* pulley = (b2PulleyJoint*)this;
   --       b2Vec2 s1 = pulley->GetGroundAnchorA();
   --       b2Vec2 s2 = pulley->GetGroundAnchorB();
   --       draw->DrawSegment(s1, p1, color);
   --       draw->DrawSegment(s2, p2, color);
   --       draw->DrawSegment(s1, s2, color);
   --    }
   --    break;
   --
   --    case e_mouseJoint:
   --    {
   --       b2Color c;
   --       c.Set(0.0f, 1.0f, 0.0f);
   --       draw->DrawPoint(p1, 4.0f, c);
   --       draw->DrawPoint(p2, 4.0f, c);
   --
   --       c.Set(0.8f, 0.8f, 0.8f);
   --       draw->DrawSegment(p1, p2, c);
   --
   --    }
   --    break;
   --
   --    default:
   --       draw->DrawSegment(x1, p1, color);
   --       draw->DrawSegment(p1, p2, color);
   --       draw->DrawSegment(x2, p2, color);
   --    }
   --  }

   procedure draw (Self : in b2Joint)
   is
   begin
      return;
   end draw;






   --    Dump this joint to the log file.
   --    virtual void Dump() { b2Dump("// Dump is not supported for this joint type.\n"); }
   --

   procedure dump (Self : in b2Joint)
   is
   begin
      b2Dump ("Dump is not supported for this joint type.");
   end dump;








end b2_Joint;
