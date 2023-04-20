with
     box2d.b2_Body,
     box2d.b2_Joint.b2_Distance_Joint,
     box2d.b2_Joint.b2_friction_Joint,
     box2d.b2_Joint.b2_gear_Joint,
     box2d.b2_Joint.b2_motor_Joint,
     box2d.b2_Joint.b2_mouse_Joint,
     box2d.b2_Joint.b2_prismatic_Joint,
     box2d.b2_Joint.b2_pulley_Joint,
     box2d.b2_Joint.b2_revolute_Joint,
     box2d.b2_Joint.b2_weld_Joint,
     box2d.b2_Joint.b2_wheel_Joint,
     box2d.b2_World,
     box2d.b2_Common,

     ada.unchecked_Deallocation;


package body box2d.b2_Joint
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



   function create (Def : in b2JointDef'Class) return access b2Joint'Class
   is
      Joint : access b2Joint'Class;
   begin
      case def.Kind
      is
         when e_distanceJoint =>
            declare
               use b2_Distance_Joint.Forge;
               Result : constant b2DistanceJoint_ptr := new b2DistanceJoint' (to_b2DistanceJoint (b2DistanceJointDef (Def)));
            begin
               Joint := Result.all'Access;
            end;

        when e_mouseJoint =>
            declare
               use b2_Mouse_Joint.Forge;
               Result : constant b2MouseJoint_ptr := new b2MouseJoint' (to_b2MouseJoint (b2MouseJointDef (Def)));
            begin
               Joint := Result.all'Access;
            end;

        when e_prismaticJoint =>
            declare
               use b2_Prismatic_Joint.Forge;
               Result : constant b2PrismaticJoint_ptr := new b2PrismaticJoint' (to_b2PrismaticJoint (b2PrismaticJointDef (Def)));
            begin
               Joint := Result.all'Access;
            end;

        when e_revoluteJoint =>
            declare
               use b2_Revolute_Joint.Forge;
               Result : constant b2RevoluteJoint_ptr := new b2RevoluteJoint' (to_b2RevoluteJoint (b2RevoluteJointDef (Def)));
            begin
               Joint := Result.all'Access;
            end;

        when e_pulleyJoint =>
            declare
               use b2_Pulley_Joint.Forge;
               Result : constant b2PulleyJoint_ptr := new b2PulleyJoint' (to_b2PulleyJoint (b2PulleyJointDef (Def)));
            begin
               Joint := Result.all'Access;
            end;

        when e_gearJoint =>
            declare
               use b2_Gear_Joint.Forge;
               Result : constant b2GearJoint_ptr := new b2GearJoint' (to_b2GearJoint (b2GearJointDef (Def)));
            begin
               Joint := Result.all'Access;
            end;

        when e_wheelJoint =>
            declare
               use b2_Wheel_Joint.Forge;
               Result : constant b2WheelJoint_ptr := new b2WheelJoint' (to_b2WheelJoint (b2WheelJointDef (Def)));
            begin
               Joint := Result.all'Access;
            end;

        when e_weldJoint =>
            declare
               use b2_Weld_Joint.Forge;
               Result : constant b2WeldJoint_ptr := new b2WeldJoint' (to_b2WeldJoint (b2WeldJointDef (Def)));
            begin
               Joint := Result.all'Access;
            end;

        when e_frictionJoint =>
            declare
               use b2_Friction_Joint.Forge;
               Result : constant b2FrictionJoint_ptr := new b2FrictionJoint' (to_b2FrictionJoint (b2FrictionJointDef (Def)));
            begin
               Joint := Result.all'Access;
            end;

        when e_motorJoint =>
            declare
               use b2_Motor_Joint.Forge;
               Result : constant b2MotorJoint_ptr := new b2MotorJoint' (to_b2MotorJoint (b2MotorJointDef (Def)));
            begin
               Joint := Result.all'Access;
            end;

        when others =>
           pragma assert (False);
        end case;


        return Joint;
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

   procedure destroy (Joint : access b2Joint'Class)
   is
      procedure free   is new ada.unchecked_Deallocation (b2Joint'Class,
                                                          b2Joint_ptr);
      Ptr : b2Joint_ptr := b2Joint_ptr (Joint);
   begin
      Joint.destruct;
      free (Ptr);

      --  case joint.m_type
      --  is
      --     when e_distanceJoint =>
      --        Joint.free; --  allocator->Free(joint, sizeof(b2DistanceJoint));
      --
      --     when e_mouseJoint =>
      --        Joint.free; --  allocator->Free(joint, sizeof(b2MouseJoint));
      --
      --     when e_prismaticJoint =>
      --        Joint.free; --  allocator->Free(joint, sizeof(b2PrismaticJoint));
      --
      --     when e_revoluteJoint =>
      --        Joint.free; --  allocator->Free(joint, sizeof(b2RevoluteJoint));
      --
      --     when e_pulleyJoint =>
      --        Joint.free; --  allocator->Free(joint, sizeof(b2PulleyJoint));
      --
      --     when e_gearJoint =>
      --        Joint.free; --  allocator->Free(joint, sizeof(b2GearJoint));
      --
      --     when e_wheelJoint =>
      --        Joint.free; --  allocator->Free(joint, sizeof(b2WheelJoint));
      --
      --     when e_weldJoint =>
      --        Joint.free; --  allocator->Free(joint, sizeof(b2WeldJoint));
      --
      --     when e_frictionJoint =>
      --        Joint.free; --  allocator->Free(joint, sizeof(b2FrictionJoint));
      --
      --     when e_motorJoint =>
      --        Joint.free; --  allocator->Free(joint, sizeof(b2MotorJoint));
      --
      --     when others =>
      --        pragma assert (False);
      --  end case;
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

      procedure define (Self : out b2Joint'Class;   def : in b2JointDef'Class)
      is
           pragma assert (def.bodyA /= def.bodyB);
      begin
           Self.m_type             := def.Kind;
           Self.m_prev             := null;
           Self.m_next             := null;
           Self.m_bodyA            := def.bodyA;
           Self.m_bodyB            := def.bodyB;
           Self.m_index            := 0;
           Self.m_collideConnected := def.collideConnected;
           Self.m_islandFlag       := False;
           Self.m_userData         := def.userData;

           Self.m_edgeA.joint := null;
           Self.m_edgeA.other := null;
           Self.m_edgeA.prev  := null;
           Self.m_edgeA.next  := null;

           Self.m_edgeB.joint := null;
           Self.m_edgeB.other := null;
           Self.m_edgeB.prev  := null;
           Self.m_edgeB.next  := null;
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

   procedure draw_any (Self : in b2Joint'Class;   draw : access b2Draw'Class)
   is
      xf1   : constant b2Transform := Self.m_bodyA.getTransform;
      xf2   : constant b2Transform := Self.m_bodyB.getTransform;

      x1    : constant b2Vec2      := xf1.p;
      x2    : constant b2Vec2      := xf2.p;

      p1    : constant b2Vec2      := Self.getAnchorA;
      p2    : constant b2Vec2      := Self.getAnchorB;

      color : constant b2Color     := to_b2Color (0.5, 0.8, 0.8);

   begin
      case Self.m_type
      is
      when e_distanceJoint =>
         draw.drawSegment (p1, p2, color);

      when e_pulleyJoint =>
         declare
            --  type b2Joint_ptr is access all b2Joint;

            --  Ptr    : b2Joint_ptr   := Self'unchecked_Access;
            Pulley : constant b2PulleyJoint := b2PulleyJoint (Self);
            s1     : constant b2Vec2        := Pulley.getGroundAnchorA;
            s2     : constant b2Vec2        := Pulley.getGroundAnchorB;
         begin
            draw.drawSegment (s1, p1, color);
            draw.drawSegment (s2, p2, color);
            draw.drawSegment (s1, s2, color);
         end;

     when e_mouseJoint =>
         declare
            c : b2Color := to_b2Color (0.0, 1.0, 0.0);
         begin
            draw.drawPoint (p1, 4.0, c);
            draw.drawPoint (p2, 4.0, c);

            c := to_b2Color  (0.8, 0.8, 0.8);
            draw.drawSegment (p1, p2, c);
         end;

     when others =>
         draw.drawSegment (x1, p1, color);
         draw.drawSegment (p1, p2, color);
         draw.drawSegment (x2, p2, color);
     end case;
   end draw_any;






   --    Dump this joint to the log file.
   --    virtual void Dump() { b2Dump("// Dump is not supported for this joint type.\n"); }
   --

   procedure dump (Self : in b2Joint)
   is
   begin
      b2Dump ("Dump is not supported for this joint type.");
   end dump;






   -----------------
   --- Protected ---
   -----------------


   procedure m_prev_is (Self : in out b2Joint;   Now : in b2Joint_ptr)
   is
   begin
      Self.m_prev := Now;
   end m_prev_is;


   procedure m_next_is (Self : in out b2Joint;   Now : in b2Joint_ptr)
   is
   begin
      Self.m_next := Now;
   end m_next_is;




   function m_prev (Self : access b2Joint) return b2Joint_ptr
   is
   begin
      return Self.m_prev;
   end m_prev;


   function m_next (Self : access b2Joint) return b2Joint_ptr
   is
   begin
      return Self.m_next;
   end m_next;




   function m_edgeA (Self : access b2Joint) return access b2JointEdge
   is
   begin
      return Self.m_edgeA'Access;
   end m_edgeA;



   function m_edgeB (Self : access b2Joint) return access b2JointEdge
   is
   begin
      return Self.m_edgeB'Access;
   end m_edgeb;



   function m_islandFlag (Self : in b2Joint) return Boolean
   is
   begin
      return Self.m_islandFlag;
   end m_islandFlag;


   procedure m_islandFlag_is (Self : in out b2Joint;   Now : in Boolean)
   is
   begin
      Self.m_islandFlag := Now;
   end m_islandFlag_is;



   procedure m_Index_is (Self : in out b2Joint;   Now : in Natural)
   is
   begin
     Self.m_Index := Now;
   end m_Index_is;


end box2d.b2_Joint;
