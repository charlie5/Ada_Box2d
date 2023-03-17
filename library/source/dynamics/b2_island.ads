with
     b2_Body,
     b2_Joint,
     b2_world_Callbacks,
     b2_Contact,
     b2_contact_Solver,
     b2_Math,
     b2_time_Step;


package b2_Island
is
   use b2_world_Callbacks,
       b2_Body,
       b2_Joint,
       b2_Contact,
       b2_contact_Solver,
       b2_Math,
       b2_time_Step;


   --  class b2Contact;
   --  class b2Joint;
   --  class b2StackAllocator;
   --  class b2ContactListener;
   --  struct b2ContactVelocityConstraint;
   --  struct b2Profile;
   --




   --  This is an internal class.
   --  class b2Island
   --  {
   --  public:
   --
   --    b2StackAllocator* m_allocator;
   --    b2ContactListener* m_listener;
   --
   --    b2Body**    m_bodies;
   --    b2Contact** m_contacts;
   --    b2Joint**   m_joints;
   --
   --    b2Position* m_positions;
   --    b2Velocity* m_velocities;
   --
   --    int32 m_bodyCount;
   --    int32 m_jointCount;
   --    int32 m_contactCount;
   --
   --    int32 m_bodyCapacity;
   --    int32 m_contactCapacity;
   --    int32 m_jointCapacity;
   --  };
   --

   type b2Bodies   is array (Natural range <>) of b2Body_ptr;
   type b2Joints   is array (Natural range <>) of b2Joint_ptr;

   type b2Bodies_ptr     is access b2Bodies;
   type b2Contacts_ptr   is access b2Contacts;
   type b2Joints_ptr     is access b2Joints;

   type b2Positions_ptr  is access b2Positions;
   type b2Velocities_ptr is access b2Velocities;



   type b2Island is tagged
      record
         m_listener   : access b2ContactListener;

         m_bodies     : b2Bodies_ptr;
         m_contacts   : b2Contacts_ptr;
         m_joints     : b2Joints_ptr;

         m_positions  : b2Positions_ptr;
         m_velocities : b2Velocities_ptr;

         m_bodyCount    : Natural;
         m_jointCount   : Natural;
         m_contactCount : Natural;

         m_bodyCapacity    : Natural;
         m_contactCapacity : Natural;
         m_jointCapacity   : Natural;
      end record;



   --    b2Island (int32 bodyCapacity, int32 contactCapacity, int32 jointCapacity,
   --              b2StackAllocator* allocator, b2ContactListener* listener);
   --

   function to_b2Island (bodyCapacity    : Natural;
                         contactCapacity : Natural;
                         jointCapacity   : Natural;
                         listener        : access b2ContactListener'Class) return b2Island;



   --    ~b2Island();
   --

   procedure destruct (Self : in out b2Island);




   --    void Clear()
   --    {
   --       m_bodyCount = 0;
   --       m_contactCount = 0;
   --       m_jointCount = 0;
   --    }
   --

   procedure clear (Self : in out b2Island)   with inline;




   --    void Solve(b2Profile* profile, const b2TimeStep& step, const b2Vec2& gravity, bool allowSleep);
   --

   procedure solve (Self : in out b2Island;   profile    : access b2Profile;
                                              step       : in     b2TimeStep;
                                              gravity    : in     b2Vec2;
                                              allowSleep : in     Boolean);




   --    void SolveTOI(const b2TimeStep& subStep, int32 toiIndexA, int32 toiIndexB);
   --

   procedure solveTOI (Self : in out b2Island;   subStep   : in b2TimeStep;
                                                 toiIndexA : in Natural;
                                                 toiIndexB : in Natural);




   --    void Add(b2Body* body)
   --    {
   --       b2Assert(m_bodyCount < m_bodyCapacity);
   --       body->m_islandIndex = m_bodyCount;
   --       m_bodies[m_bodyCount] = body;
   --       ++m_bodyCount;
   --    }
   --

   procedure add (Self : in out b2Island;   the_Body : b2Body_ptr)
     with inline;





   --    void Add(b2Contact* contact)
   --    {
   --       b2Assert(m_contactCount < m_contactCapacity);
   --       m_contacts[m_contactCount++] = contact;
   --    }
   --

   procedure add (Self : in out b2Island;   Contact : access b2Contact)
     with inline;




   --    void Add(b2Joint* joint)
   --    {
   --       b2Assert(m_jointCount < m_jointCapacity);
   --       m_joints[m_jointCount++] = joint;
   --    }
   --

   procedure add (Self : in out b2Island;   Joint : in b2Joint_ptr)
     with inline;




   --    void Report(const b2ContactVelocityConstraint* constraints);
   --

   procedure report (Self : in out b2Island;   constraints : in b2ContactVelocityConstraints);


end b2_Island;
