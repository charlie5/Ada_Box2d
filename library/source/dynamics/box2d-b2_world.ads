with
     box2d.b2_Fixture,
     box2d.b2_Contact,
     box2d.b2_contact_Manager,
     box2d.b2_Collision,
     box2d.b2_Math,
     box2d.b2_time_Step,
     --  box2d.b2_Settings,
     box2d.b2_world_Callbacks,
     box2d.b2_Body,
     box2d.b2_Draw,
     box2d.b2_Joint;


package box2d.b2_World
is
   use b2_Fixture,
       b2_Contact,
       b2_contact_Manager,
       b2_Body,
       b2_Joint,
       b2_Collision,
       b2_Math,
       --  b2_Settings,
       b2_Draw,
       b2_time_Step,
       b2_world_Callbacks;



   --  The world class manages all physics entities, dynamic simulation,
   --  and asynchronous queries. The world also contains efficient memory
   --  management facilities.
   --
   type b2World is tagged private;


   --  class b2World
   --  {
   --  public:
   --    Construct a world object.
   --    @param gravity the world gravity vector.
   --
   --    b2World(const b2Vec2& gravity);
   --

   function to_b2World (gravity : in b2Vec2) return b2World;




   --    Destruct the world. All physics entities are destroyed and all heap memory is released.
   --
   --    ~b2World();
   --

   procedure destruct (Self : in out b2World);




   --    Register a destruction listener. The listener is owned by you and must
   --    remain in scope.
   --
   --    void SetDestructionListener(b2DestructionListener* listener);
   --

   procedure setDestructionListener (Self : in out b2World;   listener : access b2DestructionListener);



   --    Register a contact filter to provide specific control over collision.
   --    Otherwise the default filter is used (b2_defaultFilter). The listener is
   --    owned by you and must remain in scope.
   --
   --    void SetContactFilter(b2ContactFilter* filter);
   --

   procedure SetContactFilter (Self : in out b2World;   filter : access b2ContactFilter);



   --    Register a contact event listener. The listener is owned by you and must
   --    remain in scope.
   --
   --    void SetContactListener(b2ContactListener* listener);
   --

   procedure setContactListener (Self : in out b2World;   listener : access b2ContactListener'Class);


   --    Register a routine for debug drawing. The debug draw functions are called
   --    inside with b2World::DebugDraw method. The debug draw object is owned
   --    by you and must remain in scope.
   --
   --    void SetDebugDraw(b2Draw* debugDraw);
   --

   procedure SetDebugDraw (Self : in out b2World;   debugDraw : access b2Draw);


   --    Create a rigid body given a definition. No reference to the definition
   --    is retained.
   --    @warning This function is locked during callbacks.
   --
   --    b2Body* CreateBody(const b2BodyDef* def);
   --

   function createBody (Self : in out b2World;   def : in b2BodyDef) return b2Body_ptr;



   --    Destroy a rigid body given a definition. No reference to the definition
   --    is retained. This function is locked during callbacks.
   --    @warning This automatically deletes all associated shapes and joints.
   --    @warning This function is locked during callbacks.
   --
   --    void DestroyBody(b2Body* body);
   --

   procedure destroyBody (Self : in out b2World;   the_Body : in out b2Body_ptr);



   --    Create a joint to constrain bodies together. No reference to the definition
   --    is retained. This may cause the connected bodies to cease colliding.
   --    @warning This function is locked during callbacks.
   --
   --    b2Joint* CreateJoint(const b2JointDef* def);
   --

   function createJoint (Self : in out b2World;   def : in b2JointDef) return b2Joint_ptr;



   --    Destroy a joint. This may cause the connected bodies to begin colliding.
   --    @warning This function is locked during callbacks.
   --
   --    void DestroyJoint(b2Joint* joint);
   --

   procedure destroyJoint (Self : in out b2World;   joint : in b2Joint_ptr);



   --    Take a time step. This performs collision detection, integration,
   --    and constraint solution.
   --    @param timeStep the amount of time to simulate, this should not vary.
   --    @param velocityIterations for the velocity constraint solver.
   --    @param positionIterations for the position constraint solver.
   --
   --    void Step (float timeStep,
   --               int32 velocityIterations,
   --               int32 positionIterations);
   --

   procedure step (Self : in out b2World;   timeStep           : in Real;
                                            velocityIterations : in Positive;
                                            positionIterations : in Positive);


   --    Manually clear the force buffer on all bodies. By default, forces are cleared automatically
   --    after each call to Step. The default behavior is modified by calling SetAutoClearForces.
   --    The purpose of this function is to support sub-stepping. Sub-stepping is often used to maintain
   --    a fixed sized time step under a variable frame-rate.
   --    When you perform sub-stepping you will disable auto clearing of forces and instead call
   --    ClearForces after all sub-steps are complete in one pass of your game loop.
   --    @see SetAutoClearForces
   --
   --    void ClearForces();
   --

   procedure clearForces (Self : in out b2World);




   --    Call this to draw shapes and other debug draw data. This is intentionally non-const.
   --
   --    void DebugDraw();
   --

   procedure DebugDraw (Self : in out b2World);




   --    Query the world for all fixtures that potentially overlap the
   --    provided AABB.
   --    @param callback a user implemented callback class.
   --    @param aabb the query box.
   --
   --    void QueryAABB(b2QueryCallback* callback, const b2AABB& aabb) const;
   --

   procedure queryAABB (Self : access constant b2World;   callback : access b2QueryCallback;
                                                          aabb     : in     b2AABB);



   --    Ray-cast the world for all fixtures in the path of the ray. Your callback
   --    controls whether you get the closest point, any point, or n-points.
   --    The ray-cast ignores shapes that contain the starting point.
   --    @param callback a user implemented callback class.
   --    @param point1 the ray starting point
   --    @param point2 the ray ending point
   --
   --    void RayCast(b2RayCastCallback* callback, const b2Vec2& point1, const b2Vec2& point2) const;
   --

   procedure RayCast (Self : access constant b2World;   callback : access b2RayCastCallback;
                                                        point1,
                      point2   : in     b2Vec2);



   --    Get the world body list. With the returned body, use b2Body::GetNext to get
   --    the next body in the world list. A nullptr body indicates the end of the list.
   --    @return the head of the world body list.
   --

   --    b2Body* GetBodyList();
   --

   function GetBodyList (Self : in b2World) return b2Body_ptr
     with inline;


   --    const b2Body* GetBodyList() const;
   --



   --    Get the world joint list. With the returned joint, use b2Joint::GetNext to get
   --    the next joint in the world list. A nullptr joint indicates the end of the list.
   --    @return the head of the world joint list.
   --

   --    b2Joint* GetJointList();
   --

   function getJointList (Self : in b2World) return b2Joint_ptr
     with inline;


   --    const b2Joint* GetJointList() const;
   --




   --    Get the world contact list. With the returned contact, use b2Contact::GetNext to get
   --    the next contact in the world list. A nullptr contact indicates the end of the list.
   --    @return the head of the world contact list.
   --    @warning contacts are created and destroyed in the middle of a time step.
   --    Use b2ContactListener to avoid missing contacts.
   --

   --    b2Contact* GetContactList();
   --

   function GetContactList (Self : in b2World) return b2Contact_ptr
     with inline;

   --    const b2Contact* GetContactList() const;
   --



   --    Enable/disable sleep.
   --

   --    void SetAllowSleeping(bool flag);
   --

   procedure setAllowSleeping (Self : in out b2World;   flag : in Boolean);



   --    bool GetAllowSleeping() const { return m_allowSleep; }
   --

   function GetAllowSleeping (Self : in b2World) return Boolean;




   --    Enable/disable warm starting. For testing.
   --

   --    void SetWarmStarting(bool flag) { m_warmStarting = flag; }
   --

   procedure SetWarmStarting (Self : in out b2World;   flag : in Boolean);



   --    bool GetWarmStarting() const { return m_warmStarting; }
   --

   function GetWarmStarting (Self : in b2World) return Boolean;




   --    Enable/disable continuous physics. For testing.
   --

   --    void SetContinuousPhysics(bool flag) { m_continuousPhysics = flag; }
   --

   procedure SetContinuousPhysics (Self : in out b2World;   flag : in Boolean);



   --    bool GetContinuousPhysics() const { return m_continuousPhysics; }
   --

   function GetContinuousPhysics (Self : in b2World) return Boolean;



   --    Enable/disable single stepped continuous physics. For testing.
   --

   --    void SetSubStepping(bool flag) { m_subStepping = flag; }
   --

   procedure SetSubStepping (Self : in out b2World;   flag : in Boolean);



   --    bool GetSubStepping() const { return m_subStepping; }
   --

   function GetSubStepping (Self : in b2World) return Boolean;



   --    Get the number of broad-phase proxies.
   --
   --    int32 GetProxyCount() const;
   --

   function getProxyCount (Self : in b2World) return Natural;



   --    Get the number of bodies.
   --
   --    int32 GetBodyCount() const;
   --

   function GetBodyCount (Self : in b2World) return Natural
     with inline;



   --    Get the number of joints.
   --
   --    int32 GetJointCount() const;
   --

   function GetJointCount (Self : in b2World) return Natural
     with inline;



   --    Get the number of contacts (each may have 0 or more contact points).
   --
   --    int32 GetContactCount() const;
   --

   function GetContactCount (Self : in b2World) return Natural
     with inline;



   --    Get the height of the dynamic tree.
   --
   --    int32 GetTreeHeight() const;
   --

   function GetTreeHeight (Self : in b2World) return Positive;



   --    Get the balance of the dynamic tree.
   --
   --    int32 GetTreeBalance() const;
   --

   function GetTreeBalance (Self : in b2World) return Natural;



   --    Get the quality metric of the dynamic tree. The smaller the better.
   --    The minimum is 1.
   --
   --    float GetTreeQuality() const;
   --

   function GetTreeQuality (Self : in b2World) return Real;



   --    Change the global gravity vector.
   --
   --    void SetGravity(const b2Vec2& gravity);
   --

   procedure SetGravity (Self : in out b2World;   gravity : in b2Vec2)
     with inline;




   --    Get the global gravity vector.
   --
   --    b2Vec2 GetGravity() const;
   --

   function GetGravity (Self : in b2World) return b2Vec2
     with inline;



   --    Is the world locked (in the middle of a time step).
   --
   --    bool IsLocked() const;
   --

   function isLocked (Self : in b2World) return Boolean
     with inline;




   --    Set flag to control automatic clearing of forces after each time step.
   --
   --    void SetAutoClearForces(bool flag);
   --

   procedure SetAutoClearForces (Self : in out b2World;   flag : in Boolean)
     with inline;



   --    Get the flag that controls automatic clearing of forces after each time step.
   --
   --    bool GetAutoClearForces() const;
   --

   function GetAutoClearForces (Self : in b2World) return Boolean
     with inline;



   --    Shift the world origin. Useful for large worlds.
   --    The body shift formula is: position -= newOrigin
   --    @param newOrigin the new origin with respect to the old origin
   --
   --    void ShiftOrigin(const b2Vec2& newOrigin);
   --

   procedure shiftOrigin (Self : in out b2World;   newOrigin : in b2Vec2);




   --    Get the contact manager for testing.
   --
   --    const b2ContactManager& GetContactManager() const;
   --

   function GetContactManager (Self : in b2World) return b2ContactManager
     with inline;



   --    Get the current profile.
   --
   --    const b2Profile& GetProfile() const;
   --

   function GetProfile (Self : in b2World) return b2Profile
     with inline;



   --    Dump the world into the log file.
   --    @warning this should be called outside of a time step.
   --
   --    void Dump();
   --

   procedure dump (Self : in out b2World);



   ------------------------------
   -- Protected b2World Functions
   --

   function m_contactManager (Self : access b2World) return access b2ContactManager
     with inline;

   procedure m_newContacts_is (Self : in out b2World;   Now : in Boolean)
     with inline;



private

   -- b2World components.
   --

   --    b2BlockAllocator m_blockAllocator;
   --    b2StackAllocator m_stackAllocator;
   --
   --    b2ContactManager m_contactManager;
   --
   --    b2Body* m_bodyList;
   --    b2Joint* m_jointList;
   --
   --    int32 m_bodyCount;
   --    int32 m_jointCount;
   --
   --    b2Vec2 m_gravity;
   --    bool m_allowSleep;
   --
   --    b2DestructionListener* m_destructionListener;
   --    b2Draw* m_debugDraw;
   --
   --    // This is used to compute the time step ratio to
   --    // support a variable time step.
   --    float m_inv_dt0;
   --
   --    bool m_newContacts;
   --    bool m_locked;
   --    bool m_clearForces;
   --
   --    // These are for debugging the solver.
   --    bool m_warmStarting;
   --    bool m_continuousPhysics;
   --    bool m_subStepping;
   --
   --    bool m_stepComplete;
   --
   --    b2Profile m_profile;


   type b2World is tagged
      record
         m_contactManager : aliased b2ContactManager;

         m_bodyList       : access  b2Body;
         m_jointList      : access  b2Joint;

         m_bodyCount  : Natural;
         m_jointCount : Natural;

         m_gravity    : b2Vec2;
         m_allowSleep : Boolean;

         m_destructionListener : access b2DestructionListener'Class;
         m_debugDraw           : access b2Draw;

         -- This is used to compute the time step ratio to support a variable time step.
         --
         m_inv_dt0 : Real;

         m_newContacts,
         m_locked,
         m_clearForces : Boolean;

         -- These are for debugging the solver.
         --
         m_warmStarting,
         m_continuousPhysics,
         m_subStepping       : Boolean;

         m_stepComplete : Boolean;

         m_profile      : b2Profile;
      end record;



--  struct b2AABB;
--  struct b2BodyDef;
--  struct b2Color;
--  struct b2JointDef;
--  class b2Body;
--  class b2Draw;
--  class b2Fixture;
--  class b2Joint;
--
--  private:
--
--    friend class b2Body;
--    friend class b2Fixture;
--    friend class b2ContactManager;
--    friend class b2Controller;
--



--    void Solve(const b2TimeStep& step);
--

   procedure Solve (Self : in out b2World;   step : in b2TimeStep);



--    void SolveTOI(const b2TimeStep& step);
--

   procedure SolveTOI (Self : in out b2World;   step : in b2TimeStep);



--    void DrawShape(b2Fixture* shape, const b2Transform& xf, const b2Color& color);
--

   procedure DrawShape (Self : in out b2World;   shape : access b2Fixture;
                                                 xf    : in     b2Transform;
                                                 color : in     b2Color);

end box2d.b2_World;
