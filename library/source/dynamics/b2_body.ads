with
     b2_Joint,
     b2_Fixture,
     b2_Math,
     b2_Shape,
     b2_Settings,
     Interfaces;

limited
with
     b2_Contact,
     --  b2_Fixture,
     b2_World;


package b2_Body
is
   use b2_Joint,
       b2_Shape,
       b2_Fixture,
       b2_Math,
       b2_Settings;


   --  class b2Fixture;
   --  class b2Joint;
   --  class b2Contact;
   --  class b2Controller;
   --  class b2World;
   --  struct b2FixtureDef;
   --  struct b2JointEdge;
   --  struct b2ContactEdge;
   --



   -------------
   -- b2BodyType
   --

   --  The body type.
   --
   --  static: zero mass, zero velocity, may be manually moved
   --  kinematic: zero mass, non-zero velocity set by user, moved by solver
   --  dynamic: positive mass, non-zero velocity determined by forces, moved by solver
   --
   --  enum b2BodyType
   --  {
   --    b2_staticBody = 0,
   --    b2_kinematicBody,
   --    b2_dynamicBody
   --  };
   --

   type b2BodyType is (b2_staticBody,
                       b2_kinematicBody,
                       b2_dynamicBody);



   ------------
   -- b2BodyDef
   --

   --  A body definition holds all the data needed to construct a rigid body.
   --  You can safely re-use body definitions. Shapes are added to a body after construction.
   --
   --  struct b2BodyDef
   --  {
   --    The body type: static, kinematic, or dynamic.
   --    Note: if a dynamic body would have zero mass, the mass is set to one.
   --
   --    b2BodyType type;
   --

   --    The world position of the body. Avoid creating bodies at the origin
   --    since this can lead to many overlapping shapes.
   --
   --    b2Vec2 position;
   --

   --    The world angle of the body in radians.
   --
   --    float angle;
   --

   --    The linear velocity of the body's origin in world co-ordinates.
   --
   --    b2Vec2 linearVelocity;
   --

   --    The angular velocity of the body.
   --
   --    float angularVelocity;
   --

   --    Linear damping is use to reduce the linear velocity. The damping parameter
   --    can be larger than 1.0f but the damping effect becomes sensitive to the
   --    time step when the damping parameter is large.
   --    Units are 1/time
   --
   --    float linearDamping;
   --

   --    Angular damping is use to reduce the angular velocity. The damping parameter
   --    can be larger than 1.0f but the damping effect becomes sensitive to the
   --    time step when the damping parameter is large.
   --    Units are 1/time
   --
   --    float angularDamping;
   --

   --    Set this flag to false if this body should never fall asleep. Note that
   --    this increases CPU usage.
   --
   --    bool allowSleep;
   --

   --    Is this body initially awake or sleeping?
   --
   --    bool awake;
   --

   --    Should this body be prevented from rotating? Useful for characters.
   --
   --    bool fixedRotation;
   --

   --    Is this a fast moving body that should be prevented from tunneling through
   --    other moving bodies? Note that all bodies are prevented from tunneling through
   --    kinematic and static bodies. This setting is only considered on dynamic bodies.
   --    @warning You should use this flag sparingly since it increases processing time.
   --
   --    bool bullet;
   --

   --    Does this body start out enabled?
   --
   --    bool enabled;
   --

   --    Use this to store application specific body data.
   --
   --    b2BodyUserData userData;
   --

   --    Scale the gravity applied to this body.
   --
   --    float gravityScale;
   --  };
   --

   type b2BodyDef is
      record
         --    The body type: static, kinematic, or dynamic.
         --    Note: if a dynamic body would have zero mass, the mass is set to one.
         --
         Kind            : b2BodyType;

         --    The world position of the body. Avoid creating bodies at the origin
         --    since this can lead to many overlapping shapes.
         --
         position        : b2Vec2;

         --    The world angle of the body in radians.
         --
         angle           : Real;

         --    The linear velocity of the body's origin in world co-ordinates.
         --
         linearVelocity  : b2Vec2;

         --    The angular velocity of the body.
         --
         angularVelocity : Real;

         --    Linear damping is use to reduce the linear velocity. The damping parameter
         --    can be larger than 1.0f but the damping effect becomes sensitive to the
         --    time step when the damping parameter is large.
         --    Units are 1/time

         linearDamping   : Real;
         --
         --    Angular damping is use to reduce the angular velocity. The damping parameter
         --    can be larger than 1.0f but the damping effect becomes sensitive to the
         --    time step when the damping parameter is large.
         --    Units are 1/time

         angularDamping  : Real;
         --
         --    Set this flag to false if this body should never fall asleep. Note that
         --    this increases CPU usage.

         allowSleep      : Boolean;

         --    Is this body initially awake or sleeping?
         --
         awake           : Boolean;

         --    Should this body be prevented from rotating? Useful for characters.
         --
         fixedRotation   : Boolean;

         --    Is this a fast moving body that should be prevented from tunneling through
         --    other moving bodies? Note that all bodies are prevented from tunneling through
         --    kinematic and static bodies. This setting is only considered on dynamic bodies.
         --    @warning You should use this flag sparingly since it increases processing time.
         --
         bullet          : Boolean;

         --    Does this body start out enabled?
         --
         enabled         : Boolean;

         --    Use this to store application specific body data.
         --
         userData        : b2BodyUserData;

         --    Scale the gravity applied to this body.
         --
         gravityScale    : Real;
      end record;


      -- This constructor sets the body definition default values.
      --

      function to_b2BodyDef return b2BodyDef;




   ---------
   -- b2Body
   --

   --  A rigid body. These are created via b2World::CreateBody.
   --



   --  class b2Body
   --  {
   --  public:
   --  }

   type b2Body is tagged private;

   type b2Body_ptr is access all b2Body;






   --    ~b2Body();
   --

   procedure destruct (Self : in out b2Body);
   procedure free     (Self : in out b2Body_ptr);




   --    Creates a fixture and attach it to this body. Use this function if you need
   --    to set some fixture parameters, like friction. Otherwise you can create the
   --    fixture directly from a shape.
   --    If the density is non-zero, this function automatically updates the mass of the body.
   --    Contacts are not created until the next time step.
   --
   --    @param def the fixture definition.
   --    @warning This function is locked during callbacks.
   --
   --    b2Fixture* CreateFixture(const b2FixtureDef* def);
   --

   function createFixture (Self : in out b2Body;   def : in b2_Fixture.b2FixtureDef) return b2_Fixture.b2Fixture_ptr;




   --    Creates a fixture from a shape and attach it to this body.
   --    This is a convenience function. Use b2FixtureDef if you need to set parameters
   --    like friction, restitution, user data, or filtering.
   --    If the density is non-zero, this function automatically updates the mass of the body.
   --
   --    @param shape the shape to be cloned.
   --    @param density the shape density (set to zero for static bodies).
   --    @warning This function is locked during callbacks.
   --
   --    b2Fixture* CreateFixture(const b2Shape* shape, float density);
   --

   function createFixture (Self : in out b2Body;   Shape   : access constant b2Shape'Class;
                                                   Density : in              Real) return b2_Fixture.b2Fixture_ptr;




   --    Destroy a fixture. This removes the fixture from the broad-phase and
   --    destroys all contacts associated with this fixture. This will
   --    automatically adjust the mass of the body if the body is dynamic and the
   --    fixture has positive density.
   --    All fixtures attached to a body are implicitly destroyed when the body is destroyed.
   --
   --    @param fixture the fixture to be removed.
   --    @warning This function is locked during callbacks.
   --
   --    void DestroyFixture(b2Fixture* fixture);
   --

   procedure destroyFixture (Self : access b2Body;   fixture : in out b2_Fixture.b2Fixture_ptr);




   --    Set the position of the body's origin and rotation.
   --    Manipulating a body's transform may cause non-physical behavior.
   --    Note: contacts are updated on the next call to b2World::Step.
   --
   --    @param position the world position of the body's local origin.
   --    @param angle the world rotation in radians.
   --
   --    void SetTransform(const b2Vec2& position, float angle);
   --

   procedure setTransform (Self : in out b2Body;   position : in b2Vec2;
                                                   Angle    : in Real);



   --    Get the body transform for the body's origin.
   --
   --    @return the world transform of the body's origin.
   --
   --    const b2Transform& GetTransform() const;
   --

   function getTransform (Self : in b2Body) return b2Transform;



   --    Get the world body origin position.
   --
   --    @return the world position of the body's origin.
   --
   --    const b2Vec2& GetPosition() const;
   --

   function getPosition (Self : in b2Body) return b2Vec2;



   --    Get the angle in radians.
   --
   --    @return the current world rotation angle in radians.
   --
   --    float GetAngle() const;
   --

   function getAngle (Self : in b2Body) return Real;


   --    Get the world position of the center of mass.
   --
   --    const b2Vec2& GetWorldCenter() const;
   --

   function getWorldCenter (Self : in b2Body) return b2Vec2;




   --    Get the local position of the center of mass.
   --
   --    const b2Vec2& GetLocalCenter() const;
   --

   function getLocalCenter (Self : in b2Body) return b2Vec2;



   --    Set the linear velocity of the center of mass.
   --
   --    @param v the new linear velocity of the center of mass.
   --
   --    void SetLinearVelocity(const b2Vec2& v);
   --

   procedure setLinearVelocity (Self : in out b2Body;   v : in b2Vec2);



   --    Get the linear velocity of the center of mass.
   --
   --    @return the linear velocity of the center of mass.
   --
   --    const b2Vec2& GetLinearVelocity() const;
   --

   function getLinearVelocity (Self : in b2Body) return b2Vec2;




   --    Set the angular velocity.
   --
   --    @param omega the new angular velocity in radians/second.
   --
   --    void SetAngularVelocity(float omega);
   --

   procedure setAngularVelocity (Self : in out b2Body;   omega : in Real);




   --    Get the angular velocity.
   --
   --    @return the angular velocity in radians/second.
   --
   --    float GetAngularVelocity() const;
   --

   function getAngularVelocity (Self : in b2Body) return Real;



   --    Apply a force at a world point. If the force is not
   --    applied at the center of mass, it will generate a torque and
   --    affect the angular velocity. This wakes up the body.
   --
   --    @param force the world force vector, usually in Newtons (N).
   --    @param point the world position of the point of application.
   --    @param wake also wake up the body
   --
   --    void ApplyForce(const b2Vec2& force, const b2Vec2& point, bool wake);
   --

   procedure applyForce (Self : in out b2Body;   force : in b2Vec2;
                                                 Point : in b2Vec2;
                                                 Wake  : in Boolean);



   --    Apply a force to the center of mass. This wakes up the body.
   --
   --    @param force the world force vector, usually in Newtons (N).
   --    @param wake also wake up the body
   --
   --    void ApplyForceToCenter(const b2Vec2& force, bool wake);
   --

   procedure applyForceToCenter (Self : in out b2Body;   force : in b2Vec2;
                                                         Wake  : in Boolean);




   --    Apply a torque. This affects the angular velocity
   --    without affecting the linear velocity of the center of mass.
   --
   --    @param torque about the z-axis (out of the screen), usually in N-m.
   --    @param wake also wake up the body
   --
   --    void ApplyTorque(float torque, bool wake);
   --

   procedure ApplyTorque (Self : in out b2Body;   Torque : in Real;
                                                  Wake   : in Boolean);



   --    Apply an impulse at a point. This immediately modifies the velocity.
   --    It also modifies the angular velocity if the point of application
   --    is not at the center of mass. This wakes up the body.
   --
   --    @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
   --    @param point the world position of the point of application.
   --    @param wake also wake up the body
   --
   --    void ApplyLinearImpulse(const b2Vec2& impulse, const b2Vec2& point, bool wake);
   --

   procedure applyLinearImpulse (Self : in out b2Body;   impulse : in b2Vec2;
                                                         Point   : in b2Vec2;
                                                         Wake    : in Boolean);



   --    Apply an impulse to the center of mass. This immediately modifies the velocity.
   --
   --    @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
   --    @param wake also wake up the body
   --
   --    void ApplyLinearImpulseToCenter(const b2Vec2& impulse, bool wake);
   --

   procedure applyLinearImpulseToCenter (Self : in out b2Body;   Impulse : in b2Vec2;
                                                                 Wake    : in Boolean);



   --    Apply an angular impulse.
   --
   --    @param impulse the angular impulse in units of kg*m*m/s
   --    @param wake also wake up the body
   --
   --    void ApplyAngularImpulse(float impulse, bool wake);
   --

   procedure applyAngularImpulse (Self : in out b2Body;   impulse : in Real;
                                  Wake    : in Boolean);



   --    Get the total mass of the body.
   --
   --    @return the mass, usually in kilograms (kg).
   --
   --    float GetMass() const;
   --

   function getMass (Self : in b2Body) return Real;




   --    Get the rotational inertia of the body about the local origin.
   --
   --    @return the rotational inertia, usually in kg-m^2.
   --
   --    float GetInertia() const;
   --

   function getInertia (Self : in b2Body) return Real;



   --    Get the mass data of the body.
   --
   --    @return a struct containing the mass, inertia and center of the body.
   --
   --    void GetMassData(b2MassData* data) const;
   --

   procedure getMassData (Self : in b2Body;   data : out b2massData);



   --    Set the mass properties to override the mass properties of the fixtures.
   --    Note that this changes the center of mass position.
   --    Note that creating or destroying fixtures can also alter the mass.
   --    This function has no effect if the body isn't dynamic.
   --
   --    @param data the mass properties.
   --
   --    void SetMassData(const b2MassData* data);
   --

   procedure setMassData (Self : in out b2Body;   data : in b2massData);




   --    This resets the mass properties to the sum of the mass properties of the fixtures.
   --    This normally does not need to be called unless you called SetMassData to override
   --    the mass and you later want to reset the mass.
   --
   --    void ResetMassData();
   --

   procedure resetMassData (Self : in out b2Body);




   --    Get the world coordinates of a point given the local coordinates.
   --
   --    @param localPoint a point on the body measured relative the the body's origin.
   --    @return the same point expressed in world coordinates.
   --
   --    b2Vec2 GetWorldPoint(const b2Vec2& localPoint) const;
   --

   function getWorldPoint (Self : in b2Body;   localPoint : in b2Vec2) return b2Vec2;



   --    Get the world coordinates of a vector given the local coordinates.
   --
   --    @param localVector a vector fixed in the body.
   --    @return the same vector expressed in world coordinates.
   --
   --    b2Vec2 GetWorldVector(const b2Vec2& localVector) const;
   --

   function getWorldVector (Self : in b2Body;   localVector : in b2Vec2) return b2Vec2;




   --    Gets a local point relative to the body's origin given a world point.
   --
   --    @param worldPoint a point in world coordinates.
   --    @return the corresponding local point relative to the body's origin.
   --
   --    b2Vec2 GetLocalPoint(const b2Vec2& worldPoint) const;
   --

   function getLocalPoint (Self : in b2Body;   worldPoint : in b2Vec2) return b2Vec2;



   --    Gets a local vector given a world vector.
   --
   --    @param worldVector a vector in world coordinates.
   --    @return the corresponding local vector.
   --
   --    b2Vec2 GetLocalVector(const b2Vec2& worldVector) const;
   --

   function getLocalVector (Self : in b2Body;   worldVector : in b2Vec2) return b2Vec2;




   --    Get the world linear velocity of a world point attached to this body.
   --
   --    @param worldPoint a point in world coordinates.
   --    @return the world velocity of a point.
   --
   --    b2Vec2 GetLinearVelocityFromWorldPoint(const b2Vec2& worldPoint) const;
   --

   function getLinearVelocityFromWorldPoint (Self : in b2Body;   worldPoint : in b2Vec2) return b2Vec2;




   --    Get the world velocity of a local point.
   --
   --    @param localPoint a point in local coordinates.
   --    @return the world velocity of a point.
   --
   --    b2Vec2 GetLinearVelocityFromLocalPoint(const b2Vec2& localPoint) const;
   --

   function getLinearVelocityFromLocalPoint (Self : in b2Body;   localPoint : in b2Vec2) return b2Vec2;



   --    Get the linear damping of the body.
   --
   --    float GetLinearDamping() const;
   --

   function getLinearDamping (Self : in b2Body) return Real;



   --    Set the linear damping of the body.
   --
   --    void SetLinearDamping(float linearDamping);
   --

   procedure setLinearDamping (Self : in out b2Body;   linearDamping : in Real);



   --    Get the angular damping of the body.
   --
   --    float GetAngularDamping() const;
   --

   function getAngularDamping (Self : in b2Body) return Real;




   --    Set the angular damping of the body.
   --
   --    void SetAngularDamping(float angularDamping);
   --

   procedure setAngularDamping (Self : in out b2Body;   angularDamping : in Real);



   --    Get the gravity scale of the body.
   --
   --    float GetGravityScale() const;
   --

   function getGravityScale (Self : in b2Body) return Real;



   --    Set the gravity scale of the body.
   --
   --    void SetGravityScale(float scale);
   --

   procedure setGravityScale (Self : in out b2Body;   Scale : in Real);



   --    Set the type of this body. This may alter the mass and velocity.
   --
   --    void SetType(b2BodyType type);
   --

   procedure setType (Self : in out b2Body;   Kind : in b2BodyType);



   --    Get the type of this body.
   --
   --    b2BodyType GetType() const;
   --

   function getType (Self : in b2Body) return b2BodyType;




   --    Should this body be treated like a bullet for continuous collision detection?
   --
   --    void SetBullet(bool flag);
   --

   procedure setBullet (Self : in out b2Body;   Flag : in Boolean);




   --    Is this body treated like a bullet for continuous collision detection?
   --
   --    bool IsBullet() const;
   --

   function IsBullet (Self : in b2Body) return Boolean;




   --    You can disable sleeping on this body. If you disable sleeping, the
   --    body will be woken.
   --
   --    void SetSleepingAllowed(bool flag);
   --

   procedure setSleepingAllowed (Self : in out b2Body;   Flag : in Boolean);




   --    Is this body allowed to sleep
   --
   --    bool IsSleepingAllowed() const;
   --

   function isSleepingAllowed (Self : in b2Body) return Boolean;




   --    Set the sleep state of the body. A sleeping body has very
   --    low CPU cost.
   --
   --    @param flag set to true to wake the body, false to put it to sleep.
   --
   --    void SetAwake(bool flag);
   --

   procedure setAwake (Self : in out b2Body;   Flag : in Boolean);




   --    Get the sleeping state of this body.
   --
   --    @return true if the body is awake.
   --
   --    bool IsAwake() const;
   --

   function IsAwake (Self : in b2Body) return Boolean;




   --    Allow a body to be disabled. A disabled body is not simulated and cannot
   --    be collided with or woken up.
   --    If you pass a flag of true, all fixtures will be added to the broad-phase.
   --    If you pass a flag of false, all fixtures will be removed from the
   --    broad-phase and all contacts will be destroyed.
   --    Fixtures and joints are otherwise unaffected. You may continue
   --    to create/destroy fixtures and joints on disabled bodies.
   --    Fixtures on a disabled body are implicitly disabled and will
   --    not participate in collisions, ray-casts, or queries.
   --    Joints connected to a disabled body are implicitly disabled.
   --    An diabled body is still owned by a b2World object and remains
   --    in the body list.
   --
   --    void SetEnabled(bool flag);
   --

   procedure setEnabled (Self : in out b2Body;   Flag : in Boolean);




   --    Get the active state of the body.
   --
   --    bool IsEnabled() const;
   --

   function isEnabled (Self : in b2Body) return Boolean;




   --    Set this body to have fixed rotation. This causes the mass
   --    to be reset.
   --
   --    void SetFixedRotation(bool flag);
   --

   procedure setFixedRotation (Self : in out b2Body;   Flag : in Boolean);




   --    Does this body have fixed rotation?
   --
   --    bool IsFixedRotation() const;
   --

   function isFixedRotation (Self : in b2Body) return Boolean;



   --    Get the list of all fixtures attached to this body.
   --
   --    b2Fixture* GetFixtureList();
   --

   function getFixtureList (Self : in out b2Body) return access b2_Fixture.b2Fixture;


   --    const b2Fixture* GetFixtureList() const;
   --

   --  function getFixtureList (Self : in b2Body) return access constant b2_Fixture.b2Fixture;




   --    Get the list of all joints attached to this body.
   --
   --    b2JointEdge* GetJointList();
   --

   function getJointList (Self : in out b2Body) return access b2_Joint.b2JointEdge;



   --    const b2JointEdge* GetJointList() const;
   --

   --  function getJointList (Self : in b2Body) return access constant b2_Joint.b2JointEdge;





   --    Get the list of all contacts attached to this body.
   --
   --    @warning this list changes during the time step and you may
   --    miss some collisions if you don't use b2ContactListener.
   --
   --    b2ContactEdge* GetContactList();
   --

   function getContactList (Self : in out b2Body) return access b2_Contact.b2ContactEdge;


   --    const b2ContactEdge* GetContactList() const;
   --

   --  function getContactList (Self : in b2Body) return b2_Contact.b2ContactEdge;



   --    Get the next body in the world's body list.
   --
   --    b2Body* GetNext();
   --

   function getNext (Self : in out b2Body) return access b2Body;


   --    const b2Body* GetNext() const;
   --

   --  function getNext (Self : in b2Body) return access constant b2Body;



   --    Get the user data pointer that was provided in the body definition.
   --
   --    b2BodyUserData& GetUserData();
   --

   function getUserData (Self : in b2Body) return b2BodyUserData;



   --    Set the user data. Use this to store your application specific data.
   --
   --    void SetUserData(void* data);
   --

   procedure setUserData (Self : in out b2Body;   data : in b2BodyUserData);


   --    Get the parent world of this body.
   --
   --    b2World* GetWorld();
   --

   function getWorld (Self : in out b2Body) return access b2_World.b2World;


   --    const b2World* GetWorld() const;
   --

   function getWorld (Self : in b2Body) return b2_World.b2World;



   --    Dump this body to a file
   --
   --    void Dump();
   --

   procedure dump (Self : in out b2Body);



   --    b2Body(const b2BodyDef* bd, b2World* world);
   --

   function to_b2Body (bd : in b2BodyDef;   world : access b2_World.b2World) return b2Body;





   -------------------------------
   --- Protected C++ functions ---
   -------------------------------


   function  m_contactList    (Self : in     b2Body)  return access b2_Contact.b2ContactEdge;
   procedure m_contactList_is (Self : in out b2Body;   Now : access b2_Contact.b2ContactEdge);


   function  m_Force     (Self : in     b2Body)     return b2Vec2;
   procedure m_Force_is  (Self : in out b2Body;   Now : in b2Vec2);

   function  m_Torque    (Self : in     b2Body)     return Real;
   procedure m_Torque_is (Self : in out b2Body;   Now : in Real);


   procedure m_prev_is (Self : in out b2Body;   Now : in b2Body_ptr);
   procedure m_next_is (Self : in out b2Body;   Now : in b2Body_ptr);

   function  m_prev    (Self : in     b2Body) return b2Body_ptr;
   function  m_next    (Self : in     b2Body) return b2Body_ptr;


   procedure m_jointList_is   (Self : in out b2Body;   Now : access b2JointEdge);
   procedure m_fixtureList_is (Self : in out b2Body;   Now : in     b2Fixture_ptr);

   function  m_sleepTime      (Self : in     b2Body)         return Real;
   procedure m_sleepTime_is   (Self : in out b2Body;   Now : in     Real);

   procedure decrement_m_fixtureCount (Self : in out b2Body);
   procedure      zero_m_fixtureCount (Self : in out b2Body);


   --    This is used to prevent connected bodies from colliding.
   --    It may lie, depending on the collideConnected flag.
   --
   --    bool ShouldCollide(const b2Body* other) const;
   --

   function shouldCollide (Self : in b2Body;   other : access b2Body) return Boolean;


   function  m_islandIndex    (Self : in     b2Body)     return Natural;
   procedure m_islandIndex_is (Self : in out b2Body;   Now : in Natural);



   function m_invMass     (Self : in b2Body) return Real;
   function m_invI        (Self : in b2Body) return Real;

   function m_xf    (Self : access b2Body) return access b2Transform;
   function m_sweep (Self : access b2Body) return access b2Sweep;



   --    void SynchronizeTransform();
   --

   procedure synchronizeTransform (Self : in out b2Body);


   --    void SynchronizeFixtures();
   --

   procedure synchronizeFixtures (Self : in out b2Body);


   --    void Advance(float t);
   --
   --  };
   --

   procedure advance (Self : in out b2Body;   t : in Real);



   -- m_flags
   --

   --  enum
   --  {
   --     e_islandFlag      = 0x0001,
   --     e_awakeFlag       = 0x0002,
   --     e_autoSleepFlag      = 0x0004,
   --     e_bulletFlag      = 0x0008,
   --     e_fixedRotationFlag  = 0x0010,
   --     e_enabledFlag     = 0x0020,
   --     e_toiFlag         = 0x0040
   --  };

   subtype Flag     is interfaces.Unsigned_16;
   subtype flag_Set is interfaces.Unsigned_16;


   e_islandFlag         : constant Flag := 16#0001#;
   e_awakeFlag          : constant Flag := 16#0002#;
   e_autoSleepFlag      : constant Flag := 16#0004#;
   e_bulletFlag         : constant Flag := 16#0008#;
   e_fixedRotationFlag  : constant Flag := 16#0010#;
   e_enabledFlag        : constant Flag := 16#0020#;
   e_toiFlag            : constant Flag := 16#0040#;

   function  m_Flags    (Self : in b2Body)      return flag_Set;
   procedure m_Flags_is (Self : in out b2Body;   Now : flag_Set);



private

   --    friend class b2World;
   --    friend class b2Island;
   --    friend class b2ContactManager;
   --    friend class b2ContactSolver;
   --    friend class b2Contact;
   --
   --    friend class b2DistanceJoint;
   --    friend class b2FrictionJoint;
   --    friend class b2GearJoint;
   --    friend class b2MotorJoint;
   --    friend class b2MouseJoint;
   --    friend class b2PrismaticJoint;
   --    friend class b2PulleyJoint;
   --    friend class b2RevoluteJoint;
   --    friend class b2RopeJoint;
   --    friend class b2WeldJoint;
   --    friend class b2WheelJoint;
   --


   -- Private 'b2body' record components.
   --
   --    b2BodyType m_type;
   --
   --    uint16 m_flags;
   --
   --    int32 m_islandIndex;
   --
   --    b2Transform m_xf;    // the body origin transform
   --    b2Sweep m_sweep;     // the swept motion for CCD
   --
   --    b2Vec2 m_linearVelocity;
   --    float m_angularVelocity;
   --
   --    b2Vec2 m_force;
   --    float m_torque;
   --
   --    b2World* m_world;
   --    b2Body* m_prev;
   --    b2Body* m_next;
   --
   --    b2Fixture* m_fixtureList;
   --    int32 m_fixtureCount;
   --
   --    b2JointEdge* m_jointList;
   --    b2ContactEdge* m_contactList;
   --
   --    float m_mass, m_invMass;
   --
   --    // Rotational inertia about the center of mass.
   --    float m_I, m_invI;
   --
   --    float m_linearDamping;
   --    float m_angularDamping;
   --    float m_gravityScale;
   --
   --    float m_sleepTime;
   --
   --    b2BodyUserData m_userData;
   --

   type b2Body is tagged
      record
         m_type            : b2BodyType;

         m_flags           : flag_Set;

         m_islandIndex     : Natural;

         m_xf              : aliased b2Transform;    -- The body origin transform.
         m_sweep           : aliased b2Sweep;        -- Rhe swept motion for CCD.

         m_linearVelocity  : b2Vec2;
         m_angularVelocity : Real;

         m_force           : b2Vec2;
         m_torque          : Real;

         m_world           : access b2_World.b2World;
         m_prev            : access b2Body;
         m_next            : access b2Body;

         m_fixtureList     : aliased b2_Fixture.b2Fixture_ptr;
         m_fixtureCount    :         Natural;

         m_jointList       : access b2_Joint.b2JointEdge;
         m_contactList     : access b2_Contact.b2ContactEdge;

         m_mass,
         m_invMass         : Real;

         -- Rotational inertia about the center of mass.
         --
         m_I,
         m_invI            : Real;

         m_linearDamping   : Real;
         m_angularDamping  : Real;
         m_gravityScale    : Real;

         m_sleepTime       : Real;
         m_userData        : b2BodyUserData;
      end record;


end b2_Body;
