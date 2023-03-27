with
     box2d.b2_broad_Phase,
     box2d.b2_Contact,
     box2d.b2_world_Callbacks,
     --  box2d.b2_Settings,

     System;


package box2d.b2_contact_Manager
is
   use b2_broad_Phase,
       b2_Contact,
       b2_world_Callbacks;
       --  b2_Settings;


   --  class b2Contact;
   --  class b2ContactFilter;
   --  class b2ContactListener;
   --  class b2BlockAllocator;


   --  Delegate of b2World.
   --

   --  class b2ContactManager
   --  {
   --  public:
   --    b2BroadPhase m_broadPhase;
   --    b2Contact* m_contactList;
   --    int32 m_contactCount;
   --    b2ContactFilter* m_contactFilter;
   --    b2ContactListener* m_contactListener;
   --    b2BlockAllocator* m_allocator;
   --  };

   type b2ContactManager is tagged
      record
           m_broadPhase      : aliased b2BroadPhase;
           m_contactList     : access  b2Contact;
           m_contactCount    :         Natural;
           m_contactFilter   : access  b2ContactFilter;
           m_contactListener : access  b2ContactListener'Class;
      end record;





   --    b2ContactManager();
   --

   function to_b2ContactManager return b2ContactManager;

   procedure destruct (Self : in out b2ContactManager);


   --    // Broad-phase callback.
   --
   --    void AddPair(void* proxyUserDataA, void* proxyUserDataB);
   --

   procedure addPair (Self : access b2ContactManager;   proxyUserDataA : void_ptr;
                                                        proxyUserDataB : void_ptr);


   --  procedure addPair (Self : in out b2ContactManager;   proxyUserDataA : system.Address;
   --                                                       proxyUserDataB : system.Address);


   --    void FindNewContacts();
   --

   procedure findNewContacts (Self : in out b2ContactManager);



   --    void Destroy(b2Contact* c);
   --

   procedure destroy (Self : in out b2ContactManager;   c : access b2Contact);



   --    void Collide();
   --

   procedure collide (Self : in out b2ContactManager);



end box2d.b2_contact_Manager;
