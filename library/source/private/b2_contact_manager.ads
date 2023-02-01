--  #include "b2_api.h"
--  #include "b2_broad_phase.h"

package b2_contact_Manager
is
   procedure dummy;


   --
--  class b2Contact;
--  class b2ContactFilter;
--  class b2ContactListener;
--  class b2BlockAllocator;
--
--  // Delegate of b2World.
--  class B2_API b2ContactManager
--  {
--  public:
--    b2ContactManager();
--
--    // Broad-phase callback.
--    void AddPair(void* proxyUserDataA, void* proxyUserDataB);
--
--    void FindNewContacts();
--
--    void Destroy(b2Contact* c);
--
--    void Collide();
--
--    b2BroadPhase m_broadPhase;
--    b2Contact* m_contactList;
--    int32 m_contactCount;
--    b2ContactFilter* m_contactFilter;
--    b2ContactListener* m_contactListener;
--    b2BlockAllocator* m_allocator;
--  };
--
end b2_contact_Manager;
