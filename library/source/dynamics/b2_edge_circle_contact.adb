--  #include "b2_edge_circle_contact.h"
--
--  #include "box2d/b2_block_allocator.h"
--  #include "box2d/b2_fixture.h"
--
--  #include <new>

package body b2_edge_circle_Contact
is
   procedure dummy is null;


   --
--  b2Contact* b2EdgeAndCircleContact::Create(b2Fixture* fixtureA, int32, b2Fixture* fixtureB, int32, b2BlockAllocator* allocator)
--  {
--    void* mem = allocator->Allocate(sizeof(b2EdgeAndCircleContact));
--    return new (mem) b2EdgeAndCircleContact(fixtureA, fixtureB);
--  }
--
--  void b2EdgeAndCircleContact::Destroy(b2Contact* contact, b2BlockAllocator* allocator)
--  {
--    ((b2EdgeAndCircleContact*)contact)->~b2EdgeAndCircleContact();
--    allocator->Free(contact, sizeof(b2EdgeAndCircleContact));
--  }
--
--  b2EdgeAndCircleContact::b2EdgeAndCircleContact(b2Fixture* fixtureA, b2Fixture* fixtureB)
--  : b2Contact(fixtureA, 0, fixtureB, 0)
--  {
--    b2Assert(m_fixtureA->GetType() == b2Shape::e_edge);
--    b2Assert(m_fixtureB->GetType() == b2Shape::e_circle);
--  }
--
--  void b2EdgeAndCircleContact::Evaluate(b2Manifold* manifold, const b2Transform& xfA, const b2Transform& xfB)
--  {
--    b2CollideEdgeAndCircle( manifold,
--                         (b2EdgeShape*)m_fixtureA->GetShape(), xfA,
--                         (b2CircleShape*)m_fixtureB->GetShape(), xfB);
--  }
end b2_edge_circle_Contact;
