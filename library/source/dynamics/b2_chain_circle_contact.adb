--  #include "b2_chain_circle_contact.h"
--  #include "box2d/b2_block_allocator.h"
--  #include "box2d/b2_fixture.h"
--  #include "box2d/b2_chain_shape.h"
--  #include "box2d/b2_edge_shape.h"
--
--  #include <new>

package body b2_chain_circle_Contact
is
   procedure dummy is null;


   --
--  b2Contact* b2ChainAndCircleContact::Create(b2Fixture* fixtureA, int32 indexA, b2Fixture* fixtureB, int32 indexB, b2BlockAllocator* allocator)
--  {
--    void* mem = allocator->Allocate(sizeof(b2ChainAndCircleContact));
--    return new (mem) b2ChainAndCircleContact(fixtureA, indexA, fixtureB, indexB);
--  }
--
--  void b2ChainAndCircleContact::Destroy(b2Contact* contact, b2BlockAllocator* allocator)
--  {
--    ((b2ChainAndCircleContact*)contact)->~b2ChainAndCircleContact();
--    allocator->Free(contact, sizeof(b2ChainAndCircleContact));
--  }
--
--  b2ChainAndCircleContact::b2ChainAndCircleContact(b2Fixture* fixtureA, int32 indexA, b2Fixture* fixtureB, int32 indexB)
--  : b2Contact(fixtureA, indexA, fixtureB, indexB)
--  {
--    b2Assert(m_fixtureA->GetType() == b2Shape::e_chain);
--    b2Assert(m_fixtureB->GetType() == b2Shape::e_circle);
--  }
--
--  void b2ChainAndCircleContact::Evaluate(b2Manifold* manifold, const b2Transform& xfA, const b2Transform& xfB)
--  {
--    b2ChainShape* chain = (b2ChainShape*)m_fixtureA->GetShape();
--    b2EdgeShape edge;
--    chain->GetChildEdge(&edge, m_indexA);
--    b2CollideEdgeAndCircle( manifold, &edge, xfA,
--                      (b2CircleShape*)m_fixtureB->GetShape(), xfB);
--  }
end b2_chain_circle_Contact;
