with
     b2_world_Callbacks,
     Interfaces;


package body b2_world_Callbacks
is
   use b2_Fixture;


   --  // Return true if contact calculations should be performed between these two shapes.
   --  // If you implement your own collision filter you may want to build from this implementation.
   --
   --  bool b2ContactFilter::ShouldCollide(b2Fixture* fixtureA, b2Fixture* fixtureB)
   --  {
   --    const b2Filter& filterA = fixtureA->GetFilterData();
   --    const b2Filter& filterB = fixtureB->GetFilterData();
   --
   --    if (filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0)
   --    {
   --       return filterA.groupIndex > 0;
   --    }
   --
   --    bool collide = (filterA.maskBits & filterB.categoryBits) != 0 && (filterA.categoryBits & filterB.maskBits) != 0;
   --    return collide;
   --  }

   function shouldCollide (Self : in out b2ContactFilter;   fixtureA,
                                                            fixtureB : access b2_Fixture.b2Fixture) return Boolean
   is
      filterA : constant b2Filter := fixtureA.getFilterData;
      filterB : constant b2Filter := fixtureB.getFilterData;

      collide : Boolean;

      use type Interfaces.Unsigned_16;
   begin
     if filterA.groupIndex = filterB.groupIndex and filterA.groupIndex /= 0
     then
        return filterA.groupIndex > 0;
     end if;

      collide :=     (filterA.maskBits     and filterB.categoryBits) /= 0
                 and (filterA.categoryBits and filterB.maskBits)     /= 0;
      return collide;
   end shouldCollide;


end b2_world_Callbacks;
