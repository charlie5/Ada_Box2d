with
     b2_Math,
     interfaces.C.Pointers;


package b2_Pointers
is
   use b2_Math;



   type b2Vec2_array is array (Natural range <>) of aliased b2_Math.b2Vec2;

   Terminator : constant b2_Math.b2Vec2 := (b2_Math.Real'Last, b2_Math.Real'Last);

   package b2Vec2_Pointers is new interfaces.C.Pointers (Index              => Natural,
                                                         Element            => b2Vec2,
                                                         Element_array      => b2Vec2_array,
                                                         Default_Terminator => Terminator);
   subtype b2Vec2_ptr is b2Vec2_Pointers.Pointer;


end b2_Pointers;
