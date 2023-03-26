with ada.Text_IO; use ada.Text_IO;


package body box2d.b2_growable_Stack
is

   package body b2GrowableStack
   is
      use ada.Containers;



      function  to_Stack return Stack
      is
      begin
         return Self : Stack
         do
            Self.Elements.reserve_Capacity (Count_type (initial_Capacity));
         end return;
      end to_Stack;


      procedure push (Self : in out Stack;   E : in Element_T)
      is
         pragma assert (Check   => Self.Elements.Capacity >= Count_type (initial_Capacity),
                        Message => "Stack has not been initialised.");
      begin
         Self.Elements.append (E);
      end push;



      function pop (Self : in out Stack) return Element_T
      is
         Top : constant Element_t := Self.Elements.last_Element;
      begin
         Self.Elements.delete_Last;
         return Top;
      end pop;



      function getCount (Self : in  Stack)   return Natural
      is
      begin
         return Natural (Self.Elements.Length);
      end getCount;



      function getCapacity (Self : in  Stack)   return Natural
      is
      begin
         return Natural (Self.Elements.Capacity);
      end getCapacity;

      procedure setCapacity (Self : in out Stack;   To : in Natural)
      is
      begin
         Self.Elements.reserve_Capacity (Count_type (To));
      end setCapacity;


   end b2GrowableStack;


end box2d.b2_growable_Stack;
