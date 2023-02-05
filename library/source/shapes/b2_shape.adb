package body b2_Shape
is

   --    Get the type of this shape. You can use this to down cast to the concrete shape.
   --    @return the shape type.
   --
   --  inline b2Shape::Type b2Shape::GetType() const
   --  {
   --    return m_type;
   --  }
   --

   function getType (Self : in b2Shape) return shape_Type
   is
   begin
      return Self.m_Type;
   end getType;


end b2_Shape;
