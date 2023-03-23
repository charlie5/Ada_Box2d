package body b2_Draw
is


   --  struct b2Color
   --  {
   --    b2Color() {}
   --    b2Color(float rIn, float gIn, float bIn, float aIn = 1.0f)
   --    {
   --       r = rIn; g = gIn; b = bIn; a = aIn;
   --    }
   --

   function to_b2Color (rIn, gIn, bIn : Real;
                        aIn           : Real := 1.0) return b2Color
   is
      Self : b2Color;
   begin
      Self.r := rIn;
      Self.g := gIn;
      Self.b := bIn;
      Self.a := aIn;

      return Self;
   end to_b2Color;



   --    void Set(float rIn, float gIn, float bIn, float aIn = 1.0f)
   --    {
   --       r = rIn; g = gIn; b = bIn; a = aIn;
   --    }
   --
   --    float r, g, b, a;
   --  };
   --

   procedure set (Self : out b2Color;   rIn, gIn, bIn : Real;
                                        aIn           : Real := 1.0)
   is
   begin
      Self.r := rIn;
      Self.g := gIn;
      Self.b := bIn;
      Self.a := aIn;
   end set;





   ----------
   --- b2Draw
   --


   --  b2Draw::b2Draw()
   --  {
   --    m_drawFlags = 0;
   --  }
   --

   procedure define (Self : out b2Draw)
   is
   begin
      Self.m_drawFlags := 0;
   end define;



   --  void b2Draw::SetFlags(uint32 flags)
   --  {
   --    m_drawFlags = flags;
   --  }
   --

   procedure setFlags (Self : in out b2Draw;   Flags : in flag_Set)
   is
   begin
      Self.m_drawFlags := flags;
   end setFlags;



   --  uint32 b2Draw::GetFlags() const
   --  {
   --    return m_drawFlags;
   --  }
   --

   function getFlags (Self : in b2Draw) return flag_Set
   is
   begin
      return Self.m_drawFlags;
   end getFlags;



   --  void b2Draw::AppendFlags(uint32 flags)
   --  {
   --    m_drawFlags |= flags;
   --  }
   --

   procedure appendFlags (Self : in out b2Draw;   Flags : in flag_Set)
   is
   begin
      Self.m_drawFlags := Self.m_drawFlags or flags;
   end appendFlags;



   --  void b2Draw::ClearFlags(uint32 flags)
   --  {
   --    m_drawFlags &= ~flags;
   --  }


   procedure clearFlags (Self : in out b2Draw;   Flags : in flag_Set)
   is
   begin
      Self.m_drawFlags := Self.m_drawFlags and not flags;
   end clearFlags;


end b2_Draw;
