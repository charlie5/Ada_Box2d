with
     b2_Math,
     b2_Settings,

     Interfaces;


package b2_Draw
is
   use b2_Math,
       b2_Settings,
       Interfaces;


   -----------
   --- b2Color
   --

   --  Color for debug drawing. Each value has the range [0,1].
   --  struct b2Color
   --  {
   --    b2Color() {}
   --    b2Color(float rIn, float gIn, float bIn, float aIn = 1.0f)
   --    {
   --       r = rIn; g = gIn; b = bIn; a = aIn;
   --    }
   --
   --    void Set(float rIn, float gIn, float bIn, float aIn = 1.0f)
   --    {
   --       r = rIn; g = gIn; b = bIn; a = aIn;
   --    }
   --
   --    float r, g, b, a;
   --  };
   --

   type b2Color is
      record
         r,
         g,
         b,
         a : Real;
      end record;

   function to_b2Color (rIn, gIn, bIn : Real;
                        aIn           : Real := 1.0) return b2Color;


   procedure set (Self : out b2Color;   rIn, gIn, bIn : Real;
                                        aIn           : Real := 1.0);




   ----------
   --- b2Draw
   --

   --  Implement and register this class with a b2World to provide debug drawing of physics
   --  entities in your game.
   --

   --  class b2Draw
   --  {

   type b2Draw is abstract tagged private;

   --  public:

   --    b2Draw();
   --

   procedure define (Self : out b2Draw);



   --    virtual ~b2Draw() {}
   --

   procedure destruct (Self : in out b2Draw) is null;



   --    enum
   --    {
   --       e_shapeBit           = 0x0001,   ///< draw shapes
   --       e_jointBit           = 0x0002,   ///< draw joint connections
   --       e_aabbBit            = 0x0004,   ///< draw axis aligned bounding boxes
   --       e_pairBit            = 0x0008,   ///< draw broad-phase pairs
   --       e_centerOfMassBit    = 0x0010 ///< draw center of mass frame
   --    };
   --

   type    Flag     is new Unsigned_32;
   subtype flag_Set is Flag;

   e_shapeBit        : constant Flag;     -- Draw shapes.
   e_jointBit        : constant Flag;     -- Draw joint connections.
   e_aabbBit         : constant Flag;     -- Draw axis aligned bounding boxes.
   e_pairBit         : constant Flag;     -- Draw broad-phase pairs.
   e_centerOfMassBit : constant Flag;     -- Draw center of mass frame.



   --    Set the drawing flags.
   --    void SetFlags(uint32 flags);
   --

   procedure setFlags (Self : in out b2Draw;   Flags : in flag_Set);


   --    Get the drawing flags.
   --    uint32 GetFlags() const;
   --

   function getFlags (Self : in b2Draw) return flag_Set;


   --    Append flags to the current flags.
   --    void AppendFlags(uint32 flags);
   --

   procedure appendFlags (Self : in out b2Draw;   Flags : in flag_Set);


   --    Clear flags from the current flags.
   --    void ClearFlags(uint32 flags);
   --

   procedure clearFlags (Self : in out b2Draw;   Flags : in flag_Set);


   --    Draw a closed polygon provided in CCW order.
   --    virtual void DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) = 0;
   --

   procedure drawPolygon (Self : in out b2Draw;   vertices    : in b2Vec2s;
                                                  --  vertexCount : in Natural;
                                                  Color       : in b2Color) is abstract;


   --    Draw a solid closed polygon provided in CCW order.
   --    virtual void DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) = 0;
   --

   procedure drawSolidPolygon (Self : in out b2Draw;   vertices    : in b2Vec2s;
                                                       --  vertexCount : in Natural;
                                                       Color       : in b2Color) is abstract;


   --    Draw a circle.
   --    virtual void DrawCircle(const b2Vec2& center, float radius, const b2Color& color) = 0;
   --

   procedure drawCircle (Self : in out b2Draw;   Center : in b2Vec2;
                                                 Radius : in Real;
                                                 Color  : in b2Color) is abstract;


   --    Draw a solid circle.
   --    virtual void DrawSolidCircle(const b2Vec2& center, float radius, const b2Vec2& axis, const b2Color& color) = 0;
   --

   procedure drawSolidCircle (Self : in out b2Draw;   Center : in b2Vec2;
                                                      Radius : in Real;
                                                      Axis   : in b2Vec2;
                                                      Color  : in b2Color) is abstract;


   --    Draw a line segment.
   --    virtual void DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) = 0;
   --

   procedure drawSegment (Self : in out b2Draw;   p1, p2 : in b2Vec2;
                                                  Color  : in b2Color) is abstract;


   --    Draw a transform. Choose your own length scale.
   --    @param xf a transform.
   --    virtual void DrawTransform(const b2Transform& xf) = 0;
   --

   procedure drawTransform (Self : in out b2Draw;   xf : in b2Transform) is abstract;


   --    Draw a point.
   --    virtual void DrawPoint(const b2Vec2& p, float size, const b2Color& color) = 0;
   --

   procedure drawPoint (Self : in out b2Draw;   p     : in b2Vec2;
                                                Size  : in Real;
                                                Color : in b2Color) is abstract;



private
   --  protected:
   --    uint32 m_drawFlags;
   --  };
   --

   e_shapeBit        : constant Flag := 16#0001#;
   e_jointBit        : constant Flag := 16#0002#;
   e_aabbBit         : constant Flag := 16#0004#;
   e_pairBit         : constant Flag := 16#0008#;
   e_centerOfMassBit : constant Flag := 16#0010#;



   type b2Draw is abstract tagged
      record
         m_drawFlags : flag_Set;
      end record;


end b2_Draw;
