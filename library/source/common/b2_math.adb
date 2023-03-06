with
     b2_Common,
     Interfaces;


package body b2_Math
is

   use Interfaces;


   --  inline bool b2IsValid(float x)
   --  {
   --    return isfinite(x);
   --  }

   function b2IsValid (x : in Real) return Boolean
   is
   begin
      return x'Valid;
   end b2IsValid;




   -------------------------------
   --  b2Vec2: A 2D column vector.
   --

   --    b2Vec2(float xIn, float yIn) : x(xIn), y(yIn) {}

   function to_b2Vec2 (x, y : in Real) return b2Vec2
   is
   begin
      return (x => x,
              y => y);
   end to_b2Vec2;



   --    void SetZero() { x = 0.0f; y = 0.0f; }

   procedure setZero (Self : out b2Vec2)
   is
   begin
      Self.x := 0.0;
      Self.y := 0.0;
   end setZero;



   --    void Set(float x_, float y_) { x = x_; y = y_; }

   procedure set (Self : out b2Vec2;   x, y : in Real)
   is
   begin
      Self.x := x;
      Self.y := y;
   end set;



   --    b2Vec2 operator -() const { b2Vec2 v; v.Set(-x, -y); return v; }

   function "-" (Self : in b2Vec2) return b2Vec2
   is
   begin
      return (-Self.x,
              -Self.y);
   end "-";



   --    float operator () (int32 i) const
   --    {
   --       return (&x)[i];
   --    }

   function Element (Self : in b2Vec2;   Index : Natural) return Real
   is
      pragma assert (   Index = 0
                     or Index = 1);
   begin
      if Index = 0 then return Self.x;
                   else return Self.y;
      end if;
   end Element;



   --    float& operator () (int32 i)
   --    {
   --       return (&x)[i];
   --    }

   function Element (Self : access b2Vec2;   Index : Natural) return access Real
   is
      pragma assert (   Index = 0
                     or Index = 1);
   begin
      if Index = 0 then return Self.x'Access;
                   else return Self.y'Access;
      end if;
   end Element;



   --    void operator += (const b2Vec2& v)
   --    {
   --       x += v.x; y += v.y;
   --    }

   procedure add_to (Self : in out b2Vec2;   Vector : in b2Vec2)
   is
   begin
      Self.x := Self.x + Vector.x;
      Self.y := Self.y + Vector.y;
   end add_to;



   --    void operator -= (const b2Vec2& v)
   --    {
   --       x -= v.x; y -= v.y;
   --    }

   procedure subtract_from (Self : in out b2Vec2;   Vector : in b2Vec2)
   is
   begin
      Self.x := Self.x - Vector.x;
      Self.y := Self.y - Vector.y;
   end subtract_from;



   --    void operator *= (float a)
   --    {
   --       x *= a; y *= a;
   --    }

   procedure multiply (Self : in out b2Vec2;   By : in Real)
   is
   begin
      Self.x := Self.x * By;
      Self.y := Self.y * By;
   end multiply;



   --    float Length() const
   --    {
   --       return b2Sqrt(x * x + y * y);
   --    }

   function Length (Self : in b2Vec2) return Real
   is
   begin
      return b2Sqrt (  Self.x * Self.x
                     + Self.y * Self.y);
   end Length;



   --    float LengthSquared() const
   --    {
   --       return x * x + y * y;
   --    }

   function LengthSquared (Self : in b2Vec2) return Real
   is
   begin
      return   Self.x * Self.x
             + Self.y * Self.y;
   end LengthSquared;



   --    float Normalize()
   --    {
   --       float length = Length();
   --       if (length < b2_epsilon)
   --       {
   --          return 0.0f;
   --       }
   --       float invLength = 1.0f / length;
   --       x *= invLength;
   --       y *= invLength;
   --
   --       return length;
   --    }

   function normalize (Self : in out b2Vec2) return Real
   is
      use b2_Common;

      l : constant Real := Length (Self);
   begin
      if l < b2_epsilon
      then
         return 0.0;
      end if;

      declare
         invLength : constant Real := 1.0 / l;
      begin
         Self.x := Self.x * invLength;
         Self.y := Self.y * invLength;
      end;

      return l;
   end Normalize;



   procedure normalize (Self : in out b2Vec2)
   is
      Unused : Real := normalize (Self)
        with Unreferenced;
   begin
      null;
   end normalize;



   --    bool IsValid() const
   --    {
   --       return b2IsValid(x) && b2IsValid(y);
   --    }

   function isValid (Self : in b2Vec2) return Boolean
   is
   begin
      return b2IsValid (Self.x)
         and b2IsValid (Self.y);
   end isValid;



   --    b2Vec2 Skew() const
   --    {
   --       return b2Vec2 (-y, x);
   --    }

   function Skew (Self : in b2Vec2) return b2Vec2
   is
   begin
      return (-Self.y,
               Self.x);
   end Skew;





   ------------------------------------------------
   --  b2Vec3 : A 2D column vector with 3 elements.
   --


   --    b2Vec3(float xIn, float yIn, float zIn) : x(xIn), y(yIn), z(zIn) {}

   function to_b2Vec3 (x, y, z : in Real) return b2Vec3
   is
   begin
      return (x, y, z);
   end to_b2Vec3;



   --    void SetZero() { x = 0.0f; y = 0.0f; z = 0.0f; }

   procedure setZero (Self : out b2Vec3)
   is
   begin
      Self := (0.0, 0.0, 0.0);
   end setZero;



   --    void Set(float x_, float y_, float z_) { x = x_; y = y_; z = z_; }

   procedure set (Self : out b2Vec3;   x, y, z : in Real)
   is
   begin
      Self := (x, y, z);
   end set;



   --    b2Vec3 operator -() const { b2Vec3 v; v.Set(-x, -y, -z); return v;}

   function "-" (Self : in b2Vec3) return b2Vec3
   is
   begin
      return (-Self.x, -Self.y, -Self.z);
   end "-";



   --    void operator += (const b2Vec3& v)
   --    {
   --       x += v.x; y += v.y; z += v.z;
   --    }

   procedure add_to (Self : in out b2Vec3;   Vector : in b2Vec3)
   is
   begin
      Self.x := Self.x + Vector.x;
      Self.y := Self.y + Vector.y;
      Self.z := Self.z + Vector.z;
   end add_to;



   --    void operator -= (const b2Vec3& v)
   --    {
   --       x -= v.x; y -= v.y; z -= v.z;
   --    }

   procedure subtract_from (Self : in out b2Vec3;   Vector : in b2Vec3)
   is
   begin
      Self.x := Self.x - Vector.x;
      Self.y := Self.y - Vector.y;
      Self.z := Self.z - Vector.z;
   end subtract_from;



   --    void operator *= (float s)
   --    {
   --       x *= s; y *= s; z *= s;
   --    }

   procedure multiply (Self : in out b2Vec3;   By : in Real)
   is
   begin
      Self.x := Self.x * By;
      Self.y := Self.y * By;
      Self.z := Self.z * By;
   end multiply;





   -----------------------------------------------------------
   --  b2Mat22: A 2-by-2 matrix. Stored in column-major order.
   --

   --    b2Mat22 (const b2Vec2& c1, const b2Vec2& c2)
   --    {
   --       ex = c1;
   --       ey = c2;
   --    }

   function to_b2Mat22 (c1, c2 : in b2Vec2) return b2Mat22
   is
   begin
      return (c1, c2);
   end to_b2Mat22;



   --    b2Mat22 (float a11, float a12, float a21, float a22)
   --    {
   --       ex.x = a11; ex.y = a21;
   --       ey.x = a12; ey.y = a22;
   --    }

   function to_b2Mat22 (a11, a12, a21, a22 : in Real) return b2Mat22
   is
   begin
      return (ex => (x => a11,
                     y => a21),
              ey => (x => a12,
                     y => a22));
   end to_b2Mat22;



   --    void Set(const b2Vec2& c1, const b2Vec2& c2)
   --    {
   --       ex = c1;
   --       ey = c2;
   --    }

   procedure set (Self : out b2Mat22;   c1, c2 : in b2Vec2)
   is
   begin
      Self.ex := c1;
      Self.ey := c2;
   end set;



   --    void SetIdentity()
   --    {
   --       ex.x = 1.0f; ey.x = 0.0f;
   --       ex.y = 0.0f; ey.y = 1.0f;
   --    }

   procedure setIdentity (Self : out b2Mat22)
   is
   begin
      Self := (ex => (1.0, 0.0),
               ey => (0.0, 1.0));
   end setIdentity;



   --    void SetZero()
   --    {
   --       ex.x = 0.0f; ey.x = 0.0f;
   --       ex.y = 0.0f; ey.y = 0.0f;
   --    }

   procedure setZero (Self : out b2Mat22)
   is
   begin
      Self := (ex => (0.0, 0.0),
               ey => (0.0, 0.0));
   end setZero;



   --    b2Mat22 GetInverse() const
   --    {
   --       float a = ex.x, b = ey.x, c = ex.y, d = ey.y;
   --       b2Mat22 B;
   --       float det = a * d - b * c;
   --       if (det != 0.0f)
   --       {
   --          det = 1.0f / det;
   --       }
   --       B.ex.x =  det * d;   B.ey.x = -det * b;
   --       B.ex.y = -det * c;   B.ey.y =  det * a;
   --       return B;
   --    }

   function getInverse (Self : in b2Mat22) return b2Mat22
   is
      a   : Real renames Self.ex.x;
      b   : Real renames Self.ey.x;
      c   : Real renames Self.ex.y;
      d   : Real renames Self.ey.y;

      det : Real := a * d  -  b * c;

      Result : b2Mat22;
   begin
      if det /= 0.0
      then
         det := 1.0 / det;
      end if;

      Result.ex.x :=  det * d;
      Result.ey.x := -det * b;
      Result.ex.y := -det * c;
      Result.ey.y :=  det * a;

      return Result;
   end getInverse;



   --    b2Vec2 Solve(const b2Vec2& b) const
   --    {
   --       float a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
   --       float det = a11 * a22 - a12 * a21;
   --       if (det != 0.0f)
   --       {
   --          det = 1.0f / det;
   --       }
   --       b2Vec2 x;
   --       x.x = det * (a22 * b.x - a12 * b.y);
   --       x.y = det * (a11 * b.y - a21 * b.x);
   --       return x;
   --    }

   function solve (Self : in b2Mat22;   b : in b2Vec2) return b2Vec2
   is
      a11 : Real renames Self.ex.x;
      a12 : Real renames Self.ey.x;
      a21 : Real renames Self.ex.y;
      a22 : Real renames Self.ey.y;

      det : Real := a11 * a22 - a12 * a21;
      x   : b2Vec2;
   begin
      if det /= 0.0
      then
         det := 1.0 / det;
      end if;

      x.x := det * (a22 * b.x - a12 * b.y);
      x.y := det * (a11 * b.y - a21 * b.x);

      return x;
   end solve;





   -----------------------------------------------------------
   --  b2Mat33: A 3-by-3 matrix. Stored in column-major order.
   --

   --    b2Mat33(const b2Vec3& c1, const b2Vec3& c2, const b2Vec3& c3)
   --    {
   --       ex = c1;
   --       ey = c2;
   --       ez = c3;
   --    }

   function to_b2Mat33 (c1, c2, c3 : in b2Vec3) return b2Mat33
   is
   begin
      return (c1, c2, c3);
   end to_b2Mat33;



   --    void SetZero()
   --    {
   --       ex.SetZero();
   --       ey.SetZero();
   --       ez.SetZero();
   --    }

   procedure setZero (Self : out b2Mat33)
   is
   begin
      setZero (Self.ex);
      setZero (Self.ey);
      setZero (Self.ez);
   end setZero;



   --  b2Vec3 b2Mat33::Solve33 (const b2Vec3&   b) const
   --  {
   --    float det = b2Dot(ex, b2Cross(ey, ez));
   --    if (det != 0.0f)
   --    {
   --       det = 1.0f / det;
   --    }
   --    b2Vec3 x;
   --    x.x = det * b2Dot(b, b2Cross(ey, ez));
   --    x.y = det * b2Dot(ex, b2Cross(b, ez));
   --    x.z = det * b2Dot(ex, b2Cross(ey, b));
   --    return x;
   --  }

   function solve33 (Self : in b2Mat33;   b : in b2Vec3) return b2Vec3
   is
      det : Real  := b2Dot (Self.ex,
                            b2Cross (Self.ey, Self.ez));
      x   : b2Vec3;
   begin
      if det /= 0.0
      then
         det := 1.0 / det;
      end if;

      x.x := det * b2Dot (b,       b2Cross (Self.ey, Self.ez));
      x.y := det * b2Dot (Self.ex, b2Cross (b,       Self.ez));
      x.z := det * b2Dot (Self.ex, b2Cross (Self.ey, b));

     return x;
   end solve33;



   --  Solve only the upper 2-by-2 matrix equation.
   --
   --  b2Vec2 b2Mat33::Solve22 (const b2Vec2&   b) const
   --  {
   --    float a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
   --    float det = a11 * a22 - a12 * a21;
   --    if (det != 0.0f)
   --    {
   --       det = 1.0f / det;
   --    }
   --    b2Vec2 x;
   --    x.x = det * (a22 * b.x - a12 * b.y);
   --    x.y = det * (a11 * b.y - a21 * b.x);
   --    return x;
   --  }

   function solve22 (Self : in b2Mat33;   b : in b2Vec2) return b2Vec2
   is
      a11 : Real renames Self.ex.x;
      a12 : Real renames Self.ey.x;
      a21 : Real renames Self.ex.y;
      a22 : Real renames Self.ey.y;

      det : Real :=   a11 * a22
                    - a12 * a21;
      x   : b2Vec2;
   begin
      if det /= 0.0
      then
        det := 1.0 / det;
      end if;

      x.x := det * (a22 * b.x - a12 * b.y);
      x.y := det * (a11 * b.y - a21 * b.x);

      return x;
   end solve22;



   --  void b2Mat33::GetInverse22 (b2Mat33*   M) const
   --  {
   --    float a = ex.x, b = ey.x, c = ex.y, d = ey.y;
   --    float det = a * d - b * c;
   --    if (det != 0.0f)
   --    {
   --       det = 1.0f / det;
   --    }
   --
   --    M->ex.x =  det * d;  M->ey.x = -det * b; M->ex.z = 0.0f;
   --    M->ex.y = -det * c;  M->ey.y =  det * a; M->ey.z = 0.0f;
   --    M->ez.x = 0.0f; M->ez.y = 0.0f; M->ez.z = 0.0f;
   --  }

   procedure getInverse22 (Self : in b2Mat33;   M : out b2Mat33)
   is
      a  : Real renames Self.ex.x;
      b  : Real renames Self.ey.x;
      c  : Real renames Self.ex.y;
      d  : Real renames Self.ey.y;

      det: Real :=   a * d
                   - b * c;
   begin
     if det /= 0.0
     then
        det := 1.0 / det;
     end if;

     M.ex.x :=  det * d;   M.ey.x := -det * b;   M.ex.z := 0.0;
     M.ex.y := -det * c;   M.ey.y :=  det * a;   M.ey.z := 0.0;
     M.ez.x := 0.0;        M.ez.y := 0.0;        M.ez.z := 0.0;
   end getInverse22;



   --  void b2Mat33::GetSymInverse33 (b2Mat33*   M) const
   --  {
   --    float det = b2Dot(ex, b2Cross(ey, ez));
   --    if (det != 0.0f)
   --    {
   --       det = 1.0f / det;
   --    }
   --
   --    float a11 = ex.x, a12 = ey.x, a13 = ez.x;
   --    float a22 = ey.y, a23 = ez.y;
   --    float a33 = ez.z;
   --
   --    M->ex.x = det * (a22 * a33 - a23 * a23);
   --    M->ex.y = det * (a13 * a23 - a12 * a33);
   --    M->ex.z = det * (a12 * a23 - a13 * a22);
   --
   --    M->ey.x = M->ex.y;
   --    M->ey.y = det * (a11 * a33 - a13 * a13);
   --    M->ey.z = det * (a13 * a12 - a11 * a23);
   --
   --    M->ez.x = M->ex.z;
   --    M->ez.y = M->ey.z;
   --    M->ez.z = det * (a11 * a22 - a12 * a12);
   --  }

   procedure getSymInverse33 (Self : in b2Mat33;   M : out b2Mat33)
   is
      det : Real := b2Dot (Self.ex,
                           b2Cross (Self.ey, Self.ez));

      a11 : Real renames Self.ex.x;
      a12 : Real renames Self.ey.x;
      a13 : Real renames Self.ez.x;
      a22 : Real renames Self.ey.y;
      a23 : Real renames Self.ez.y;
      a33 : Real renames Self.ez.z;
   begin
      if det /= 0.0
      then
         det := 1.0 / det;
      end if;

     M.ex.x := det * (a22 * a33 - a23 * a23);
     M.ex.y := det * (a13 * a23 - a12 * a33);
     M.ex.z := det * (a12 * a23 - a13 * a22);

     M.ey.x := M.ex.y;
     M.ey.y := det * (a11 * a33 - a13 * a13);
     M.ey.z := det * (a13 * a12 - a11 * a23);

     M.ez.x := M.ex.z;
     M.ez.y := M.ey.z;
     M.ez.z := det * (a11 * a22 - a12 * a12);
   end getSymInverse33;





   ------------
   --  Rotation
   --


   --    explicit b2Rot(float angle)
   --    {
   --       TODO_ERIN optimize
   --       s = sinf(angle);
   --       c = cosf(angle);
   --    }

   function to_b2Rot (Angle : in Real) return b2Rot
   is
      use Functions;
   begin
      return (s => sin (Angle),
              c => cos (Angle));
   end to_b2Rot;



   --    void Set(float angle)
   --    {
   --       TODO_ERIN optimize
   --       s = sinf(angle);
   --       c = cosf(angle);
   --    }

   procedure set (Self : out b2Rot;   Angle : in Real)
   is
      use Functions;
   begin
      Self.s := sin (Angle);
      Self.c := cos (Angle);
   end set;



   --    void SetIdentity()
   --    {
   --       s = 0.0f;
   --       c = 1.0f;
   --    }

   procedure setIdentity (Self : out b2Rot)
   is
   begin
      Self.s := 0.0;
      Self.c := 1.0;
   end setIdentity;



   --    float GetAngle() const
   --    {
   --       return b2Atan2(s, c);
   --    }

   function getAngle (Self : in b2Rot) return Real
   is
   begin
      return b2Atan2 (Self.s, Self.c);
   end getAngle;



   --    b2Vec2 GetXAxis() const
   --    {
   --       return b2Vec2(c, s);
   --    }

   function getXAxis (Self : in b2Rot) return b2Vec2
   is
   begin
      return b2Vec2' (Self.c, Self.s);
   end getXAxis;



   --    b2Vec2 GetYAxis() const
   --    {
   --       return b2Vec2(-s, c);
   --    }

   function getYAxis (Self : in b2Rot) return b2Vec2
   is
   begin
      return b2Vec2' (-Self.s, Self.c);
   end getYAxis;





   ---------------
   --  b2Transform


   --    b2Transform (const b2Vec2& position, const b2Rot& rotation) : p(position), q(rotation) {}

   function to_b2Transform (Position : in b2Vec2;   Rotation : in b2Rot) return b2Transform
   is
   begin
      return (p => Position,
              q => Rotation);
   end to_b2Transform;



   --    void SetIdentity()
   --    {
   --       p.SetZero();
   --       q.SetIdentity();
   --    }

   procedure setIdentity (Self : out b2Transform)
   is
   begin
      setZero     (Self.p);
      setIdentity (Self.q);
   end setIdentity;



   --    void Set(const b2Vec2& position, float angle)
   --    {
   --       p = position;
   --       q.Set(angle);
   --    }

   procedure set (Self : out b2Transform;   Position : in b2Vec2;
                                            Angle    : in Real)
   is
   begin
        Self.p := Position;
        set(Self.q, Angle);
   end set;





   ----------
   -- b2Sweep


   --  https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future
   --
   --  inline void b2Sweep::GetTransform (b2Transform* xf, float beta) const
   --  {
   --    xf->p = (1.0f - beta) * c0 + beta * c;
   --    float angle = (1.0f - beta) * a0 + beta * a;
   --    xf->q.Set(angle);
   --
   --    // Shift to origin
   --    xf->p -= b2Mul(xf->q, localCenter);
   --  }

   procedure getTransform (Self : in b2Sweep;   Transform :    out b2Transform;
                                                Beta      : in     Real)
   is
      Angle : constant Real :=   (1.0 - beta) * Self.a0
                               + beta * Self.a;
   begin
      Transform.p :=   (1.0 - beta) * Self.c0
                     + beta * Self.c;

      set (Transform.q, Angle);

      subtract_from (Transform.p,                                -- Shift to origin
                     b2Mul (Transform.q, Self.localCenter));
   end getTransform;



   --  inline void b2Sweep::Advance(float alpha)
   --  {
   --    b2Assert(alpha0 < 1.0f);
   --    float beta = (alpha - alpha0) / (1.0f - alpha0);
   --    c0 += beta * (c - c0);
   --    a0 += beta * (a - a0);
   --    alpha0 = alpha;
   --  }

   procedure advance (Self : in out b2Sweep;   Alpha : in Real)
   is
      pragma assert (Self.alpha0 < 1.0);

      beta : constant Real :=   (Alpha - Self.alpha0)
                              / (1.0   - Self.alpha0);
   begin
      Self.c0 := Self.c0 + beta * (Self.c - Self.c0);
      Self.a0 := Self.a0 + beta * (Self.a - Self.a0);

      Self.alpha0 := Alpha;
   end advance;



   --  inline void b2Sweep::Normalize()
   --  {
   --    float twoPi = 2.0f * b2_pi;
   --    float d =  twoPi * floorf(a0 / twoPi);
   --    a0 -= d;
   --    a  -= d;
   --  }

   procedure normalize (Self : in out b2Sweep)
   is
      use b2_Common;

      twoPi : constant Real := 2.0   * b2_Pi;
      d     : constant Real := twoPi * Real'Floor (Self.a0 / twoPi);
   begin
      Self.a0 := Self.a0 - d;
      Self.a  := Self.a  - d;
   end normalize;





   --------------
   --  Operations


   --  inline float b2Dot(const b2Vec2& a, const b2Vec2& b)
   --  {
   --    return a.x * b.x + a.y * b.y;
   --  }

   function b2Dot (a, b : in b2Vec2) return Real
   is
   begin
      return   a.x * b.x
             + a.y * b.y;
   end b2Dot;



   --  inline float b2Cross(const b2Vec2& a, const b2Vec2& b)
   --  {
   --    return a.x * b.y - a.y * b.x;
   --  }

   function b2Cross (a, b : in b2Vec2) return Real
   is
   begin
      return   a.x * b.y
             - a.y * b.x;
   end b2Cross;



   --  inline b2Vec2 b2Cross(const b2Vec2& a, float s)
   --  {
   --    return b2Vec2(s * a.y, -s * a.x);
   --  }

   function b2Cross (a : in b2Vec2;   s : in Real) return b2Vec2
   is
   begin
      return b2Vec2' ( s * a.y,
                      -s * a.x);
   end b2Cross;



   --  inline b2Vec2 b2Cross(float s, const b2Vec2& a)
   --  {
   --    return b2Vec2(-s * a.y, s * a.x);
   --  }

   function b2Cross (s : in Real;   a : in b2Vec2) return b2Vec2
   is
   begin
      return b2Vec2' (-s * a.y,
                       s * a.x);
   end b2Cross;



   --  inline b2Vec2 b2Mul(const b2Mat22& A, const b2Vec2& v)
   --  {
   --    return b2Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
   --  }

   function b2Mul (A : in b2Mat22;   v : in b2Vec2) return b2Vec2
   is
   begin
      return b2Vec2' (  A.ex.x * v.x
                      + A.ey.x * v.y,
                        A.ex.y * v.x
                      + A.ey.y * v.y);
   end b2Mul;



   --  inline b2Vec2 b2MulT(const b2Mat22& A, const b2Vec2& v)
   --  {
   --    return b2Vec2(b2Dot(v, A.ex), b2Dot(v, A.ey));
   --  }

   function b2MulT (A : in b2Mat22;   v : in b2Vec2) return b2Vec2
   is
   begin
      return b2Vec2' (b2Dot (v, A.ex),
                      b2Dot (v, A.ey));
   end b2MulT;



   --  inline b2Vec2 operator + (const b2Vec2& a, const b2Vec2& b)
   --  {
   --    return b2Vec2(a.x + b.x, a.y + b.y);
   --  }

   function "+" (a, b : in b2Vec2) return b2Vec2
   is
   begin
      return b2Vec2' (a.x + b.x,
                      a.y + b.y);
   end "+";



   --  inline b2Vec2 operator - (const b2Vec2& a, const b2Vec2& b)
   --  {
   --    return b2Vec2(a.x - b.x, a.y - b.y);
   --  }

   function "-" (a, b : in b2Vec2) return b2Vec2
   is
   begin
      return b2Vec2' (a.x - b.x,
                      a.y - b.y);
   end "-";



   --  inline b2Vec2 operator * (float s, const b2Vec2& a)
   --  {
   --    return b2Vec2(s * a.x, s * a.y);
   --  }

   function "*" (s : in Real;   a : in b2Vec2) return b2Vec2
   is
   begin
      return b2Vec2' (s * a.x,
                      s * a.y);
   end "*";



   --  inline bool operator == (const b2Vec2& a, const b2Vec2& b)
   --  {
   --    return a.x == b.x && a.y == b.y;
   --  }

   overriding
   function "=" (a, b: in b2Vec2) return Boolean
   is
   begin
      return     a.x = b.x
             and a.y = b.y;
   end "=";



   --  inline bool operator != (const b2Vec2& a, const b2Vec2& b)
   --  {
   --    return a.x != b.x || a.y != b.y;
   --  }

   --  Explicit definition of inequality is not allowed in Ada.



   --  inline float b2Distance(const b2Vec2& a, const b2Vec2& b)
   --  {
   --    b2Vec2 c = a - b;
   --    return c.Length();
   --  }

   function b2Distance (a, b: in b2Vec2) return Real
   is
      c : constant b2Vec2 := a - b;
   begin
        return Length (c);
   end b2Distance;



   --  inline float b2DistanceSquared(const b2Vec2& a, const b2Vec2& b)
   --  {
   --    b2Vec2 c = a - b;
   --    return b2Dot(c, c);
   --  }

   function b2DistanceSquared (a, b: in b2Vec2) return Real
   is
      c : constant b2Vec2 := a - b;
   begin
      return b2Dot (c, c);
   end b2DistanceSquared;



   --  inline b2Vec3 operator * (float s, const b2Vec3& a)
   --  {
   --    return b2Vec3(s * a.x, s * a.y, s * a.z);
   --  }

   function "*" (s : in Real;   a : in b2Vec3) return b2Vec3
   is
   begin
      return b2Vec3' (s * a.x,
                      s * a.y,
                      s * a.z);
   end "*";



   --  inline b2Vec3 operator + (const b2Vec3& a, const b2Vec3& b)
   --  {
   --    return b2Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
   --  }

   function "+" (a, b: in b2Vec3) return b2Vec3
   is
   begin
      return b2Vec3' (a.x + b.x,
                      a.y + b.y,
                      a.z + b.z);
   end "+";



   --  inline b2Vec3 operator - (const b2Vec3& a, const b2Vec3& b)
   --  {
   --    return b2Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
   --  }

   function "-" (a, b: in b2Vec3) return b2Vec3
   is
   begin
      return b2Vec3' (a.x - b.x,
                      a.y - b.y,
                      a.z - b.z);
   end "-";



   --  inline float b2Dot(const b2Vec3& a, const b2Vec3& b)
   --  {
   --    return a.x * b.x + a.y * b.y + a.z * b.z;
   --  }

   function b2Dot (a, b: in b2Vec3) return Real
   is
   begin
      return   a.x * b.x
             + a.y * b.y
             + a.z * b.z;
   end b2Dot;



   --  inline b2Vec3 b2Cross(const b2Vec3& a, const b2Vec3& b)
   --  {
   --    return b2Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
   --  }

   function b2Cross (a, b: in b2Vec3) return b2Vec3
   is
   begin
      return b2Vec3' (a.y * b.z - a.z * b.y,
                      a.z * b.x - a.x * b.z,
                      a.x * b.y - a.y * b.x);
   end b2Cross;



   --  inline b2Mat22 operator + (const b2Mat22& A, const b2Mat22& B)
   --  {
   --    return b2Mat22(A.ex + B.ex, A.ey + B.ey);
   --  }

   function "+" (A, B: in b2Mat22) return b2Mat22
   is
   begin
      return b2Mat22' (A.ex + B.ex,
                       A.ey + B.ey);
   end "+";



   --  inline b2Mat22 b2Mul(const b2Mat22& A, const b2Mat22& B)
   --  {
   --    return b2Mat22(b2Mul(A, B.ex), b2Mul(A, B.ey));
   --  }

   function b2Mul (A, B: in b2Mat22) return b2Mat22
   is
   begin
      return b2Mat22' (b2Mul (A, B.ex),
                       b2Mul (A, B.ey));
   end b2Mul;



   --  inline b2Mat22 b2MulT(const b2Mat22& A, const b2Mat22& B)
   --  {
   --    b2Vec2 c1(b2Dot(A.ex, B.ex), b2Dot(A.ey, B.ex));
   --    b2Vec2 c2(b2Dot(A.ex, B.ey), b2Dot(A.ey, B.ey));
   --    return b2Mat22(c1, c2);
   --  }

   function b2MulT (A, B: in b2Mat22) return b2Mat22
   is
      c1 : constant b2Vec2 := (b2Dot (A.ex, B.ex),  b2Dot (A.ey, B.ex));
      c2 : constant b2Vec2 := (b2Dot (A.ex, B.ey),  b2Dot (A.ey, B.ey));
   begin
      return b2Mat22' (c1, c2);
   end b2MulT;



   --  inline b2Vec3 b2Mul(const b2Mat33& A, const b2Vec3& v)
   --  {
   --    return v.x * A.ex + v.y * A.ey + v.z * A.ez;
   --  }

   function b2Mul (A : in b2Mat33;   v : b2Vec3) return b2Vec3
   is
   begin
      return   v.x * A.ex
             + v.y * A.ey
             + v.z * A.ez;
   end b2Mul;



   --  inline b2Vec2 b2Mul22(const b2Mat33& A, const b2Vec2& v)
   --  {
   --    return b2Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
   --  }

   function b2Mul22 (A : in b2Mat33;   v : b2Vec2) return b2Vec2
   is
   begin
      return b2Vec2' (A.ex.x * v.x  +  A.ey.x * v.y,
                      A.ex.y * v.x  +  A.ey.y * v.y);
   end b2Mul22;



   --  inline b2Rot b2Mul(const b2Rot& q, const b2Rot& r)
   --  {
   --    // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
   --    // [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
   --    // s = qs * rc + qc * rs
   --    // c = qc * rc - qs * rs
   --    b2Rot qr;
   --    qr.s = q.s * r.c + q.c * r.s;
   --    qr.c = q.c * r.c - q.s * r.s;
   --    return qr;
   --  }

   function b2Mul (q, r: in b2Rot) return b2Rot
   is
      --    [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
      --    [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
      --    s = qs * rc + qc * rs
      --    c = qc * rc - qs * rs

      qr : b2Rot;
   begin
      qr.s := q.s * r.c  +  q.c * r.s;
      qr.c := q.c * r.c  -  q.s * r.s;

      return qr;
   end b2Mul;



   --  inline b2Rot b2MulT(const b2Rot& q, const b2Rot& r)
   --  {
   --    // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
   --    // [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
   --    // s = qc * rs - qs * rc
   --    // c = qc * rc + qs * rs
   --    b2Rot qr;
   --    qr.s = q.c * r.s - q.s * r.c;
   --    qr.c = q.c * r.c + q.s * r.s;
   --    return qr;
   --  }

   function b2MulT (q, r: in b2Rot) return b2Rot
   is
      --    [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
      --    [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
      --    s = qc * rs - qs * rc
      --    c = qc * rc + qs * rs

      qr : b2Rot;
   begin
      qr.s := q.c * r.s  -  q.s * r.c;
      qr.c := q.c * r.c  +  q.s * r.s;

      return qr;
   end b2MulT;



   --  inline b2Vec2 b2Mul(const b2Rot& q, const b2Vec2& v)
   --  {
   --    return b2Vec2(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y);
   --  }

   function b2Mul (q : in b2Rot;   v : b2Vec2) return b2Vec2
   is
   begin
      return b2Vec2' (q.c * v.x  -  q.s * v.y,
                      q.s * v.x  +  q.c * v.y);
   end b2Mul;



   --  inline b2Vec2 b2MulT(const b2Rot& q, const b2Vec2& v)
   --  {
   --    return b2Vec2(q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y);
   --  }

   function b2MulT (q : in b2Rot;   v : b2Vec2) return b2Vec2
   is
   begin
      return b2Vec2' ( q.c * v.x  +  q.s * v.y,
                      -q.s * v.x  +  q.c * v.y);
   end b2MulT;



   --  inline b2Vec2 b2Mul(const b2Transform& T, const b2Vec2& v)
   --  {
   --    float x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
   --    float y = (T.q.s * v.x + T.q.c * v.y) + T.p.y;
   --
   --    return b2Vec2(x, y);
   --  }

   function b2Mul (T : in b2Transform;   v : b2Vec2) return b2Vec2
   is
      x : constant Real := (T.q.c * v.x  -  T.q.s * v.y)  + T.p.x;
      y : constant Real := (T.q.s * v.x  +  T.q.c * v.y)  + T.p.y;
   begin
        return b2Vec2' (x, y);
   end b2Mul;



   --  inline b2Vec2 b2MulT(const b2Transform& T, const b2Vec2& v)
   --  {
   --    float px = v.x - T.p.x;
   --    float py = v.y - T.p.y;
   --    float x = (T.q.c * px + T.q.s * py);
   --    float y = (-T.q.s * px + T.q.c * py);
   --
   --    return b2Vec2(x, y);
   --  }

   function b2MulT (T : in b2Transform;   v : b2Vec2) return b2Vec2
   is
      px : constant Real := v.x - T.p.x;
      py : constant Real := v.y - T.p.y;

      x  : constant Real := ( T.q.c * px  +  T.q.s * py);
      y  : constant Real := (-T.q.s * px  +  T.q.c * py);
   begin
      return b2Vec2' (x, y);
   end b2MulT;



   --  v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
   --     = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
   --
   --  inline b2Transform b2Mul(const b2Transform& A, const b2Transform& B)
   --  {
   --    b2Transform C;
   --    C.q = b2Mul(A.q, B.q);
   --    C.p = b2Mul(A.q, B.p) + A.p;
   --    return C;
   --  }

   function b2Mul (A, B : in b2Transform) return b2Transform
   is
      C : b2Transform;
   begin
      C.q := b2Mul (A.q, B.q);
      C.p := b2Mul (A.q, B.p)  +  A.p;

      return C;
   end b2Mul;



   --  v2 = A.q' * (B.q * v1 + B.p - A.p)
   --     = A.q' * B.q * v1 + A.q' * (B.p - A.p)
   --
   --  inline b2Transform b2MulT(const b2Transform& A, const b2Transform& B)
   --  {
   --    b2Transform C;
   --    C.q = b2MulT(A.q, B.q);
   --    C.p = b2MulT(A.q, B.p - A.p);
   --    return C;
   --  }

   function b2MulT (A, B : in b2Transform) return b2Transform
   is
      C : b2Transform;
   begin
      C.q := b2MulT (A.q,  B.q);
      C.p := b2MulT (A.q,  B.p - A.p);

      return C;
   end b2MulT;



   --  template <typename T>
   --  inline T b2Abs(T a)
   --  {
   --    return a > T(0) ? a : -a;
   --  }

   -- 'abs' is catered for in Ada.



   --  inline b2Vec2 b2Abs(const b2Vec2& a)
   --  {
   --    return b2Vec2(b2Abs(a.x), b2Abs(a.y));
   --  }

   function b2Abs (a : in b2Vec2) return b2Vec2
   is
   begin
      return b2Vec2' (abs (a.x),
                      abs (a.y));
   end b2Abs;



   --  inline b2Mat22 b2Abs(const b2Mat22& A)
   --  {
   --    return b2Mat22(b2Abs(A.ex), b2Abs(A.ey));
   --  }

   function b2Abs (A : in b2Mat22) return b2Mat22
   is
   begin
      return b2Mat22' (b2Abs (A.ex),
                       b2Abs (A.ey));
   end b2Abs;



   --  template <typename T>
   --  inline T b2Min(T a, T b)
   --  {
   --    return a < b ? a : b;
   --  }

   -- 'min' is catered for in Ada.



   --  inline b2Vec2 b2Min(const b2Vec2& a, const b2Vec2& b)
   --  {
   --    return b2Vec2(b2Min(a.x, b.x), b2Min(a.y, b.y));
   --  }

   function b2Min (a, b : in b2Vec2) return b2Vec2
   is
   begin
      return b2Vec2' (Real'Min (a.x, b.x),
                      Real'Min (a.y, b.y));
   end b2Min;



   --  template <typename T>
   --  inline T b2Max(T a, T b)
   --  {
   --    return a > b ? a : b;
   --  }

   -- 'max' is catered for in Ada.



   --  inline b2Vec2 b2Max(const b2Vec2& a, const b2Vec2& b)
   --  {
   --    return b2Vec2(b2Max(a.x, b.x), b2Max(a.y, b.y));
   --  }

   function b2Max (a, b : in b2Vec2) return b2Vec2
   is
   begin
      return b2Vec2' (Real'Max (a.x, b.x),
                      Real'Max (a.y, b.y));
   end b2Max;




   --  template <typename T>
   --  inline T b2Clamp(T a, T low, T high)
   --  {
   --    return b2Max(low, b2Min(a, high));
   --  }

   function b2Clamp (a, low, high : in Real) return Real
   is
   begin
      return Real'Max (low,
                       Real'Min (a, high));
   end b2Clamp;




   --  inline b2Vec2 b2Clamp(const b2Vec2& a, const b2Vec2& low, const b2Vec2& high)
   --  {
   --    return b2Max(low, b2Min(a, high));
   --  }

   function b2Clamp (a, low, high : in b2Vec2) return b2Vec2
   is
   begin
      return b2Max (low,
                    b2Min (a, high));
   end b2Clamp;




   --  template<typename T> inline void b2Swap(T& a, T& b)
   --  {
   --    T tmp = a;
   --    a = b;
   --    b = tmp;
   --  }

   procedure b2Swap (a, b : in out Real)
   is
      tmp : constant Real := a;
   begin
      a := b;
      b := tmp;
   end b2Swap;




   --  "Next Largest Power of 2
   --  Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
   --  that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
   --  the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
   --  largest power of 2. For a 32-bit value:"
   --
   --  inline uint32 b2NextPowerOfTwo(uint32 x)
   --  {
   --    x |= (x >> 1);
   --    x |= (x >> 2);
   --    x |= (x >> 4);
   --    x |= (x >> 8);
   --    x |= (x >> 16);
   --    return x + 1;
   --  }

   function b2NextPowerOfTwo (x : in uint32) return uint32
   is
      a : uint32 := x;
   begin
      a := a or shift_Right (x,  1);
      a := a or shift_Right (x,  2);
      a := a or shift_Right (x,  4);
      a := a or shift_Right (x,  8);
      a := a or shift_Right (x, 16);

      return a + 1;
   end b2NextPowerOfTwo;




   --  inline bool b2IsPowerOfTwo(uint32 x)
   --  {
   --    bool result = x > 0 && (x & (x - 1)) == 0;
   --    return result;
   --  }

   function b2IsPowerOfTwo (x : in uint32) return Boolean
   is
      Result : constant Boolean :=      x > 0
                                   and (x and (x - 1)) = 0;
   begin
      return Result;
   end b2IsPowerOfTwo;


end b2_Math;
