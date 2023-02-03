--  #include <math.h>

with
     b2_Settings,
     b2_Types,
     ada.Numerics.generic_elementary_Functions;


package b2_Math
is
   use b2_Types;


   subtype Real  is b2_Settings.Real;
   type    Reals is array (Natural range <>) of Real;


   package Functions is new ada.Numerics.generic_elementary_Functions (Real);



   --  This function is used to ensure that a floating point number is not a NaN or infinity.
   --
   function b2IsValid (x : in Real) return Boolean
     with Inline;



   --  #define b2Sqrt(x)      sqrtf  (x)
   --  #define b2Atan2(y, x)  atan2f (y, x)

   function b2SqRt  (x    : in Real) return Real renames Functions.SqRt;
   function b2Atan2 (y, x : in Real) return Real renames Functions.arcTan;





   ---------
   -- b2Vec2
   --

   --  A 2D column vector.
   --

   --  struct b2Vec2
   --  {
   --    float x, y;
   --  };

   type b2Vec2 is
      record
         x, y : aliased Real;     -- Don't give these initial default vaues for performance.
      end record;

   type b2Vec2s is array (Natural range <>) of b2Vec2;


   --    Construct using coordinates.
   --
   function to_b2Vec2 (x, y : in Real) return b2Vec2;


   --    Set this vector to all zeros.
   --
   procedure setZero (Self : out b2Vec2);


   --    Set this vector to some specified coordinates.
   --
   procedure set (Self : out b2Vec2;   x, y : in Real);


   --    Negate this vector.
   --
   function "-" (Self : in b2Vec2) return b2Vec2;


   -- Read from an indexed element.
   --
   function Element (Self : in b2Vec2;   Index : int32) return Real;


   --    Write to an indexed element.
   --
   function Element (Self : access b2Vec2;   Index : int32) return access Real;


   --    Add a vector to this vector.
   --
   procedure add_to (Self : in out b2Vec2;   Vector : in b2Vec2);


   --    Subtract a vector from this vector.
   --
   procedure subtract_from (Self : in out b2Vec2;   Vector : in b2Vec2);


   --    Multiply this vector by a scalar.
   --
   procedure multiply (Self : in out b2Vec2;   By : in Real);


   --    Get the length of this vector (the norm).
   --
   function Length (Self : in b2Vec2) return Real;


   --    Get the length squared. For performance, use this instead of
   --    b2Vec2.Length (if possible).
   --
   function LengthSquared (Self : in b2Vec2) return Real;


   --    Convert this vector into a unit vector. Returns the length.
   --
   function Normalize (Self : in out b2Vec2) return Real;


   --    Does this vector contain finite coordinates?
   --
   function isValid (Self : in b2Vec2) return Boolean;


   --    Get the skew vector such that   dot (skew_vec, other) = cross (vec, other)
   --
   function Skew (Self : in b2Vec2) return b2Vec2;





   ---------
   -- b2Vec3
   --

   --  A 2D column vector with 3 elements.
   --

   --  struct b2Vec3
   --  {
   --    float x, y, z;
   --  }

   type b2Vec3 is
      record
         x, y, z : Real;     -- Don't give these initial default vaues for performance.
      end record;


   --    Construct using coordinates.
   --
   function to_b2Vec3 (x, y, z : in Real) return b2Vec3;


   --    Set this vector to all zeros.
   --
   procedure setZero (Self : out b2Vec3);


   --    Set this vector to some specified coordinates.
   --
   procedure set (Self : out b2Vec3;   x, y, z : in Real);


   --    Negate this vector.
   --
   function "-" (Self : in b2Vec3) return b2Vec3;


   --    Add a vector to this vector.
   --
   procedure add_to (Self : in out b2Vec3;   Vector : in b2Vec3);


   --    Subtract a vector from this vector.
   --
   procedure subtract_from (Self : in out b2Vec3;   Vector : in b2Vec3);


   --    Multiply this vector by a scalar.
   --
   procedure multiply (Self : in out b2Vec3;   By : in Real);




   ----------
   -- b2Mat22
   --

   --  A 2-by-2 matrix. Stored in column-major order.
   --

   --  struct b2Mat22
   --  {
   --    b2Vec2 ex, ey;
   --  };

   type b2Mat22 is
      record
         ex, ey : b2Vec2;     -- Don't give these initial default vaues for performance.
      end record;


   --    Construct this matrix using columns.
   --
   function to_b2Mat22 (c1, c2 : in b2Vec2) return b2Mat22;


   --    Construct this matrix using scalars.
   --
   function to_b2Mat22 (a11, a12, a21, a22 : in Real) return b2Mat22;


   --    Initialize this matrix using columns.
   --
   procedure set (Self : out b2Mat22;   c1, c2 : in b2Vec2);


   --    Set this to the identity matrix.
   --
   procedure setIdentity (Self : out b2Mat22);


   --    Set this matrix to all zeros.
   --
   procedure setZero (Self : out b2Mat22);


   function getInverse (Self : in b2Mat22) return b2Mat22;


   --    Solve A * x = b, where b is a column vector. This is more efficient
   --    than computing the inverse in one-shot cases.
   --
   function solve (Self : in b2Mat22;   b : in b2Vec2) return b2Vec2;





   ----------
   -- b2Mat33
   --

   --  A 3-by-3 matrix. Stored in column-major order.
   --

   --  struct b2Mat33
   --  {
   --    b2Vec3 ex, ey, ez;
   --  };

   type b2Mat33 is
      record
         ex, ey, ez : b2Vec3;     -- Don't give these initial default vaues for performance.
      end record;


   --    Construct this matrix using columns.
   --
   function to_b2Mat33 (c1, c2, c3 : in b2Vec3) return b2Mat33;


   --    Set this matrix to all zeros.
   --
   procedure setZero (Self : out b2Mat33);


   --    Solve A * x = b, where b is a column vector. This is more efficient
   --    than computing the inverse in one-shot cases.
   --
   function solve33 (Self : in b2Mat33;   b : in b2Vec3) return b2Vec3;


   --    Solve A * x = b, where b is a column vector. This is more efficient
   --    than computing the inverse in one-shot cases. Solve only the upper
   --    2-by-2 matrix equation.
   --
   function solve22 (Self : in b2Mat33;   b : in b2Vec2) return b2Vec2;


   --    Get the inverse of this matrix as a 2-by-2.
   --    Returns the zero matrix if singular.
   --
   procedure getInverse22 (Self : in b2Mat33;   M : out b2Mat33);


   --    Get the symmetric inverse of this matrix as a 3-by-3.
   --    Returns the zero matrix if singular.
   --
   procedure getSymInverse33 (Self : in b2Mat33;   M : out b2Mat33);





   --------
   -- b2Rot
   --

   --  Rotation
   --

   --  struct b2Rot
   --  {
   --    /// Sine and cosine
   --    float s, c;
   --  };

   type b2Rot is
      record
         s, c : Real;     -- Sine and cosine. Don't give these initial default vaues for performance.
      end record;


   --    Initialize from an angle in radians.
   --
   function to_b2Rot (Angle : in Real) return b2Rot;


   --    Set using an angle in radians.
   --
   procedure set (Self : out b2Rot;   Angle : in Real);


   --    Set to the identity rotation
   --
   procedure setIdentity (Self : out b2Rot);


   --    Get the angle in radians.
   --
   function getAngle (Self : in b2Rot) return Real;


   --    Get the x-axis
   --
   function getXAxis (Self : in b2Rot) return b2Vec2;


   --    Get the y-axis
   --
   function getYAxis (Self : in b2Rot) return b2Vec2;





   --------------
   -- b2Transform
   --

   --  A transform contains translation and rotation. It is used to represent
   --  the position and orientation of rigid frames.

   --  struct b2Transform
   --  {
   --    b2Vec2 p;
   --    b2Rot q;
   --  };

   type b2Transform is
      record
         -- Don't give these initial default vaues for performance.
         p : b2Vec2;     -- Position
         q : b2Rot;      -- Rotation
      end record;


   --    Initialize using a position vector and a rotation.
   --
   function to_b2Transform (Position : in b2Vec2;   Rotation : in b2Rot) return b2Transform;


   --    Set this to the identity transform.
   --
   procedure setIdentity (Self : out b2Transform);


   --    Set this based on the position and angle.
   --
   procedure set (Self : out b2Transform;   Position : in b2Vec2;
                                            Angle    : in Real);





   ----------
   -- b2Sweep
   --

   --  This describes the motion of a body/shape for TOI computation.
   --  Shapes are defined with respect to the body origin, which may
   --  no coincide with the center of mass. However, to support dynamics
   --  we must interpolate the center of mass position.
   --

   --  struct b2Sweep
   --  {
   --    /// Fraction of the current time step in the range [0,1]
   --    /// c0 and a0 are the positions at alpha0.
   --    float  alpha0;
   --
   --    b2Vec2 localCenter;  ///< local center of mass position
   --    b2Vec2 c0, c;     ///< center world positions
   --    float  a0, a;      ///< world angles
   --  };

   type b2Sweep is
      record
         alpha0      : Real;
         localCenter : b2Vec2;     -- Local center of mass position.
         c0, c       : b2Vec2;     -- Center world positions.
         a0, a       : Real;       -- World angles.
      end record;


   --    Get the interpolated transform at a specific time.
   --    @param transform the output transform
   --    @param beta is a factor in [0,1], where 0 indicates alpha0.
   --
   --    https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future
   --
   procedure getTransform (Self : in b2Sweep;   Transform :    out b2Transform;
                                                Beta      : in     Real)
     with Inline;



   --    Advance the sweep forward, yielding a new initial state.
   --    @param alpha the new initial time.
   --
   procedure advance (Self : in out b2Sweep;   Alpha : in Real)
     with Inline;



   --    Normalize an angle in radians to be between -pi and pi.
   --
   procedure normalize (Self : in out b2Sweep)
     with Inline;





   -------------------
   --  Useful constant
   --
   --  extern const b2Vec2 b2Vec2_zero;

   b2Vec2_zero : constant b2Vec2;





   -------------
   -- Operations
   --

   --  Perform the dot product on two vectors.
   --
   function b2Dot (a, b : in b2Vec2) return Real
     with Inline;



   --  Perform the cross product on two vectors. In 2D this produces a scalar.
   --
   function b2Cross (a, b : in b2Vec2) return Real
     with Inline;



   --  Perform the cross product on a vector and a scalar. In 2D this produces a vector.
   --
   function b2Cross (a : in b2Vec2;   s : in Real) return b2Vec2
     with Inline;



   --  Perform the cross product on a scalar and a vector. In 2D this produces a vector.
   --
   function b2Cross (s : in Real;   a : in b2Vec2) return b2Vec2
     with Inline;



   --  Multiply a matrix times a vector. If a rotation matrix is provided,
   --  then this transforms the vector from one frame to another.
   --
   function b2Mul (A : in b2Mat22;   v : in b2Vec2) return b2Vec2
     with Inline;



   --  Multiply a matrix transpose times a vector. If a rotation matrix is provided,
   --  then this transforms the vector from one frame to another (inverse transform).
   --
   function b2MulT (A : in b2Mat22;   v : in b2Vec2) return b2Vec2
     with Inline;



   --  Add two vectors component-wise.
   --
   function "+" (a, b : in b2Vec2) return b2Vec2
     with Inline;



   --  Subtract two vectors component-wise.
   --
   function "-" (a, b : in b2Vec2) return b2Vec2
     with Inline;



   function "*" (s : in Real;   a : in b2Vec2) return b2Vec2
     with Inline;



   overriding
   function "=" (a, b: in b2Vec2) return Boolean
     with Inline;



   --  inline bool operator != (const b2Vec2& a, const b2Vec2& b)
   --
   --  Explicit definition of inequality is not allowed in Ada.



   function b2Distance (a, b: in b2Vec2) return Real
     with Inline;



   function b2DistanceSquared (a, b: in b2Vec2) return Real
     with Inline;



   function "*" (s : in Real;   a : in b2Vec3) return b2Vec3
     with Inline;



   --  Add two vectors component-wise.
   --
   function "+" (a, b: in b2Vec3) return b2Vec3
     with Inline;



   --  Subtract two vectors component-wise.
   --
   function "-" (a, b: in b2Vec3) return b2Vec3
     with Inline;



   --  Perform the dot product on two vectors.
   --
   function b2Dot (a, b: in b2Vec3) return Real
     with Inline;



   --  Perform the cross product on two vectors.
   --
   function b2Cross (a, b: in b2Vec3) return b2Vec3
     with Inline;



   function "+" (A, B: in b2Mat22) return b2Mat22
     with Inline;



   --  A * B
   --
   function b2Mul (A, B: in b2Mat22) return b2Mat22
     with Inline;



   --  A^T * B
   --
   function b2MulT (A, B: in b2Mat22) return b2Mat22
     with Inline;



   --  Multiply a matrix times a vector.
   --
   function b2Mul (A : in b2Mat33;   v : b2Vec3) return b2Vec3
     with Inline;



   --   Multiply a matrix times a vector.
   --
   function b2Mul22 (A : in b2Mat33;   v : b2Vec2) return b2Vec2
     with Inline;



   --  Multiply two rotations: q * r
   --
   function b2Mul (q, r: in b2Rot) return b2Rot
     with Inline;



   --  Transpose multiply two rotations: qT * r
   --
   function b2MulT (q, r: in b2Rot) return b2Rot
     with Inline;



   --  Rotate a vector
   --
   function b2Mul (q : in b2Rot;   v : b2Vec2) return b2Vec2
     with Inline;



   --  Inverse rotate a vector
   --
   function b2MulT (q : in b2Rot;   v : b2Vec2) return b2Vec2
     with Inline;



   function b2Mul (T : in b2Transform;   v : b2Vec2) return b2Vec2
     with Inline;



   function b2MulT (T : in b2Transform;   v : b2Vec2) return b2Vec2
     with Inline;



   --  v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
   --     = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
   --
   function b2Mul (A, B : in b2Transform) return b2Transform
     with Inline;



   --  v2 = A.q' * (B.q * v1 + B.p - A.p)
   --     = A.q' * B.q * v1 + A.q' * (B.p - A.p)
   --
   function b2MulT (A, B : in b2Transform) return b2Transform
     with Inline;



   --  template <typename T>
   --  inline T b2Abs(T a)
   --  {
   --    return a > T(0) ? a : -a;
   --  }

   -- 'abs' is catered for in Ada.



   function b2Abs (a : in b2Vec2)  return b2Vec2;
   function b2Abs (A : in b2Mat22) return b2Mat22;



   --  template <typename T>
   --  inline T b2Min(T a, T b)
   --  {
   --    return a < b ? a : b;
   --  }

   -- 'min' is catered for in Ada.


   function b2Min (a, b : in b2Vec2) return b2Vec2;



   --  template <typename T>
   --  inline T b2Max(T a, T b)
   --  {
   --    return a > b ? a : b;
   --  }

   -- 'max' is catered for in Ada.


   function b2Max (a, b : in b2Vec2) return b2Vec2;



   --  template <typename T>
   --  inline T b2Clamp(T a, T low, T high)
   --  {
   --    return b2Max(low, b2Min(a, high));
   --  }

   function b2Clamp (a, low, high : in Real)   return Real;
   function b2Clamp (a, low, high : in b2Vec2) return b2Vec2;



   --  template<typename T> inline void b2Swap(T& a, T& b)
   --  {
   --    T tmp = a;
   --    a = b;
   --    b = tmp;
   --  }

   procedure b2Swap (a, b : in out Real);



   --  Next Largest Power of 2.
   --
   function b2NextPowerOfTwo (x : in uint32) return uint32;
   function b2IsPowerOfTwo   (x : in uint32) return Boolean;



private

   --  const b2Vec2   b2Vec2_zero (0.0f, 0.0f);
   --
   b2Vec2_zero : constant b2Vec2 := (0.0, 0.0);


end b2_Math;
