with
     Interfaces;


package b2_Types
is
   --  typedef signed char  int8;
   --  typedef signed short int16;
   --  typedef signed int   int32;

   subtype int8  is interfaces.Integer_8;
   subtype int16 is interfaces.Integer_16;
   subtype int32 is interfaces.Integer_32;


   --  typedef unsigned char  uint8;
   --  typedef unsigned short uint16;
   --  typedef unsigned int   uint32;

   subtype uint8  is interfaces.Unsigned_8;
   subtype uint16 is interfaces.Unsigned_16;
   subtype uint32 is interfaces.Unsigned_32;

end b2_Types;
