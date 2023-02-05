with
     b2_Settings,
     ada.Text_IO,
     ada.unchecked_Deallocation;


package body b2_Settings
is

   --  b2Version b2_version = {2, 4, 0};     (moved to b2_Common).




   --------------------
   -- Memory Allocation
   --

   -- Default allocation functions.
   --


   -- inline void* b2Alloc (int32 size)
   -- {
   --   return b2Alloc_Default(size);
   -- }

   function b2alloc (Size : in int32) return void_ptr
   is
   begin
      return b2alloc_default (size);
   end b2Alloc;



   -- inline void b2Free (void* mem)
   -- {
   --   b2Free_Default (mem);
   -- }

   procedure b2free (Mem : in out void_ptr)
   is
   begin
      b2free_default (Mem);
   end b2free;



   --  Memory allocators. Modify these to use your own allocator.
   --

   --  void* b2Alloc_Default (int32 size)
   --  {
   --    return malloc (size);
   --  }

   function b2alloc_default (Size : in int32) return void_ptr
   is
      use type int32;

      type uint8s is array (0 .. Size - 1) of aliased uint8;

      Block : access uint8s := new uint8s;
   begin
      return Block (Block'First)'Access;
   end b2alloc_default;



   --  void b2Free_Default (void* mem)
   --  {
   --    free(mem);
   --  }

   procedure b2free_default (Mem : in out void_ptr)
   is
      procedure free is new ada.unchecked_Deallocation (uint8, void_ptr);
   begin
      free (Mem);
   end b2free_default;





   ----------------------------
   -- Default logging function.
   --
   --  // You can modify this to use your logging facility.
   --  void b2Log_Default(const char* string, va_list args)
   --  {
   --    vprintf(string, args);
   --  }



   procedure b2Log_Default (Message : in String)
   is
      use ada.Text_IO;
   begin
      put_Line (Message);
   end b2Log_Default;



   -- inline void b2Log (const char*   string, ...)
   -- {
   --   va_list   args;
   --
   --   va_start (args, string);
   --   b2Log_Default (string, args);
   --   va_end (args);
   -- }

   procedure b2Log (Message : in String)
   is
   begin
      b2Log_Default (Message);
   end b2Log;



   --  FILE*   b2_dumpFile = nullptr;

   --  void b2OpenDump(const char* fileName)
   --  {
   --    b2Assert(b2_dumpFile == nullptr);
   --    b2_dumpFile = fopen(fileName, "w");
   --  }


   --  void b2Dump(const char* string, ...)
   --  {
   --    if (b2_dumpFile == nullptr)
   --    {
   --       return;
   --    }
   --
   --    va_list args;
   --    va_start(args, string);
   --    vfprintf(b2_dumpFile, string, args);
   --    va_end(args);
   --  }


   --  void b2CloseDump()
   --  {
   --    fclose(b2_dumpFile);
   --    b2_dumpFile = nullptr;
   --  }

end b2_Settings;
