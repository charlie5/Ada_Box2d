with
     b2_Settings,
     ada.Text_IO;


package body b2_Settings
is

   --  b2Version b2_version = {2, 4, 0};     (moved to b2_Common).



   --  // Memory allocators. Modify these to use your own allocator.
   --  void* b2Alloc_Default(int32 size)
   --  {
   --    return malloc(size);
   --  }
   --
   --  void b2Free_Default(void* mem)
   --  {
   --    free(mem);
   --  }
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
