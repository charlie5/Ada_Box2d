with
     ada.Text_IO;


package body b2_Common
is
   use ada.Text_IO;

   --------
   -- Dumps
   --

   --  FILE*   b2_dumpFile = nullptr;
   --
   b2_dumpFile : ada.Text_IO.File_type;



   --  void b2OpenDump(const char* fileName)
   --  {
   --    b2Assert(b2_dumpFile == nullptr);
   --    b2_dumpFile = fopen(fileName, "w");
   --  }
   --

   procedure b2OpenDump (fileName : in String)
   is
      pragma assert (not is_open (b2_dumpFile));
   begin
      create (b2_dumpFile, out_File, fileName);
   end b2OpenDump;




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
   --

   procedure b2Dump (Message : in String)
   is
   begin
      if not is_open (b2_dumpFile)
      then
         return;
      end if;

      put_Line (b2_dumpFile, Message);
   end b2Dump;




   --  void b2CloseDump()
   --  {
   --    fclose(b2_dumpFile);
   --    b2_dumpFile = nullptr;
   --  }
   --

   procedure b2CloseDump
   is
   begin
      close (b2_dumpFile);
   end b2CloseDump;



end b2_Common;
