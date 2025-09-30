procedure BIOS_poll_receive;
function BIOS_carrier_present:  boolean;
function BIOS_receive_ready: boolean;
function BIOS_receive_data:  char;
procedure BIOS_transmit_data(s:    longstring);
procedure BIOS_init_com(chan: integer);
procedure BIOS_uninit_com;
procedure BIOS_flush_com;

(*
 * Copyright 1987, 1989 Samuel H. Smith;  All rights reserved
 *
 * This is a component of the ProDoor System.
 * Do not distribute modified versions without my permission.
 * Do not remove or alter this notice or any other copyright notice.
 * If you use this in your own program you must distribute source code.
 * Do not use any of this in a commercial product.
 *
 *)

const
   com_chan:      integer = 0;
   local:         boolean = true;  {local mode, no com port}
   bios_comm:     boolean = true;  {use bios for com port i/o}
   bios_echo:     boolean = true;  {echo com port to screen in bios mode?}


function carrier_present:  boolean;

function receive_ready: boolean;

function receive_data:  char;

procedure transmit_data(s:    longstring);

procedure init_com;

procedure flush_com;

procedure lower_dtr;

procedure raise_dtr;

procedure uninit_com;


(*
 * Copyright 1987, 1989 Samuel H. Smith;  All rights reserved
 *
 * This is a component of the ProDoor System.
 * Do not distribute modified versions without my permission.
 * Do not remove or alter this notice or any other copyright notice.
 * If you use this in your own program you must distribute source code.
 * Do not use any of this in a commercial product.
 *
 *)

const
   extcount:  byte = 90;        {max number of extended conferences (up to 215)}
var
   extsize:   word;             {actual extuser record size}

procedure determine_extsize(fd: dos_handle);

procedure read_extrec(fd:  dos_handle);
procedure load_extrec;

procedure write_extrec(fd:  dos_handle);
procedure save_extrec;


(*
 * Copyright 1987, 1989 Samuel H. Smith;  All rights reserved
 *
 * This is a component of the ProDoor System.
 * Do not distribute modified versions without my permission.
 * Do not remove or alter this notice or any other copyright notice.
 * If you use this in your own program you must distribute source code.
 * Do not use any of this in a commercial product.
 *
 *)

const
   carrier_lost = #$E3;              (* code returned with carrier is lost *)

   com_current_chan: integer = 0;    (* current communication channel *)

   port_base:    integer = -1;  (* base port number for 8250 chip *)
                                (* value = -1 until init is finished *)

   port_irq:     integer = -1;  (* port irq number *)

   old_vector:   pointer = nil; (* pointer to original com interrupt handler *)
   
   XOFF_char:    char = ^S;     (* XOFF character code *)

   disable_cts_check: boolean = false; {false if RTS handshake is needed}
   even_parity:   boolean = false; {strip parity?}



var
   port_intr:    integer;       (* interrupt number for 8250 chip *)
   intr_mask:    integer;       (* interrupt controller initialization code *)

   prev_LCR:     integer;       (* previous LCR contents *)
   prev_IER:     integer;       (* previous IER contents *)
   prev_MCR:     integer;       (* previous MCR contents *)
   prev_ICTL:    integer;       (* previous ICTL contents *)

   xmit_active:  boolean;       (* is the transmitter active now?
                                   (is a THRE interrupt expected?) *)

   XOFF_active:  boolean;       (* has XOFF suspended transmit? *)

   rxque:        queue_rec;     (* receive data queue *)
   txque:        queue_rec;     (* transmit data queue *)

   reg:          registers;     (* register package *)

   bios_bastab:  array[0..3] of word absolute $40:0;
                                (* bios table of com port bases for each
                                   port com1..com4 *)


(*
 * Uart register definitions
 *
 *)

const
   ICTL = $21;                  (* system interrupt controller i/o port *)

   RBR = 0;  (* receive buffer register *)
   THR = 0;  (* transmit holding register *)

   DLM = 1;  (* divisor latch MSB *)
   IER = 1;  (* interrupt enable register *)
      IER_DAV     = $01;       (* data available interrupt *)
      IER_THRE    = $02;       (* THR empty interrupt *)
      IER_LSRC    = $04;       (* line status change interrupt *)
      IER_MSR     = $08;       (* modem status interrupt *)


   IIR = 2;  (* interrupt identification register *)
      IIR_PENDING = $01;       (* low when interrupt pending *)

      IIR_MASK    = $06;       (* mask for interrupt identification *)
        IIR_MSR     = $00;       (* modem status change interrupt *)
        IIR_THRE    = $02;       (* transmit holding reg empty interrupt *)
        IIR_DAV     = $04;       (* data available interrupt *)
        IIR_LSR     = $06;       (* line status change interrupt *)


   LCR = 3;  (* line control register *)
      LCR_5BITS   = $00;       (* 5 data bits *)
      LCR_7BITS   = $02;       (* 7 data bits *)
      LCR_8BITS   = $03;       (* 8 data bits *)

      LCR_1STOP   = $00;       (* 1 stop bit *)
      LCR_2STOP   = $04;       (* 2 stop bits *)

      LCR_NPARITY = $00;       (* no parity *)
      LCR_EPARITY = $38;       (* even parity *)

      LCR_NOBREAK = $00;       (* break disabled *)
      LCR_BREAK   = $40;       (* break enabled *)

     {LCR_NORMAL  = $00;}      (* normal *)
      LCR_ABDL    = $80;       (* address baud divisor latch *)


   MCR = 4;  (* modem control register *)
      MCR_DTR     = $01;       (* active DTR *)
      MCR_RTS     = $02;       (* active RTS *)
      MCR_OUT1    = $04;       (* enable OUT1 *)
      MCR_OUT2    = $08;       (* enable OUT2 -- COM INTERRUPT ENABLE *)
      MCR_LOOP    = $10;       (* loopback mode *)


   LSR = 5;  (* line status register *)
     LSR_DAV      = $01;       (* data available *)
     LSR_OERR     = $02;       (* overrun error *)
     LSR_PERR     = $04;       (* parity error *)
     LSR_FERR     = $08;       (* framing error *)
     LSR_BREAK    = $10;       (* break received *)
     LSR_THRE     = $20;       (* THR empty *)
     LSR_TSRE     = $40;       (* transmit shift register empty *)

     LOERR_count:       integer = 0;    {overrun error count}
     LPERR_count:       integer = 0;    {parity error count}
     LFERR_count:       integer = 0;    {framing error count}
     LBREAK_count:      integer = 0;    {break received count}


   MSR = 6;  (* modem status register *)
     MSR_DCTS     = $01;       (* delta CTS *)
     MSR_DDSR     = $02;       (* delta DSR *)
     MSR_DRING    = $04;       (* delta ring *)
     MSR_DRLSD    = $08;       (* delta receive line signal detect *)
     MSR_CTS      = $10;       (* clear to send *)
     MSR_DSR      = $20;       (* data set ready *)
     MSR_RING     = $40;       (* ring detect *)
     MSR_RLSD     = $80;       (* receive line signal detect *)


   COM_BASE_TABLE: ARRAY[0..2] OF WORD = ($3F8,$2F8,$3E8);
   COM_IRQ_TABLE:  ARRAY[0..2] OF BYTE = (4, 3, 4);

   IRQ_MASK_TABLE: ARRAY[0..7] OF BYTE = ($01,$02,$04,$08,$10,$20,$40,$80);
   IRQ_VECT_TABLE: ARRAY[0..7] OF BYTE = ($08,$09,$0A,$0B,$0C,$0D,$0E,$0F);


procedure push_flags;
   inline($9C);

procedure pop_flags;
   inline($9D);

procedure disable_int;
   inline($FA);

procedure enable_int;
   inline($FB);

procedure io_delay;
   inline($EB/$00);     {jmp $+2}

procedure INTR_service_transmit;
procedure INTR_poll_transmit;
procedure INTR_service_receive;
procedure INTR_check_interrupts;

procedure cancel_xoff;
procedure control_k;
procedure INTR_lower_dtr;
procedure INTR_raise_dtr;
procedure INTR_select_port(chan: integer);
procedure INTR_init_com(chan: integer);
procedure INTR_uninit_com;
procedure INTR_set_baud_rate(speed: word);

procedure INTR_flush_com;
procedure INTR_transmit_data(s:    longstring);
function  INTR_receive_ready: boolean;
function  INTR_receive_data:  char;
procedure verify_txque_space;


(*
 * Copyright 1987, 1989 Samuel H. Smith;  All rights reserved
 *
 * This is a component of the ProDoor System.
 * Do not distribute modified versions without my permission.
 * Do not remove or alter this notice or any other copyright notice.
 * If you use this in your own program you must distribute source code.
 * Do not use any of this in a commercial product.
 *
 *)

const
   carrier_lost = #$E3;            (* code returned with carrier is lost *)
   com_chan:      integer = 0;
   local:         boolean = true;  {local mode, no com port}
   bios_comm:     boolean = true;  {use bios for com port i/o}
   bios_echo:     boolean = true;  {echo com port to screen in bios mode?}
   disable_cts_check: boolean = true; {false if RTS handshake is needed}
   even_parity:   boolean = false; {strip parity?}
   xoff_char:     char = ^S;

function carrier_present:  boolean;
function receive_ready: boolean;
function receive_data:  char;
procedure transmit_data(s:    longstring);
procedure init_com;
procedure flush_com;
procedure lower_dtr;
procedure raise_dtr;
procedure uninit_com;
procedure disable_int;
procedure enable_int;
procedure cancel_xoff;
procedure control_k;
procedure verify_txque_space;

procedure select_main_board;
procedure abandon_conference;
procedure rearc_scratchfile;
procedure capture_conference(n: integer);
procedure capture_new_mail;
   procedure automatic_logoff;

procedure build_download_list;
procedure open_mail_capture;
procedure buffer_mail_capture;
procedure close_mail_capture;
procedure capture_current_message;
procedure chat_mode;
procedure operator_page;
   procedure sysop_view_log;
 
procedure displn_dir(var line: longstring);

(*
 * Copyright 1987, 1989 Samuel H. Smith;  All rights reserved
 *
 * This is a component of the ProDoor System.
 * Do not distribute modified versions without my permission.
 * Do not remove or alter this notice or any other copyright notice.
 * If you use this in your own program you must distribute source code.
 * Do not use any of this in a commercial product.
 *
 *)

var
   current_line: string;
   prev_prompt:  string;
        
procedure pdisp (msg:  string240);
procedure pdispln(msg:  string240);
procedure disp (msg:  string240);
procedure dispc( c: char );
procedure displn(msg:  string240);
procedure newline;
procedure spaces(n: byte);
procedure space;
procedure beep;
procedure erase_prompt (len: integer);
procedure repeat_prompt;

procedure get_cmdline_raw(prelength: integer);

procedure no_hotkeys;
procedure get_cmdline;  {get cmdline without hotkeys}
procedure get_hcmdline; {get cmdline with hotkeys}

procedure prompt_def(prompt: string80; default: string80);
procedure get_def(prompt: string80; default: string80);
procedure get_defn(prompt: string80; default: string80);
procedure get_defnh(prompt: string80; default: string80);
procedure get_defen(prompt: string80);
procedure get_defyn(prompt: string80; default: boolean);
procedure get_defbl(prompt: string80);
procedure get_int(prompt: string80; var n: byte);

function key_ready: boolean;
function get_key: char;
function time_key(ms: integer): char;

procedure drop_carrier;
procedure force_offhook;

procedure check_carrier_loss;

procedure line_input(var line:  string;
                     maxlen:    integer;
                     echo:      boolean;
                     autocr:    boolean);

procedure input(var line:  string;
                maxlen:    integer);

procedure force_new_prompt;

procedure get_chars(prompt: string;
                    var dest;
                    size: integer;
                    echo: boolean);

procedure load_config_file;
procedure create_door;
procedure conference_registration;

(*
 * Copyright 1987, 1989 Samuel H. Smith;  All rights reserved
 *
 * This is a component of the ProDoor System.
 * Do not distribute modified versions without my permission.
 * Do not remove or alter this notice or any other copyright notice.
 * If you use this in your own program you must distribute source code.
 * Do not use any of this in a commercial product.
 *
 *)

(*
 * PCB ProDOOR sysop control panel handlers (3-1-89)
 *
 *)

(*
 * control function codes
 *
 *)
       
const
   fun_idle    = '0';      {set_function function codes}
   fun_batchdl = '1';
   fun_batchul = '2';
   fun_private = '3';
   fun_reply   = '4';
   fun_textview= '5';
   fun_arcview = '5';
   fun_xtract  = '6';
   fun_chat    = '7';
   fun_arcmail = '8';
   fun_lib     = '9';
   fun_rearc   = ':';
   fun_test    = ';';
   fun_confreg = '<';
   fun_unkill  = '=';   {all sysop functions in mail reader}
   fun_sysop   = fun_unkill;
   fun_nodechat= '>';
   fun_door    = '[';
   

procedure adjust_timing;
   (* adjust time-left based on current function crediting *)

procedure set_function(func: char);
   (* select this function for current time and bytecount crediting *)

function check_level(func:     char): boolean;       {function letter}
   (* verify access level for a function, select this function for
      current time and bytecount crediting *)

function verify_level(func:     char): boolean;       {function letter}
   (* verify access level for a function, select this function for
      current time and bytecount crediting, warning if not allowed *)

{procedure control_dump;}
   (* dump the contents of the sysop control table *)


(*
 * Copyright 1987, 1989 Samuel H. Smith;  All rights reserved
 *
 * This is a component of the ProDoor System.
 * Do not distribute modified versions without my permission.
 * Do not remove or alter this notice or any other copyright notice.
 * If you use this in your own program you must distribute source code.
 * Do not use any of this in a commercial product.
 *
 *)

type
   display_formats = (dir_colorize, 
                      display_normal, 
                      remove_semicolons,
                      number_lines);
   
procedure display_file_raw(name: filenames;
                           form: display_formats);
   {display the specified file.  handles special cases for
    graphics files and missing files}

procedure display_file(name: filenames);

procedure display_dirfile(name: filenames);
   {display the specified directory file.  handles special cases for
    graphics files and missing files}

procedure display_resultfile;
   {display the resultfile from archive testing; remove pathnames from
    all lines; detect invalid archives and delete them}

   procedure edit_message;
   procedure edit_header;

(*
 * Copyright 1987, 1989 Samuel H. Smith;  All rights reserved
 *
 * This is a component of the ProDoor System.
 * Do not distribute modified versions without my permission.
 * Do not remove or alter this notice or any other copyright notice.
 * If you use this in your own program you must distribute source code.
 * Do not use any of this in a commercial product.
 *
 *)

(*
 * PCB ProDOOR enter message module for ProMail unit (3-1-89)
 *
 *)

type
   message_entry_modes = (new_message, 
                          reply_message, 
                          reply_originator,
                          comment_message,
                          duplicate_message);
   
procedure save_message(mode: message_entry_modes);
procedure show_margins;
procedure show_line_number(n: integer);
procedure continue_entry;
procedure edit_line;

procedure insert_line(contents: string);
   {open a new line at the cursor}

procedure insert_text;
   {insert a line}

procedure delete_line;
   {delete the line at the cursor}

procedure delete_text;
   {delete a line}

procedure quote_from_original;

procedure display_original;
   {display original message with optional quoting}

procedure count_lines;

procedure enter_message(mode: message_entry_modes);


(*****************
function lock_msgfile: boolean;
procedure save_index;
procedure save_text(mode: message_entry_modes);
procedure unlock_msgfile;
procedure list_message;
procedure enter_header(mode: message_entry_modes);
******************)

procedure error_handler;
procedure install_error_handler;
var
   ExitSave: pointer;   {pointer to next exitproc in the chain}
procedure estimate_transfer_time;

(*
 * Copyright 1987, 1989 Samuel H. Smith;  All rights reserved
 *
 * This is a component of the ProDoor System.
 * Do not distribute modified versions without my permission.
 * Do not remove or alter this notice or any other copyright notice.
 * If you use this in your own program you must distribute source code.
 * Do not use any of this in a commercial product.
 *
 *)

function valid_filename(name: filenames): boolean;
   (* test a specified filename for validity, return false if it is
      invalid (also prints a message to that effect *)

procedure find_file(target: filenames;
                    files:  integer);
   {attempt to locate the specified file based on the
    available file directories.  list matching files in 'select'}

function ok_name(target: filenames): boolean;
   {is the specified filename ok for the selected protocol?
    return the exact name if it is}

procedure set_scratch_type;
procedure select_archive(action: string30);

procedure flag_files;
procedure autoflag_scratch;
function flag_warning(quit: boolean): boolean;
function is_free_file(name: string65): boolean;

(*
 * Copyright 1987, 1989 Samuel H. Smith;  All rights reserved
 *
 * This is a component of the ProDoor System.
 * Do not distribute modified versions without my permission.
 * Do not remove or alter this notice or any other copyright notice.
 * If you use this in your own program you must distribute source code.
 * Do not use any of this in a commercial product.
 *
 *)

function verify_access(fname:      anystring;           {function name}
                       flevel:     integer;             {minimum level}
                       fpassword:  anystring;           {password if any}
                       fmessage:   anystring)           {failure message}
                          : boolean;

function file_allowed(path:       anystring;            {name to verify}
                      secfile:    filenames)            {fsec/upsec name}
                         : boolean;
                         


(*
 * Copyright 1987, 1989 Samuel H. Smith;  All rights reserved
 *
 * This is a component of the ProDoor System.
 * Do not distribute modified versions without my permission.
 * Do not remove or alter this notice or any other copyright notice.
 * If you use this in your own program you must distribute source code.
 * Do not use any of this in a commercial product.
 *
 *)

const
   event_now = false;
   event_possible = true;

procedure fill_chars( var dest; 
                      source:    anystring;
                      size:      integer);
procedure lfill_chars( var dest;
                       source:    anystring;
                       size:      integer);
   {fill_chars with leading space on source}

procedure save_name_list;
procedure load_name_list;

procedure save_pointers(name: filenames);
procedure load_pointers(name: filenames);

procedure prepare_word_wrap(var par: string; var pos: integer; len: integer);

procedure print_text(s: anystring);
procedure make_raw_log_entry(entry: anystring);
procedure make_log_entry (entry: anystring; echo: boolean);

function download_k_allowed: word;

procedure get_infocount(path:       filenames;
                        reclen:     longint;
                        var count:  integer);

procedure get_dirn(n:         integer;
                   var name:  filenames;
                   var descr: anystring);
function dir_count: integer;

function minutes_before_event: integer;
function event_run_needed(possible: boolean): boolean;

function time_used: integer;
function minutes_left: integer;
procedure check_time_left;
procedure display_time(used: boolean);
procedure display_time_left;
procedure adjust_time_allowed(addseconds: longint);


(*
 * Copyright 1987, 1989 Samuel H. Smith;  All rights reserved
 *
 * This is a component of the ProDoor System.
 * Do not distribute modified versions without my permission.
 * Do not remove or alter this notice or any other copyright notice.
 * If you use this in your own program you must distribute source code.
 * Do not use any of this in a commercial product.
 *
 *)

type
   user_ix_rec = record
      urec: word;
      name: char25;
   end;

var
   user_ix:  user_ix_rec;

procedure load_conf(n: integer);

procedure get_user_rec(var user: pcb_user_rec; recn: word);
procedure get_user_info(var user: pcb_user_rec; name: char25);
procedure load_user_rec;
procedure load_extuser;
procedure put_user_rec(var user: pcb_user_rec; recn: word);
procedure save_user_rec;
procedure save_extuser;

procedure load_pcbsys_file;
procedure save_pcbsys_file;
procedure save_offline_pcbsys_file;

procedure build_scratchnames;

procedure load_cnames_file;
procedure load_pcbdat_file;

procedure high_ascii_filter(var c: char);

function get_pcbtext(n: integer): anystring;

procedure display_conference_news;
function lookup_conference_number(name: string10): integer;
procedure join_conference;
procedure request_library;
   procedure log_file_transfer;
 

procedure log_upload_name(logfile:  anystring;
                          source:   anystring;
                          dest:     anystring;
                          size:     longint;
                          descr:    string);

procedure log_download(entry: anystring);

function next_extended_descr(var descr: string): anystring;
procedure display_extended_description(descr: string);


(*
 * Copyright 1987, 1989 Samuel H. Smith;  All rights reserved
 *
 * This is a component of the ProDoor System.
 * Do not distribute modified versions without my permission.
 * Do not remove or alter this notice or any other copyright notice.
 * If you use this in your own program you must distribute source code.
 * Do not use any of this in a commercial product.
 *
 *)

(*
 * PCB ProDOOR mail access module (3-1-89)
 *
 *)

const
   msgmaxlen = 72;      {maximum line length in new message entry}
   maxlines = 102;      {maximum lines per message}

   maxtext = (maxlines+1)*(msgmaxlen+8);
                        {maximum text size per message maxlen*maxlines}

   blksiz = 128;        {size of each message block}
   maxblocks = (maxtext div blksiz)+1;
                        {maximum number of blocks per message}

   no_msg = $FFFF;      {message position indicating no valid message}

   cap_bufsize = 10240; {mail capture buffer size}
   maxinbuf = 128;      {input buffer records *128}
   maxixbuf = 128;      {index buffer records *4}
   
   maxthread = 1000;    {maximum range of thread memory}


(* layout of the message control file records for PCBoard *)

type
   message_rec = record
      case integer of

      {file header record}
         0: (himsg:    single;    {highest message on file}
             lowmsg:   single;    {low msg number in message base}
             msgcnt:   single;    {number of active messages}
             callers:  single;    {number of callers on system}
             lockflag: char6;     {LOCKED if file being updated}
             fill1:    array[1..105] of char);
                                  {reserved for future use}

      {message header record}
         1: (StatusCode:  char;     {protect, unprotect flag '*' or blank}
             Number:      single;   {message number}
             ReferTo:     single;   {reference message number}
             blocks:      byte;     {number of 128 byte text blocks}
             Date:        char8;    {mm-dd-yy}
             Time:        char5;    {hh:mm}
             WhoTo:       char25;
             ReadDate:    single;   {yymmdd numeric date of reply message}
             ReadTime:    char5;    {hh:mm of reply}
             HasReplys:   char;     {'R' is ALL message has replys}
             WhoFrom:     char25;
             Subject:     char25;
             Password:    char12;   {blank=none}
             status:      char;     {dead_msg(226) or live_msg(225)}
             echoflag:    char;     {'E' if msg to be echoed}
             filler:      char6);   {reserved}

      {message text record}
         2: (body:      array[1..128] of char); {body of the message, space fill}
   end;


   blockarray = array[1..maxblocks] of message_rec;
   rawarray   = array[1..maxtext] of char;
   textarray  = array[1..maxlines] of string80;
   threadarray= array[1..maxthread] of boolean;
   cap_bufrec = array[1..cap_bufsize] of byte;
   
const
   dead_msg    = #226;           {message status codes}
   live_msg    = #225;
   endline     = #227;           {end of line character in message files}

var
   cap_buffer:   ^cap_bufrec;
   cap_count:    integer;
   
   selectedfile:      filenames;
   messagebase_file:  filenames;

   mbfd:         buffered_file;
   ixfd:         buffered_file;
   header:       message_rec;
   mheader:      message_rec;

   curmsg:       word;
   basemsg:      word;
   lastmsg:      word;
   memorymsg:    word;
   priormsg:     word;
   newmsgs:      word;
   yourmsgs:     integer;

   msgpos:       word;

   txtblocks:    integer;
   maxpos:       integer;
   block:        ^blockarray;
   raw:          ^rawarray absolute block;

   lines:        ^textarray;
   linecnt:      integer;

   threadseen:   ^threadarray;  {message thread memory}
   threadbase:   longint;

   privatep:     boolean;       {message private?}
   groupp:       boolean;       {message has a group password?}
   readp:        boolean;       {message has been read?}
   tomep:        boolean;       {message is to me?}
   frommep:      boolean;       {message is from me?}
   toallp:       boolean;       {message is to ALL?}
   kill_allowed: boolean;       {user allowed to kill this message}

   WhoTo:        char25;        {to: address after prepare_line}
   Subject:      char25;        {subject after prepare_line}

   search_key:   anystring;
   nextjoin:     string20;      {set to J nn at end of message base}
   direction:    char;          {+ or -}
   
   lastread:     ^single;       {pointer to current lastread counter}

   fromUser:     pcb_user_rec;  {user record of message sender}
   lookup_info:  boolean;       {find city for each user?}
   have_city:    boolean;

   non_stop:     boolean;       {currently in non-stop mode?}


const
   pprevcmd:     string2 = 'R';
   prevcmd:      string2 = 'R';         {previous command letter}
   zip_active:   boolean = false;       {cancel search after first find?}

   substitute:   boolean = true;        {allow @...@ substitutes?}


function sysopfun_allowed: boolean;
function message_allowed: boolean;
function meets_criteria: boolean;

procedure display_header;
procedure display_text;
procedure display_loaded_message;
procedure get_text;
procedure load_message(killed: boolean);
procedure save_message_header;
procedure set_lastread;
procedure set_read_flag;

procedure decode_status;
procedure advance;
procedure get_index(killed: boolean);
procedure check_message(killed: boolean);
function select_conference(conf: integer): boolean;
procedure display_conference_info;
procedure open_conference;
procedure reopen_messagebase;
procedure close_conference;
procedure alloc_mail;
procedure free_mail;

function locate_next_personal {(par: string2)}: boolean;

procedure main_menu;
procedure abort_program(reason: string);
procedure usage (error: anystring);

(*
 * Copyright 1987, 1989 Samuel H. Smith;  All rights reserved
 *
 * This is a component of the ProDoor System.
 * Do not distribute modified versions without my permission.
 * Do not remove or alter this notice or any other copyright notice.
 * If you use this in your own program you must distribute source code.
 * Do not use any of this in a commercial product.
 *
 *)

procedure popup_prompt(prompt:      string80;
                       var answer:  string;
                       maxlen:      integer);
   {prompt for an input and remove the prompt afterwards}

procedure popup_cmdline(prompt:      string80;
                        defalt:      string80);
   {prompt for a command line and remove the prompt afterwards}

procedure force_enter;
procedure force_enter2;
procedure wait_for_enter;

function nomore: boolean;
procedure set_node_status(newcode: char);
procedure node_status_display;
procedure node_chat;
procedure check_chat_request;

const
  node_available        = 'A';
  node_unavailable      = 'U';
  node_in_door          = 'D';
  node_entering_msg     = 'E';
  node_transfer         = 'T';
  node_group_chat       = 'G';
  node_logoff_pending   = 'L';
  node_dropping_2dos    = 'X';
  node_no_caller        = ' ';
  node_chat_request     = 'R';
  node_going_down       = '@';  {not a standard code}

  chat_poll: boolean = true;    {true if polling for chat}

procedure prepare_line(var line: string);
procedure prepare_protocol_table;
procedure remove_variables(var line: string);

const
   delete_variables: boolean = false;

procedure view_profile;
procedure get_protocol(modes: transfer_modes);

(*
 * Copyright 1987, 1989 Samuel H. Smith;  All rights reserved
 *
 * This is a component of the ProDoor System.
 * Do not distribute modified versions without my permission.
 * Do not remove or alter this notice or any other copyright notice.
 * If you use this in your own program you must distribute source code.
 * Do not use any of this in a commercial product.
 *
 *)

(*
 * PCB ProDOOR read message module (3-1-89)
 *
 *)

procedure read_messages;

   procedure sysop_remote;
 
procedure select_names;
procedure display_conference_status(func: char);
   {display status of all conferences; optionally exclude those
    with no new messages in them}

type
   status_formats = (normal_format, help_format, pgup_format, pgdn_format);

procedure update_status_display (format:  status_formats);
   {display current status information on the last 2 lines of the screen}

procedure init_status_display;
   {prepare the screen for status displays}

procedure transfer_status_display;
   {prepare the status display area for execution of a protocol driver}

procedure delete_status_display;
   {completely remove status display from the screen}

procedure shell_to_dos;
   {allow the sysop to drop to DOS for a moment}

procedure toggle(var v: char2);
   {toggle a variable and update status display accordingly}

procedure dispatch_function_key(c: char);
   {sysop function key dispatch}

procedure process_function_keys;
   {read next local function code and dispatch it}


procedure display_protocol_table;
procedure test_archive;

procedure get_upload_description(prompt: string80;
                                 var desc: string);

function expert: boolean;

function scan_nextpar(var cmdline: string): string;
procedure get_nextpar;
procedure unget_par;

procedure not_understood;
procedure not_available;
procedure file_not_found (name: anystring);
procedure file_missing;

procedure keypause;

function pro_command: anystring;
function pro_title: anystring;
function pro_code: char;
function pro_files: integer;
function pro_mode: transfer_modes;

function estimated_time(size: longint): real;

procedure open_capture;
procedure close_capture;

procedure clean_playpen;
procedure clean_scratch;

procedure disp_margin(left,width: integer);

function expdate(yymmdd: string8): string8;     {convert to mm-dd-yy}
function mmddyy_to_yymmdd(mmddyy: string8): string8;
function yymmdd_to_mmddyy(yymmdd: string8): string8;

function todays_date_mmddyy: string8;
function todays_date_yymmdd: string8;

function dtok(d: double): string8;
function stok(s: single): string8;

function posc(c: char; var s: string): integer;

procedure check_command_line(var command: string);
procedure execute(command: string);

procedure disconnect_warning(left: integer);
procedure check_disconnect_warning;

function file_is_new(name: dos_filename): boolean;

(*
 * PCB ProDoor visual message entry/edit module for ProMail unit (3-1-89)
 *
 *)

const
   topscreen = 7;       {first screen line for text entry}
   maxscrlines = 40;    {maximum number of display lines}

   insert_mode: boolean = true;

var
   statline:    integer;        {line for statue messages}
   scrlines:    integer;        {number of screen lines for text entry}
   scrollsiz:   integer;        {number of lines to scroll by}
   topline:     integer;        {message line number at top of screen}
   cline:       integer;        {current message line number}
   ccol:        integer;        {current column number}

   phyline:     array[1..maxscrlines] of string[72];
                                {physical display text}

   pleft:       integer;        {previous value of minutes_left}

procedure visual_edit;
   procedure write_user_info;
 
procedure get_xcmd(xfile: filenames);

procedure log_onoff(what: string30; state: boolean);
procedure expert_toggle;
procedure mode_toggle;
procedure snoop_toggle;
procedure hotkey_toggle;
procedure scan_toggle;

const
   max_conf = 39;               {highest conference number}
   conf_limit = max_conf+1;

   min_econf = conf_limit;      {lowest extended conference number}
   max_econf = 255;             {highest extended conference number}

   max_extcount = max_econf - min_econf;
                                {highest extcount for extuser file}

   {bit values in conf[n].ext_flags}
   ext_scan       = $01;         {are we scanning this conf?}
   ext_dly        = $02;         {download only your own messages}
   ext_qnet       = $04;         {qnet status}
   ext_sysop      = $08;         {sysop status}
   ext_joined     = $10;         {have we joined today?}
   ext_ro         = $20;         {read/only status}
   ext_priv       = $40;         {updloads private??}
   ext_member     = $80;         {are a member of this conf?}


(* layout of extended user information file *)
type
   extuser_fixed_rec = record
      extcount: byte;         {number of extended conferences in first rec}
      spare0:   char4;        {unallocated spares}
      name:     char25;       {user name}
      lockreg:  boolean;      {lock conference registrations?}
      lastdate: char8;        {last date of access mm-dd-yy}
      spare2:   char40;       {unallocated spares}
      spare3:   char40;       {used by qmail}
   end;

   extuser_conf_rec = record
      flags:    byte;         {flag bits}
      lastread: single;       {last message read}
   end;

   extuser_rec = record
      fixed:   extuser_fixed_rec;      {fixed user info}

      conf:    array[0..max_econf]     {extended conference info}
               of extuser_conf_rec;
   end;

(*
 * PCBoard file configuration file declarations (3-1-89)
 *
 *)

const
   min_control = '0';
   max_control = '[';

type
   sysop_control_rec = record
      minlevel:      integer;       (* minimum security level for access *)
      timefact:      real;          (* time charge(+) or credit(-) factor *)
      bytefact:      real;          (* bytecount charge(+) or credit(-) fact *)
   end;

   control_table = array[min_control..max_control] of sysop_control_rec;

const
   min_desc  = 15;           (* minimum upload description length *)
   max_files = 50;           (* maximum number of files per transfer *)
   max_proto = 100;          (* number of protocols in the protocol table *)
   max_dir   = 200;          (* maximum number of download directories *)
      
type
   name_table  = record
      entry: array[1..max_files] of varstring;
      count: integer;
   end;

   dir_table  = record
      entry: array[1..max_dir] of varstring;
      count: integer;
   end;

   transfer_modes = (TX,RX);      (* data transfer modes *)

   protocol_description = record
      code:    char;              (* single letter protocol code *)
      mode:    transfer_modes;    (* the protocol mode TX or RX *)
      files:   integer;           (* the maximum number of filenames *)
      title:   varstring;         (* the menu title/protocol name *)
      command: varstring;         (* the command prefix *)
      peffic:  real;              (* protocol efficiency *)
      efree:   boolean;           (* error free connection required? *)
   end;


   (* layout of PCBPRO.CNF configuration *)
   config_rec = record
      overhead:        real;       (* overhead time to load protocol and
                                      reload logger (seconds) *)

      playpen_dir:     filenames;  (* path of "playpen" directory for uploads *)
      scratch_prefix:  string8;    (* scratch filename prefix *)

      trashcan_list:   filenames;  (* path of held-upload list \pcb\main\trash *)
      trashcan_dir:    filenames;  (* location of trashed-upload subdir *)

      scratch_dir:     filenames;  (* location of scratch.arc *)
      mail_prefix:     string8;    (* ARCM filename prefix *)

      listing_command: filenames;  (* list arc members *)
      typetext_command:filenames;  (* get text member to stdout *)
      xtract_command:  filenames;  (* extract from arc to arc *)
      test_command:    filenames;  (* test archive *)
      rearc_command:   filenames;  (* repack archive *)
      badarc_keymsg:   filenames;  (* message for arc test failure *)

      protocol_table:  array[1..max_proto] of protocol_description;
      protocol_count:  integer;
   end;


const
   max_capcount: integer = 400;  {maximum number of msgs to capture at once}

const
   sysfile              = 'PCBOARD.SYS'; {system file}
   pcboard_dat_file     = 'PCBOARD.DAT'; {setup file}

   doorfile             = '$DOOR.BAT';   {door created for each transfer}
   namefile             = '$DOOR.NAM';   {list of names, used in logging}
   doorlogfile          = '$DOOR.LOG';   {logfile for DSZ or outside batch}
   resultfile           = '$DOOR.RES';   {result message filename}
   newptrfile           = '$PTRS.NEW';   {new message pointers}
   oldptrfile           = '$PTRS.OLD';   {old message pointers}

   signon_file          = 'PROSTRT';      {filenames of message files}
   protocol_help_file   = 'PROHELP';
   main_menu_file       = 'PROMENU';
   mail_menu_file       = 'PROREAD';
   mail_help_file       = 'PROMAIL';
   enter_help_file      = 'PROENTR';
   closing_door_file    = 'PROCLOS';

   extract_help_file    = 'PROEXT';
   rearc_file           = 'PROARC';   {REARC explanation to user}
   test_archive_file    = 'PROTEST';  {TEST explanation to user}
   arcm_help_file       = 'PROARCM';  {ARCM explanation to user}
   visual_help_file     = 'PROVIS';   {visual edit help file}
   freefile_list        = 'PROFREE';  {list of free d/l files}
   usermail_file        = 'PROUSER';  {user has mail/other flags}

   enter_chat_file      = 'PROCHAT';
   chat_request_file    = 'PROREQ';
   dump_user_message    = 'PRODUMP';  {after F8 key is pressed}

   newuser_file         = 'PRONEW';   {display first time}

   library_menu         = 'PROLIB';
   library_table        = 'LIBDEF';
   library_help_file    = 'LIBHELP';

(*
 * PCBoard file interface declarations - SPECIFIC TO PCB 14.0
 *
 * (3-1-89)
 *
 *)


type
   {layout of the PCBOARD.SYS file while doors are open}
   pcb_sys_rec = record
    {1  }display:        char2;          {display on console?  -1 or 0}
    {3  }printer:        char2;          {print log?           -1 or 0}
    {5  }page_bell:      char2;          {bother sysop?        -1 or 0}
    {7  }alarm:          char2;          {caller alarm sound?  -1 or 0}
    {9  }sysop_next:     char;           {force sysop on next? 'N', 'X' or ' '}

    case integer of
    1: (
    {10 }errcheck:       char2;          {error check/correcting modem? -1 or 0}
    {12 }graphics:       char;           {ansi graphics mode?   'Y','N','7'}
    {13 }nodechat:       char;           {node chat status 'U' or 'A'}
    {14 }openbps:        char5;          {BPS rate to open modem port at}
    {19 }connectbps:     char5;          {BPS connect rate or 'Local'}
    {24 }usernum:        integer;        {record number in user file}
    {26 }firstname:      char15;         {caller's first name}
    {41 }password:       char12;         {caller's password}
    {53 }time_on:        integer;        {when the user logged on in MINUTES}
    {55 }prev_used:      integer;        {minutes used in prev calls today, <0}
    {57 }time_logged:    char5;          {hh:mm time the user logged on}
    {62 }time_limit:     integer;        {maximum minutes allowed from PWRD}
    {64 }down_limit:     integer;        {daily download limit/1024 from PWRD}
    {66 }curconf:        byte;           {active conference when door opened}
    {67 }joined:         bitmap;         {areas user has been in}
    {72 }ydone:          bitmap;         {areas user has done 'Y' on}
    {77 }time_added:     integer;        {highest conference added time in mins}
    {79 }time_credit:    integer;        {upload/chat time credit in minutes}
    {81 }slanguage:      char4;          {language used, blank, .FRE etc}
    {85 }name:           char25;         {caller's full name}
    {110}sminsleft:      integer;        {minutes left when door opened}
    {112}snodenum:       byte;           {current node number}
    {113}seventtime:     char5;          {hh:mm event time}
    {118}seventactive:   char2;          {event time active? "-1" or "0 "}
    {120}sslide:         char2;          {slide event? "-1" or " 0"}
    {122}smemmsg:        single;         {memorized message number}
    {126}scomport:       char;           {com port number '0','1','2'}
    {127}fill99:         char2           {filler UNDOCUMENTED}
    {record size: 128}
      );

    2: (
      offline_filler:    array[1..119] of char      {filler, spaces}
      );
   end;

   {layout of the USERS file}
   pcb_user_rec = record
    {1  }name:          char25;
    {26 }city:          char24;
    {50 }passwd:        char12;         {no spaces allowed}
    {62 }busphone:      char13;
    {75 }phone:         char13;
    {88 }date:          char6;          {yymmdd of last call}
    {94 }time:          char5;          {hh:mm  of last call}
    {99 }expert:        char;           {pcboard expert status Y or N}
    {100}protocol:      char;           {X, C, Y, N}
    {101}space1:        char;           {space - reserved}
    {102}filedate:      char6;          {yymmdd of last file directory}
    {108}level:         byte;           {security level}
    {109}total_calls:   integer;        {number of times on system}
    {111}pagelen:       byte;           {page length}
    {112}uploads:       integer;        {number of uploads}
    {114}downloads:     integer;        {number of downloads}
    {116}downbytes:     double;         {daily download bytes so far}
    {124}usercomment:   char30;         {user entered comment field}
    {154}sysopcomment:  char30;         {sysop maintained comment field}
    {184}lastused:      integer;        {minutes used so far today}
    {186}expdate:       char6;          {yymmdd expiration date}
    {192}explevel:      byte;           {expired security level}
    {193}curconf:       byte;           {current conference number}
    {194}conferences:   bitmap;         {area registration 1-39 (5 bytes)}
    {199}expconf:       bitmap;         {expired conference registration}
    {204}scanconf:      bitmap;         {user configured scan conferences}
    {209}downtotal:     double;         {total bytes downloaded, all calls}
    {217}uptotal:       double;         {total bytes uploaded, all calls}
    {225}dead:          char;           {positive delete flag, Y or N}

    {226}lastread:      array[0..39] of single;
                                        {last message pointer, main+39 conf's}

    {386}reserved:      char5;          {reserved for future use}

(*
 * THE FOLLOWING USERS FILE BYTES ARE TAKEN OVER BY PRODOOR
 * FOR STORAGE OF PRODOOR-SPECIFIC DATA FOR A USER.  OTHER DOOR
 * PROGRAMS SHOULD TAKE CARE TO NOT CONFLICT WITH THESE BYTE
 * POSITIONS!
 *)
    {391}extrarec:      word;           {record number for extra user record}

    {393}flags:         byte;           {prodoor user flag bits}

    {394}mailconf:      byte;           {conference user has mail in}
    {395}scratchnum:    byte;           {scratch file number - incremented for
                                         each use of a scratch file}
    {396}dooruse:       byte;           {times in prodoor, up to 255}
    {397}earned_k:      word;           {prodoor; earned kbytes}


    {399}reserve3:      word;           {used by qmail??}
    {total size: 400}
   end;


const
   {bit values in conf_flags}
   conf_scan_blts    = 1;     {qmail include blts?}
   conf_scan_files   = 2;     {qmail scan new files?}

type
(* layout of CNAMES/CONFINFO record *)
   pcbconf_rec = record
      conf_name:              string[10];

      conf_private_ul:        boolean;
      conf_private_mail:      boolean;
      conf_echo_mail:         boolean;
      conf_addsec:            integer;
      conf_addtime:           integer;    {minutes}
      conf_msgblocks:         integer;    {1-32 message blocks; 1024 msgs each}

      conf_msgfile:           string[31];  {conference message base pathname}
      conf_public_uplist:     string[31];  {public uploads listing pathname}
      conf_updir:             string[31];  {conference upload dir}
      conf_private_uplist:    string[31];  {private uploads listing pathname}
      conf_private_updir:     string[31];  {private uploads dir}

      conf_newsfile:          string[31];  {conference news pathname}

      conf_doormenu:          string[31];  {door menu pathname}
      conf_doorinfo:          string[31];  {door info data pathname}

      conf_bltmenu:           string[31];  {bulletin menu pathname}
      conf_bltinfo:           string[31];  {bulletin info pathname}

      conf_minsec:            integer;     {minimum security to join
                                            if conference is non-public}

      conf_dirmenu:           string[31];  {file area menu pathname}
      conf_dirinfo:           string[31];  {file area info pathname}

      conf_autojoin:          boolean;     {auto-rejoin on logon?}

      conf_spare2:            byte;
      conf_dlinfo:            string[31];  {download path info pathname}

      conf_public:            boolean;     {is this conference public?}

      conf_packopt:           string[15];  {propack options}

      conf_flags:             byte;        {Qmail flag bits}

      conf_spare3:            byte;        {spare flag bits}

      conf_msgs_read:         single;      {Number of messages downloaded/read
                                            from this conference}

      conf_msgs_entered:      single;      {Number of messages uploaded to this
                                            conference}

      conf_spare4:            char6;       {unallocated}
   end;

(* layout of pcboard.dat *)
   pcbsetup_rec = record
      sysop_name:             varstring;  {sysop display name}
      use_realname:           boolean;    {use real name for sysop?}
      local_graphics:         boolean;    {graphics on in local mode?}
      security_dir:           varstring;  {security messages location}
      chat_dir:               varstring;  {node chat files location}
      pcbtext_dir:            varstring;  {pcbtext file location}
      userix_dir:             varstring;  {user index location}
      users_path:             varstring;  {user file pathname}
      caller_path:            varstring;  {caller log pathname}
      cnames_path:            varstring;  {cnames file pathname}
      pwrd_path:              varstring;  {pwrd file pathname}
      fsec_path:              varstring;  {fsec file pathname}
      upsec_path:             varstring;  {upsec file pathname}
      tcan_path:              varstring;  {tcan file pathname}
      welcome_path:           varstring;  {welcome file pathname}
      newuser_path:           varstring;  {newuser file pathname}
      closed_path:            varstring;  {closed file pathname}
      warning_path:           varstring;  {30 day expiration warning pathname}
      expired_path:           varstring;  {expired user message pathname}
      usernet_path:           varstring;  {usernet.dat file pathname}
      conference_menu:        varstring;  {conference menu pathname}
      tranlog_path:           varstring;  {down/upload transfer log pathname}
      logoff_path:            varstring;  {logoff message pathname}
      language_path:          varstring;  {multi-lingual data file}
      hayesv_modem:           boolean;    {hayes-v modem?}
      initial_speed:          word;       {modem initial/top open speed}
      lock_speed:             boolean;    {lock modem at initial speed}
      modem_initial:          varstring;  {modem initial command}
      modem_offhook:          varstring;  {modem off-hook command}
      reset_modem:            boolean;    {reset modem during recycle?}
      recycle_offhook:        boolean;    {modem offhook during recycle?}
      allow_300:              boolean;    {allow 300 baud connections?}
      start_300:              string5;    {starting hh:mm for 300 baud}
      stop_300:               string5;    {ending hh:mm for 300 baud}
      disable_blanker:        boolean;    {disable 3 minute screen blanker}
      disable_filter:         boolean;    {disable high-bit filter}
      disable_quick:          boolean;    {disable quick logon/join}
      multi_lingual:          boolean;    {run in multi-lingual mode?}
      only_pwchange:          boolean;    {allow only password changes?}
      closed_board:           boolean;    {run in closed-board mode?}
      disable_graphics:       boolean;    {disable graphics mode?}
      dos_recycle:            boolean;    {exit to dos after each caller}
      subscription_mode:      boolean;    {enable expired user checks}
      allow_esc_codes:        boolean;    {allow escape codes in messages}
      validate_to:            boolean;    {validate "TO:" in messages}
      enforce_time_limit:     boolean;    {enforce daily time limits?}
      new_news:               boolean;    {display only NEW news files?}
      timeout_minutes:        integer;    {keyboard timeout in minutes}
      under_network:          boolean;    {running multi-node?}
      node_number:            varstring;  {node number}
      network_timeout:        integer;    {network timeout in seconds}
      chat_delay:             integer;    {node chat delay in seconds}
      system_name:            varstring;  {name of the bbs system}
      macro_string:           array[1..10] of varstring;
                                          {shift-F1 thru F10 macros}
      public_conferences:     string40;   {public conferences}
      msg_lines:              integer;    {maximum message lines (1-99)}
      ansi_intensity:         varstring;  {default intensity}
      ansi_color:             varstring;  {color for inputs/filedisps}
      event_active:           boolean;    {is event active?}
      event_time:             string8;    {event time hh:mm}
      event_buffer:           integer;    {minutes buffer before event}
      event_noul:             boolean;    {disable uploads before event}
      slide_event:            boolean;    {slide event time if needed}
      disable_freecheck:      boolean;    {disable free space check}
      printer_num:            integer;    {printer port number 1..3}
      min_upload_free:        integer;    {stop uploads if free space less (K)}

      newuser_level:          integer;    {level given to new users}
   end;


(* layout of usernet.dat *)

   usernet_rec = record
      status:  char2;      {chatting-with-node-number or status code}
      nodeno:  char2;      {this-node-number or "R " if chat request sent}
      name:    char25;     {name of caller on this node}
      city:    char24;     {location of caller on this node}
   end;


(* flag bits in user.flags byte *)

const
   flag_hasmail = $01;   {user has mail waiting?}
  {flag_expert  = $02;}  {is user an expert?}
   flag_hotkeys = $04;   {does user want hotkeys?}
   flag_init    = $20;   {set before prodoor runs the first time}
   flag_oldfmt  = $40;   {set by old versions of prodoor, must be clear in new}

{some commonly used message strings}

(********

const
   local_msg       = 'Local';
   upload_msg      = 'Upload ';
   download_msg    = 'Download ';
   ul_dir_has      = ' upload dir has ';
   k_free          = 'k free';
   k_max_per_xfr   = 'k max per transfer';
   no_ul_space     = 'Insufficient disk space for uploading!';
   wild_ok         = 'Wildcards are Okay, ';
   is_assumed      = ' is assumed.';
   enter_up_to     = 'Enter up to ';
   end_list_with   = ' filespecs.  End the list with a blank line.';
   enter_desc_of   = 'Please enter a description of (';
   mins_left_msg   = ' min. left';
   filespec_msg    = 'Filespec';
   checking        = 'Checking ... ';
   enter_to_end    = '  (Enter) alone to end.';
   begin_descrip   = 'Begin description with (/) to make upload ''Private''.';
   longer_descrip  = 'Enter a longer description of the file please!';
   completed_using = ' Completed using ';
   aborted_using   = ' Aborted using ';
   msgs_captured   = ' messages captured.';
   conf_joined     = ' Conference Joined.';
   changes_saved   = 'Changes saved ...';
   please_wait     = 'Processing your request.  Please wait ...';
   ctrlk_aborts    = ') ... (Ctrl-K) aborts';
*******)


const
   dotpak:          string[4]  = '.ZIP';

const
   all25:           char25     = 'ALL                      ';
   sysop25:         char25     = 'SYSOP                    ';

const
   allkeys:         string[62] = '!"#$%&''()*+, -./ :<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`';
   all_stars:       string[62] = '**************************************************************';
   enter_eq:        string[8]  = '(Enter)=';
   enter_eq_none:   string[14] = '(Enter)=none? ';
   enter_eq_no:     string[13] = '(Enter)=no? ';
   enter_eq_yes:    string[13] = '(Enter)=yes? ';

var
   enter_eq_yesno:  array[false..true] of string[13] absolute enter_eq_no;

procedure BIOS_poll_receive;
function BIOS_carrier_present:  boolean;
function BIOS_receive_ready: boolean;
function BIOS_receive_data:  char;
procedure BIOS_transmit_data(s:    longstring);
procedure BIOS_init_com(chan: integer);
procedure BIOS_uninit_com;
procedure BIOS_flush_com;

const
   com_chan:      integer = 0;
   local:         boolean = true;  {local mode, no com port}
   bios_comm:     boolean = true;  {use bios for com port i/o}
   bios_echo:     boolean = true;  {echo com port to screen in bios mode?}


function carrier_present:  boolean;

function receive_ready: boolean;

function receive_data:  char;

procedure transmit_data(s:    longstring);

procedure init_com;

procedure flush_com;

procedure lower_dtr;

procedure raise_dtr;

procedure uninit_com;

const
   extcount:  byte = 90;        {max number of extended conferences (up to 215)}
var
   extsize:   word;             {actual extuser record size}

procedure determine_extsize(fd: dos_handle);

procedure read_extrec(fd:  dos_handle);
procedure load_extrec;

procedure write_extrec(fd:  dos_handle);
procedure save_extrec;

const
   carrier_lost = #$E3;              (* code returned with carrier is lost *)

   com_current_chan: integer = 0;    (* current communication channel *)

   port_base:    integer = -1;  (* base port number for 8250 chip *)
                                (* value = -1 until init is finished *)

   port_irq:     integer = -1;  (* port irq number *)

   old_vector:   pointer = nil; (* pointer to original com interrupt handler *)
   
   XOFF_char:    char = ^S;     (* XOFF character code *)

   disable_cts_check: boolean = false; {false if RTS handshake is needed}
   even_parity:   boolean = false; {strip parity?}

var
   port_intr:    integer;       (* interrupt number for 8250 chip *)
   intr_mask:    integer;       (* interrupt controller initialization code *)

   prev_LCR:     integer;       (* previous LCR contents *)
   prev_IER:     integer;       (* previous IER contents *)
   prev_MCR:     integer;       (* previous MCR contents *)
   prev_ICTL:    integer;       (* previous ICTL contents *)

   xmit_active:  boolean;       (* is the transmitter active now?
                                   (is a THRE interrupt expected?) *)

   XOFF_active:  boolean;       (* has XOFF suspended transmit? *)

   rxque:        queue_rec;     (* receive data queue *)
   txque:        queue_rec;     (* transmit data queue *)

   reg:          registers;     (* register package *)

   bios_bastab:  array[0..3] of word absolute $40:0;
                                (* bios table of com port bases for each
                                   port com1..com4 *)


(*
 * Uart register definitions
 *
 *)

const
   ICTL = $21;                  (* system interrupt controller i/o port *)

   RBR = 0;  (* receive buffer register *)
   THR = 0;  (* transmit holding register *)

   DLM = 1;  (* divisor latch MSB *)
   IER = 1;  (* interrupt enable register *)
      IER_DAV     = $01;       (* data available interrupt *)
      IER_THRE    = $02;       (* THR empty interrupt *)
      IER_LSRC    = $04;       (* line status change interrupt *)
      IER_MSR     = $08;       (* modem status interrupt *)


   IIR = 2;  (* interrupt identification register *)
      IIR_PENDING = $01;       (* low when interrupt pending *)

      IIR_MASK    = $06;       (* mask for interrupt identification *)
        IIR_MSR     = $00;       (* modem status change interrupt *)
        IIR_THRE    = $02;       (* transmit holding reg empty interrupt *)
        IIR_DAV     = $04;       (* data available interrupt *)
        IIR_LSR     = $06;       (* line status change interrupt *)


   LCR = 3;  (* line control register *)
      LCR_5BITS   = $00;       (* 5 data bits *)
      LCR_7BITS   = $02;       (* 7 data bits *)
      LCR_8BITS   = $03;       (* 8 data bits *)

      LCR_1STOP   = $00;       (* 1 stop bit *)
      LCR_2STOP   = $04;       (* 2 stop bits *)

      LCR_NPARITY = $00;       (* no parity *)
      LCR_EPARITY = $38;       (* even parity *)

      LCR_NOBREAK = $00;       (* break disabled *)
      LCR_BREAK   = $40;       (* break enabled *)

     {LCR_NORMAL  = $00;}      (* normal *)
      LCR_ABDL    = $80;       (* address baud divisor latch *)


   MCR = 4;  (* modem control register *)
      MCR_DTR     = $01;       (* active DTR *)
      MCR_RTS     = $02;       (* active RTS *)
      MCR_OUT1    = $04;       (* enable OUT1 *)
      MCR_OUT2    = $08;       (* enable OUT2 -- COM INTERRUPT ENABLE *)
      MCR_LOOP    = $10;       (* loopback mode *)


   LSR = 5;  (* line status register *)
     LSR_DAV      = $01;       (* data available *)
     LSR_OERR     = $02;       (* overrun error *)
     LSR_PERR     = $04;       (* parity error *)
     LSR_FERR     = $08;       (* framing error *)
     LSR_BREAK    = $10;       (* break received *)
     LSR_THRE     = $20;       (* THR empty *)
     LSR_TSRE     = $40;       (* transmit shift register empty *)

     LOERR_count:       integer = 0;    {overrun error count}
     LPERR_count:       integer = 0;    {parity error count}
     LFERR_count:       integer = 0;    {framing error count}
     LBREAK_count:      integer = 0;    {break received count}


   MSR = 6;  (* modem status register *)
     MSR_DCTS     = $01;       (* delta CTS *)
     MSR_DDSR     = $02;       (* delta DSR *)
     MSR_DRING    = $04;       (* delta ring *)
     MSR_DRLSD    = $08;       (* delta receive line signal detect *)
     MSR_CTS      = $10;       (* clear to send *)
     MSR_DSR      = $20;       (* data set ready *)
     MSR_RING     = $40;       (* ring detect *)
     MSR_RLSD     = $80;       (* receive line signal detect *)


   COM_BASE_TABLE: ARRAY[0..2] OF WORD = ($3F8,$2F8,$3E8);
   COM_IRQ_TABLE:  ARRAY[0..2] OF BYTE = (4, 3, 4);

   IRQ_MASK_TABLE: ARRAY[0..7] OF BYTE = ($01,$02,$04,$08,$10,$20,$40,$80);
   IRQ_VECT_TABLE: ARRAY[0..7] OF BYTE = ($08,$09,$0A,$0B,$0C,$0D,$0E,$0F);


procedure push_flags;
   inline($9C);

procedure pop_flags;
   inline($9D);

procedure disable_int;
   inline($FA);

procedure enable_int;
   inline($FB);

procedure io_delay;
   inline($EB/$00);     {jmp $+2}

procedure INTR_service_transmit;
procedure INTR_poll_transmit;
procedure INTR_service_receive;
procedure INTR_check_interrupts;

procedure cancel_xoff;
procedure control_k;
procedure INTR_lower_dtr;
procedure INTR_raise_dtr;
procedure INTR_select_port(chan: integer);
procedure INTR_init_com(chan: integer);
procedure INTR_uninit_com;
procedure INTR_set_baud_rate(speed: word);

procedure INTR_flush_com;
procedure INTR_transmit_data(s:    longstring);
function  INTR_receive_ready: boolean;
function  INTR_receive_data:  char;
procedure verify_txque_space;

const
   carrier_lost = #$E3;            (* code returned with carrier is lost *)
   com_chan:      integer = 0;
   local:         boolean = true;  {local mode, no com port}
   bios_comm:     boolean = true;  {use bios for com port i/o}
   bios_echo:     boolean = true;  {echo com port to screen in bios mode?}
   disable_cts_check: boolean = true; {false if RTS handshake is needed}
   even_parity:   boolean = false; {strip parity?}
   xoff_char:     char = ^S;

function carrier_present:  boolean;
function receive_ready: boolean;
function receive_data:  char;
procedure transmit_data(s:    longstring);
procedure init_com;
procedure flush_com;
procedure lower_dtr;
procedure raise_dtr;
procedure uninit_com;
procedure disable_int;
procedure enable_int;
procedure cancel_xoff;
procedure control_k;
procedure verify_txque_space;

procedure select_main_board;
procedure abandon_conference;
procedure rearc_scratchfile;
procedure capture_conference(n: integer);
procedure capture_new_mail;
   procedure automatic_logoff;

procedure build_download_list;
procedure open_mail_capture;
procedure buffer_mail_capture;
procedure close_mail_capture;
procedure capture_current_message;
procedure chat_mode;
procedure operator_page;
   procedure sysop_view_log;
 
procedure displn_dir(var line: longstring);

var
   current_line: string;
   prev_prompt:  string;
        
procedure pdisp (msg:  string240);
procedure pdispln(msg:  string240);
procedure disp (msg:  string240);
procedure dispc( c: char );
procedure displn(msg:  string240);
procedure newline;
procedure spaces(n: byte);
procedure space;
procedure beep;
procedure erase_prompt (len: integer);
procedure repeat_prompt;

procedure get_cmdline_raw(prelength: integer);

procedure no_hotkeys;
procedure get_cmdline;  {get cmdline without hotkeys}
procedure get_hcmdline; {get cmdline with hotkeys}

procedure prompt_def(prompt: string80; default: string80);
procedure get_def(prompt: string80; default: string80);
procedure get_defn(prompt: string80; default: string80);
procedure get_defnh(prompt: string80; default: string80);
procedure get_defen(prompt: string80);
procedure get_defyn(prompt: string80; default: boolean);
procedure get_defbl(prompt: string80);
procedure get_int(prompt: string80; var n: byte);

function key_ready: boolean;
function get_key: char;
function time_key(ms: integer): char;

procedure drop_carrier;
procedure force_offhook;

procedure check_carrier_loss;

procedure line_input(var line:  string;
                     maxlen:    integer;
                     echo:      boolean;
                     autocr:    boolean);

procedure input(var line:  string;
                maxlen:    integer);

procedure force_new_prompt;

procedure get_chars(prompt: string;
                    var dest;
                    size: integer;
                    echo: boolean);

procedure load_config_file;
procedure create_door;
procedure conference_registration;

(*
 * PCB ProDOOR sysop control panel handlers (3-1-89)
 *
 *)

(*
 * control function codes
 *
 *)
       
const
   fun_idle    = '0';      {set_function function codes}
   fun_batchdl = '1';
   fun_batchul = '2';
   fun_private = '3';
   fun_reply   = '4';
   fun_textview= '5';
   fun_arcview = '5';
   fun_xtract  = '6';
   fun_chat    = '7';
   fun_arcmail = '8';
   fun_lib     = '9';
   fun_rearc   = ':';
   fun_test    = ';';
   fun_confreg = '<';
   fun_unkill  = '=';   {all sysop functions in mail reader}
   fun_sysop   = fun_unkill;
   fun_nodechat= '>';
   fun_door    = '[';
   

procedure adjust_timing;
   (* adjust time-left based on current function crediting *)

procedure set_function(func: char);
   (* select this function for current time and bytecount crediting *)

function check_level(func:     char): boolean;       {function letter}
   (* verify access level for a function, select this function for
      current time and bytecount crediting *)

function verify_level(func:     char): boolean;       {function letter}
   (* verify access level for a function, select this function for
      current time and bytecount crediting, warning if not allowed *)

{procedure control_dump;}
   (* dump the contents of the sysop control table *)

type
   display_formats = (dir_colorize, 
                      display_normal, 
                      remove_semicolons,
                      number_lines);
   
procedure display_file_raw(name: filenames;
                           form: display_formats);
   {display the specified file.  handles special cases for
    graphics files and missing files}

procedure display_file(name: filenames);

procedure display_dirfile(name: filenames);
   {display the specified directory file.  handles special cases for
    graphics files and missing files}

procedure display_resultfile;
   {display the resultfile from archive testing; remove pathnames from
    all lines; detect invalid archives and delete them}

   procedure edit_message;
   procedure edit_header;

(*
 * PCB ProDOOR enter message module for ProMail unit (3-1-89)
 *
 *)

type
   message_entry_modes = (new_message, 
                          reply_message, 
                          reply_originator,
                          comment_message,
                          duplicate_message);
   
procedure save_message(mode: message_entry_modes);
procedure show_margins;
procedure show_line_number(n: integer);
procedure continue_entry;
procedure edit_line;

procedure insert_line(contents: string);
   {open a new line at the cursor}

procedure insert_text;
   {insert a line}

procedure delete_line;
   {delete the line at the cursor}

procedure delete_text;
   {delete a line}

procedure quote_from_original;

procedure display_original;
   {display original message with optional quoting}

procedure count_lines;

procedure enter_message(mode: message_entry_modes);

(*****************
function lock_msgfile: boolean;
procedure save_index;
procedure save_text(mode: message_entry_modes);
procedure unlock_msgfile;
procedure list_message;
procedure enter_header(mode: message_entry_modes);
******************)

procedure error_handler;
procedure install_error_handler;
var
   ExitSave: pointer;   {pointer to next exitproc in the chain}
procedure estimate_transfer_time;

function valid_filename(name: filenames): boolean;
   (* test a specified filename for validity, return false if it is
      invalid (also prints a message to that effect *)

procedure find_file(target: filenames;
                    files:  integer);
   {attempt to locate the specified file based on the
    available file directories.  list matching files in 'select'}

function ok_name(target: filenames): boolean;
   {is the specified filename ok for the selected protocol?
    return the exact name if it is}

procedure set_scratch_type;
procedure select_archive(action: string30);

procedure flag_files;
procedure autoflag_scratch;
function flag_warning(quit: boolean): boolean;
function is_free_file(name: string65): boolean;

function verify_access(fname:      anystring;           {function name}
                       flevel:     integer;             {minimum level}
                       fpassword:  anystring;           {password if any}
                       fmessage:   anystring)           {failure message}
                          : boolean;

function file_allowed(path:       anystring;            {name to verify}
                      secfile:    filenames)            {fsec/upsec name}
                         : boolean;
                         
const
   event_now = false;
   event_possible = true;

procedure fill_chars( var dest; 
                      source:    anystring;
                      size:      integer);
procedure lfill_chars( var dest;
                       source:    anystring;
                       size:      integer);
   {fill_chars with leading space on source}

procedure save_name_list;
procedure load_name_list;

procedure save_pointers(name: filenames);
procedure load_pointers(name: filenames);

procedure prepare_word_wrap(var par: string; var pos: integer; len: integer);

procedure print_text(s: anystring);
procedure make_raw_log_entry(entry: anystring);
procedure make_log_entry (entry: anystring; echo: boolean);

function download_k_allowed: word;

procedure get_infocount(path:       filenames;
                        reclen:     longint;
                        var count:  integer);

procedure get_dirn(n:         integer;
                   var name:  filenames;
                   var descr: anystring);
function dir_count: integer;

function minutes_before_event: integer;
function event_run_needed(possible: boolean): boolean;

function time_used: integer;
function minutes_left: integer;
procedure check_time_left;
procedure display_time(used: boolean);
procedure display_time_left;
procedure adjust_time_allowed(addseconds: longint);

type
   user_ix_rec = record
      urec: word;
      name: char25;
   end;

var
   user_ix:  user_ix_rec;

procedure load_conf(n: integer);

procedure get_user_rec(var user: pcb_user_rec; recn: word);
procedure get_user_info(var user: pcb_user_rec; name: char25);
procedure load_user_rec;
procedure load_extuser;
procedure put_user_rec(var user: pcb_user_rec; recn: word);
procedure save_user_rec;
procedure save_extuser;

procedure load_pcbsys_file;
procedure save_pcbsys_file;
procedure save_offline_pcbsys_file;

procedure build_scratchnames;

procedure load_cnames_file;
procedure load_pcbdat_file;

procedure high_ascii_filter(var c: char);

function get_pcbtext(n: integer): anystring;

procedure display_conference_news;
function lookup_conference_number(name: string10): integer;
procedure join_conference;
procedure request_library;
   procedure log_file_transfer;
 

procedure log_upload_name(logfile:  anystring;
                          source:   anystring;
                          dest:     anystring;
                          size:     longint;
                          descr:    string);

procedure log_download(entry: anystring);

function next_extended_descr(var descr: string): anystring;
procedure display_extended_description(descr: string);

(*
 * PCB ProDOOR mail access module (3-1-89)
 *
 *)

const
   msgmaxlen = 72;      {maximum line length in new message entry}
   maxlines = 102;      {maximum lines per message}

   maxtext = (maxlines+1)*(msgmaxlen+8);
                        {maximum text size per message maxlen*maxlines}

   blksiz = 128;        {size of each message block}
   maxblocks = (maxtext div blksiz)+1;
                        {maximum number of blocks per message}

   no_msg = $FFFF;      {message position indicating no valid message}

   cap_bufsize = 10240; {mail capture buffer size}
   maxinbuf = 128;      {input buffer records *128}
   maxixbuf = 128;      {index buffer records *4}
   
   maxthread = 1000;    {maximum range of thread memory}


(* layout of the message control file records for PCBoard *)

type
   message_rec = record
      case integer of

      {file header record}
         0: (himsg:    single;    {highest message on file}
             lowmsg:   single;    {low msg number in message base}
             msgcnt:   single;    {number of active messages}
             callers:  single;    {number of callers on system}
             lockflag: char6;     {LOCKED if file being updated}
             fill1:    array[1..105] of char);
                                  {reserved for future use}

      {message header record}
         1: (StatusCode:  char;     {protect, unprotect flag '*' or blank}
             Number:      single;   {message number}
             ReferTo:     single;   {reference message number}
             blocks:      byte;     {number of 128 byte text blocks}
             Date:        char8;    {mm-dd-yy}
             Time:        char5;    {hh:mm}
             WhoTo:       char25;
             ReadDate:    single;   {yymmdd numeric date of reply message}
             ReadTime:    char5;    {hh:mm of reply}
             HasReplys:   char;     {'R' is ALL message has replys}
             WhoFrom:     char25;
             Subject:     char25;
             Password:    char12;   {blank=none}
             status:      char;     {dead_msg(226) or live_msg(225)}
             echoflag:    char;     {'E' if msg to be echoed}
             filler:      char6);   {reserved}

      {message text record}
         2: (body:      array[1..128] of char); {body of the message, space fill}
   end;


   blockarray = array[1..maxblocks] of message_rec;
   rawarray   = array[1..maxtext] of char;
   textarray  = array[1..maxlines] of string80;
   threadarray= array[1..maxthread] of boolean;
   cap_bufrec = array[1..cap_bufsize] of byte;
   
const
   dead_msg    = #226;           {message status codes}
   live_msg    = #225;
   endline     = #227;           {end of line character in message files}

var
   cap_buffer:   ^cap_bufrec;
   cap_count:    integer;
   
   selectedfile:      filenames;
   messagebase_file:  filenames;

   mbfd:         buffered_file;
   ixfd:         buffered_file;
   header:       message_rec;
   mheader:      message_rec;

   curmsg:       word;
   basemsg:      word;
   lastmsg:      word;
   memorymsg:    word;
   priormsg:     word;
   newmsgs:      word;
   yourmsgs:     integer;

   msgpos:       word;

   txtblocks:    integer;
   maxpos:       integer;
   block:        ^blockarray;
   raw:          ^rawarray absolute block;

   lines:        ^textarray;
   linecnt:      integer;

   threadseen:   ^threadarray;  {message thread memory}
   threadbase:   longint;

   privatep:     boolean;       {message private?}
   groupp:       boolean;       {message has a group password?}
   readp:        boolean;       {message has been read?}
   tomep:        boolean;       {message is to me?}
   frommep:      boolean;       {message is from me?}
   toallp:       boolean;       {message is to ALL?}
   kill_allowed: boolean;       {user allowed to kill this message}

   WhoTo:        char25;        {to: address after prepare_line}
   Subject:      char25;        {subject after prepare_line}

   search_key:   anystring;
   nextjoin:     string20;      {set to J nn at end of message base}
   direction:    char;          {+ or -}
   
   lastread:     ^single;       {pointer to current lastread counter}

   fromUser:     pcb_user_rec;  {user record of message sender}
   lookup_info:  boolean;       {find city for each user?}
   have_city:    boolean;

   non_stop:     boolean;       {currently in non-stop mode?}


const
   pprevcmd:     string2 = 'R';
   prevcmd:      string2 = 'R';         {previous command letter}
   zip_active:   boolean = false;       {cancel search after first find?}

   substitute:   boolean = true;        {allow @...@ substitutes?}


function sysopfun_allowed: boolean;
function message_allowed: boolean;
function meets_criteria: boolean;

procedure display_header;
procedure display_text;
procedure display_loaded_message;
procedure get_text;
procedure load_message(killed: boolean);
procedure save_message_header;
procedure set_lastread;
procedure set_read_flag;

procedure decode_status;
procedure advance;
procedure get_index(killed: boolean);
procedure check_message(killed: boolean);
function select_conference(conf: integer): boolean;
procedure display_conference_info;
procedure open_conference;
procedure reopen_messagebase;
procedure close_conference;
procedure alloc_mail;
procedure free_mail;

function locate_next_personal {(par: string2)}: boolean;

procedure main_menu;
procedure abort_program(reason: string);
procedure usage (error: anystring);

procedure popup_prompt(prompt:      string80;
                       var answer:  string;
                       maxlen:      integer);
   {prompt for an input and remove the prompt afterwards}

procedure popup_cmdline(prompt:      string80;
                        defalt:      string80);
   {prompt for a command line and remove the prompt afterwards}

procedure force_enter;
procedure force_enter2;
procedure wait_for_enter;

function nomore: boolean;
procedure set_node_status(newcode: char);
procedure node_status_display;
procedure node_chat;
procedure check_chat_request;

const
  node_available        = 'A';
  node_unavailable      = 'U';
  node_in_door          = 'D';
  node_entering_msg     = 'E';
  node_transfer         = 'T';
  node_group_chat       = 'G';
  node_logoff_pending   = 'L';
  node_dropping_2dos    = 'X';
  node_no_caller        = ' ';
  node_chat_request     = 'R';
  node_going_down       = '@';  {not a standard code}

  chat_poll: boolean = true;    {true if polling for chat}

procedure prepare_line(var line: string);
procedure prepare_protocol_table;
procedure remove_variables(var line: string);

const
   delete_variables: boolean = false;

procedure view_profile;
procedure get_protocol(modes: transfer_modes);

(*
 * PCB ProDOOR read message module (3-1-89)
 *
 *)

procedure read_messages;

   procedure sysop_remote;
 
procedure select_names;
procedure display_conference_status(func: char);
   {display status of all conferences; optionally exclude those
    with no new messages in them}

type
   status_formats = (normal_format, help_format, pgup_format, pgdn_format);

procedure update_status_display (format:  status_formats);
   {display current status information on the last 2 lines of the screen}

procedure init_status_display;
   {prepare the screen for status displays}

procedure transfer_status_display;
   {prepare the status display area for execution of a protocol driver}

procedure delete_status_display;
   {completely remove status display from the screen}

procedure shell_to_dos;
   {allow the sysop to drop to DOS for a moment}

procedure toggle(var v: char2);
   {toggle a variable and update status display accordingly}

procedure dispatch_function_key(c: char);
   {sysop function key dispatch}

procedure process_function_keys;
   {read next local function code and dispatch it}


procedure display_protocol_table;
procedure test_archive;

procedure get_upload_description(prompt: string80;
                                 var desc: string);

function expert: boolean;

function scan_nextpar(var cmdline: string): string;
procedure get_nextpar;
procedure unget_par;

procedure not_understood;
procedure not_available;
procedure file_not_found (name: anystring);
procedure file_missing;

procedure keypause;

function pro_command: anystring;
function pro_title: anystring;
function pro_code: char;
function pro_files: integer;
function pro_mode: transfer_modes;

function estimated_time(size: longint): real;

procedure open_capture;
procedure close_capture;

procedure clean_playpen;
procedure clean_scratch;

procedure disp_margin(left,width: integer);

function expdate(yymmdd: string8): string8;     {convert to mm-dd-yy}
function mmddyy_to_yymmdd(mmddyy: string8): string8;
function yymmdd_to_mmddyy(yymmdd: string8): string8;

function todays_date_mmddyy: string8;
function todays_date_yymmdd: string8;

function dtok(d: double): string8;
function stok(s: single): string8;

function posc(c: char; var s: string): integer;

procedure check_command_line(var command: string);
procedure execute(command: string);

procedure disconnect_warning(left: integer);
procedure check_disconnect_warning;

function file_is_new(name: dos_filename): boolean;

(*
 * PCB ProDoor visual message entry/edit module for ProMail unit (3-1-89)
 *
 *)

const
   topscreen = 7;       {first screen line for text entry}
   maxscrlines = 40;    {maximum number of display lines}

   insert_mode: boolean = true;

var
   statline:    integer;        {line for statue messages}
   scrlines:    integer;        {number of screen lines for text entry}
   scrollsiz:   integer;        {number of lines to scroll by}
   topline:     integer;        {message line number at top of screen}
   cline:       integer;        {current message line number}
   ccol:        integer;        {current column number}

   phyline:     array[1..maxscrlines] of string[72];
                                {physical display text}

   pleft:       integer;        {previous value of minutes_left}

procedure visual_edit;
   procedure write_user_info;
 
procedure get_xcmd(xfile: filenames);

procedure log_onoff(what: string30; state: boolean);
procedure expert_toggle;
procedure mode_toggle;
procedure snoop_toggle;
procedure hotkey_toggle;
procedure scan_toggle;
