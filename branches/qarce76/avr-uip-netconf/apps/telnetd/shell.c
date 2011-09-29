 /*
 * Copyright (c) 2003, Adam Dunkels.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the uIP TCP/IP stack.
 *
 * $Id: shell.c,v 1.1 2006/06/07 09:43:54 adam Exp $
 *
 */
#include "global-conf.h"

#if TELNETD_TIME_CMD
#include "ds1302.h"
#endif

#include "shell.h"

#include <string.h>

struct ptentry {
  char *commandstr;
  void (* pfunc)(char *str);
};

#ifndef SHELL_PROMPT
#define SHELL_PROMPT "uIP 1.0> "
#endif

/*---------------------------------------------------------------------------*/
static void
parse(register char *str, struct ptentry *t)
{
  struct ptentry *p;
  for(p = t; p->commandstr != NULL; ++p) {
    if(strncmp(p->commandstr, str, strlen(p->commandstr)) == 0) {
      break;
    }
  }

  p->pfunc(str);
}
/*---------------------------------------------------------------------------*/
static void
inttostr(register char *str, unsigned int i)
{
  str[0] = '0' + i / 100;
  if(str[0] == '0') {
    str[0] = ' ';
  }
  str[1] = '0' + (i / 10) % 10;
  if(str[0] == ' ' && str[1] == '0') {
    str[1] = ' ';
  }
  str[2] = '0' + i % 10;
  str[3] = ' ';
  str[4] = 0;
}
static void
settime(char *str)
{
	uint8_t timestore_write[8] = {0x00,0x38,0x10,0x10,0x11,0x04,0x10,0};
	send_time_to_rtc(timestore_write);
	shell_output("time set", "");
}
/*---------------------------------------------------------------------------*/
static void
isotime(char *str)
{
  uint8_t timestore_read[7];
  uint8_t iso_timestore[16];

  read_time(timestore_read);
  iso_time(timestore_read, iso_timestore);
/*  iso_timestore[0] = 'a' + timestore_read[0];
  iso_timestore[1] = 'a' + timestore_read[1];
  iso_timestore[2] = 'a' + timestore_read[2];
  iso_timestore[3] = 'a' + timestore_read[3];
  iso_timestore[4] = '\0';
*/
  shell_output(iso_timestore, "");
}
/*---------------------------------------------------------------------------*/
static void
help(char *str)
{
  shell_output("Available commands:", "");
#ifdef TELNETD_TIME_CMD
  shell_output("isotime - show the current system time in iso format", "");
  shell_output("settime - set the current system time", "");
  shell_output("gettime - get the current system time", "");
  shell_output("rtcisotime - show the current RTC time in iso format", "");
  shell_output("rtcsettime - set the current RTC time", "");
  shell_output("rtcgettime - set the current RTC time", "");
  shell_output("rtctosys   - copy the RTC time to the system time", "");
  shell_output("systortc   - copy the system time to the RTC chip", "");
#endif
  shell_output("stats   - show network statistics", "");
  shell_output("conn    - show TCP connections", "");
  shell_output("help, ? - show help", "");
  shell_output("exit    - exit shell", "");
}
/*---------------------------------------------------------------------------*/
static void
unknown(char *str)
{
  if(strlen(str) > 0) {
    shell_output("Unknown command: ", str);
	*str = 0;
  }
}
/*---------------------------------------------------------------------------*/
static struct ptentry parsetab[] =
  {{"stats", help},
   {"conn", help},
   {"help", help},
   {"exit", shell_quit},
   {"?", help},
#ifdef TELNETD_TIME_CMD
   {"time", isotime},
   {"settime", settime},
#endif

   /* Default action */
   {NULL, unknown}};
/*---------------------------------------------------------------------------*/
void
shell_init(void)
{
}
/*---------------------------------------------------------------------------*/
void
shell_start(void)
{
  shell_output("uIP command shell", "");
  shell_output("Type '?' and return for help", "");
  shell_prompt(SHELL_PROMPT);
}
/*---------------------------------------------------------------------------*/
void
shell_input(char *cmd)
{
  parse(cmd, parsetab);
  shell_prompt(SHELL_PROMPT);
}
/*---------------------------------------------------------------------------*/
