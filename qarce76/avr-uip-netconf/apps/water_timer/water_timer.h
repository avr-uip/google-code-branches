#ifndef __WATER_TIMER_H__
#define __WATER_TIMER_H__
#include "psock.h"



void water_timer_appcall(void);
void water_timer_init(void);

/*
#if defined PORT_APP_MAPPER
	#define SIMPLE_HTTPD_APP_CALL_MAP {simple_httpd_appcall, 80, 0},
	struct simple_httpd_state httpd_state_list[UIP_CONF_MAX_CONNECTIONS];
#else
	#define SIMPLE_HTTPD_APP_CALL_MAP
	#define UIP_APPCALL simple_httpd_appcall
	typedef struct simple_httpd_state uip_tcp_appstate_t;
#endif
*/

#endif /* __WATER_TIMER_H__ */
