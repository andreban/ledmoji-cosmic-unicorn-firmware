#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

#include "lwipopts_examples_common.h"

/* TCP WND must be at least 16 kb to match TLS record size
   or you will get a warning "altcp_tls: TCP_WND is smaller than the RX decrypion buffer, connection RX might stall!" */
#undef TCP_WND
#define TCP_WND  16384

#define LWIP_ALTCP               1
#define LWIP_ALTCP_TLS           1
#define LWIP_ALTCP_TLS_MBEDTLS   1

// Workaround for "*** PANIC *** sys_timeout: timeout != NULL, pool MEMP_SYS_TIMEOUT is empty", caused by v1.5.0 of the
// Pico SDK.
// See https://github.com/raspberrypi/pico-sdk/issues/1281 for details
#define MEMP_NUM_SYS_TIMEOUT     (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 8)

#define LWIP_DEBUG 1
#define ALTCP_MBEDTLS_DEBUG  LWIP_DBG_ON

#endif
