/*****************************************************************************
 * @file        Machine_UDP.h
 *
 * @brief:      This  Module is for 
 *
 * @defgroup Machine name
 * @addtogroup modules
 * @ingroup common
 * @{
 * */

#ifndef __MACHINE_UDP_MODULE__
#define __MACHINE_UDP_MODULE__

/********************************************************************************
 * INCLUDE DIRECTIVES
 ********************************************************************************/
// #ifdef __cplusplus
// extern "C" {
// #endif

/********************************************************************************
 * DEFINITIONS, ENUMS, STRUCTURES AND TYPEDEFS
 ********************************************************************************/

 #define MACHINE_LOOP_PERIOD_MS      100

void Machine_Init(void);
void Machine_loop(void);
void Machine_ProcessData(uint8_t * udpdata);
void Machine_ProcessConfig(uint8_t * udpdata);
void Machine_ProcessRelayConfig(uint8_t * udpData);

/* Machine can send data to Aog by message data[3] == 123, data[5,6] are section status - je to zprava hello from machine helloFromMachine */

// #ifdef __cplusplus
// }
// #endif

#endif //__MACHINE_UDP_MODULE__


