/*****************************************************************************
 * @file        CAN_bus.h
 *
 * @brief:      This  Module is for driver of can
 *
 * @defgroup Can name
 * @addtogroup modules
 * @ingroup common
 * @{
 * */

#ifndef __CAN_BUS_MODULE__
#define __CAN_BUS_MODULE__

/********************************************************************************
 * INCLUDE DIRECTIVES
 ********************************************************************************/


/********************************************************************************
 * DEFINITIONS, ENUMS, STRUCTURES AND TYPEDEFS
 ********************************************************************************/

 

void CanBus_Init(void);
void CanBus_Task(void);
bool CanBus_IsLeftSideActive(void);
bool CanBus_IsRightSideActive(void);
bool CanBus_IsSeedingActive(void);





#endif //__CAN_BUS_MODULE__


