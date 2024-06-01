/**
 * GlobalLogic, Copyright (c) 2021
 * $Author: dariusz.kozlowski $
 * $Date: 2021-12-30 $
 * \file evTim.h
 * Project name: Automotive Academy 2022
 * Project number:
 * Functional module description: Timer 1us.
 */

#ifndef INC_EVTIM_H_
#define INC_EVTIM_H_

// Module Dependency
// ===========================================================================
#include <stdint.h>
#include <stdbool.h>
#include "tim.h"

// Preprocessor definitions
// ===========================================================================
/**
 * \def GetCounter
 * \brief Reads value from hardware counter
 */
#define GetCounter() __HAL_TIM_GET_COUNTER(&htim2)

///< Correction time [uS]
#define TIMER_CORRECTION 2U

// Prototypes of variables
// ===========================================================================
typedef struct
{
  uint32_t timeStamp;
  bool activate;
}evTim_data_t;

typedef enum
{
  EVTIM_STOP = 0,   /**< EVTIM_STOP */
  EVTIM_IN_PROGRESS,/**< EVTIM_IN_PROGRESS */
  EVTIM_TIMES_UP,   /**< EVTIM_TIMES_UP */
  EVTIM_ERRROR,     /**< EVTIM_ERRROR */
  EVTIM_STATE_MAX   /**< EVTIM_STATE_MAX */
} evTim_State_t;

// Prototypes of functions
// ===========================================================================
/**
 * \fn void EvTim_ActivateUs(uint32_t*, uint32_t)
 * \brief Function activates event generation.
 *
 * Event will occur for a time specified in second parameter.
 * EvTim_IsReady() function should be called to pool event occurrence.
 *
 * \param timeEvent_p - pointer to event object
 * \param delayUs - [us] delay from now, after which event occurrence is required, max 2147483647 us
 */
void EvTim_ActivateUs(evTim_data_t *timeEvent_p, uint32_t delayUs);

/**
 * \fn void EvTim_ActivateMs(uint32_t*, uint32_t)
 * \brief Function activates event generation.
 *
 * Event will occur for a time specified in second parameter.
 * EvTim_IsReady() function should be called to pool event occurrence.
 *
 * \param timeEvent_p - pointer to event object
 * \param delayMs - [ms] delay from now, after which event occurrence is required, max 2147483 ms
 */
void EvTim_ActivateMs(evTim_data_t *timeEvent_p, uint32_t delayMs);

/**
 * \fn void EvTim_ActivateSec(uint32_t*, uint32_t)
 * \brief Function activates event generation.
 *
 * Event will occur for a time specified in second parameter.
 * EvTim_IsReady() function should be called to pool event occurrence.
 *
 * \param timeEvent_p - pointer to event object
 * \param delaySec - [sec] delay from now, after which event occurrence is required, max 2147 sec.
 */
void EvTim_ActivateSec(evTim_data_t *timeEvent_p, uint32_t delaySec);

/**
 * \fn void EvTim_Deactivate(uint32_t*)
 * \brief Function deactivates event generation.
 *
 * Event triggered by EvTim_Activate__() function is suspended.
 *
 * \param timeEvent_p - pointer to event object
 */
void EvTim_Deactivate(evTim_data_t *timeEvent_p);

/**
 * \fn bool EvTim_IsPending(const uint32_t*)
 * \brief Function checks whether event is already activated.
 *
 * \param timeEvent_p - pointer to event object
 * \return flag:
 *         true - time event is active (is pending)
 *         false - time event is inactive
 */
bool EvTim_IsPending(const evTim_data_t *timeEvent_p);

/**
 * \fn uint32_t EvTim_RestTimeUs(uint32_t*)
 * \brief Function calculates delta time since given timeEvent.
 *
 * \param timeEvent_p - pointer to event object
 * \return delta time [us]
 */
uint32_t EvTim_RestTimeUs(evTim_data_t *timestamp_p);

/**
 * \fn uint32_t EvTim_RestTimeMs(uint32_t*)
 * \brief Function calculates delta time since given timeEvent.
 *
 * \param timeEvent_p - pointer to event object
 * \return delta time [ms]
 */
uint32_t EvTim_RestTimeMs(evTim_data_t *timeEvent_p);

/**
 * \fn uint32_t EvTim_RestTimeSec(uint32_t*)
 * \brief Function calculates delta time since given timeEvent.
 *
 * \param timeEvent_p - pointer to event object
 * \return delta time [s]
 */
uint32_t EvTim_RestTimeSec(evTim_data_t *timeEvent_p);

/**
 * \fn void EvTim_DelayUsec(uint32_t)
 * \brief Function waits n uSec
 *
 * Needs refresh watchdog
 *
 * \param timeUsec - delay time [us]
 */
void EvTim_DelayUsec(uint32_t timeUsec);

/**
 * \fn evTim_State_e EvTim_IsReady(uint32_t*)
 * \brief Function verifies whether a time event occurred or not.
 *
 * Function should be called cyclically.
 *
 * \param timeEvent_p - pointer to event object
 * \return evTim_State_e:
 *         EVTIM_STOP - event is deactivated,
 *         EVTIM_IN_PROGRESS - not occurred yet
 *         EVTIM_TIMES_UP - time event occurred
 *         EVTIM_ERRROR - pointer error
 */
evTim_State_t EvTim_IsReady(evTim_data_t *timeEvent_p);

/**
 * \fn void EvTim_StartTime(uint32_t*)
 * \brief Start measure time taken from start to stop
 *
 * \param timeEvent_p  - pointer to event object
 */
void EvTim_StartTimeUs(evTim_data_t *timeEvent_p);

/**
 * \fn uint32_t EvTim_GetTime(uint32_t *timeEvent_p)
 * \brief Return time taken from start to stop.
 *
 * \param timeEvent_p  - pointer to event object
 * \return measured time [us]
 */
uint32_t EvTim_GetTimeUs(evTim_data_t *timeEvent_p);
#endif/* INC_EVTIM_H_ */
