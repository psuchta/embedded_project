/**
 * GlobalLogic, Copyright (c) 2021
 * $Author: dariusz.kozlowski $
 * $Date: 2021-12-30 $
 * \file evTim.c
 * Project name: Automotive Academy 2022
 * Project number:
 * Functional module description: Timer 1us.
 */

// Module Dependency
// ===========================================================================
#include "evTim.h"

// Preprocessor definitions
// ===========================================================================

// Definitions of variables
// ===========================================================================

// Prototypes of functions
// ===========================================================================

// Definitions of functions
// ===========================================================================
void EvTim_ActivateUs(evTim_data_t *timeEvent_p, uint32_t delayUs)
{
  timeEvent_p->timeStamp = GetCounter() + delayUs - TIMER_CORRECTION;
  timeEvent_p->activate = true;
}

void EvTim_ActivateMs(evTim_data_t *timeEvent_p, uint32_t delayMs)
{
  EvTim_ActivateUs(timeEvent_p, delayMs * 1000U);
}

void EvTim_ActivateSec(evTim_data_t *timeEvent_p, uint32_t delaySec)
{
  EvTim_ActivateUs(timeEvent_p, delaySec * 1000000U);
}

void EvTim_Deactivate(evTim_data_t *timeEvent_p)
{
  timeEvent_p->activate = false;
}

bool EvTim_IsPending(const evTim_data_t *timeEvent_p)
{
  return timeEvent_p->activate;
}

evTim_State_t EvTim_IsReady(evTim_data_t *timeEvent_p)
{
  evTim_State_t ret = EVTIM_ERRROR;

  if(timeEvent_p != NULL)
  {
    if(timeEvent_p->activate)
    {
      if((timeEvent_p->timeStamp - GetCounter()) & 0x80000000U)
      {
        timeEvent_p->activate = false;
        ret = EVTIM_TIMES_UP;
      }
      else
      {
        ret = EVTIM_IN_PROGRESS;
      }
    }
    else
    {
      ret = EVTIM_STOP;
    }
  }

  return ret;
}

uint32_t EvTim_RestTimeUs(evTim_data_t *timeEvent_p)
{
  uint32_t restTime = 0U;

  if((timeEvent_p != NULL) && timeEvent_p->activate)
  {
    restTime = timeEvent_p->timeStamp - GetCounter();

    if(restTime & 0x80000000U)
    {
      timeEvent_p->activate = false;
      restTime = 0U;
    }
  }

  return restTime;
}

uint32_t EvTim_RestTimeMs(evTim_data_t *timeEvent_p)
{
  return (EvTim_RestTimeUs(timeEvent_p) + 500U) / 1000U;
}

uint32_t EvTim_RestTimeSec(evTim_data_t *timeEvent_p)
{
  return (EvTim_RestTimeUs(timeEvent_p) + 500000U) / 1000000U;
}

void EvTim_DelayUsec(uint32_t timeUsec)
{
  evTim_data_t myTimer;
  EvTim_ActivateUs(&myTimer, timeUsec);

  while(EvTim_IsReady(&myTimer) == EVTIM_IN_PROGRESS)
  {
    // WDG_CntRefresh();
  }
}

void EvTim_StartTimeUs(evTim_data_t *timeEvent_p)
{
  timeEvent_p->timeStamp = GetCounter();
}

uint32_t EvTim_GetTimeUs(evTim_data_t *timeEvent_p)
{
  return (GetCounter() - timeEvent_p->timeStamp) & 0x7FFFFFFFU;
}
