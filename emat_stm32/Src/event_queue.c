/**
  ******************************************************************************
  * File Name          : event_queue.c
  * Description        : Simple events queue.
  ******************************************************************************

  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "event_queue.h"
typedef enum QUEUE_EVENT_TAG {
	NO_EVENT = -1,
	TIMER1_EXPIRED,
	TIMER2_EXPIRED,
	CMD_WIDTH
} queue_event_e;
typedef struct QUEUE_ELEMENT_TAG {
	queue_event_e event;
	uint32_t param;
} queue_element_s;

#define EVENT_QUEUE_SIZE 32
static queue_element_s queue[EVENT_QUEUE_SIZE];
static queue_element_s *pAdd = queue;
static queue_element_s *pGet = queue;

void PutEvent(queue_event_e event, uint32_t param)
{
	pAdd->event = event;
	pAdd->param = param;
	pAdd = (pAdd + 1) & (EVENT_QUEUE_SIZE - 1);
}

void GetEvent(queue_element_s *ev)
{
	if (pAdd == pGet) {
		ev->event = NO_EVENT;
		ev->param = 0;
	} else {
		ev->event = pGet->event;
		ev->param = pGet->param;
		pGet = (pGet + 1) & (EVENT_QUEUE_SIZE - 1);
	}
}

/******END OF FILE****/
