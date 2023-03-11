/*
 * circular_queue.h
 *
 *  Created on: Jul 21, 2022
 *      Author: 유지현
 */

#ifndef INC_CIRCULAR_QUEUE_H_
#define INC_CIRCULAR_QUEUE_H_

#include <stdio.h>

#include "define.h"

typedef struct Queue {
	uint8_t history[MAX_HISTORY_BUFFER_LENGTH][MAX_CMD_BUFFER_LENGTH];
	int front;
	int rear;
	int usage;
	int cursor;
}Queue;


enum
{
	UP,
	DOWN,
};


extern Queue que;


void QueueInit(Queue* q);
int IsEmpty(Queue* q);
int IsFull(Queue* q);
void Enqueue(Queue* q, uint8_t* str, uint8_t length);
int Dequeue(Queue* q);
int move_cursor(Queue* q, int dir);
void print_history(Queue* q, int num);


#endif /* INC_CIRCULAR_QUEUE_H_ */
