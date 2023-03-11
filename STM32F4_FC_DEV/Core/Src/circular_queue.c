/*
 * circular_queue.c
 *
 *  Created on: 2022. 7. 22.
 *      Author: 유지현
 */


#include <stdio.h>
#include <string.h>

#include "circular_queue.h"


void QueueInit(Queue* q){
	q->front = 0;
	q->rear = 0;
	q->cursor = 0;
	q->usage = 0;
}


int IsEmpty(Queue* q){
	return (q->front == q->rear);
	//return (q->usage == 0);
}


int IsFull(Queue* q){
	return (q->front == ((q->rear + 1) % MAX_HISTORY_BUFFER_LENGTH));
	//return (q->usage == MAX_HISTORY_BUFFER);
}


void Enqueue(Queue* q, uint8_t* str, uint8_t length){
	if(IsFull(q)){
		Dequeue(q);
	}
	memcpy(q->history[q->rear], str, length);
	q->rear = (q->rear + 1) % MAX_HISTORY_BUFFER_LENGTH;
	q->usage++;
}


int Dequeue(Queue* q){
	if (IsEmpty(q)) {
		return 0;
	}
	else {
		q->front = (q->front + 1) % MAX_HISTORY_BUFFER_LENGTH;
		q->usage--;
		return 1;
	}
}


int move_cursor(Queue* q, int dir){
	if (dir == UP){
		if(q->cursor == q->front){
			return 0;
		}

		if(q->cursor == 0){
			q->cursor = MAX_HISTORY_BUFFER_LENGTH - 1;
		}else{
			q->cursor = q->cursor - 1;
		}
		return 1;
	}
	else if(dir == DOWN){
		if(((q->cursor + 1) % MAX_HISTORY_BUFFER_LENGTH) == q->rear){
			return 0;
		}

		q->cursor = (q->cursor + 1) % MAX_HISTORY_BUFFER_LENGTH;
		return 1;
	}
	return 0;
}



void print_history(Queue* q, int num){
	if(q->usage == 0){
		return;
	}

	int idx = 1;

	if(num){
		for(int i = 0; i < num; ++i){
			move_cursor(q, UP);
		}

		while(q->cursor != q->rear){
			printf("%d %s\r\n", idx++, q->history[q->cursor]);
			q->cursor = (q->cursor + 1) % MAX_HISTORY_BUFFER_LENGTH;
		}
	}else{
		q->cursor = q->front;
		while(q->cursor != q->rear){

			printf("%d %s\r\n", idx++, q->history[q->cursor]);

			//printf("******\n");
			q->cursor = (q->cursor + 1) % MAX_HISTORY_BUFFER_LENGTH;
		}

	}
}
