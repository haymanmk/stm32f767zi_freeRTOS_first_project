#ifndef DOUBLE_BUFFER_MEM_MANAGE_H
#define DOUBLE_BUFFER_MEM_MANAGE_H

#include "main.h"

#define LINKED_LIST_SIZE 5U
#define DOUBLE_BUFFER_SIZE 32

// A known value to initialize the double buffer
#define DOUBLE_BUFFER_INIT_VALUE 0xf5f5f5f5UL

typedef uint32_t doubleBufferArray_t[DOUBLE_BUFFER_SIZE];

typedef struct linkedListNode
{
    doubleBufferArray_t doubleBuffer;
    uint32_t* owner;
    struct linkedListNode *next;
} linkedListNode_t;

typedef enum channel{
    CH1=0,
    CH2,
    CH3,
    CH4,
    MAX_CHANNEL,
} channel_t;

static linkedListNode_t MEM_POOL_LINKED_LIST[MAX_CHANNEL][LINKED_LIST_SIZE];

void initLinkedList(linkedListNode_t *linkedListHead);
linkedListNode_t *initNode(linkedListNode_t *node);
void freeNode(doubleBufferArray_t **buf, channel_t channel);

#endif // DOUBLE_BUFFER_MEM_MANAGE_H