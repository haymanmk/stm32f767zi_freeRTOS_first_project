#include <stdint.h>
#include <stdlib.h>

#include "double_buffer_mem_manage.h"

static linkedListNode_t *LINKED_LIST_HEAD_TABLE[MAX_CHANNEL];

void initMemPool()
{
    // initialize the linked list in the memory pool
    for (uint8_t i = 0; i < MAX_CHANNEL; i++)
    {
        initLinkedList(MEM_POOL_LINKED_LIST[i]);
        LINKED_LIST_HEAD_TABLE[i] = &MEM_POOL_LINKED_LIST[i][0];
    }
}

void initLinkedList(linkedListNode_t *memp_desc)
{
    uint32_t addressIncrement = sizeof(memp_desc[0]);
    linkedListNode_t *currentNode = initNode(&memp_desc[0]);

    linkedListNode_t *newNode = NULL;

    for (uint8_t i = 0; i < LINKED_LIST_SIZE - 1; i++)
    {
        newNode = initNode((linkedListNode_t *)((uint32_t)currentNode + addressIncrement));

        currentNode->next = newNode;
        currentNode = newNode;
    }
}

linkedListNode_t *initNode(linkedListNode_t *node)
{
    // fill the double buffer with a known value
    for (uint8_t i = 0; i < DOUBLE_BUFFER_SIZE; i++)
    {
        (node->doubleBuffer)[i] = DOUBLE_BUFFER_INIT_VALUE;
    }

    node->next = NULL;
    node->owner = NULL;

    return node;
}

void getFreeNode(doubleBufferArray_t **buf, channel_t channel)
{
    linkedListNode_t **linkedListHead = &LINKED_LIST_HEAD_TABLE[channel];

    if ((*linkedListHead) == NULL)
    {
        return NULL;
    }

    linkedListNode_t *freeNode = *linkedListHead;
    *linkedListHead = (*linkedListHead)->next;
    freeNode->next = NULL;

    // return the double buffer array
    *buf = &(freeNode->doubleBuffer);

    // remember the owner of the buffer
    freeNode->owner = (uint32_t *)buf;
}

void freeNode(doubleBufferArray_t **buf, channel_t channel)
{
    linkedListNode_t **linkedListHead = &LINKED_LIST_HEAD_TABLE[channel];

    linkedListNode_t *node = (linkedListNode_t *)(*buf);

    *buf = NULL;

    // check if buf is a valid pointer
    if (node == NULL)
    {
        return;
    }

    // check if the node is already in the linked list
    if (node->next != NULL)
    {
        return;
    }

    node->next = *linkedListHead;
    node->owner = NULL;
    *linkedListHead = node;
}