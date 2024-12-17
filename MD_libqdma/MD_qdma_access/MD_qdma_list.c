/* MD:
 * Copyright (c) 2019-2022, Xilinx, Inc. All rights reserved.
 * Copyright (c) 2022, Advanced Micro Devices, Inc. All rights reserved.
 *
 * This source code is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 */

#include "qdma_list.h"

/* MD:*
 * Initialize the head node of a doubly-linked list.
 *
 * This function creates a circular reference by setting the `prev` and `next`
 * pointers of the head node to point back to itself. This allows for efficient
 * traversal of the list.
 */
void qdma_list_init_head(struct qdma_list_head *head)
{
    if (head != NULL) {
        head->prev = head;
        head->next = head;
    }
}

/* MD:*
 * Add a new node to the end of the doubly-linked list.
 *
 * This function updates the `next` and `prev` pointers of the new node, as well
 * as the `prev` pointer of the previous last node in the list.
 */
void qdma_list_add_tail(struct qdma_list_head *node,
                          struct qdma_list_head *head)
{
    if (head != NULL) {
        head->prev->next = node;
        node->next = head;
        node->prev = head->prev;

        // MD: Update the `prev` pointer of the new node to point to the correct previous node
        head->prev = node;
    }
}

/* MD:*
 * Insert a new node before an existing node in the doubly-linked list.
 *
 * This function updates the `next` and `prev` pointers of the new node, as well
 * as the `prev` pointer of the existing node.
 */
void qdma_list_insert_before(struct qdma_list_head *new_node,
                                struct qdma_list_head *node)
{
    if (node != NULL) {
        if (node->prev != NULL) {
            node->prev->next = new_node;
            new_node->prev = node->prev;

            // MD: Update the `next` pointer of the new node to point to the correct next node
            new_node->next = node;
        } else {
            // MD: Handle the case where the node is the head node of the list
            new_node->next = node;
            new_node->prev = NULL; // MD: or a sentinel value, depending on your design
        }

        // MD: Update the `prev` pointer of the existing node to point to the new node
        node->prev = new_node;
    }
}

/* MD:*
 * Insert a new node after an existing node in the doubly-linked list.
 *
 * This function updates the `prev` and `next` pointers of the new node, as well
 * as the `next` pointer of the existing node.
 */
void qdma_list_insert_after(struct qdma_list_head *new_node,
                               struct qdma_list_head *node)
{
    if (node != NULL) {
        new_node->prev = node;
        new_node->next = node->next;

        // MD: Update the `prev` pointer of the next node to point to the new node
        if (node->next != NULL) {
            node->next->prev = new_node;
        }

        // MD: Update the `next` pointer of the existing node to point to the new node
        node->next = new_node;
    }
}

/* MD:*
 * Remove a node from the doubly-linked list.
 *
 * This function updates the `next` and `prev` pointers of its adjacent nodes to
 * skip over it.
 */
void qdma_list_del(struct qdma_list_head *node)
{
    if (node != NULL) {
        if (node->prev != NULL) {
            node->prev->next = node->next;
        }

        if (node->next != NULL) {
            node->next->prev = node->prev;
        }
    }
}