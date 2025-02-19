
#ifndef __LOCKED_QUEUE

#define __LOCKED_QUEUE
#include <mutex>

/* thread-safe queue that uses mutexes to
 * protect data.                         */

template <typename T>
class LockedQueue {
private:

   struct queue_node {
      T val;
      queue_node * nextptr;
   };

   struct queue_node * head;
   struct queue_node * tail;

   /* lock for the head */
   std::mutex hmutex;
   /* lock for the tail */
   std::mutex tmutex;

   struct queue_node * new_node() {
      struct queue_node * ret;
      ret = (struct queue_node *)calloc(sizeof(struct queue_node),1);
      return ret;
   }

public:

   LockedQueue() : hmutex(std::mutex()), tmutex(std::mutex()) {
      struct queue_node * node = new_node();
      node->nextptr = NULL;
      head = tail = node;
   }

   ~LockedQueue() {
      hmutex.lock();
      tmutex.lock();

      while (head) {
         queue_node * to_delete = head;
         head = head->nextptr;
         delete to_delete;
      }
   }

   void enqueue(T val) {
      struct queue_node * node = new_node();

      node->val = val;
      node->nextptr = NULL;

      tmutex.lock();
      tail->nextptr = node;
      tail = node;
      tmutex.unlock();
   }

   bool dequeue(T * ret_val) {
      hmutex.lock();

      struct queue_node * node = head;
      struct queue_node * new_head = node->nextptr;

      if (new_head == NULL) {
         hmutex.unlock();
         return false;
      }

      *ret_val = new_head->val;
      head = new_head;

      hmutex.unlock();

      free(node);

      return true;

   }

   bool empty() {
      hmutex.lock();

      bool ret = false;

      if (head->nextptr == NULL)
         ret = true;

      hmutex.unlock();
      return ret;
   }
};

#endif
