
#ifndef THREADSAFE_QUEUES

#define THREADSAFE_QUEUES

#include <atomic>
#include <mutex>
#include <cstdlib>

/*
 * lock-free queue. 
 * Untested :/
 */
template <typename T>
class lf_queue {
   struct lf_queue_node {
      T val;
      std::atomic<struct lf_queue_node *> nextptr;
   };
   struct lf_queue_node * new_node();
   std::atomic<struct lf_queue_node *> head;
   std::atomic<struct lf_queue_node *> tail;
public:

   lf_queue();
   void enqueue(T val);
   bool dequeue(T * val);
   bool empty();
};

/*
 * lock based queue.
 * Untested :/
 */
template <typename T>
class ll_queue {
   struct ll_queue_node {
      T val;
      ll_queue_node * nextptr;
   };
   struct ll_queue_node * new_node();
   struct ll_queue_node * head;
   struct ll_queue_node * tail;
   std::mutex hmutex;
   std::mutex tmutex;
public:

   ll_queue();
   void enqueue(T val);
   bool dequeue(T * val);
   bool empty();
};

template <typename T>
struct lf_queue<T>::lf_queue_node * lf_queue<T>::new_node() {
   struct lf_queue_node * ret;
   ret = (struct lf_queue_node *)calloc(sizeof(struct lf_queue_node),1);
   ret->nextptr = std::atomic<struct lf_queue_node *>();
   return ret;
}

/*
 * Construct a new queue
 */
template <typename T>
lf_queue<T>::lf_queue() {
   struct lf_queue_node * node = new_node();
   node->nextptr = NULL;
   head = tail = node;
}

/*
 * Add an element to the queue.
 */
template <typename T>
void lf_queue<T>::enqueue(T val) {
   struct lf_queue_node * node = new_node();
   node->val = val;
   node->nextptr = NULL;
   struct lf_queue_node * tail;
   struct lf_queue_node * next;
   for ( ; ; ) {
      tail = this->tail;
      next = tail->nextptr;
      if (tail == this->tail) {
         if (next->nextptr == NULL) {
            if (tail->nextptr.compare_exchange_strong(next,node))
               break;
         }
         else {
            this->tail.compare_exchange_strong(tail,next->nextptr);
         }
      }
   }
   this->tail.compare_exchange_strong(tail,node);
}

/*
 * Remove an element from the queue.
 */
template <typename T>
bool lf_queue<T>::dequeue(T * val) {
   struct lf_queue_node * head;
   struct lf_queue_node * tail;
   struct lf_queue_node * next;
   for ( ; ; ) {
      head = this->head;
      tail = this->tail;
      next = head->nextptr;
      if (head == this->head) {
         if (head->nextptr == tail->nextptr) {
            if (next->nextptr == NULL) {
               return false;
            }
            this->head.compare_exchange_strong(tail,next->nextptr);
         }
         else {
            *val = next->val;
            if (this->head.compare_exchange_strong(head,next))
               break;
         }
      }
   }
   free(head->nextptr);
   return true;
}

/*
 * Check if the queue contains any more elements.
 */
template <typename T>
bool lf_queue<T>::empty() {
   struct lf_queue_node * head;
   struct lf_queue_node * tail;
   struct lf_queue_node * next;
   for ( ; ; ) {
      head = this->head;
      tail = this->tail;
      next = head->nextptr;
      if (head == this->head) {
         if (head->nextptr == tail->nextptr) {
            if (next->nextptr == NULL) {
               return true;
            }
            this->head.compare_exchange_strong(tail,next->nextptr);
         }
         else
            return false;
      }
   }
}

template <typename T>
struct ll_queue<T>::ll_queue_node * ll_queue<T>::new_node() {
   return (struct ll_queue_node *)calloc(sizeof(struct ll_queue_node),1);
}

/*
 * Construct a new queue
 */
template <typename T>
ll_queue<T>::ll_queue()
   : hmutex(std::mutex()), tmutex(std::mutex()) {
   struct ll_queue_node * node = new_node();
   node->nextptr = NULL;
   head = tail = node;
}

/*
 * Add an element to the queue.
 */
template <typename T>
void ll_queue<T>::enqueue(T val) {
   struct ll_queue_node * node = new_node();
   node->val = val;
   node->nextptr = NULL;
   tmutex.lock();
      tail->nextptr = node;
      tail = node;
   tmutex.unlock();
}

/*
 * Remove an element from the queue.
 */
template <typename T>
bool ll_queue<T>::dequeue(T * val) {
   hmutex.lock();
      struct ll_queue_node * node = head;
      struct ll_queue_node * new_head = node->nextptr;
      if (new_head == NULL) {
         hmutex.unlock();
         return false;
      }
      *val = new_head->val;
      head = new_head;
   hmutex.unlock();
   free(node);
   return true;
}

/*
 * Check if the queue contains any more elements.
 */
template <typename T>
bool ll_queue<T>::empty() {
   hmutex.lock();
      if (head->nextptr == NULL) {
         hmutex.unlock();
         return true;
      }
   hmutex.unlock();
   return false;
}

#endif

