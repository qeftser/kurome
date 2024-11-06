
#ifndef KUROME_BINARY_HEAP

#define KUROME_BINARY_HEAP
#include <vector>
#include <cstdint>
#include <cstdlib>
#include <unordered_map>
#include <cstdio>

template<typename V>
class binary_heap {
public:

   binary_heap() {
      elements = std::vector<pair *>();
      members = std::unordered_map<uint64_t,pair *>();
      total = 0;
   }

   ~binary_heap() {
   }

   void insert(uint64_t key, V val) {
      struct pair * nev = (struct pair *)malloc(sizeof(struct pair));
      nev->key = key; nev->pos = total; nev->val = val;
      elements.push_back(nev);
      members[elements[total]->key] = elements[total];
      up_heap(total);
      ++total;
   }

   V extract(void) {
      struct pair * temp;
      --total;
      temp = elements[0];
      elements[0] = elements[total];
      members.erase(elements[total]->key);
      elements.pop_back();
      down_heap(0);
      V ret = temp->val;
      free(temp);
      return ret;
   }

   void update_key(uint64_t old, uint64_t nev) {
      struct pair * curr = members.at(old);
      curr->key = nev;
      if (old > nev)
         up_heap(curr->pos);
      else
         down_heap(curr->pos);
   }

   bool contains(uint64_t key) {
      return members.count(key);
   }

   bool empty() {
      return !total;
   }

   void print() {
      printf("<");
      for (pair * p : elements) {
         printf("%ld ",p->key);
      }
      printf(">\n");
   }

private:

   struct pair {
      uint64_t key;
      int pos;
      V val;
   };

   std::vector<pair *> elements;
   std::unordered_map<uint64_t,pair *> members;
   struct pair * temp;
   int total;

   inline const int left(const int i) const {
      return (i+i)+1;
   }

   inline const int right(const int i) const {
      return (i+i)+2;
   }

   inline const int parent(const int i) const {
      return (i-1)>>1;
   }

   inline const void up_heap(const int i) {
      if (!i)
         return;
      if (elements[i]->key < elements[parent(i)]->key) {
         temp = elements[i];
         elements[i] = elements[parent(i)];
         elements[parent(i)] = temp;
         elements[parent(i)]->pos = elements[i]->pos;
         elements[i]->pos = temp->pos;
         up_heap(parent(i));
      }
   }

   inline const void down_heap(const int i) {
      if (left(i) >= total)
         return;
      if (right(i) >= total) {
         if (elements[left(i)]->key < elements[i]->key) {
            temp = elements[left(i)];
            elements[left(i)] = elements[i];
            elements[i] = temp;
            elements[i]->pos = elements[left(i)]->pos;
            elements[left(i)]->pos = temp->pos;
            down_heap(left(i));
         }
      }
      else {
         if (elements[left(i)]->key < elements[i]->key) {
            if (elements[right(i)]->key < elements[left(i)]->key) {
               temp = elements[right(i)];
               elements[right(i)] = elements[i];
               elements[i] = temp;
               elements[i]->pos = elements[right(i)]->pos;
               elements[right(i)]->pos = temp->pos;
               down_heap(right(i));
            }
            else {
               temp = elements[left(i)];
               elements[left(i)] = elements[i];
               elements[i] = temp;
               elements[i]->pos = elements[left(i)]->pos;
               elements[left(i)]->pos = temp->pos;
               down_heap(left(i));
            }
         }
         else {
            if (elements[right(i)]->key < elements[i]->key) {
               temp = elements[right(i)];
               elements[right(i)] = elements[i];
               elements[i] = temp;
               elements[i]->pos = elements[right(i)]->pos;
               elements[right(i)]->pos = temp->pos;
               down_heap(right(i));
            }
         }
      }
   }
};

#endif
