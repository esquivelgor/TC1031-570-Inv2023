#ifndef _OAHASHTABLE_H_
#define _OAHASHTABLE_H_

#include <vector>
#include <utility>
#include "LinkedList.h"

template <class T> 
class OAHashTable {
  private:
    std::vector<LinkedList<std::pair<int, T>>> table;
    int numElements;
    // Preferible usar un numero primo
    // https://numerosprimos.org/numeros-primos-de-1-a-100000/
    int maxSize;

  public:
    OAHashTable();
    OAHashTable(int selectedMaxSize);
    ~OAHashTable();
    void setMaxSize(int selectedMaxSize);
    int getHashIndex(int keyVal); // funcion hash
    int getNumElements();
    void print();
    void add(int keyValue, T dataValue);
    T find(int keyValue);
    void remove(int keyValue);
};

template <class T>
OAHashTable<T>::OAHashTable() {
  maxSize = 0;
  numElements = 0;
  table.resize(maxSize);
}

template <class T>
OAHashTable<T>::~OAHashTable() {
  maxSize = 0;
  numElements = 0;
  table.resize(maxSize);
}

template <class T>
OAHashTable<T>::OAHashTable(int selectedMaxSize) {
  maxSize = selectedMaxSize;
  numElements = 0;
  table.resize(maxSize);
}

template <class T>
void OAHashTable<T>::setMaxSize(int selectedMaxSize) {
  maxSize = selectedMaxSize;
  numElements = 0;
  table.resize(maxSize);
}

template <class T>
int OAHashTable<T>::getHashIndex(int keyVal) {
  // metodo de residuales (llave mod maxSize)
  return keyVal % maxSize;
}

template<class T>
int OAHashTable<T>::getNumElements() {
  return numElements;
}

template<class T>
void OAHashTable<T>::add(int keyValue, T dataValue) {
  int index = getHashIndex(keyValue);
  table[index].addLast(std::make_pair(keyValue, dataValue));  
}

template<class T>
void OAHashTable<T>::print() {
  for(int i = 0; i < maxSize; i++){
    std::cout << "Celda " << i << ": ";
    NodeLinkedList<std::pair<int, T>> *p = table[i].getHead();
    while (p != nullptr) {
      std::pair<int, T> par = p->data;
      std::cout << "{" << par.first << ", " << par.second << "} ";
      p = p->next;
    }
    std::cout << std::endl;
  }
}

template <class T> 
T OAHashTable<T>::find(int keyValue) {
  int index = getHashIndex(keyValue);
  NodeLinkedList<std::pair<int, T>> *p = table[index].getHead();
  
  while (p != nullptr) {
    std::pair<int, T> pair = p->data;
    if (pair.first == keyValue) {
      return pair.second;
    }
    p = p->next;
  }
  return {};
}

template <class T> 
void OAHashTable<T>::remove(int keyValue){
  int index = getHashIndex(keyValue);
  NodeLinkedList<std::pair<int, T>> *p = table[index].getHead(); 
  
  while (p!=nullptr){ 
    std::pair<int, T> pair = p->data;
    if(pair.first == keyValue){
      table[index].deleteData(pair);
    }
    
    p = p->next;
  }
}

#endif  // _OAHASHTABLE_H_