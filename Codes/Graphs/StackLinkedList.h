#ifndef _STACKLINKEDLIST_H_
#define _STACKLINKEDLIST_H_

#include "StackNode.h"
#include <iostream>
#include <stdexcept>

template <class T> class StackLinkedList {
private:
  StackNode<T> *top;
  StackNode<T> *tail;
  int numElements;

public:
  StackLinkedList();
  ~StackLinkedList();
  int getNumElements();
  void print();
  void push(T value);
  void pop();
  T getTop();
  bool isEmpty();
};

template <class T> StackLinkedList<T>::StackLinkedList() {
  // std::cout << "---> Creando un Stack vacio: " << this << std::endl;
  top = nullptr;
  tail = nullptr;
  numElements = 0;
}

template <class T> StackLinkedList<T>::~StackLinkedList() {
  // std::cout << "---> Liberando memoria del Stack: " << this << std::endl;
  StackNode<T> *p, *q;
  p = top;
  while (p != nullptr) {
    q = p->next;
    delete p;
    p = q;
  }
  top = nullptr;
  tail = nullptr;
  numElements = 0;
}

template <class T> int StackLinkedList<T>::getNumElements() {
  return numElements;
}

template <class T> void StackLinkedList<T>::print() {
  StackNode<T> *ptr = top;
  while (ptr != nullptr) {
    std::cout << ptr->data << " ";
    ptr = ptr->next;
  }
  std::cout << std::endl;
}

template <class T> void StackLinkedList<T>::push(T value) {
  // 1. crea un nuevoNodo
  StackNode<T> *newNode = new StackNode<T>(value);
  // 2. apunto nuevoNodo->next a donde esta head
  newNode->next = top;
  // 3. Actualizo head para que apunte al nuevoNodo
  top = newNode;
  if (numElements == 0)
    tail = newNode;
  numElements++;
}

template <class T> void StackLinkedList<T>::pop() {
  if (isEmpty())
    throw std::out_of_range("El Stack esta vacio");
  else {
    StackNode<T> *p = top;
    top = p->next;
    delete p;
    numElements--;
  }
}

template <class T> T StackLinkedList<T>::getTop() {
  if (isEmpty())
    throw std::out_of_range("El Stack esta vacio");
  else
    return top->data;
}

template <class T> bool StackLinkedList<T>::isEmpty() {
  return (top == nullptr);
}

#endif // _STACKLINKEDLIST_H_
