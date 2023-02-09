#ifndef _QUEUELINKEDLIST_H_
#define _QUEUELINKEDLIST_H_

#include <iostream>
#include <stdexcept>
#include "QueueNode.h"

template <class T>
class QueueLinkedList {
  private:
    QueueNode<T> *front;
    QueueNode<T> *rear;
    int numElements;

  public:
    QueueLinkedList();
    ~QueueLinkedList();
    int getNumElements();
    void print();
    void enqueue(T value);
    void dequeue();
    T getFront();
    bool isEmpty();

};

template <class T>
QueueLinkedList<T>::QueueLinkedList() {
  //std::cout << "---> Creando un Queue vacio: " << this << std::endl;
  front = nullptr;
  rear = nullptr;
  numElements = 0;
}

template<class T>
QueueLinkedList<T>::~QueueLinkedList() {
  //std::cout << "---> Liberando memoria del Queue: " << this << std::endl;
  QueueNode<T> *p, *q;
  p = front;
  while (p != nullptr) {
    q = p->next;
    delete p;
    p = q;
  }
  front = nullptr;
  rear = nullptr;
  numElements = 0;
}

template <class T> 
int QueueLinkedList<T>::getNumElements() {
  return numElements;
}

template<class T>
void QueueLinkedList<T>::print() {
  QueueNode<T> *ptr = front;
  while (ptr != nullptr) {
      std::cout << ptr->data << " ";
      ptr = ptr->next;
  }
  std::cout << std::endl;
}

template <class T> 
void QueueLinkedList<T>::enqueue(T value) {
  QueueNode<T> *newNode = new QueueNode<T>(value);
  // La fila esta vacia
  if (isEmpty()) {
    front = newNode;
    rear = newNode;
  }
  else {  
    rear->next = newNode;
    rear = newNode;
  }
  numElements++;
} 

template <class T> 
void QueueLinkedList<T>::dequeue() {
  if (isEmpty()) 
    throw std::out_of_range("El Queue esta vacio");
  else {
    QueueNode<T> *p = front;
    front = front->next;
    if (front == nullptr)
      rear = nullptr;
    delete p;
    numElements--;
  }
}

template <class T> 
T QueueLinkedList<T>::getFront() {
  if (isEmpty()) 
    throw std::out_of_range("El Queue esta vacio");
  else
    return front->data;
}


template <class T> 
bool QueueLinkedList<T>::isEmpty() {
  return (front == nullptr);
}



#endif // _QUEUELINKEDLIST_H_
