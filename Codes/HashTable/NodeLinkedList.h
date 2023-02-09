#ifndef _NODELINKEDLIST_H_
#define _NODELINKEDLIST_H_

template <class T>
class NodeLinkedList {
  public:
    T data;
    NodeLinkedList<T> *next;

    NodeLinkedList();
    NodeLinkedList(T value);
};
//O(1)
template <class T>
NodeLinkedList<T>::NodeLinkedList() : data{}, next{nullptr} {}
//O(2)
template <class T>
NodeLinkedList<T>::NodeLinkedList(T value) : data{value}, next{nullptr} {}


#endif // _NODELINKEDLIST_H_


