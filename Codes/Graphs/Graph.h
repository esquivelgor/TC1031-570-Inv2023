#ifndef _GRAPH_H
#define _GRAPH_H

#include "LinkedList.h"
#include "QueueLinkedList.h"
#include "StackLinkedList.h"
#include <iostream>
#include <queue>
#include <set>
#include <string>
#include <utility>
#include <vector>

#define INF 0x3f3f3f3f

class Graph {
private:
  int numNodes;
  int numEdges;
  // Lista de adyacencia (vector de listas de pares (vertice, peso))
  std::vector<LinkedList<std::pair<int, int>>> adjList;

  void split(std::string line, std::vector<int> &res);
  void topologicalSortUtility(int v, std::set<int> &visited,
                              StackLinkedList<int> &stackResult);

public:
  Graph();
  ~Graph();
  void loadDirWeightedGraph(std::istream &input);
  void print();
  void BFS(int v);
  void DFS(int v);
  void topologicalSort();
  void dijkstraAlgorithm(int v);
};

#endif // _GRAPH_H
