/**
 * Ejemplo que implementa Grafos ponderados dirigidos representados
 * como una Lista de adyacencia y sus recorridos BFS y DFS
 *
 * Compilacion para debug:
 *    g++ -std=c++17 -g -o main *.cpp
 * Ejecucion con valgrind:
 *    nix-env -iA nixpkgs.valgrind
 *    valgrind --leak-check=full ./main < TestCases/graph01.txt
 *
 * Compilacion para ejecucion:
 *    g++ -std=c++17 -O3 -o main *.cpp
 * Ejecucion:
 *    ./main < TestCases/test01.txt
 **/

#include "Graph.h"
#include <iostream>
#include <sstream>

int main() {

  std::stringstream inputInfo;
  inputInfo << std::cin.rdbuf();
  // Construye un grafo a partir de la consola usando
  // representacion de Lista de adyacencia
  Graph g1;
  g1.loadDirWeightedGraph(inputInfo);
  g1.print();
  //g1.BFS(1);
  //g1.DFS(1);
  g1.topologicalSort();
  g1.dijkstraAlgorithm(7);
  return 0;
}