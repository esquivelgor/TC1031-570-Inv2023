/** Act 5.2 - Actividad Integral sobre el uso de códigos hash (Evidencia Competencia)
 *
 * Instituto Tecnológico y de Estudios Superiores de Monterrey
 * Campus Guadalajara
 *
 * Guillermo Esquivel Ortiz - A01625621
 * Moisés Hiram Pineda Campos - A01625510
 * Alma Paulina González Sandoval - A01631256
 *
 * 7 de Febrero del 2023
 *
 *****************************************************************
 *
 * Compilacion para debug:
 *    g++ -std=c++17 -g -o main *.cpp
 * Ejecucion con valgrind:
 *    nix-env -iA nixpkgs.valgrind
 *    valgrind --leak-check=full ./main < TestCases/graph01.txt
 *
 * Compilacion y ejecucion:
 *    g++ -std=c++17 -Wall *.cpp && ./a.out < bitacoraGrafos.txt
 **/

#include "Graph.h"
#include "IpData.h"
#include <iostream>
#include <sstream>

int main() {
  std::stringstream inputInfo;
  inputInfo << std::cin.rdbuf();
  Graph g1;
  std::string ip;

  // <--------------> Cargar Gráfica <------------------------>
  // Complejidad Computacional: O(N)
  g1.loadDirWeightedGraph(inputInfo);

  // <-------------> Procesar la Información<------------->
  // Complejidad Computacional: O(N)
  g1.processData();


  int coll = g1.hashTable();
  std::cout << "Colisiones totales: " << coll << std::endl;

  // <-------------> Algoritmo getIpSummary (Uso de ) <------------------->
  // Complejidad Computacional: O((V+E)logV)  
  g1.getIpSummary("36.50.52.170");  
  return 0;
}