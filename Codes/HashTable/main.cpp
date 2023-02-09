 /** Act 5.1 - Implementación individual de operaciones sobre conjuntos
 *
 * Instituto Tecnológico y de Estudios Superiores de Monterrey
 * Campus Guadalajara
 *
 * Guillermo Esquivel Ortiz - A01625621
 * Moisés Hiram Pineda Campos - A01625510
 * Alma Paulina González Sandoval - A01631256
 *
 * 4 de Febrero del 2023
 *
 *****************************************************************
 *
 * Compilacion para debug:
 *    g++ -std=c++17 -g -o main *.cpp
 * Ejecucion con valgrind:
 *    nix-env -iA nixpkgs.valgrind
 *    valgrind --leak-check=full ./main
 *
 * Compilacion para ejecucion:
 *    g++ -std=c++17 -O3 -o main *.cpp
 * Ejecucion:
 *    ./main
 **/

#include <iostream>
#include <string>
#include "OAHashTable.h"

int main() {
  OAHashTable<std::string> myOAHT(7);
  //myOAHT.setMaxSize(7);
  
  //------------------------Add---------------------------------
  //Complejidad Computacional: O(1)    
  std::cout << "<---- Metodo Add ----> " << std::endl;
  myOAHT.add(5, "Hola");
  myOAHT.add(6, "Memo");
  myOAHT.add(15, "Moy");
  myOAHT.add(7, "Pau");
  myOAHT.print();

  //-------------------------Find--------------------------------
  //Complejidad Computacional: O(n)
  std::cout << "\n<---- Metodo Find ----> " << std::endl;
  std::cout << "Llave 5: " << myOAHT.find(5) << std::endl;

  //-------------------------Remove------------------------------
  //Complejidad Computacional: O(n)
  std::cout << "\n<---- Metodo Remove ----> " << std::endl;
  std::cout << "Eliminamos llave 7" << std::endl;
  myOAHT.remove(7);
  
  //-------------------------Print-----------------------------
  //Complejidad Computacional: O(n)
  std::cout << "\n<---- Metodo Print ----> " << std::endl;
  myOAHT.print();

  return 0;
}