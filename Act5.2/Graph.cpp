#include "Graph.h"
#include "Ip.h"
#include "IpData.h"
#include <iterator>
#include <unistd.h>
#include <utility>

Graph::Graph() {
  numNodes = 0;
  numEdges = 0;
  adjList.clear();
}

Graph::~Graph() {
  numNodes = 0;
  numEdges = 0;
  adjList.clear();
}

// Complejidad computacional O(N)
void Graph::splitStr(std::string line, std::vector<std::string> &vec,
                     std::string deli) {
  size_t strPos = line.find(deli);
  size_t lastPos = 0;
  while (strPos != std::string::npos) {
    vec.push_back(line.substr(lastPos, strPos - lastPos));
    lastPos = strPos + 1;
    strPos = line.find(deli, lastPos);
  }
  vec.push_back(line.substr(lastPos, line.size() - lastPos));
}

// Reference Asesoria 4 de Febrero
// Complejidad computacional O(N)
int Graph::findIpIndex(std::string ipString) {
  Ip tmpIp(ipString, 0);
  std::map<unsigned int, Ip>::iterator it;
  int ipIndex = -1;
  it = mapIp.find(tmpIp.getIpValue());
  if (it != mapIp.end())
    ipIndex = it->second.getIpIndex();
  return ipIndex;
}

// Complejidad computacional O(N)
void Graph::loadDirWeightedGraph(std::istream &input) {
  std::string line;
  int i = 0;
  while (std::getline(input, line)) {
    if (i == 0) {
      std::vector<std::string> result;
      splitStr(line, result, " ");
      numNodes = stoi(result[0]);
      numEdges = stoi(result[1]);

      adjList.resize(numNodes + 1);
      listIp.resize(numNodes + 1);
      for (int k = 1; k <= numNodes; k++) {
        LinkedList<std::pair<int, int>> tmpList;
        adjList[k] = tmpList;
      }
    } else if (i > 0 && i <= numNodes) {
      Ip nodo(line, i);
      mapIp.insert({nodo.getIpValue(), nodo});
    } else if (i > numNodes) {
      // Reading edges of the graph
      std::vector<std::string> result, date, ipPort1, ipPort2;
      std::string tmp;
      // Separate each space from the line
      splitStr(line, result, " ");
      // Separate hours/min/sec
      splitStr(result[2], date, ":");
      // Separate each ip and port
      splitStr(result[3], ipPort1, ":");
      splitStr(result[4], ipPort2, ":");

      std::stringstream ss;
      for (auto it = result.begin() + 6; it != result.end(); it++) {
        if (it != result.begin()) {
          ss << " ";
        }
        ss << *it;
      }
      Ip ip1(ipPort1[0], 0);

      int ip1Index = findIpIndex(ipPort1[0]);
      int ip2Index = findIpIndex(ipPort2[0]);
      if (ip1Index != -1 && ip2Index != -1) {
        adjList[ip1Index].addLast({ip2Index, stoi(result[5])});
        mapIp[ip1.getIpValue()].addToOutDegree();
      } else {
        throw std::out_of_range("Incorrect IPs in the edge");
      }
    }
    i++;
  }
}

// Complejidad computacional O(N**2)
void Graph::print() {
  std::cout << "numNode: " << numNodes << std::endl;
  std::cout << "numEdges: " << numEdges << std::endl;
  std::cout << "Adjacency List" << std::endl;
  for (int nodeU = 1; nodeU <= numNodes; nodeU++) {
    std::cout << "vertex " << nodeU << ": ";
    NodeLinkedList<std::pair<int, int>> *ptr = adjList[nodeU].getHead();
    while (ptr != nullptr) {
      std::pair<int, int> par = ptr->data;
      std::cout << "{" << par.first << "," << par.second << "} ";
      ptr = ptr->next;
    }
    std::cout << std::endl;
  }
}

// Complejidad computacional O(n)
void Graph::heap() {
  ipHeap.setCapacity(numNodes);
  for (ptr = mapIp.begin(); ptr != mapIp.end(); ptr++) {
    IpData val(ptr->second);
    ipHeap.push(val);
  }
}

// Complejidad computacional O(n)
void Graph::processData() {
  heap();
  for (int i = 0; i < numNodes - 1; i++) {
    /* Para obtener grados_ips.txt ejecutar el sh, quitar el comentario debajo y
     comentar el contenido del if */

    /* g++ -std=c++17 -Wall *.cpp && ./a.out < bitacoraGrafos.txt >
     grados_ips.txt */
    // std::cout << ipHeap.top() << std::endl;

    // Mayores grados_ips.txt
    /* g++ -std=c++17 -Wall *.cpp && ./a.out < bitacoraGrafos.txt >
     mayores_grados_ips.txt */
    // if (i < 5) {
    //   std::cout << "Top " << i + 1 << ": ";
    //   std::cout << ipHeap.top() << std::endl;
    // }

    // Bootsmaster
    if (i == 0) {
      std::cout << "Bootsmaster: ";
      std::cout << ipHeap.top() << std::endl;
      IpData val = ipHeap.top();
      indexBM = findIpIndex(val.getIp());
    }
    ipHeap.pop();
  }
}

// Complejidad Computacional: O((V+E)logV)
void Graph::dijkstraAlgorithmBM() {
  // Crear una priority queue del STL de C++
  // https://www.geeksforgeeks.org/implement-min-heap-using-stl/
  std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>,
                      std::greater<std::pair<int, int>>>
      pq;
  // vector de distancias con el resultado del algoritmo
  std::vector<int> dist(numNodes + 1, INF);
  // Insertar el nodo de origen v en el priority queue (pares dist, vertice)
  // e inicializar su distancia a cero
  pq.push(std::make_pair(0, indexBM)); // pares (dist, vertice)
  dist[indexBM] = 0;
  // Mientras el priority queue no este vacio
  while (!pq.empty()) {
    // Extraemos un vertice del priority queue
    int nodeU = pq.top().second; // pares (dist, vertice)
    pq.pop();
    // Obtener los vecinos del vertice nodeU
    NodeLinkedList<std::pair<int, int>> *ptr = adjList[nodeU].getHead();
    while (ptr != nullptr) {
      std::pair<int, int> par = ptr->data;
      int nodeV = par.first; // nodeV es vecino de nodeU
      int peso = par.second; // peso de la arista (nodeU, nodeV)
      // Si hay un camino mas corto hacia nodeV pasando por nodeU
      if (dist[nodeV] > dist[nodeU] + peso) {
        // Updating distance of v
        dist[nodeV] = dist[nodeU] + peso;
        pq.push(std::make_pair(dist[nodeV], nodeV));
      }
      ptr = ptr->next;
    }
  }
  /* Para obtener distancia_boostmaster.txt ejecutar el sh, quitar el comentario
     debajo y comentar el contenido proximo a este */

  /* g++ -std=c++17 -Wall *.cpp && ./a.out < bitacoraGrafos.txt >
   distancia_boostmaster.txt */

  // std::cout << "Vertex\tDistance from Boost Master" << std::endl;

  // for (ptr = mapIp.begin(); ptr != mapIp.end(); ptr++) {
  //   std::cout << "Ip: " << ptr->second.getIp();
  //   std::cout << " | Distancia: " << dist[ptr->second.getIpIndex()]
  //             << std::endl;
  // }
  int mD = -1, mI = -1;
  std::vector<std::pair<std::string, int>> ipD;
  for (ptr = mapIp.begin(); ptr != mapIp.end(); ptr++) {
    ipD.push_back(
        make_pair(ptr->second.getIp(), dist[ptr->second.getIpIndex()]));
    if (dist[ptr->second.getIpIndex()] > mD) {
      mD = dist[ptr->second.getIpIndex()];
      mI = ptr->second.getIpIndex();
    }
  }
  std::cout << "Ip con menor probabilidad a ataque: " << ipD[mI - 1].first
            << " a " << mD << " unidades de distancia." << std::endl;
}

void Graph::dijkstraAlgorithm(int v) {
  // Crear una priority queue del STL de C++
  // https://www.geeksforgeeks.org/implement-min-heap-using-stl/
  std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>,
                      std::greater<std::pair<int, int>>>
      pq;
  // vector de distancias con el resultado del algoritmo
  std::vector<int> dist(numNodes + 1, INF);
  // Insertar el nodo de origen v en el priority queue (pares dist, vertice)
  // e inicializar su distancia a cero
  pq.push(std::make_pair(0, v)); // pares (dist, vertice)
  dist[v] = 0;
  // Mientras el priority queue no este vacio
  while (!pq.empty()) {
    // Extraemos un vertice del priority queue
    int nodeU = pq.top().second; // pares (dist, vertice)
    pq.pop();
    // Obtener los vecinos del vertice nodeU
    NodeLinkedList<std::pair<int, int>> *ptr = adjList[nodeU].getHead();
    while (ptr != nullptr) {
      std::pair<int, int> par = ptr->data;
      int nodeV = par.first; // nodeV es vecino de nodeU
      int peso = par.second; // peso de la arista (nodeU, nodeV)
      // Si hay un camino mas corto hacia nodeV pasando por nodeU
      if (dist[nodeV] > dist[nodeU] + peso) {
        // Updating distance of v
        dist[nodeV] = dist[nodeU] + peso;
        pq.push(std::make_pair(dist[nodeV], nodeV));
      }
      ptr = ptr->next;
    }
  }
  // Imprimir las distancias mas cortas entre v y todos los nodos del grafo
  std::cout << "Vertex\tDistance from source" << std::endl;
  for (int i = 1; i <= numNodes; i++)
    if (dist[i] != INF) {
      std::cout << i << "\t" << dist[i] << std::endl;
    } else
      std::cout << i << "\tINF" << std::endl;
}

int Graph::hashTable() {
  hT.setMaxSize(maxSizeHT);
  for (ptr = mapIp.begin(); ptr != mapIp.end(); ptr++) {
    hT.add(ptr->first, ptr->second);
  }
  return hT.getColl();
}

void Graph::getIpSummary(std::string key) {
  Ip ip(key, 0);
  int n = hT.find(ip.getIpValue());
  if (n != -1) {
    std::cout << "Ip: ";
    hT.printN(n);
    for (ptr = mapIp.begin(); ptr != mapIp.end(); ptr++) {
      if (ptr->second.getIp() == key) {
        dijkstraAlgorithm(ptr->second.getIpIndex());
        continue;
      }
    }
  } else
    std::cout << "Ip no encontrada" << std::endl;
}
