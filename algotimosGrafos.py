import heapq

def print_graph(graph):
    print("\n--- Representación del Grafo ---")
    for node, edges in graph.items():
        print(f"{node} -> {edges}")
    print()

# Algoritmo de Dijkstra
def dijkstra(graph, start):
    print("\nAlgoritmo de Dijkstra - Camino más corto desde el nodo:", start)
    distances = {node: float('infinity') for node in graph}
    distances[start] = 0
    priority_queue = [(0, start)]
    
    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)
        
        if current_distance > distances[current_node]:
            continue

        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))
    
    return distances

# Algoritmo de Floyd-Warshall
def floyd_warshall(graph):
    print("\nAlgoritmo de Floyd-Warshall - Caminos mínimos entre todos los pares de nodos")
    nodes = list(graph.keys())
    dist = {node: {neighbor: float('infinity') for neighbor in nodes} for node in nodes}
    
    for node in nodes:
        dist[node][node] = 0

    for u in graph:
        for v, weight in graph[u].items():
            dist[u][v] = weight

    for k in nodes:
        for i in nodes:
            for j in nodes:
                dist[i][j] = min(dist[i][j], dist[i][k] + dist[k][j])
    
    return dist

# Algoritmo de Warshall
def warshall(graph):
    print("\nAlgoritmo de Warshall - Cerradura transitiva del grafo")
    nodes = list(graph.keys())
    reach = {node: {neighbor: 0 for neighbor in nodes} for node in nodes}

    for u in graph:
        for v in graph[u]:
            reach[u][v] = 1

    for k in nodes:
        for i in nodes:
            for j in nodes:
                reach[i][j] = reach[i][j] or (reach[i][k] and reach[k][j])
    
    return reach

# Algoritmo de Kruskal
class DisjointSet:
    def __init__(self, n):
        self.parent = list(range(n))
        self.rank = [0] * n
    
    def find(self, u):
        if self.parent[u] != u:
            self.parent[u] = self.find(self.parent[u])
        return self.parent[u]
    
    def union(self, u, v):
        root_u = self.find(u)
        root_v = self.find(v)
        if root_u != root_v:
            if self.rank[root_u] > self.rank[root_v]:
                self.parent[root_v] = root_u
            elif self.rank[root_u] < self.rank[root_v]:
                self.parent[root_u] = root_v
            else:
                self.parent[root_v] = root_u
                self.rank[root_u] += 1

def kruskal(num_nodes, edges):
    print("\nAlgoritmo de Kruskal - Árbol de expansión mínima")
    print("\n--- Lista de aristas del grafo (nodos y pesos) ---")
    for edge in edges:
        print(f"{edge[0]} - {edge[1]} : {edge[2]}")
    edges.sort(key=lambda x: x[2])
    dsu = DisjointSet(num_nodes)
    mst = []
    mst_cost = 0
    
    for u, v, weight in edges:
        if dsu.find(u) != dsu.find(v):
            dsu.union(u, v)
            mst.append((u, v, weight))
            mst_cost += weight
    
    return mst, mst_cost

# Menú Principal
def main_menu():
    while True:
        print("\n--- Menú de Algoritmos de Grafos ---")
        print("1. Algoritmo de Dijkstra (Camino más corto desde un nodo)")
        print("2. Algoritmo de Floyd-Warshall (Camino más corto entre todos los pares)")
        print("3. Algoritmo de Warshall (Cerradura transitiva)")
        print("4. Algoritmo de Kruskal (Árbol de expansión mínima)")
        print("5. Salir")
        
        choice = input("Seleccione una opción: ")
        
        if choice == '1':
            graph = {
                'A': {'B': 1, 'C': 4},
                'B': {'C': 2, 'D': 5},
                'C': {'D': 1},
                'D': {}
            }
            print_graph(graph)
            start_node = input("Ingrese el nodo de inicio: ")
            if start_node in graph:
                distances = dijkstra(graph, start_node)
                print("Distancias más cortas desde el nodo", start_node, ":", distances)
            else:
                print("Nodo de inicio no válido.")

        elif choice == '2':
            graph = {
                'A': {'B': 3, 'C': 8, 'D': -4},
                'B': {'C': 1, 'D': 7},
                'C': {'A': 4},
                'D': {'C': 6}
            }
            print_graph(graph)
            distances = floyd_warshall(graph)
            print("Distancias mínimas entre todos los pares de nodos:")
            for i in distances:
                print(i, distances[i])

        elif choice == '3':
            graph = {
                'A': {'B': 1},
                'B': {'C': 1},
                'C': {'A': 1, 'D': 1},
                'D': {}
            }
            print_graph(graph)
            reachability = warshall(graph)
            print("Cerradura transitiva del grafo:")
            for i in reachability:
                print(i, reachability[i])

        elif choice == '4':
            edges = [
                (0, 1, 1),
                (0, 2, 3),
                (1, 2, 2),
                (1, 3, 4),
                (2, 3, 5)
            ]
            num_nodes = 4
            mst, cost = kruskal(num_nodes, edges)
            print("Aristas del árbol de expansión mínima:", mst)
            print("Costo total del árbol de expansión mínima:", cost)

        elif choice == '5':
            print("Saliendo del programa.")
            break
        
        else:
            print("Opción no válida. Intente nuevamente.")

main_menu()
