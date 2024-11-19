# Implementacion del algoritmo A* para encontrar la distancia
# Mas corta entre dos nodos en un grafo ponderado

# Autor: Pedro Sotelo Arce
# Marticula: A01285371

# Ultima fecha de modificacion
# 18/11/2024

# Importamos librerias necesarias

import networkx as nx
import math
from heapq import heappush, heappop
import matplotlib.pyplot as plt

def leer_grafo(filename):
    """
    Leer un grafo dirigido desde un archivo de texto
    Paramentros
    - Filename (str) path al archivo
    - G (networkx.DiGraph) un grafo dirigido con posiciones y pesos
    """

    G = nx.DiGraph()
    with open(filename,'r') as file:
        lines = file.readlines()

        # Parsear los nodos
        selecion_Nodo = True
        for line in lines:
            line = line.strip()
            if not line or line == "# Nodes:":
                continue # Saltarse lineas vacias o la de nodos

            if line == "# Edges:":
                selecion_Nodo = False
                continue

            if selecion_Nodo: # Asignar los valores como vienen del generador
                node_id, x, y = line.split()
                G.add_node(int(node_id), pos = (float(x), float(y)))
            else:
                u, v, weight = line.split()
                G.add_edge(int(u), int(v), weight=float(weight))

    return G

# Definir la heuristica (Distancia euclidiana)
def heuristica(nodo, goal, G):
    """
    Calcular la heuristica, en este caso la distancia entre 2 nodos
    Parametros
    - Nodo (int) nodo actual
    - Goal (int) nodo objetivo
    - G la grafica

    Retorna un float que contiene la distancia entre nodo y goal
    """

    x1, y1 = G.nodes[nodo]['pos']
    x2, y2 = G.nodes[goal]['pos']

    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Implementacion del algortimo A*

def a_star(G, inicio, final):
    """
    Utilizando el algoritmo a estrella calculara la ruta mas corta en un grafo
    Parametros:
    G : el grafo
    Inicio: Nodo inicial
    Final: Nodo final
    Retorna la ruta mas corta en forma de lista
    """
    open_set = [] # Cola de prioridad
    heappush(open_set, (0, inicio))
    came_from = {}
    g_score = {node: float('inf') for node in G.nodes}
    g_score[inicio] = 0
    f_score = {node: float('inf') for node in G.nodes}
    f_score[inicio] = heuristica(inicio, final, G)

    while open_set:
        _, current = heappop(open_set)

        if current == final:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(inicio)
            return path[::-1] 
        
        for neighbor in G.neighbors(current):
            tentativeG = g_score[current] + G[current][neighbor]['weight']

            if tentativeG < g_score[neighbor]:
                # Actualizar la lista (path) y los costos
                came_from[neighbor] = current
                g_score[neighbor] = tentativeG
                f_score[neighbor] = tentativeG + heuristica(neighbor, final, G)

                # Agregar vecino a open set
                if neighbor not in [item[1] for item in open_set]:
                    heappush(open_set, (f_score[neighbor], neighbor))

    return None

# Graficamos

def plot_path(G, path):
    """
    Funcion para graficar y visualizar el path elegido
    Parametros:
    - G : el grafo
    - path : La lista de la ruta mas corta
    """

    pos = nx.get_node_attributes(G,'pos')
    edge_labels = {(u,v): f"{d['weight']:.2f}" for u, v, d in G.edges(data = True)}

    plt.figure(figsize=(10,7))
    nx.draw(G, pos, with_labels=True, node_size=500, node_color='lightblue', arrows=True)
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)

    if path:
        path_edges = list(zip(path, path[1:]))
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='red', width=2)

    plt.title("Grafica con el camino mas corto")
    plt.show()

# Probando el correcto funcionamiento del programa

filename = "directed_graph.txt"
G = leer_grafo(filename)

# Comprobacion adicional
print("Nodos:")
print(G.nodes(data=True))
print("Aristas:")
print(G.edges(data=True))

# Definimos nodo inicial y final
startNode = 2
finalNode = 7

# Ejecutamos A*
path = a_star(G, startNode, finalNode)

if path:
    print(f"La ruta mas corta desde {startNode} a  {finalNode}: {path}")

else:
    print("No se encontro ruta")

# Visualizar el grafo 
plot_path(G, path)
