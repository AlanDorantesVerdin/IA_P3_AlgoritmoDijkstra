import matplotlib.pyplot as plt
import networkx as nx
import heapq


class DijkstraVisual:
    """Clase para ejecutar y visualizar paso a paso el algoritmo de Dijkstra."""

    def __init__(self, graph, start, end):
        # Guardar referencias básicas
        self.graph = graph
        self.start = start
        self.end = end

        # Lista donde acumularemos estados (snapshots) del algoritmo
        # Cada snapshot es un diccionario con claves: 'current','visited','distances',...
        self.steps = []
        # Índice del paso mostrado actualmente
        self.current_step = 0

    # === Implementación del algoritmo de Dijkstra con captura de pasos ===
    def dijkstra_with_steps(self):
        """Ejecuta Dijkstra y guarda pasos intermedios para visualización."""

        # ------------------ PASO 1: Inicialización ------------------
        # Inicializamos todas las distancias a infinito salvo el nodo inicial
        distances = {node: float('inf') for node in self.graph.nodes()}
        distances[self.start] = 0

        # 'previous' almacenará el predecesor inmediato en el camino más corto
        previous = {node: None for node in self.graph.nodes()}

        # Cola de prioridad (min-heap) con tuplas (distancia_acumulada, nodo)
        # heapq permite extraer siempre el nodo con menor distancia conocida.
        pq = [(0, self.start)]

        # Conjunto de nodos completamente procesados (no se revisarán sus aristas otra vez)
        visited = set()

        # Guardar snapshot inicial para la visualización
        self.steps.append({
            'current': None,               # nodo actualmente en foco (None al inicio)
            'visited': set(),              # nodos ya procesados
            'distances': distances.copy(), # distancias actualmente conocidas
            'previous': previous.copy(),   # predecesores para reconstruir caminos
            'queue': pq.copy(),            # contenido actual de la cola (orden ascendente por distancia)
            'message': f'Inicio: Nodo origen = {self.start}'
        })

        # ------------------ PASO 2: Bucle principal ------------------
        while pq:
            # Obtener el nodo con la menor distancia acumulada
            current_distance, current_node = heapq.heappop(pq)

            # Si el nodo ya fue procesado (puede haber entradas duplicadas en la cola), ignorarlo
            if current_node in visited:
                continue

            # Marcar como procesado
            visited.add(current_node)

            # Guardar snapshot tras extraer el nodo
            self.steps.append({
                'current': current_node,
                'visited': visited.copy(),
                'distances': distances.copy(),
                'previous': previous.copy(),
                'queue': pq.copy(),
                'message': f'Visitando nodo: {current_node} (distancia: {current_distance})'
            })

            # Si alcanzamos el nodo destino, terminamos pronto (optimización válida para grafos no dirigidos/positivos)
            if current_node == self.end:
                break

            # ------------------ PASO 3: Relajación de aristas ------------------
            # Revisar cada vecino del nodo actual
            for neighbor in self.graph.neighbors(current_node):
                # Saltar vecinos ya procesados
                if neighbor in visited:
                    continue

                # Obtener el peso de la arista (se asume que existe la clave 'weight')
                weight = self.graph[current_node][neighbor]['weight']

                # Distancia alternativa pasando por current_node
                distance = current_distance + weight

                # Si encontramos una distancia más corta, la actualizamos y empujamos a la cola
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current_node
                    heapq.heappush(pq, (distance, neighbor))

                    # Guardar snapshot que muestra la actualización de un vecino concreto
                    self.steps.append({
                        'current': current_node,
                        'visited': visited.copy(),
                        'distances': distances.copy(),
                        'previous': previous.copy(),
                        'queue': pq.copy(),
                        'edge_highlight': (current_node, neighbor),
                        'message': f'Actualizando {neighbor}: nueva distancia = {distance}'
                    })

        # ------------------ PASO 4: Reconstrucción del camino más corto ------------------
        # Reconstruimos la ruta desde `end` hacia `start` usando `previous`
        path = []
        node = self.end
        while node is not None:
            path.append(node)
            node = previous[node]
        path.reverse()  # invertir para tener origen -> destino

        # Guardar snapshot final con el camino encontrado y la distancia total
        self.steps.append({
            'current': None,
            'visited': visited.copy(),
            'distances': distances.copy(),
            'previous': previous.copy(),
            'queue': [],
            'final_path': path,
            'message': f'Camino más corto: {" -> ".join(map(str, path))} (distancia: {distances[self.end]})'
        })

        return path, distances[self.end]

    def visualize(self):
        """Prepara la visualización y muestra la interfaz interactiva."""

        # Ejecutar algoritmo y capturar pasos
        self.dijkstra_with_steps()

        # Crear figura con tema oscuro (colores definidos más abajo)
        self.fig, self.ax = plt.subplots(figsize=(14, 10), facecolor='#2b2b2b')
        self.ax.set_facecolor('#2b2b2b')

        # Registrar manejador de eventos (flecha derecha, 'q', etc.)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        # Calcular posiciones de los nodos (layout) para que el grafo no cambie entre pasos
        self.pos = nx.spring_layout(self.graph, seed=42, k=2)

        # Dibujar el primer paso
        self.draw_step()
        plt.tight_layout()
        plt.show()

    def draw_step(self):
        """Dibuja el paso actual tomando la información desde `self.steps[self.current_step]`."""

        # Limpiar e iniciar el lienzo
        self.ax.clear()

        # Asegurar que el índice de paso está en rango
        if self.current_step >= len(self.steps):
            self.current_step = len(self.steps) - 1

        step = self.steps[self.current_step]

        # Título general
        self.ax.set_title(f"\n\n\nAlgoritmo de Dijkstra\n\n",
                         fontsize=16, fontweight='bold', color="#837B7B")

        # Texto principal con número de paso y mensaje descriptivo
        self.ax.text(0.5, 0.95, f"\n\n\nPaso {self.current_step + 1}/{len(self.steps)}\n{step['message']}\n\n",
                     transform=self.ax.transAxes, ha='center', fontsize=12, color="#837B7B", fontweight='bold')

        # Instrucciones cortas en la esquina inferior
        self.ax.text(0.1, 0.05, "[Presiona → para avanzar, 'q' para salir]",
                     transform=self.ax.transAxes, ha='center', fontsize=10, color="#837B7B")

        # Determinar color de cada nodo según su estado en este paso
        node_colors = []
        for node in self.graph.nodes():
            # Si estamos en el snapshot final y el nodo está en el camino final
            if 'final_path' in step and node in step['final_path']:
                node_colors.append('#FFD700')    # dorado
            elif node == self.start:
                node_colors.append("#00FF80A8")  # verde (origen)
            elif node == self.end:
                node_colors.append('#FF1493')    # rosa (destino)
            elif node == step['current']:
                node_colors.append('#FF8C00')    # naranja (nodo en foco)
            elif node in step['visited']:
                node_colors.append('#4169E1')    # azul (visitados)
            else:
                node_colors.append('#696969')    # gris (no visitados)

        # Preparar colores y grosores de las aristas (resaltar camino o arista evaluada)
        edges = self.graph.edges()
        edge_colors = []
        edge_widths = []

        for edge in edges:

            # Si el snapshot contiene un 'final_path', resaltamos las aristas que forman el camino final
            if 'final_path' in step:
                path = step['final_path']
                if any((path[i], path[i+1]) == edge or (path[i+1], path[i]) == edge for i in range(len(path)-1)):
                    edge_colors.append('#FFD700')
                    edge_widths.append(4)
                else:
                    edge_colors.append('#555555')
                    edge_widths.append(1)

            # Si el snapshot resalta una arista en evaluación
            elif 'edge_highlight' in step and (edge == step['edge_highlight'] or edge == (step['edge_highlight'][1], step['edge_highlight'][0])):
                edge_colors.append('#FF4500')
                edge_widths.append(3)
            else:
                edge_colors.append('#555555')
                edge_widths.append(1)

        # Dibujar aristas y nodos con networkx
        nx.draw_networkx_edges(self.graph, self.pos, ax=self.ax,
                               edge_color=edge_colors, width=edge_widths, alpha=0.6)

        nx.draw_networkx_nodes(self.graph, self.pos, ax=self.ax,
                               node_color=node_colors, node_size=800,
                               edgecolors='white', linewidths=2)

        # Etiquetas de nodos (letras/nombres)
        nx.draw_networkx_labels(self.graph, self.pos, ax=self.ax,
                                font_size=12, font_weight='bold', font_color='white')

        # Dibujar etiquetas de peso en las aristas
        edge_labels = nx.get_edge_attributes(self.graph, 'weight')
        nx.draw_networkx_edge_labels(self.graph, self.pos, edge_labels, ax=self.ax,
                                    font_size=10, font_color='white',
                                    bbox=dict(boxstyle='round,pad=0.3', facecolor='#2b2b2b', edgecolor='none', alpha=0.8))

        # Texto con la distancia acumulada mostrada en cada paso
        if step['current'] is not None:
            current_distance = step['distances'][step['current']]
            distance_text = f"Distancia acumulada al nodo {step['current']}: {current_distance:.1f}"
        elif 'final_path' in step:
            final_distance = step['distances'][self.end]
            distance_text = f"Distancia total del camino más corto: {final_distance:.1f}"
        else:
            distance_text = "Iniciando algoritmo..."

        # Mostrar la caja con la distancia en la parte superior izquierda
        self.ax.text(0.02, 0.98, distance_text, transform=self.ax.transAxes,
                     fontsize=10, verticalalignment='center', color="#BEB1B1", fontweight='bold',
                     bbox=dict(boxstyle='round', facecolor='#404040', alpha=0.9, edgecolor="#8B8686"))

        # Remover ejes para una presentación limpia
        self.ax.axis('off')
        self.fig.canvas.draw()

    def on_key(self, event):
        """Manejador de eventos de teclado para la visualización interactiva."""

        if event.key == 'right':
            if self.current_step < len(self.steps) - 1:
                self.current_step += 1
                self.draw_step()
        elif event.key == 'q':
            plt.close()


# ------------------ Ejemplo de uso ------------------

if __name__ == "__main__":
    
    # Crear un grafo sencillo de ejemplo con pesos en las aristas
    G = nx.Graph()

    # Lista de aristas en formato (u, v, peso)
    edges = [
        ('A', 'B', 4),
        ('A', 'C', 2),
        ('B', 'C', 1),
        ('B', 'D', 5),
        ('C', 'D', 8),
        ('C', 'E', 10),
        ('D', 'E', 2),
        ('D', 'F', 6),
        ('E', 'F', 3)
    ]

    # Agregar aristas al grafo con el atributo 'weight'
    for u, v, w in edges:
        G.add_edge(u, v, weight=w)

    # Mensajes informativos por consola
    print("Iniciando visualización del algoritmo de Dijkstra...")
    print("Controles:")
    print("  → (flecha derecha): Avanzar al siguiente paso")
    print("  q: Cerrar visualización")

    # Crear instancia y lanzar la visualización
    dijkstra_viz = DijkstraVisual(G, start='A', end='F')
    dijkstra_viz.visualize()