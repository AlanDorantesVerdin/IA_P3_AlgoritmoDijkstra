import matplotlib.pyplot as plt
import networkx as nx
import heapq
from collections import defaultdict

class DijkstraVisual:
    def __init__(self, graph, start, end):
        self.graph = graph
        self.start = start
        self.end = end
        self.steps = []
        self.current_step = 0
        
    def dijkstra_with_steps(self):
        """
        ============================================================
        IMPLEMENTACIÓN DEL ALGORITMO DE DIJKSTRA
        ============================================================
        Este método contiene la lógica central del algoritmo de Dijkstra
        para encontrar el camino más corto desde un nodo origen a un destino.
        """
        
        # PASO 1: Inicialización
        # Establecer todas las distancias como infinito, excepto el nodo inicial (0)
        distances = {node: float('inf') for node in self.graph.nodes()}
        distances[self.start] = 0
        
        # Diccionario para rastrear el nodo previo en el camino más corto
        previous = {node: None for node in self.graph.nodes()}
        
        # Cola de prioridad: contiene tuplas (distancia, nodo)
        # Se usa heap para obtener siempre el nodo con menor distancia
        pq = [(0, self.start)]
        
        # Conjunto de nodos ya visitados (procesados completamente)
        visited = set()
        
        # Guardar paso inicial para visualización
        self.steps.append({
            'current': None,
            'visited': set(),
            'distances': distances.copy(),
            'previous': previous.copy(),
            'queue': [(0, self.start)],
            'message': f'Inicio: Nodo origen = {self.start}'
        })
        
        # PASO 2: Procesamiento del algoritmo de Dijkstra
        # Mientras haya nodos en la cola de prioridad
        while pq:
            # Extraer el nodo con la menor distancia acumulada
            current_distance, current_node = heapq.heappop(pq)
            
            # Si ya visitamos este nodo, lo ignoramos (puede haber duplicados en la cola)
            if current_node in visited:
                continue
            
            # Marcar el nodo como visitado
            visited.add(current_node)
            
            # Guardar paso para visualización
            self.steps.append({
                'current': current_node,
                'visited': visited.copy(),
                'distances': distances.copy(),
                'previous': previous.copy(),
                'queue': pq.copy(),
                'message': f'Visitando nodo: {current_node} (distancia: {current_distance})'
            })
            
            # Si llegamos al destino, podemos terminar
            if current_node == self.end:
                break
            
            # PASO 3: Relajación de aristas
            # Explorar todos los vecinos del nodo actual
            for neighbor in self.graph.neighbors(current_node):
                # Ignorar vecinos ya visitados
                if neighbor in visited:
                    continue
                
                # Obtener el peso de la arista entre current_node y neighbor
                weight = self.graph[current_node][neighbor]['weight']
                
                # Calcular la nueva distancia: distancia al nodo actual + peso de la arista
                distance = current_distance + weight
                
                # Si encontramos un camino más corto al vecino, actualizamos
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current_node
                    
                    # Agregar el vecino a la cola de prioridad con su nueva distancia
                    heapq.heappush(pq, (distance, neighbor))
                    
                    # Guardar paso de actualización para visualización
                    self.steps.append({
                        'current': current_node,
                        'visited': visited.copy(),
                        'distances': distances.copy(),
                        'previous': previous.copy(),
                        'queue': pq.copy(),
                        'edge_highlight': (current_node, neighbor),
                        'message': f'Actualizando {neighbor}: nueva distancia = {distance}'
                    })
        
        # PASO 4: Reconstrucción del camino más corto
        # Partimos del destino y retrocedemos usando el diccionario 'previous'
        path = []
        node = self.end
        while node is not None:
            path.append(node)
            node = previous[node]
        path.reverse()  # Invertir para obtener el camino desde origen a destino
        
        # Guardar paso final con el camino completo
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
        """Visualiza el algoritmo paso a paso"""
        self.dijkstra_with_steps()
        
        # Configurar la figura con tema oscuro
        self.fig, self.ax = plt.subplots(figsize=(14, 10), facecolor='#2b2b2b')
        self.ax.set_facecolor('#2b2b2b')
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        
        # Calcular posiciones de los nodos
        self.pos = nx.spring_layout(self.graph, seed=42, k=2)
        
        # Mostrar primer paso
        self.draw_step()
        plt.tight_layout()
        plt.show()
    
    def draw_step(self):
        """Dibuja un paso específico del algoritmo"""
        self.ax.clear()
        
        if self.current_step >= len(self.steps):
            self.current_step = len(self.steps) - 1
        
        step = self.steps[self.current_step]
        
        # Título con información del paso
        self.ax.set_title(f"\n\n\nAlgoritmo de Dijkstra\n\n",
                         fontsize=16, fontweight='bold', color="#837B7B"
        )

        # Información del paso
        self.ax.text(0.5, 0.95, f"\n\n\nPaso {self.current_step + 1}/{len(self.steps)}\n{step['message']}\n\n",
                    transform=self.ax.transAxes, ha='center', fontsize=12, color="#837B7B", fontweight='bold'
        )

        # Instrucciones esquina inferior
        self.ax.text(0.1, 0.05, "[Presiona → para avanzar, 'q' para salir]", 
                    transform=self.ax.transAxes, ha='center', fontsize=10, color="#837B7B")
        
        # Colores de los nodos (paleta oscura)
        node_colors = []
        for node in self.graph.nodes():
            if 'final_path' in step and node in step['final_path']:
                node_colors.append('#FFD700')  # Dorado para camino final
            elif node == self.start:
                node_colors.append("#00FF80A8")  # Verde para inicio
            elif node == self.end:
                node_colors.append('#FF1493')  # Rosa brillante para destino
            elif node == step['current']:
                node_colors.append('#FF8C00')  # Naranja brillante para nodo actual
            elif node in step['visited']:
                node_colors.append('#4169E1')  # Azul royal para visitados
            else:
                node_colors.append('#696969')  # Gris oscuro para no visitados
        
        # Dibujar aristas
        edges = self.graph.edges()
        edge_colors = []
        edge_widths = []
        
        for edge in edges:
            if 'final_path' in step:
                # Resaltar aristas del camino final
                path = step['final_path']
                if any((path[i], path[i+1]) == edge or (path[i+1], path[i]) == edge 
                       for i in range(len(path)-1)):
                    edge_colors.append('#FFD700')
                    edge_widths.append(4)
                else:
                    edge_colors.append('#555555')
                    edge_widths.append(1)
            elif 'edge_highlight' in step and (edge == step['edge_highlight'] or 
                                                edge == (step['edge_highlight'][1], step['edge_highlight'][0])):
                edge_colors.append('#FF4500')  # Rojo-naranja para arista siendo evaluada
                edge_widths.append(3)
            else:
                edge_colors.append('#555555')
                edge_widths.append(1)
        
        nx.draw_networkx_edges(self.graph, self.pos, ax=self.ax, 
                               edge_color=edge_colors, width=edge_widths, alpha=0.6)
        
        # Dibujar nodos
        nx.draw_networkx_nodes(self.graph, self.pos, ax=self.ax,
                               node_color=node_colors, node_size=800,
                               edgecolors='white', linewidths=2)
        
        # Etiquetas de nodos
        nx.draw_networkx_labels(self.graph, self.pos, ax=self.ax,
                                font_size=12, font_weight='bold', font_color='white')
        
        # Etiquetas de pesos en aristas
        edge_labels = nx.get_edge_attributes(self.graph, 'weight')
        nx.draw_networkx_edge_labels(self.graph, self.pos, edge_labels, ax=self.ax,
                                    font_size=10, font_color='white',
                                    bbox=dict(boxstyle='round,pad=0.3', facecolor='#2b2b2b', 
                                    edgecolor='none', alpha=0.8))
        
        # Mostrar distancia total acumulada
        if step['current'] is not None:
            current_distance = step['distances'][step['current']]
            distance_text = f"Distancia acumulada al nodo {step['current']}: {current_distance:.1f}"
        elif 'final_path' in step:
            final_distance = step['distances'][self.end]
            distance_text = f"Distancia total del camino más corto: {final_distance:.1f}"
        else:
            distance_text = "Iniciando algoritmo..."
        
        self.ax.text(0.02, 0.98, distance_text, transform=self.ax.transAxes,
                     fontsize=10, verticalalignment='center', color="#BEB1B1", fontweight='bold',
                     bbox=dict(boxstyle='round', facecolor='#404040', alpha=0.9, edgecolor="#8B8686", ))
        
        self.ax.axis('off')
        self.fig.canvas.draw()
    
    def on_key(self, event):
        """Maneja eventos de teclado"""
        if event.key == 'right':
            if self.current_step < len(self.steps) - 1:
                self.current_step += 1
                self.draw_step()
        elif event.key == 'q':
            plt.close()


# Ejemplo de uso
if __name__ == "__main__":
    # Crear grafo de ejemplo
    G = nx.Graph()
    
    # Agregar aristas con pesos
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
    
    for u, v, w in edges:
        G.add_edge(u, v, weight=w)
    
    # Ejecutar visualización
    print("Iniciando visualización del algoritmo de Dijkstra...")
    print("Controles:")
    print("  → (flecha derecha): Avanzar al siguiente paso")
    print("  q: Cerrar visualización")
    
    dijkstra_viz = DijkstraVisual(G, start='A', end='F')
    dijkstra_viz.visualize()