from abc import ABC, abstractmethod
import math

class State(ABC):
    
    def __init__(self, graph, parent=None, current_node=None, goal_node=None):
        self.graph = graph  # Reference na networkx graf mape
        self.parent = parent  # Roditeljsko stanje
        self.current_node = current_node  # ID trenutnog cvora (raskrsnice)
        self.goal_node = goal_node  # ID ciljnog cvora (mesto nesrece)
        
        # Povecavamo dubinu/nivo pretrage 
        self.depth = parent.depth + 1 if parent is not None else 1

    @abstractmethod
    def get_next_states(self):
        pass

    @abstractmethod
    def is_final_state(self):
        pass

    @abstractmethod
    def unique_hash(self):
        pass

    @abstractmethod
    def get_cost_estimate(self):
        pass

    @abstractmethod
    def get_current_cost(self):
        pass


class MapNodeState(State):
    
    def __init__(self, graph, parent=None, current_node=None, goal_node=None, edge_cost=0):
        super().__init__(graph, parent, current_node, goal_node)
        
        # Racunanje stvarne cene g(n) u metrima
        if parent is None:
            self.cost = 0
        else:
            self.cost = parent.cost + edge_cost

    def get_next_states(self):
        
        next_states = []
        
        # Prolazimo kroz sve naslednike (susede) trenutnog cvora u usmerenom grafu
        for neighbor in self.graph.successors(self.current_node):
            # Posto je graf MultiDiGraph, uzimamo prvu granu izmedju dva cvora [0]
            edge_data = self.graph[self.current_node][neighbor][0]
            
            # Uzimamo duzinu ulice u metrima (ako slucajno ne postoji, stavljamo 0)
            length = edge_data.get('length', 0)
            
            # Kreiramo novo stanje i dodajemo ga u listu
            next_state = MapNodeState(
                graph=self.graph, 
                parent=self, 
                current_node=neighbor, 
                goal_node=self.goal_node, 
                edge_cost=length
            )
            next_states.append(next_state)
            
        return next_states

    def is_final_state(self):
       
        return self.current_node == self.goal_node

    def unique_hash(self):
       
        return str(self.current_node)

    def get_current_cost(self):
       
        return self.cost

    def get_cost_estimate(self):
       
        curr_lat = self.graph.nodes[self.current_node]['y']
        curr_lon = self.graph.nodes[self.current_node]['x']
        
        goal_lat = self.graph.nodes[self.goal_node]['y']
        goal_lon = self.graph.nodes[self.goal_node]['x']
        
        R = 6371000.0
        
        phi1 = math.radians(curr_lat)
        phi2 = math.radians(goal_lat)
        delta_phi = math.radians(goal_lat - curr_lat)
        delta_lambda = math.radians(goal_lon - curr_lon)
        
        a = math.sin(delta_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        distance = R * c
        return distance

    def __repr__(self):
        return f"MapNodeState(node={self.current_node}, cost={self.cost:.2f}m, depth={self.depth})"