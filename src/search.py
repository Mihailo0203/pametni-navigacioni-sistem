from collections import deque
from abc import ABC, abstractmethod

class Search(ABC):
    
    def search(self, initial_state):
        """
        Glavna petlja za pretragu prostora stanja.
        Vraća: putanju (lista ID-jeva čvorova), listu procesiranih stanja, i listu neprocesiranih stanja.
        """
        states_list = deque([initial_state])  # deque - brza lista
        states_set = {initial_state.unique_hash()}  # set - za brzu pretragu stanja u O(1)

        processed_list = deque([])  
        processed_set = set()  

        while len(states_list) > 0:
            # Preuzimanje sledećeg stanja zavisno od konkretnog algoritma
            curr_state = self.select_state(states_list)  
            states_set.remove(curr_state.unique_hash())  

            processed_list.append(curr_state)  
            processed_set.add(curr_state.unique_hash())  

            # Ako smo stigli do mesta nesreće, rekonstruišemo put
            if curr_state.is_final_state():  
                return Search.reconstruct_path(curr_state), processed_list, states_list

            # Generisanje sledećih raskrsnica (stanja)
            for new_state in curr_state.get_next_states():
                # Provera da li je stanje već viđeno, a nije obrađeno
                if new_state.unique_hash() in states_set:
                    old_state = next((x for x in states_list if x.unique_hash() == new_state.unique_hash()), None)
                    if old_state and old_state.get_current_cost() > new_state.get_current_cost():
                        states_list.remove(old_state)
                        states_set.remove(old_state.unique_hash())
                        states_list.append(new_state)
                        states_set.add(new_state.unique_hash())
                    continue
                
                # Provera da li je stanje već potpuno obrađeno
                if new_state.unique_hash() in processed_set:
                    old_state = next((x for x in processed_list if x.unique_hash() == new_state.unique_hash()), None)
                    if old_state and old_state.get_current_cost() > new_state.get_current_cost():
                        states_list.append(new_state)
                        states_set.add(new_state.unique_hash())
                    continue
                
                # Ako je potpuno novo stanje, dodajemo ga na listu
                states_list.append(new_state)
                states_set.add(new_state.unique_hash())
                    
        return None, processed_list, states_list

    @staticmethod
    def reconstruct_path(final_state):
        """
        Rekonstruiše putanju unazad prateći roditelje.
        Za razliku od vezbi, ovde vracamo listu ID-jeva cvorova jer je to potrebno za crtanje mape.
        """
        path = []
        while final_state is not None:
            path.append(final_state.current_node)
            final_state = final_state.parent
        return list(reversed(path))

    @abstractmethod
    def select_state(self, states):
        pass


class BreadthFirstSearch(Search):
    def select_state(self, states):
        return states.popleft()


class DepthFirstSearch(Search):
    def select_state(self, states):
        return states.pop()


class UniformCostSearch(Search):
    def select_state(self, states):
        min_state = min(states, key=lambda x: x.get_current_cost())
        states.remove(min_state)
        return min_state


class AStarSearch(Search):
    def select_state(self, states):
        min_state = min(states, key=lambda x: x.get_current_cost() + x.get_cost_estimate())
        states.remove(min_state)
        return min_state
    
class IterativeDeepeningAStar(Search):
    def select_state(self, states):
        pass 

    def search(self, initial_state):
        # Почетна граница (bound) је хеуристика почетног чвора
        bound = initial_state.get_current_cost() + initial_state.get_cost_estimate()
        processed_list = [] # Листа за метрику (просторна сложеност)
        
        while True:
            # Стек за Итеративни DFS: чува тупл (стање, сет_посећених_id_јева_на_путањи)
            # Сет користимо да спречимо алгоритам да се врти у круг у истој улици
            stack = [(initial_state, {initial_state.unique_hash()})]
            
            min_exceeded = float('inf')
            found_goal = None
            
            while stack:
                curr_state, current_path_set = stack.pop()
                processed_list.append(curr_state)
                
                f = curr_state.get_current_cost() + curr_state.get_cost_estimate()
                
                # Ако тренутна цена премашује границу, бележимо је за следећу итерацију и сечемо грану
                if f > bound:
                    if f < min_exceeded:
                        min_exceeded = f
                    continue
                    
                if curr_state.is_final_state():
                    found_goal = curr_state
                    break
                    
                # Додавање следећих стања на стек
                for next_state in curr_state.get_next_states():
                    if next_state.unique_hash() not in current_path_set:
                        # Правимо копију сета за нову грану претраге
                        new_path_set = set(current_path_set)
                        new_path_set.add(next_state.unique_hash())
                        stack.append((next_state, new_path_set))
                        
            # Ако смо нашли циљ, враћамо путању
            if found_goal is not None:
                return Search.reconstruct_path(found_goal), processed_list, []
                
            # Ако нема више чворова за обилазак, путања не постоји
            if min_exceeded == float('inf'):
                return None, processed_list, []
                
            # Повећавамо границу за следећу итерацију
            bound = min_exceeded

class GreedySearch(Search):
    def select_state(self, states):
        min_state = min(states, key=lambda x: x.get_cost_estimate())
        states.remove(min_state)
        return min_state
    
class BidirectionalAStar(Search):
    def select_state(self, states):
        pass 

    def search(self, initial_state):
        forward_states = [initial_state]
        forward_visited = {initial_state.unique_hash(): initial_state}
        
        goal_state = initial_state.__class__(
            graph=initial_state.graph, 
            parent=None, 
            current_node=initial_state.goal_node, 
            goal_node=initial_state.current_node, 
            edge_cost=0
        )
        backward_states = [goal_state]
        backward_visited = {goal_state.unique_hash(): goal_state}
        
        processed_list = []
        
        # Inicijalizujemo najkraći put na "beskonačno"
        best_cost = float('inf')
        best_collision_fwd = None
        best_collision_bwd = None
        
        while forward_states and backward_states:
            # Uvek biramo čvor sa najmanjom f = g + h vrednošću
            current_forward = min(forward_states, key=lambda x: x.get_current_cost() + x.get_cost_estimate())
            current_backward = min(backward_states, key=lambda x: x.get_current_cost() + x.get_cost_estimate())
            
            f_forward = current_forward.get_current_cost() + current_forward.get_cost_estimate()
            f_backward = current_backward.get_current_cost() + current_backward.get_cost_estimate()
            
            # USLOV ZA KRAJ: Ako najmanja procenjena cena u oba reda premašuje 
            # trenutno najbolji pronađen put, garantovano nema kraćeg puta!
            if f_forward >= best_cost and f_backward >= best_cost:
                break
                
            # --- KORAK NAPRED ---
            if f_forward < best_cost:
                forward_states.remove(current_forward)
                processed_list.append(current_forward)
                
                # Provera sudara
                hash_fwd = current_forward.unique_hash()
                if hash_fwd in backward_visited:
                    total_cost = current_forward.get_current_cost() + backward_visited[hash_fwd].get_current_cost()
                    # Ažuriramo ako smo našli bolju prečicu
                    if total_cost < best_cost:
                        best_cost = total_cost
                        best_collision_fwd = current_forward
                        best_collision_bwd = backward_visited[hash_fwd]
                        
                for next_state in current_forward.get_next_states():
                    hash_val = next_state.unique_hash()
                    if hash_val not in forward_visited or forward_visited[hash_val].get_current_cost() > next_state.get_current_cost():
                        forward_visited[hash_val] = next_state
                        forward_states.append(next_state)
            else:
                forward_states.remove(current_forward)
                
            # --- KORAK NAZAD ---
            if f_backward < best_cost:
                backward_states.remove(current_backward)
                processed_list.append(current_backward)
                
                # Provera sudara
                hash_bwd = current_backward.unique_hash()
                if hash_bwd in forward_visited:
                    total_cost = current_backward.get_current_cost() + forward_visited[hash_bwd].get_current_cost()
                    if total_cost < best_cost:
                        best_cost = total_cost
                        best_collision_bwd = current_backward
                        best_collision_fwd = forward_visited[hash_bwd]
                        
                for predecessor in initial_state.graph.predecessors(current_backward.current_node):
                    edge_data = initial_state.graph[predecessor][current_backward.current_node][0]
                    length = edge_data.get('length', 0)
                    
                    new_back_state = initial_state.__class__(
                        graph=initial_state.graph,
                        parent=current_backward,
                        current_node=predecessor,
                        goal_node=initial_state.current_node,
                        edge_cost=length
                    )
                    
                    hash_val = new_back_state.unique_hash()
                    if hash_val not in backward_visited or backward_visited[hash_val].get_current_cost() > new_back_state.get_current_cost():
                        backward_visited[hash_val] = new_back_state
                        backward_states.append(new_back_state)
            else:
                backward_states.remove(current_backward)
                
        # Spajanje savršene putanje na kraju
        if best_collision_fwd and best_collision_bwd:
            return self._reconstruct_bidirectional_path(best_collision_fwd, best_collision_bwd), processed_list, []
            
        return None, processed_list, []

    def _reconstruct_bidirectional_path(self, forward_state, backward_state):
        path_forward = []
        curr = forward_state
        while curr is not None:
            path_forward.append(curr.current_node)
            curr = curr.parent
        path_forward = list(reversed(path_forward))
        
        path_backward = []
        curr = backward_state.parent
        while curr is not None:
            path_backward.append(curr.current_node)
            curr = curr.parent
            
        return path_forward + path_backward