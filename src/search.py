from collections import deque
from abc import ABC, abstractmethod

class Search(ABC):
    """
    Apstraktna klasa za pretragu na mapi grada.
    """
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