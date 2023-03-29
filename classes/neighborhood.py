import random
import math
from copy import deepcopy

class Neighbor(object):
    """Uses generator method to generate randomly the best neighbor out of the
    randomly picked neighborhood n. 
    """
            
			
			
    def generator(self, n, current_tour, vehicle_count, distance):
        N_tour = deepcopy(current_tour)
        j = random.randint(0, vehicle_count-1)
        if n == 0: 
            N_tour[j]= self.run_2opt(current_tour[j], distance)
            N_dist = distance.get_distance_all_tours(N_tour) 
           
        elif n==1: 
            found = False
            used = set()
            remaining_vehicles = set(range(0,vehicle_count))
            used.add(j)
            remaining_vehicles = remaining_vehicles - used #Creates new set, not great for large ones
            while ((found == False) and remaining_vehicles):
                node_index = random.randint(0, len(remaining_vehicles)-1) # weil kein range nimmt er auch die letzte Zahl sonst
                k = list(remaining_vehicles)[node_index]
                used.add(k)
                N_tour[j], N_tour[k], found = self.run_swap_move(
                        current_tour[j], current_tour[k], distance, found)
                N_dist = distance.get_distance_all_tours(N_tour) 
                remaining_vehicles = remaining_vehicles - used #Creates new set, not great for large ones

        elif n==2:  
            found = False
            used = set()
            remaining_vehicles = set(range(0,vehicle_count))
            j = random.randint(0,vehicle_count-1) #Random Robot Selection
            used.add(j)
            remaining_vehicles = remaining_vehicles - used #Creates new set, not great for large ones
            while ((found == False) and remaining_vehicles):
                node_index = random.randint(0, len(remaining_vehicles)-1)
                k = list(remaining_vehicles)[node_index]
                used.add(k)
                N_tour[j], N_tour[k], found = self.run_relocate_move(
                        current_tour[j], current_tour[k], distance, found)
                N_dist = distance.get_distance_all_tours(N_tour) 
                remaining_vehicles = remaining_vehicles - used #Creates new set, not great for large ones
        return N_dist, N_tour



    def generator_nearby(self, n, current_tour, vehicle_indices, distance):

        if len(vehicle_indices)==1:
            n=0
        if n == 0: 
            N_tour = current_tour.copy()
            v = random.choice(vehicle_indices)
            N_tour[v]= self.run_2opt(current_tour[v], distance)
            N_dist = distance.get_distance_all_tours(N_tour) 
           
        elif n==1: 
            found = False
            used = set()
            remaining_vehicles = set(vehicle_indices)
            j = random.choice(vehicle_indices)#Random Robot Selection
            used.add(j)
            remaining_vehicles = remaining_vehicles - used #Creates new set, not great for large ones
            while ((found == False) and remaining_vehicles):
                node_index = random.randint(0, len(remaining_vehicles)-1) # weil kein range nimmt er auch die letzte Zahl sonst
                k = list(remaining_vehicles)[node_index]
                used.add(k)
                N_tour = current_tour.copy()
                N_tour[j], N_tour[k], found = self.run_swap_move(
                        current_tour[j], current_tour[k], distance, found)
                N_dist = distance.get_distance_all_tours(N_tour) 
                remaining_vehicles = remaining_vehicles - used #Creates new set, not great for large ones

        elif n==2:  
            found = False
            used = set()
            remaining_vehicles = set(vehicle_indices)
            j = random.choice(vehicle_indices)#Random Robot Selection
            used.add(j)
            while ((found == False) and remaining_vehicles):
                remaining_vehicles = remaining_vehicles - used #Creates new set, not great for large ones
                node_index = random.randint(0, len(remaining_vehicles)-1) 
                k = list(remaining_vehicles)[node_index]
                used.add(k)
                N_tour = current_tour.copy()
                N_tour[j], N_tour[k], found = self.run_relocate_move(
                        current_tour[j], current_tour[k], distance, found)
                N_dist = distance.get_distance_all_tours(N_tour) 
        return N_dist, N_tour



    def construction_heuristics(self, vehicle_count, vehicle_tours, distance_class, cities, depot):
        """Local_Cheapest_Arc: Greedy-Construction-Heuristic, takes nearest cites
           to build a tour, until every vehicle is fairly filled. 
        Output: Route per vehicle v, with depot as first item
        """
        remaining_customers = set(cities)
        fair_distribution = math.floor(len(cities)/len(depot))
        for v in range(vehicle_count):
            vehicle_tours.append([])
            used = set()
            i=0
            best_dist = math.inf
            vehicle_tours[v].append(depot[v])
            for to_node in remaining_customers:
                dist_temp = distance_class.d_mat[v,to_node.index]
                if dist_temp < best_dist:
                    best_node = to_node
                    best_dist = dist_temp
            vehicle_tours[v].append(best_node)
            used.add(best_node)
            remaining_customers -= used
            while remaining_customers and i<fair_distribution:
                i += 1
                best_dist = math.inf
                for to_node in remaining_customers:
                    dist_temp = distance_class.d_mat[best_node.index,to_node.index]
                    if dist_temp < best_dist:
                        best_node_temp = to_node
                        best_dist = dist_temp
                vehicle_tours[v].append(best_node_temp)
                used.add(best_node_temp)
                remaining_customers -= used
                best_node = best_node_temp
                
        return vehicle_tours
    
	
	
    def construction_heuristics_labeled(self, vehicle_count, vehicle_tours, distance_class, cities, depot):
        """Same as original ch(), just checks if vehicle label belongs 
        to customer label. Last Block is for remaining customers, because
        the approximation of fair_distribution is not true: there are maybe
        less customers for one vehicle than the other, so not everybody can get 
        fair_distribution amount of customers."""
        remaining_customers = set(cities)
        fair_distribution = math.floor(len(cities)/len(depot))
        for v in range(vehicle_count):
            vehicle_tours.append([])
            used = set()
            i=0
            best_dist = math.inf
            vehicle_tours[v].append(depot[v])
            found = False
            for to_node in remaining_customers:
                if (set(depot[v].label).intersection(to_node.label)):
                    found=True
                    dist_temp = distance_class.d_mat[v,to_node.index]
                    if dist_temp < best_dist:
                        best_node = to_node
                    best_dist = dist_temp
                        
            if found==True:
                vehicle_tours[v].append(best_node)
                used.add(best_node)
                remaining_customers -= used
            while remaining_customers and i<fair_distribution and found == True:
                i += 1
                best_dist = math.inf
                found = False
                for to_node in remaining_customers:
                    if (set(depot[v].label).intersection(to_node.label)):
                        found=True
                        dist_temp = distance_class.d_mat[
                                best_node.index,to_node.index]
                        if dist_temp < best_dist:
                            best_node_temp = to_node
                            best_dist = dist_temp
                if (found==True):
                    vehicle_tours[v].append(best_node_temp)
                    used.add(best_node_temp)
                    remaining_customers -= used
                    best_node = best_node_temp
        #If remaining_customers, than allocate them one by one, so not one 
        # vehicle gets every node
        while remaining_customers:
            for v in range(vehicle_count):
                i=0
                found=True
                used= set()
                best_node = vehicle_tours[v][-1]
                while remaining_customers and i<1 and found == True:
                    i += 1
                    best_dist = math.inf
                    found = False
                    for to_node in remaining_customers:
                        if (set(depot[v].label).intersection(to_node.label)):
                            found=True
                            dist_temp = distance_class.d_mat[
                                    best_node.index, to_node.index]
                            if dist_temp < best_dist:
                                best_node_temp = to_node
                                best_dist = dist_temp
                    if (found==True):
                        vehicle_tours[v].append(best_node_temp)
                        used.add(best_node_temp)
                        remaining_customers -= used
            
        return vehicle_tours, remaining_customers
    
	
	
    def construction_heuristics_dynamic(self, dynamic_schedule, distance, free_customers):
#       Dynamic Schedule kommt als Input. D.H, nur noch die Robots, die die freee
#       customers auch bearbeiten können. Trotzdem macht es sinn, eine Abfrage der Labels zu machen,
#        im Falle von label: (0,1), also mittig liegende Punkte und Punkte die links (0) liegen,
#        zwar alle Roboter in Frage kommen, aber nicht jeden Punkt jeder Free_customer bearbeiten kann
        """Loops over all customers. Takes smallest route out of all routes 
        belonging to customer label. Inserts customer"""
        new_routes_index=set()     
        
        for customer in free_customers:
#          Alle Indexe aus dem dynamic_schedule, die für den customer in Frage kommen 
           v_index = [idx for idx,x in enumerate(dynamic_schedule) if x[0].label[0] in customer.label]
#          Finde den Index, mit der kleinsten Zeit. Speichert index in 
#          v_min_idx 
           v_min = math.inf
           for v in v_index:
               current_cost = (distance.get_distance_tour(dynamic_schedule[v]))
               if current_cost < v_min:
                   v_min = current_cost
                   v_min_index = v
#           v_cost = [distance.get_distance_tour(i) for i in dynamic_schedule if i[0].label[0] in customer.label]
#           v_min_index_fake = v_cost.index(min(v_cost))
#           v_min_index = v_index[v_min_index_fake] #VEHICLE INDEX!!!         
           
#          Loop über alle Customer der Route mit minimaler Reisedauer: 
#          Ziel ist es die nächste Stadt der Route zum free_cust zu finden
           best_cost=math.inf
           for j in range(0, len(dynamic_schedule[v_min_index])):
               current_cost=distance.d_mat[customer.index, dynamic_schedule[v_min_index][j].index]
               if current_cost < best_cost:
                   best_cost=current_cost
                   best_index = j
                   
#          Abfrage: Wenn best_index die letzte Stadt ist, dannn füge customer
#          vor die letzte Stadt ein                      
           if len(dynamic_schedule[v_min_index]) == best_index+1:
               best_adjacent_index = best_index
               
#          Wenn Depot am nächsten ist, füge es nach depot ein                              
           elif (best_index == 0):
               best_adjacent_index = best_index+1
               
  #        Ansonsten guck ob vor oder hinter der besten Stadt besser ist           
           elif (distance.d_mat[dynamic_schedule[v_min_index][best_index].index
                 , dynamic_schedule[v_min_index][best_index+1].index] <
                 distance.d_mat[dynamic_schedule[v_min_index][best_index].index,
                        dynamic_schedule[v_min_index][best_index-1].index]):
               best_adjacent_index = best_index+1
           else: best_adjacent_index = best_index
           
           dynamic_schedule[v_min_index].insert(best_adjacent_index,customer)
           new_routes_index.add(v_min_index)
           
        return dynamic_schedule, new_routes_index
    
	
	
    def swap_2opt(self, route, i, k):
        assert i >= 0 and i < (len(route) - 1)
        assert k > i and k < len(route)
        new_route = route[0:i]
        new_route.extend(reversed(route[i:k + 1]))
        new_route.extend(route[k+1:])
        assert len(new_route) == len(route)
        return new_route
    
	
	
    def run_2opt(self, route, distance):
        
        """
    	improves an existing route using the 2-opt swap until no improved route is found
    	Searches for the best i, k to swap. Controls the inputs for swap_2opt()
    	route - route to improve    
        [A B C D E] i=0, k=1 -> [B A C D E]
        [A B C D E] i=1, k=4 -> [A E D C B]
        """
        best_route = route
        best_distance = math.inf
        for i in range(len(best_route) - 1):
            for k in range(i+1, len(best_route)):
                new_route = self.swap_2opt(route, i, k)
                new_distance = distance.get_distance_tour(new_route)
                if new_distance < best_distance:
                    best_distance = new_distance
                    best_route = new_route
                    
        start = [ind for ind,e in enumerate(best_route) if e.index==route[0].index] #sort
        best_route = best_route[start[0]:]+best_route[:start[0]]
        
        assert len(best_route) == len(route)
        return best_route



    def run_2opt_hc(self, route, distance):
        """
    	improves an existing route using the 2-opt swap until no improved route is found
    	best path found will differ depending of the start node of the list of nodes
        representing the input tour. returns the best path found
    	route - route to improve    
        """
        improvement = True
        best_route = route.copy()
        best_distance = distance.get_distance_tour(best_route)
        while improvement:
            improvement = False
            for i in range(len(best_route) - 1):
                for k in range(i+1, len(best_route)):
                    
                    new_route = self.swap_2opt(best_route, i, k)
                    new_distance = distance.get_distance_tour(new_route)
                    if new_distance < best_distance:
                        best_distance = new_distance
                        best_route = new_route
                        improvement = True
                        break #improvement found, return to the top of the while loop
                if improvement:
                    break    
        start = [ind for ind,e in enumerate(best_route) if e.index==route[0].index ] #sort
        best_route = best_route[start[0]:]+best_route[:start[0]]
        assert len(best_route) == len(route)
    
        return best_route
    
	
	
    def swap_move(self, route1, route2, i , k):
        route1 = route1.copy()
        route2 = route2.copy()
        route1[i], route2[k] = route2[k], route1[i]
        return route1, route2
    
	
	
    def run_swap_move(self, route1, route2, distance, found):
        best_route1 = route1.copy()
        best_route2 = route2.copy()
        best_distance = math.inf
        for i in range(1, len(best_route1)):
            for k in range(1, len(best_route2)):
                if (set(route1[0].label).intersection(set(route2[k].label)) 
                and set(route1[i].label).intersection(set(route2[0].label))):
                    found = True
                    new_route1, new_route2 = self.swap_move(
                                                route1, route2, i, k)
                    new_distance = distance.get_distance_all_tours(
                                            [new_route1, new_route2])
                    if new_distance < best_distance: 
                        best_distance = new_distance
                        best_route1, best_route2 = new_route1, new_route2
                        
        return best_route1, best_route2, found
    
	
	
    def relocate_move(self, route1, route2, i , k):
        route1 = route1.copy()
        route2 = route2.copy()
        route1.insert(i, route2[k])
        route2.remove(route2[k])
        return route1, route2
    
	
	
    def run_relocate_move(self, route1, route2, distance, found):
        """Runs swap_move() until improvement is found, cancels
           if no improvement route1: taking, route2: giving"""
        best_route1 = route1.copy()
        best_route2 = route2.copy()
        best_distance = math.inf
        for i in range(1, len(route1)+1):
            for k in range(1, len(route2)):
                if (set(route1[0].label).intersection(set(route2[k].label))):
                    found = True
                    new_route1, new_route2 = self.relocate_move(route1, route2, i, k)
                    new_distance = distance.get_distance_all_tours([new_route1, new_route2])
                    if new_distance < best_distance: 
                        best_distance = new_distance
                        best_route1, best_route2 = new_route1, new_route2
                        
        return best_route1, best_route2, found