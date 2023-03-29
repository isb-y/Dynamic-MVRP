import random

class Neighborhood(object):
    
	
	
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
    	best path found will differ depending of the start node of the list of nodes
        representing the input tour. returns the best path found
    	route - route to improve    
        """
        best_route = route
        best_distance = 99999
		
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
    
	
	
    def swap_move(self, route1, route2, i , k):
        route1 = route1.copy()
        route2 = route2.copy()
		
        route1[i], route2[k] = route2[k], route1[i]
		
        return route1, route2
    
	
	
    def run_swap_move(self, route1, route2, distance):
        best_route1, best_route2 = route1, route2
        best_distance = 999999
		
        for i in range(1, len(best_route1)):
            for k in range(1, len(best_route2)):
			
                new_route1, new_route2 = self.swap_move(route1, route2, i, k)
                new_distance = distance.get_distance_all_tours([new_route1, new_route2])
				
                if new_distance < best_distance: 
                    best_distance = new_distance
                    best_route1, best_route2 = new_route1, new_route2
                        
        return best_route1, best_route2
    
	
	
    def relocate_move(self, route1, route2, i , k):
	
        route1 = route1.copy()
        route2 = route2.copy()
		
        route1.insert(i, route2[k])
        route2.remove(route2[k])
		
        return route1, route2
    
	
	
    def run_relocate_move(self, route1, route2, distance):
        """Runs swap_move() until improvement is found, cancels if no improvement"""
        best_route1, best_route2 = route1, route2
        best_distance = 99999
		
        for i in range(1, len(route1)+1):
            for k in range(1, len(route2)):
			
                new_route1, new_route2 = self.relocate_move(route1, route2, i, k)
                new_distance = distance.get_distance_all_tours([new_route1, new_route2])
				
                if new_distance < best_distance: 
                    best_distance = new_distance
                    best_route1, best_route2 = new_route1, new_route2
					
        return best_route1, best_route2
    
	
	
    def run_2opt_hc(self, route, distance):
        """
    	improves an existing route using the 2-opt swap until no improved route is found
    	best path found will differ depending of the start node of the list of nodes
        representing the input tour. returns the best path found
    	route - route to improve    
        """
        improvement = True
        best_route = route
        best_distance = distance.get_distance_tour(route)
		
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
    
	
	
    def construction_heuristics_dynamic(self, dynamic_schedule, distance, free_customers):
        """Takes smallest route out of route for route in customer.c. Insert customer out of free
           customers, fill it with nearest city."""
           
        for customer in free_customers:
		
           v_index = [x for x,i in enumerate(dynamic_schedule) if i[0].c[0] in customer.c]
           v_cost = [distance.get_distance_tour(i) for i in dynamic_schedule if i[0].c[0] in customer.c]
           v_min_index_fake = v_cost.index(min(v_cost))
           v_min_index = v_index[v_min_index_fake] #VEHICLE INDEX!!!
           
           best_cost=9999
		   
           for j in range(0,len(dynamic_schedule[v_min_index_fake])):
               current_cost=distance.d_mat[customer.index, dynamic_schedule[v_min_index][j].index]
			   
               if current_cost < best_cost:
                   best_cost=current_cost
                   best_index = j
				   
           if len(dynamic_schedule[v_min_index]) == best_index+1:
               best_adjacent_index = best_index
			   
           elif best_index == 0:
               best_adjacent_index = best_index+1
			   
           elif (distance.d_mat[dynamic_schedule[v_min_index][best_index].index, dynamic_schedule[v_min_index][best_index+1].index] <
                 distance.d_mat[dynamic_schedule[v_min_index][best_index].index, dynamic_schedule[v_min_index][best_index-1].index]):
               best_adjacent_index = best_index+1
			   
           else: best_adjacent_index = best_index
		   
           dynamic_schedule[v_min_index].insert(best_adjacent_index,customer)
		   
        return dynamic_schedule
        
		
		
    def generator(self, n, current_tour, vehicle_count, distance):
	
        if n == 0: 
            N_tour = current_tour.copy()
            v = random.randint(0, vehicle_count-1)
            N_tour[v]= self.run_2opt(current_tour[v], distance)
            N_dist = distance.get_distance_all_tours(N_tour) 
           
        elif n==1: 
            j, k = random.sample(range(0 ,len(current_tour)), 2)
            N_tour = current_tour.copy()
            N_tour[j], N_tour[k] = self.run_swap_move(current_tour[j], current_tour[k], distance)
            N_dist = distance.get_distance_all_tours(N_tour)
       
        elif n==2:   
            j, k = random.sample(range(0,vehicle_count),2) 
            N_tour = current_tour.copy()
            N_tour[j], N_tour[k] = self.run_relocate_move(current_tour[j], current_tour[k], distance)
            N_dist = distance.get_distance_all_tours(N_tour) 
            
        return N_dist, N_tour