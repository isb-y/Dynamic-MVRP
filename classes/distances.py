import numpy as np
import math

class DistanceEvaluator(object):
    def __init__(self, customers):
        self._v_bahn = 1#/(263.8770884478724))
        self._d_mat = self._distance_matrix(customers) * self._v_bahn
    
    def _distance_matrix(self, customers):
        number_of_locations = len(customers)
        self._d_mat = np.zeros((number_of_locations, number_of_locations))
        for from_node in range(number_of_locations):
            for to_node in range(number_of_locations):
                if from_node == to_node:
                   self._d_mat[from_node, to_node] = math.inf
                else:
                    self._d_mat[from_node, to_node] = (
                        self.euclidean_distance(
                            customers[from_node],
                            customers[to_node]))
        return self.d_mat
    
    def get_distance_all_tours(self, vehicle_tours):
        """Returns distance of tour with maximum distance"""
        best_dist = 0
        for v in vehicle_tours:
            if (len(v) == 1): 
                max_dist = 0
                continue
            max_dist = 0 
            for from_node in range(0, len(v)-1):
                to_node=from_node+1
                max_dist += self.d_mat[v[from_node].index,v[to_node].index] \
                            + v[to_node].time
            max_dist += self.d_mat[v[-1].index, v[0].index] 
            if(max_dist>best_dist): best_dist=max_dist
        return best_dist

    def get_distance_tour(self, vehicle_tour):
        """Returns distance of input tour"""
        dist = 0
        if (len(vehicle_tour) < 2):
            return dist
        for from_node in range(0, len(vehicle_tour)-1):
            to_node=from_node+1
            dist += self.d_mat[vehicle_tour[from_node].index, \
                               vehicle_tour[to_node].index]+ vehicle_tour[to_node].time
        dist += self.d_mat[vehicle_tour[-1].index,vehicle_tour[0].index] + vehicle_tour[0].time
        return dist
    
    def euclidean_distance(self, customer1, customer2):
        return math.sqrt((customer1.x - customer2.x)**2 + (customer1.y - customer2.y)**2)    
    
    @property
    def d_mat(self):
        return self._d_mat