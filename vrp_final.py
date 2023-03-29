from classes.distances import DistanceEvaluator
from classes.neighborhood import Neighbor
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from collections import namedtuple
from copy import deepcopy
import random
import json
import os
import time



def parser():
    #PARSING
    if (os.path.exists("Y:\\Masterarbeit\\07_Code\\Python\\vrp")):
        path2dir = "Y:\\Masterarbeit\\07_Code\\Python\\vrp"
        path2image = "Y:\\MASTERARBEIT\\05_Abschlussarbeit\\Latex\\Thesis\\images"
        
    else:
        os.path.exists("C:\\Users\\Saltuk\\Desktop\\Masterarbeit\\07_Code\\Python\\vrp")
        path2dir = "C:\\Users\\Saltuk\\Desktop\\Masterarbeit\\07_Code\\Python\\vrp"
        path2image = "C:\\Users\\Saltuk\\Desktop\\Masterarbeit\\05_Abschlussarbeit\\Latex\\Thesis\\images" 
        
    Customer = namedtuple("Customer", ['index', 'x', 'y', 'time', 'label'])
    
    file_location = path2dir + "\\data_own\\vrp_101_10_1_label.txt"  
    
    with open(file_location, 'r') as input_data_file:
        input_data = input_data_file.read()
    
    lines = input_data.split('\n')
    
    parts = lines[0].split()
    customer_count = int(parts[0])
    
    vehicle_count = int(parts[1])
    customers = []
    
    for i in range(1, customer_count+1):
	
        line = lines[i]
        parts = line.split()
		
        parts[-1] = parts[-1].split(',')
        parts[-1] = [int(i) for i in parts[-1]]
        parts[-1] = tuple(parts[-1])
		
        customers.append(Customer(i-1, float(parts[0]),
                         float(parts[1]), float(parts[2]), parts[3]))
        
    return customers, vehicle_count, path2dir, path2image    



def run_solver(vehicle_count, vehicle_tours, distance, cities,
               depot, neighborhood, L_h, itermax, idle):
    
    #Generate Initial Tour
    start = time.time()
	
    vehicle_tours, remaining_customers = (
            neighborhood.construction_heuristics_labeled(
            vehicle_count, vehicle_tours, distance, cities, depot))
			
    end = time.time()
    
    print('CH TIME IS:   {}'.format(end - start))
	
    #LAHC-Algorithm starts here
    current_tour = deepcopy(vehicle_tours)
    current_dist = distance.get_distance_all_tours(current_tour)
	
    best_tour = current_tour
    best_dist    = current_dist 
	
    f_k = [best_dist]*L_h 
    iter_idle=0
    iter_ = 0
	
    xx = [0]
    yy = [current_dist]  
    
    start1 = time.time()
	
    while iter_ < itermax and iter_idle < itermax*idle:
        print(iter_)
        iter_ += 1

        n=random.randint(0,2) 
        start = time.time()
        N_dist, N_tour = neighborhood.generator(n, current_tour,
                                                vehicle_count, distance)
        end = time.time()
        two_opt = end - start
        print('2opt_time:   {}'.format(two_opt))
		
        if N_dist >= current_dist:
            iter_idle += 1
			
        else:
            iter_idle = 0
            
        if N_dist <= current_dist or N_dist < f_k[iter_ % L_h] :
            current_tour = N_tour
            current_dist = N_dist
            
        if current_dist < f_k[iter_ % L_h]:
            f_k[iter_ % L_h] = current_dist
			
        yy.append(current_dist)
        xx.append(iter_)
        
		#If better sol is found in last iteration
    if(current_dist < best_dist):
        best_dist = current_dist
        best_tour = current_tour
        
    #Perform 2-opt last time 
    for j in range(len(best_tour)):
        best_tour[j] = neighborhood.run_2opt_hc(best_tour[j], distance)
    end1 = time.time()
    print('LOOP TIME IS:   {}'.format(end1 - start1))   
    print([distance.get_distance_tour(i) for i in best_tour])
    return best_tour, best_dist, xx, yy



def plotter(xx, yy, distance, vehicle_tours, path2image):
    
    x1 = [i/10000 for i in xx]
    x1=x1[::100]
    y1 = [i+2.738465725434576 for i in yy]
    y1=y1[::100]
    fig = plt.figure()
    fig.set_size_inches(5, 4)
    plt.plot(x1,y1,label=r'$Kosten über Iteration$', 
             linewidth=0.5, color='black')
    plt.xlabel('Iteration in $10^{4}$')
    plt.ylabel('Taktzeit in $s$')
    plt.tight_layout()
    plt.grid(True, alpha=0.2)
#    plt.savefig(path2image + '\\overall_plan_iter2.pdf', 
#            #This is simple recomendation for publication plots
#            dpi=1000, 
#            # Plot will be occupy a maximum of available space
#            bbox_inches='tight', 
#            )
    
# =============================================================================
    
    fig = plt.figure()
    fig.set_size_inches(6, 4)
    
    ax = fig.add_subplot(111)
    ax.set_title('Master-Schedule')
    ax.set_xlabel('$X$-Koordinate in $mm$')
    ax.set_ylabel('$Z$-Koordinate in $mm$')
    
#    c = ['darkviolet', 'k', 'y', 'm', 'g','r','salmon', 'lightslategrey', 'darkorange', 'c']
    for v in range(len(vehicle_tours)):
        x = [i.x for i in vehicle_tours[v]]

        y = [i.y for i in vehicle_tours[v]]

        a = ax.plot(x[1:],y[1:],'-o', lw=1, markersize=1, markerfacecolor='k',
                markeredgecolor = 'k')
        last_color = a[-1].get_color()
        ax.plot(x[0],y[0], 's', color='w',markeredgecolor = 'k', markersize=10)
        
        ax.plot(x[0:2],y[0:2], '-', color = last_color, linewidth=1)
        
        ax.plot([x[-1], x[0]], [y[-1], y[0]], '-', color=last_color, linewidth=1)
                    
    legend_elements = [Line2D([0], [0], marker='o', color='k', lw=0, label='WPS-Punkt',
                              markerfacecolor='w', markersize=2),
                       Line2D([0], [0], marker='s', color='k', lw=0, label='Roboter-Base',
                              markerfacecolor='w', markersize=10)]
                        
    ax.legend(handles=legend_elements, loc='upper right', frameon=True)
    plt.grid(True, alpha=0.2)
    plt.tight_layout()
    fig.set_size_inches(6, 4)
#    plt.savefig(path2image + '\\overall_plan2.pdf', 
#            #This is simple recomendation for publication plots
#            dpi=1000, 
#            # Plot will be occupy a maximum of available space
#            bbox_inches='tight', 
#            )
    
	
	
def write_to_file(vehicle_tours, path2dir):
    master_schedule_json= [[json.dumps(i._asdict()) for i in v] for v in vehicle_tours]
    with open(path2dir + '\\master_schedule\\master_scheduletest.json', 'w') as outfile:
        json.dump(master_schedule_json, outfile)
        
		
		
def main():
    customers, vehicle_count, path2dir, path2image = parser()
    depot    = customers[:vehicle_count]
    cities   = customers[vehicle_count:] 
    distance = DistanceEvaluator(customers)
    neighborhood = Neighbor()
    vehicle_tours = []
    L_h = 500
    itermax = 1000000
    idle = 0.02
    
    vehicle_tours, best_dist, xx, yy = (
            run_solver(vehicle_count, vehicle_tours,distance, cities, depot,
                neighborhood, L_h, itermax, idle))
    
    plotter(xx, yy, distance, vehicle_tours, path2image)
#    write_to_file(vehicle_tours, path2dir)
    return vehicle_tours, best_dist, customers, distance, xx, yy
        
	  
		
if __name__ == '__main__':
  vehicle_tours, best_dist, customers, distance, xx, yy = main()
  