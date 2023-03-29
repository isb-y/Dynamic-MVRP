from classes.distances import DistanceEvaluator
from classes.neighborhood import Neighbor
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from collections import namedtuple
from copy import deepcopy
import numpy as np
import random
import json
import os
#mpl.style.use('default')
plt.rcParams.update({'font.size': 12})
plt.rc('text', usetex=True)
plt.rc('font', family='serif', serif='Computer Modern Roman')
#plt.rc('family',)
def parser():
    
    if (os.path.exists("Y:\\Masterarbeit\\07_Code\\Python\\vrp")):
        path2dir = "Y:\\Masterarbeit\\07_Code\\Python\\vrp"
        path2image = "Y:\\MASTERARBEIT\\05_Abschlussarbeit\\Latex\\Thesis\\images"
        
    else:
        os.path.exists("C:\\Users\\Saltuk\\Desktop\\Masterarbeit\\07_Code\\Python\\vrp")
        path2dir = "C:\\Users\\Saltuk\\Desktop\\Masterarbeit\\07_Code\\Python\\vrp"
        path2image = "C:\\Users\\Saltuk\\Desktop\\Masterarbeit\\05_Abschlussarbeit\\Latex\\Thesis2\\images" 
        
    Customer=namedtuple("Customer", ['index', 'x', 'y', 'time', 'label'])
    
    file_location_schedule = path2dir +  "\\master_schedule\\master_schedule.json"  
    file_location_customers = path2dir +  "\\data_own\\vrp_tiguan_new.txt"
        
    Customer=namedtuple("Customer", ['index', 'x', 'y', 'time', 'label'])
    
    #LOAD MASTER SCHEDULE
    with open(file_location_schedule) as json_data:
        master_schedule_json=json.load(json_data)
    master_schedule = [[Customer(**json.loads(i)) for i in j]
                               for j in master_schedule_json]
    #LOAD CUSTOMERS FOR DISTANCE MATRIX
    with open(file_location_customers, 'r') as input_data_file:
        input_data = input_data_file.read()
        
    lines = input_data.split('\n')
    
    parts = lines[0].split()
    customer_count = int(parts[0])
    vehicle_count = int(parts[1])
    customers = []  
    
    for i in range(2, customer_count+1):
        line = lines[i]
        parts = line.split()
        parts[-1] = parts[-1].split(',')
        parts[-1] = [int(i) for i in parts[-1]]
        parts[-1] = tuple(parts[-1])
        customers.append(Customer(i-2, float(parts[0]),
                         float(parts[1]), float(parts[2]), parts[3]))
        
    return customers, vehicle_count, master_schedule, path2dir, path2image

def event_manager(dynamic_schedule, vehicle_fail_index, j):
    
#   Gib die zu erledigenden Customer frei: 
    free_customers = dynamic_schedule[vehicle_fail_index][1:]  
    
#   Speichere die Labels, also die zugehörigen Klassen der Customer
    free_customer_labels = set()
    for customer in free_customers:
        free_customer_labels=free_customer_labels.union(customer.label)
    print('free_cust_labels:   \n {}'.format(free_customer_labels))       
#   Die Roboter als Anlagen: [0 10], [1 11]... 
    vehicle_station_list = [0,10,1,11,2,12,3,13,4,14,5,15,6,16,7,17,8,18,9,19]  
    
##   Die Position des Ausgefallenen Roboters in der vehicle_station_list
    vehicle_fail_idx = [i for i,x in enumerate(
            vehicle_station_list) if x==vehicle_fail_index]
        
#   Die noch verbleibenden Roboter zB.: v_f_i=6, dann: [17,8,18,9,19] 
#        [16,7,17,8,18,9,19] [15,16,7,17,8,18,9,19] ...
#   Hier kann man steuern, welche Roboter berücksichtigt werden       
    vehicle_station_list.pop(vehicle_fail_idx[0])
    available_vehicles = vehicle_station_list[j:]
    print('AVAILABLE VEHICLES: \n  {}'.format(available_vehicles))
    
#   Lösche den Roboter aus dem neuen Schedule raus
    del dynamic_schedule[vehicle_fail_index]
    
#   Neues Schedule mit nur übrigen Stationen
    dyn_sched = [i for i in dynamic_schedule if (
            i[0].index in available_vehicles)]

    # Aus übrigen Stationen, wähle die Roboter, die die Punkte schweißen können
#     Gibt die Indices für die in Frage kommenden Roboter raus bezogen auf dyn_sched:
#     Also wenn vehicle.index=6 an erster Stelle steht in dyn_sched und in Frage kommt
#     dann ist vehicle_indices = 0 unter anderem.
    
    # Wenn die free_customers label:0,1 haben, kommen alle Roboter in Frage, die noch übrig sind
    # label 0: linke Roboter
    # label 1: rechte Roboter
    vehicle_indices = [i for i,x in enumerate(dyn_sched) if (
            set(x[0].label).intersection(free_customer_labels))]
#    print('DYN_SCHED:   \n {}'.format(dyn_sched))    
    print('VEHICLE_INDICES ARE:   \n {}'.format(vehicle_indices))
    
    return dynamic_schedule, dyn_sched, free_customers, vehicle_indices  
    
def run_solver(dynamic_schedule, distance, free_customers,
               neighborhood, L_h, itermax, idle, vehicle_indices):

    vehicle_tours_reschedule, new_routes_index =\
    neighborhood.construction_heuristics_dynamic(
        dynamic_schedule, distance, free_customers)
      
    current_tour = deepcopy(vehicle_tours_reschedule)
    current_dist = distance.get_distance_all_tours(current_tour) 
    best_tour = current_tour.copy()
#    print(best_tour)
    best_dist    = current_dist 
    f_k = [best_dist]*L_h 
    iter_idle=0
    iter_ = 0
    xx = [0]
    yy = [current_dist]  
    
    while iter_ < itermax and iter_idle < itermax*idle:
        if(current_dist < best_dist):
            best_dist = current_dist
            best_tour = current_tour
        iter_ += 1

        n=random.randint(0,2)          
        N_dist, N_tour = neighborhood.generator_nearby(n, current_tour,
                                            vehicle_indices, distance)
        
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
        
    if(current_dist < best_dist):
        best_dist = current_dist
        best_tour = current_tour
  
    for j in range(len(best_tour)):
        best_tour[j] = neighborhood.run_2opt_hc(
                best_tour[j], distance) 

    return best_tour, best_dist, xx, yy

def plotter(xx, yy, free_customers, vehicle_tours_reschedule,
            vehicle_indices, path2image, j):
    #You must select the correct size of the plot in advance
    fig = plt.figure()
    #You must select the correct size of the plot in advance
    fig.set_size_inches(6, 4)
    plt.plot(xx,yy,label=r'$Kosten über Iteration$',linewidth=0.5,
             color='black')
    plt.xlabel('Iteration in 10^{-4}')
    plt.ylabel('Taktzeit in s')
    plt.tight_layout()
#    plt.savefig(path2image + '\\vf_4_iter'+str(j)+'.pdf', 
#            #This is simple recomendation for publication plots
#            dpi=1000, 
#            # Plot will be occupy a maximum of available space
#            bbox_inches='tight', 
#            )
#    plt.clf()
    
    fig = plt.figure()
    fig.set_size_inches(6, 4)
    ax = fig.add_subplot(111)
    fig.subplots_adjust(top=0.85)
    ax.set_xlabel('$x$-Koordinate in $mm$')
    ax.set_ylabel('$z$-Koordinate in $mm$')
    
    for v in range(len(vehicle_tours_reschedule)):
        x = [i.x for i in vehicle_tours_reschedule[v]]

        y = [i.y for i in vehicle_tours_reschedule[v]]

        a = ax.plot(x[1:],y[1:],'-o', lw=0.5, markersize=1, markerfacecolor='w',
                markeredgecolor = 'k')
        last_color = a[-1].get_color()
        ax.plot(x[0],y[0], 's', color='w',markeredgecolor = 'k', markersize=10)
        
        ax.plot(x[0:2],y[0:2], '--', color = last_color, linewidth=0.2)
        
        ax.plot([x[-1], x[0]], [y[-1], y[0]], '--', color=last_color, linewidth=0.2)


    x=[i.x for i in free_customers]
    y=[i.y for i in free_customers]
    ax.plot(x,y,'o', lw=0.5, markersize=3,markeredgewidth=0.5, markerfacecolor='r', 
            markeredgecolor = 'k')  
    
    legend_elements = [Line2D([0], [0], marker='o', color='k', lw=0,
                              label='Schweisspunkt',
                              markerfacecolor='w', markersize=2),
                       Line2D([0], [0], marker='s', color='k', lw=0,
                              label='Roboter-Base',
                              markerfacecolor='w', markersize=10)]
                       
    ax.legend(handles=legend_elements, loc='upper right', frameon=True)
    plt.grid(True, alpha=0.2)
    plt.tight_layout()
    plt.show()

    plt.savefig(path2image + '\\vf_4_plan_'+str(j)+'.pdf', 
            #This is simple recomendation for publication plots
            dpi=1000, 
            # Plot will be occupy a maximum of available space
            bbox_inches='tight', 
            )
    plt.clf()
    
def main():
    customers, vehicle_count, master_schedule, path2dir, path2image = parser()
    distance = DistanceEvaluator(customers)
    neighborhood = Neighbor()
    vehicle_fail_index  = 0
    idle = 0.02
    L_h = 50
    itermax = 10000
    schedule_times = []
    for j in range(0,vehicle_fail_index*2+2,2):
        if j==(vehicle_fail_index*2)+2:
            j+=1
        dynamic_schedule=deepcopy(master_schedule)
        dynamic_schedule, dyn_sched, free_customers, vehicle_indices = \
        event_manager(dynamic_schedule, vehicle_fail_index, j)
        
        vehicle_tours_reschedule, max_dist, xx, yy = run_solver(
                dyn_sched, distance, free_customers,neighborhood,
                L_h, itermax, idle, vehicle_indices)
        
        plotter(xx, yy, free_customers, vehicle_tours_reschedule,
                vehicle_indices, path2image, j)  
        
        print()
        print(max_dist)
        print()
        
        schedule_times.append([distance.get_distance_tour(i) \
                              for i in vehicle_tours_reschedule])
    
    schedule_times = [[np.round(float(i), 2) for i in nested]\
                       for nested in schedule_times]

    return master_schedule, vehicle_tours_reschedule, max_dist,\
            distance, free_customers, vehicle_indices, schedule_times
 
if __name__ == '__main__':
    master_schedule, vehicle_tours_reschedule, max_dist,\
            distance, free_customers, vehicle_indices, schedule_times = main()  
