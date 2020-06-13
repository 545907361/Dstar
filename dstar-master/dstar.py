import numpy as np
import time
import threading
import drawer
import matplotlib as mpl
mpl.use('Qt4Agg')
import matplotlib.pyplot as plt
import operator

import cv2
START = 10
GOAL  = 0
OBSTACLE = 1
FREE = 3
CLOSE = 8

INF = float("inf")

class map_cell:
    def __init__(self,x,y):
        
        self.back_point = self
        self.path = False
        self.tag = 'NEW'
        self.stat = 'FREE' # FREE OBSTACLE GOAL START
        self.h = 0
        self.k = 0
        self.row = x # rows
        self.col = y # cols

    def cost(self,to_ceil):                  
        a = abs(to_ceil.row - self.row)
        b = abs(to_ceil.col - self.col)
        if a > 1 or b > 1:
            print('COST ERROR')
            return -1
        elif to_ceil.stat  == 'OBSTACLE' or self.stat == 'OBSTACLE':
            return INF
        elif a == 0 or b == 0:
            return 1
        else:
            return 1.4

class grid_map:
    def __init__(self,rows,cols):
        self.map_data = []
        self.map_show_data = np.zeros((rows, cols))
        self.map_show_data.fill(FREE)

        for i in range(rows):
            line = []
            for j in range(cols):
                cell = map_cell(i,j)
                line.append(cell)
            self.map_data.append(line)

    def set_goal(self,y,x):
        self.map_data[y][x].stat = 'GOAL'
        self.goal = self.map_data[y][x]
        
    def set_start(self,y,x):
        self.map_data[y][x].stat = 'START'
        self.start = self.map_data[y][x]
                    

class dstar_planner(threading.Thread):
    def __init__(self,grid_map):
        threading.Thread.__init__(self)
        self.open = []
        self.drawer = drawer.map_drawer(grid_map)
        self.grid_map = grid_map
    def insert(self,cell,h_new):
        if cell.tag == 'NEW':
            cell.k = h_new
        elif cell.tag == 'OPEN':
            cell.k = min(h_new,cell.k)
        elif cell.tag == 'CLOSE':
            cell.k = min(h_new,cell.h)
        else:
            print('error_insert')
        cell.h = h_new
        if cell.tag != 'OPEN':
            cell.tag = 'OPEN'
            self.open.append(cell)
        self.open.sort(key=operator.attrgetter('k'))
    def neighbor(self,cell):
        tmp = []
        rows = len(self.grid_map.map_data)
        cols = len(self.grid_map.map_data[0])
        for i in range(-1,2):
            for j in range(-1,2):
                if (cell.row + i < rows ) and (cell.row + i >= 0 ) and (cell.col + j < cols ) and (cell.col + j >= 0 ): 
                    if i == 0 and j == 0:
                        continue
                    #if self.grid_map.map_data[cell.row + i][cell.col + j].tag == 'CLOSE':
                    #    continue
                    tmp.append(self.grid_map.map_data[cell.row + i][cell.col + j])
        return tmp

    def clear_close(self):
        rows = len(self.grid_map.map_data)
        cols = len(self.grid_map.map_data[0])
        for i in range(rows):
            for j in range(cols):
                if self.grid_map.map_data[i][j].tag == 'CLOSE':
                    self.grid_map.map_data[i][j].tag = 'NEW'

    def process_state(self,end_cell):
        if len(self.open) == 0:
            return -1
        self.open.sort(key=operator.attrgetter('k'))
        cell_c = self.open[0]
        del self.open[0]
        cell_c.tag = 'CLOSE'
        if cell_c == end_cell:
            return -1
        if cell_c == None or cell_c == end_cell:
            return -1
        k_old = cell_c.k
        neighbor = self.neighbor(cell_c)
        if k_old < cell_c.h:
            for cell in neighbor:
                if cell.tag == 'NEW':
                    #print 'TODO'
                    cell_c.back_point = cell
                    #cell_c.h = cell_c.cost(cell) + cell.h
                    self.insert(cell, cell_c.cost(cell) + cell_c.h)
                elif  (cell.back_point != cell_c and cell_c.h > cell_c.cost(cell) + cell.h):
                    cell_c.back_point = cell
                    #cell_c.h = cell_c.cost(cell) + cell.h
                    self.insert(cell, cell.h)
                elif cell.back_point == cell_c and cell.h != cell_c.cost(cell) + cell_c.h:
                    self.insert(cell, cell_c.cost(cell) + cell_c.h)
                    
        if k_old == cell_c.h:
            for cell in neighbor:
                if cell.tag == 'NEW' or (cell.back_point == cell_c and cell.h != cell_c.cost(cell) + cell_c.h) or (cell.back_point != cell_c and cell.h >  cell_c.cost(cell) + cell_c.h):
                    cell.back_point = cell_c
                    self.insert(cell, cell_c.cost(cell) + cell_c.h)
                    
        return k_old
    def init_plan(self):
        k = 0
        while k != -1:
            #raw_input('Press any key to next step ...')
            k = planner.process_state(self.grid_map.start)
            #self.drawer.update_plot()

    def sensor(self,cell):
            if (cell.k == INF and cell.stat != 'OBSTACLE') or (cell.k != INF and cell.stat == 'OBSTACLE'):
                self.insert(cell,cell.h)
                neighbor = self.neighbor(cell)
                for c in neighbor:
                    if c.back_point == cell:
                        self.insert(c,cell.cost(c) + cell.h)
                    else:
                        self.insert(c,c.h)
                return True
            else:
                return False

    def run(self):
        print('============= init plan start ===============')
        self.init_plan()
        self.drawer.update_plot()
        print('============= init plan end =================')
        cell_c = self.grid_map.start
        while cell_c != self.grid_map.goal:
            next_cell = cell_c.back_point
            if self.sensor(next_cell):
                print('oooh! have a OBSTACLE, try to compute new path')
                k = 0
                while k != -1:
                    k = self.process_state(None)
                    if cell_c.h < k:
                        break
                    #raw_input()
                    #self.drawer.update_plot()
            cell_c = cell_c.back_point
            cell_c.path = True
            self.drawer.update_plot()
            print('Press any key to go...')
            input()

                
if __name__ == '__main__':
    grid_map = grid_map(10,10)
    grid_map.set_goal(0,6)
    grid_map.set_start(5,1)
    grid_map.map_data[0][1].stat = 'OBSTACLE'
    grid_map.map_data[0][2].stat = 'OBSTACLE'
    grid_map.map_data[1][1].stat = 'OBSTACLE'
    grid_map.map_data[1][2].stat = 'OBSTACLE'
    grid_map.map_data[2][1].stat = 'OBSTACLE'
    grid_map.map_data[2][2].stat = 'OBSTACLE'
    grid_map.map_data[3][1].stat = 'OBSTACLE'
    grid_map.map_data[3][2].stat = 'OBSTACLE'
    grid_map.map_data[4][3].stat = 'OBSTACLE'


    planner = dstar_planner(grid_map)
    planner.insert(grid_map.goal,0)
    planner.setDaemon(True)
    planner.start()
    try : 
        planner.drawer.run()
    except KeyboardInterrupt:
        print("Bye")
