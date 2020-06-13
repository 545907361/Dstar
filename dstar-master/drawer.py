import matplotlib.pyplot as plt
import threading
START = 10
GOAL  = 0
OBSTACLE = 1
FREE = 3
CLOSE = 8
PATH = 8

class map_drawer(threading.Thread):
    def __init__(self,g_map):
        threading.Thread.__init__(self)
        self.g_map = g_map
        self.fig, self.ax = plt.subplots()
        cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)
    def onclick(self,event):
        x, y = event.xdata, event.ydata
        if x == None or y == None:
             return
        ix = int(x+0.5)
        iy = int(y+0.5)
        redraw_flag = 0
        cell = self.g_map.map_data[iy][ix]
        if(event.button == 1):
            if cell.stat != 'GOAL' and cell.stat != 'START':
                cell.stat = 'OBSTACLE'
                redraw_flag = 1
        elif(event.button == 3):
            if cell.stat != 'GOAL' and cell.stat != 'START':
                cell.stat = 'FREE'
        else:
            #print 'Hi, %s. You are %d years old' % ('John', 25)
            print('cell(%d %d): h=%.1f k=%.1f' % (ix, iy, cell.h, cell.k))
        self.update_plot()

    def update_plot(self):
        plt.cla()
        map_data = self.g_map.map_data
        map_show_data = self.g_map.map_show_data
        for i in range(len(map_data)):
            for j in range(len(map_data[0])):
                
                if map_data[i][j].stat == 'FREE':   
                    map_show_data[i,j] = FREE
                elif map_data[i][j].stat == 'OBSTACLE':   
                    map_show_data[i,j] = OBSTACLE

                #if map_data[i][j].tag == 'CLOSE':
                #    self.ax.text(j, i, 'X', fontsize = 40 ,va='center', ha='center',color='red')
                if map_data[i][j].path:
                    map_show_data[i,j] = PATH
                if map_data[i][j].stat == 'GOAL':
                    self.ax.text(j, i, 'Goal', va='center', ha='center',color='white')
                    map_show_data[i,j] = GOAL
                elif map_data[i][j].stat == 'START':
                    self.ax.text(j, i, 'Start', va='center', ha='center',color='white')
                    map_show_data[i,j] = START
                    
                if map_data[i][j].back_point != map_data[i][j]:
                    a = map_data[i][j].back_point.row - map_data[i][j].row 
                    b = map_data[i][j].back_point.col - map_data[i][j].col
                    self.ax.quiver( map_data[i][j].col, map_data[i][j].row,b,-a,color ='white');
        self.ax.imshow(map_show_data, interpolation='none') 
        self.fig.canvas.draw()
    def run(self):
        self.update_plot()
        plt.show()
        
