#coding=utf-8
 
import math
import numpy as np
from decimal import Decimal
import math
import time
import math
import multiprocessing
import sys
import pathlib
import pickle
import os
import datetime
import shutil
import matplotlib.style
from matplotlib.offsetbox import AnchoredText
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Rectangle
from matplotlib.animation import FFMpegWriter
import cv2
import random
import threading
from plot import plot_trajectory
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
#from qt_material import apply_stylesheet


from PyQt5.QtCore import Qt, pyqtSlot, QThread, pyqtSignal, QObject
from PyQt5.QtWidgets import (QVBoxLayout, QHBoxLayout, QFormLayout, QGroupBox,
                             QComboBox, QSpinBox, QDoubleSpinBox, QLabel,
                             QProgressBar, QPushButton, QRadioButton,
                             QCheckBox, QFrame, QWidget, QApplication,
                             QMainWindow, QSizePolicy, QFileDialog, QTableWidget,
                             QStackedWidget, QDialog, QTableWidgetItem,
                             QDoubleSpinBox)
from test import PLAN



class Model2(QObject):
    finished = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.data = None
            
    def solve(self,Num, ini_x , ini_v , 
            target , r_min,epsilon, h , K , episodes ):
    
        result,agent_list = PLAN(Num=Num, ini_x = ini_x,
                    ini_v = ini_v, target = target,r_min=r_min,epsilon=epsilon,h=h,K=K,episodes=episodes)
        self.data = result
        self.agent_list = agent_list
        self.r_min = r_min 
        self.finished.emit()
        
    def output(self, drone_id, index):
        drone_loc = self.data[index]['position_list'][drone_id][-1]
        drone_future_path = self.data[index]['pos_list'][drone_id]
        terminal_index = self.data[index]['terminal_index_list'][drone_id]
        return drone_loc, drone_future_path, terminal_index
    
    def start_point(self, drone_id):
        return self.data[0]['position_list'][drone_id][0]

    def save_trajectory(self):
        plot_trajectory(self.agent_list,self.r_min)
        print('trajectory has been saved in \'trajecotry.svg\' ')
    
    @property
    def goal(self):
        return self.data['goal']

    @property
    def dronecount(self):
        
        return len(self.data[0]['terminal_index_list'])
    
    @property
    def count(self):
        return len(self.data) - 1
    

class MatrixDialog(QDialog):
    
    
    def __init__(self, features, row_count, title,  matrix=None):
        super().__init__()
        self.matrix = matrix
        self.column_count = len(features)
        self.row_count = row_count
        self.features = features
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)
        self.setWindowTitle(title)
        
        self.__createTable()
        
    def __createTable(self):
        
        self.tbl = QTableWidget()
        self.tbl.setRowCount(self.row_count)
        self.tbl.setColumnCount(self.column_count)
        
        self.tbl.setHorizontalHeaderLabels(self.features)
        self.tbl.setVerticalHeaderLabels(list(map(str, range(self.row_count))))
        
        for i in range(self.row_count):
            for j in range(self.column_count):
                if self.data is not None:
                    try:
                        text = str(self.matrix[i, j])
                    except:
                        text = '0'
                item = QTableWidgetItem(text)
                self.tbl.setItem(i, j, item)
        self.layout.addWidget(self.tbl)

    def read_table(self):
        table_data = np.zeros((self.row_count, self.column_count))
        for i in range(self.row_count):
            for j in range(self.column_count):
                item = self.tbl.item(i, j)
                table_data[i, j] = self.get_data(item)
        return table_data
    
    def data(self):
        return self.read_table()

    def get_data(self, item):
        text = item.text()
        try:
            value = float(text)
        except ValueError:
            if '/' in text:
                value = eval(text)
            else:
                value = 0
        return value


class RunDrone2(QThread):
    sig_drone = pyqtSignal(object, object, object, list, int)
    
    def __init__(self, drones, model, fps):
        super().__init__()
        self.drone_num = len(drones)
        self.drones = drones
        self.model = model
        self.abort = False
        self.waiting_time = 1/fps
    
    @pyqtSlot()
    def run(self):
        i = 0
        while True:
            time.sleep(self.waiting_time)
            if self.abort:
                break
            drone_trace = []
            drone_pos = []
            drone_future = []
            drone_terminal = []
            for j in range(self.drone_num):
                pos, future_path, terminal_index = self.model.output(j, i)
                drone_pos.append(pos)
                drone_future.append(future_path)
                drone_trace.append(self.drones[j].trace_data)
                drone_terminal.append(terminal_index)
                
                self.drones[j].move(pos)
                self.drones[j].future_path = future_path
            self.sig_drone.emit(
                drone_pos,
                drone_trace,
                drone_future,
                drone_terminal,
                i
            )
            i += 1
            if i >= self.model.count:
                break
            
    @pyqtSlot()
    def stop(self):
        self.abort = True


class Drone(object):
    
    def __init__(self, pos, radius):
        
        self.pos = list(pos)
        self.radius = radius
        self.trace_data = []
        self.__future_path = None
        
    def move(self, new_pos):
        if len(new_pos) > 0:
            ###############
            self.pos[0] = new_pos[0]
            self.pos[1] = new_pos[1]
            self.trace_data.append(new_pos)
           
    @property
    def future_path(self):
        return self.__future_path
    
    @future_path.setter
    def future_path(self, path):
        self.__future_path = path


class DroneSimulatorPlot(FigureCanvas):
    COLORMAP = [ '#1f77b4',
        '#ff7f0e', 
        '#2ca02c', 
        '#d62728',
        '#9467bd',
        '#8c564b', 
        '#e377c2',
        '#7f7f7f', 
        '#bcbd22',
        '#17becf',
        '#2F4F4F',
        '#CD5C5C',
        '#ADD8E6',
        '#663399',
        '#8FBC8F',
        '#00CED1',
        '#6A5ACD',
        '#808000',
        '#A0522D',
        '#FF4500',
        '#708090',
        '#BDB76B',
        '#FF6347',
        '#E9967A',
        '#F5DEB3',
        '#FFB6C1',
        '#556B2F',
        '#008080',
        '#7FFF00',
        '#FFA500',
        '#FF8C00',
        '#00FF7F',
        '#C0C0C0',
        '#483D8B',
        '#F08080',
        '#D3D3D3',
        '#66CDAA',
        '#FA8072',
        '#F4A460',
        '#48D1CC',
        '#8A2BE2',
        '#2E8B57']
                    
    def __init__(self):
        self.fig = Figure(figsize=(10, 10), dpi=200)
        self.axes = self.fig.add_subplot(111, aspect='equal')
        self.axes.set_xlim([-0.5,2.5])
        self.axes.set_ylim([-0.5,2.5])
        self.axes.set_xlabel('x(m)')
        self.axes.set_ylabel('y(m)')

        super().__init__(self.fig)
    
        self.setMinimumWidth(700)
        self.setMinimumHeight(700)
        FigureCanvas.setSizePolicy(self,
                                   QSizePolicy.Expanding,
                                   QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)
        
        self.__drone = {}
        self.__traces = {}
        self.__future_path = {}
        self.__paths = []
        self.__radius = 0.3

    ############## 这是画线阶段
    def paint_drone2(self, drone_pos, drone_trace, drone_future, drone_terminal,i):
         
        for drone_id, (pos, trace_data, future_path, terminal_index) in enumerate(zip(
            drone_pos, drone_trace, drone_future, drone_terminal
        )):
            
            try:
                self.__future_path[drone_id].remove()
                self.__traces[drone_id].remove()
                self.__drone[drone_id].remove()
            except (AttributeError, ValueError, KeyError):
                pass
             
            self.__drone[drone_id] = Circle(pos, radius = self.radius, edgecolor='black',
                                            facecolor=self.COLORMAP[drone_id], zorder=4)

            # if terminal_index > 0:
            #     self.__future_path[drone_id] = Line2D(*zip(*future_path), linestyle=":",
            #                                         color=self.COLORMAP[drone_id], lw=2)
            #     self.axes.add_line(self.__future_path[drone_id])

            self.__future_path[drone_id] = Line2D(*zip(*future_path), linestyle=":",
                color=self.COLORMAP[drone_id], lw=2)
            
            self.axes.add_line(self.__future_path[drone_id])

            self.__traces[drone_id] = Line2D(*zip(*trace_data), linestyle="-", color=self.COLORMAP[drone_id], lw=2)
            self.axes.add_artist(self.__drone[drone_id])
            self.axes.add_line(self.__traces[drone_id])

        self.draw()
        self.fig.savefig('savefig/sanpshot%s.svg'%(i))
    
    def paint_legend(self, num):
        pass
        """
        for i in range(num):
            self.axes.scatter([], [], c=self.COLORMAP[i], alpha=0.7, edgecolors='black',
                        label= 'robot ' + str(i+1))
        self.axes.legend(scatterpoints=1, frameon=False, labelspacing=1)
        """
        
    def paint_goal(self, data):
        #############
        self.axes.cla()
        i=0
        for x, y in data:
            i+=1
            self.axes.plot(x, y, marker="d", markersize=7, color=self.COLORMAP[i-1],markeredgecolor='k', zorder=5)
        self.axes.set_xlim([-0.5,2.5])
        self.axes.set_ylim([-0.5,2.5])
        self.axes.set_xlabel('x(m)')
        self.axes.set_ylabel('y(m)')
            
    def paint_drone_finished(self, drone_id):
        self.__drone[drone_id].set_color('tomato')
        self.draw()
    

        
    @property
    def radius(self):
        return self.__radius
    
    @radius.setter
    def radius(self, value):
        self.__radius = value

    @property
    def frame(self):
        rgba_buff=self.buffer_rgba()
        (w,h)=self.get_width_height()
        mat=np.frombuffer(rgba_buff, dtype = np.uint8).reshape((h,w,4))
        return mat
    @property
    #def frame(self):
    #    mat=np.asarray(self.buffer_rgba())
        #        mat = np.array(self.renderer.buffer_rgba())
    #    return mat
    
    @property
    def frame_size(self):
        return self.get_width_height()
    

class Panel(QFrame):
    
    def __init__(self):
        super().__init__()
        self._layout = QVBoxLayout()
        self.setLayout(self._layout)
        self._layout.setContentsMargins(0, 0, 0, 0)


class ControlPanel(Panel):
    
    def __init__(self, panel_animation):
        super().__init__()
        self.panel_animation = panel_animation
        # self.model = Model()
        self.model2 = Model2()
        self.__setup_ui()
        self.__init_widgets()
          
    def __setup_ui(self):
        model_box = QGroupBox('PLAN')
        model_box_layout = QVBoxLayout()
        model_box.setLayout(model_box_layout)
        
        execution_box = QGroupBox(' ')
        execution_box_layout = QVBoxLayout()
        execution_box.setLayout(execution_box_layout)
        # execution_box.setContentsMargins(0, 0, 0, 0)
        
        self.model_selector = QComboBox()
        self.model_selector.addItems(['2D Planning'])
        self.model_selector.currentIndexChanged.connect(self.__display_option)
        
        self.solve_btn = QPushButton('Solve')
        self.solve_btn.clicked.connect(self.solve_model)
        
        self.start_btn = QPushButton('Start animation')
        self.start_btn.clicked.connect(self.run)
        
        self.stop_btn = QPushButton('Stop animation')
        
        self.savevideo_btn = QPushButton('Save animation')
        self.savevideo_btn.clicked.connect(self.panel_animation.save_video)

        self.savetrack_btn = QPushButton('Save trajectory')
        self.savetrack_btn.clicked.connect(self.model2.save_trajectory)

        model_box_layout.addWidget(self.model_selector)
        
        execution_box_layout.addWidget((self.solve_btn))
        execution_box_layout.addWidget(self.start_btn)
        execution_box_layout.addWidget(self.stop_btn)
        execution_box_layout.addWidget(self.savevideo_btn)
        execution_box_layout.addWidget(self.savetrack_btn)
        
        parameter_box = QGroupBox('Animation setting')
        parameter_box_layout = QFormLayout(labelAlignment=Qt.AlignLeft, fieldGrowthPolicy=QFormLayout.AllNonFixedFieldsGrow)
        parameter_box.setLayout(parameter_box_layout)
        
        self.stack = QStackedWidget()
    
        
        #========================================
        model2_box = QGroupBox('MODEL')
        model2_layout = QFormLayout(labelAlignment=Qt.AlignLeft, fieldGrowthPolicy=QFormLayout.AllNonFixedFieldsGrow)
        model2_box.setLayout(model2_layout)

        self.episodes = QSpinBox()
        self.episodes.setRange(0, 1000)
        self.episodes.setValue(100)
        
        self.Num = QSpinBox()
        self.Num.setRange(0, 100)
        self.Num.setValue(1)
        
        self.h = QDoubleSpinBox()
        self.h.setRange(0,1)
        self.h.setValue(0.2)
        self.h.setSingleStep(0.01)


        
        self.K = QSpinBox()
        self.K.setRange(1, 30)
        self.K.setValue(12)

        self.r_min = QDoubleSpinBox()
        self.r_min.setRange(0, 10)
        self.r_min.setValue(0.3)
        self.r_min.setSingleStep(0.01)
    
        self.epsilon = QDoubleSpinBox()
        self.epsilon.setRange(0,5)
        self.epsilon.setValue(0.1)
        self.epsilon.setSingleStep(0.01)
        
        self.goal_btn = QPushButton('Set target')
        self.goal_btn.clicked.connect(self.set_goal)
        self.goal = [
            np.array([2.0,2.0]),np.array([0.0,2.0]),\
            np.array([2.0,0.0]),np.array([0.0,0.0]),\
            np.array([2.0,1.0]),np.array([0.0,1.5]),\
            np.array([1.0,2.0]),np.array([0.0,2.0])
            ]
    
        
        self.ini_state = [
            [0.0,0.0,0.0,0.0],
            [2.0,0.0,0.0,0.0],
            [0.0,2.0,0.0,0.0],
            [2.0,2.0,0.0,0.0],
            [0.0,0.5,0.0,0.0],
            [1.5,1.0,0.0,0.0],
            [0.0,1.5,0.0,0.0],
            [2.0,2.0,0.0,0.0]
        ]
        
            
    

        self.ini_x=None 
        self.ini_v=None

        self.drone_prop_btn = QPushButton('Set original sate')
        self.drone_prop_btn.clicked.connect(self.set_drone_prop)

        self.random_ini_btn = QPushButton('Random initial state')
        self.random_ini_btn.clicked.connect(self.set_random_ini)

        self.random_target_btn = QPushButton('Random target')
        self.random_target_btn.clicked.connect(self.set_random_target)
        
        model2_layout.addRow('Max step', self.episodes)
        model2_layout.addRow('Number', self.Num)
        model2_layout.addRow('Initial state', self.drone_prop_btn)
        model2_layout.addRow('--->',self.random_ini_btn)
        model2_layout.addRow('h', self.h)
        model2_layout.addRow('K', self.K)
        model2_layout.addRow('r_min', self.r_min)
        model2_layout.addRow('epsilon', self.epsilon)
        model2_layout.addRow('target', self.goal_btn)
        model2_layout.addRow('--->',self.random_target_btn)


        
        
        self.stack.addWidget(model2_box)
        self.stack.setCurrentIndex(0)
        
        
        
        self.time = QDoubleSpinBox()
        self.time.setRange(0,5)
        self.time.setValue(1)
        self.time.setSingleStep(0.1)
        
        parameter_box_layout.addRow('Rate to play：', self.time)

       
       
        parameter_box.setSizePolicy(QSizePolicy.Preferred,
                                   QSizePolicy.Expanding) 
        
        self._layout.addWidget(model_box)
        self._layout.addWidget(self.stack)
        
        self._layout.addWidget(parameter_box)
        self._layout.addWidget(execution_box)
        
    def __display_option(self, i):
        self.stack.setCurrentIndex(i)
        self.radius.setValue(0.3)
        
        
    def set_drone_prop(self):
        drone_num = self.Num.value()
        if drone_num:
            
            features = ['x', 'y', 'v_x', 'v_y']
            dialog = MatrixDialog(features, drone_num, ' ', np.array(self.ini_state))

            if not dialog.exec_():
            
                self.ini_x = [i[:2] for i in dialog.data()]
                self.ini_v = [i[2:4] for i in dialog.data()]
                self.ini_state = [i for i in dialog.data()]
        else:
            print("Please set number of robot")


    def set_goal(self):
        goal_num = self.Num.value()
        if goal_num:
            features = ['x', 'y']
            dialog = MatrixDialog(features, goal_num, ' ', np.array(self.goal))

            if not dialog.exec_():
                self.goal = [i for i in dialog.data()]
        else:
            print("Please set number of robot")


    def set_random_ini(self):
        Num = self.Num.value()
        r_min = self.r_min.value()
        
        if Num:
            for times in range(100):

                ini_x=[]

                for i in range(Num):

                    for j in range(1000): 
                    
                        ini=np.random.rand(2)*np.array([2,2])
                        RIGHT=True

                        for k in range(len(ini_x)):
                            if(np.linalg.norm(ini-ini_x[k])<r_min+0.05):
                                RIGHT=False 
                        
                        if RIGHT:
                            ini_x+=[ini]
                            break 

                if len(ini_x)==Num:
                    print("get intial positions")
                    break 

                if (times+1)%10==0:
                    print('Try %s times, cannot find. Keep trying.'%(times+1))

            if times == 99:
                print('cannot find initial positions, please decrease the number of robots')
        else:

            print("Please set the number of robots")

        self.ini_x=ini_x 

        self.ini_v=[]
        for i in range(Num):
            self.ini_v+=[np.zeros(2)] 



    def set_random_target(self):
        r_min=self.r_min.value()
        Num = self.Num.value()
        epsilon = self.epsilon.value()
        if Num:
            for times in range(100):

                target=[]

                for i in range(Num):

                    for j in range(1000): 
                    
                        tar=np.random.rand(2)*np.array([2,2])
                        RIGHT=True

                        for k in range(len(target)):
                            if(np.linalg.norm(tar-target[k])<r_min+2*epsilon):
                                RIGHT=False 
                        
                        if RIGHT:
                            target+=[tar]
                            break

                if len(target)==Num:
                    print("get target positions")
                    break
                if (times+1)%10==0:
                    print('Try %s times, cannot find. Keep trying.'%(times+1))
            if times == 99:
                print('cannot find target positions, please decrease the number of robots')
        else:
            
            print("Please set the number of robots")

        self.goal=target
        

                  
    
    def __set_background(self, *map_data):
        self.panel_animation.setup_background(*map_data)
        
    def __set_goal(self, data):
        self.panel_animation.setup_goal(data)
        
    def __set_legend(self, num):
        self.panel_animation.setup_legend(num)
        
    @pyqtSlot(list, list)
    def __move_drone(self, pos, data):
        self.panel_animation.move_drone(pos, data)
        
    @pyqtSlot(object, object, object, list,int)
    def __move_drone2(self, pos, trace_data, future_path, terminal_index,i):
        self.panel_animation.move_drone2(pos, trace_data, future_path, terminal_index,i)
        

    def solve_model(self):
        
        self.__init_widgets()
        self.thread = QThread()
        self.thread.finished.connect(self.__reset_widgets)
        
        if self.ini_state == 0 or self.goal == 0:
            return
        
        
        Num = self.Num.value()
        r_min = self.r_min.value()
        epsilon = self.epsilon.value()
        h = self.h.value()
        K = self.K.value()
        episodes=self.episodes.value()

        ini_x=self.ini_x
        ini_v=self.ini_v


        target = self.goal

        if ini_x == None:
            print("Please set initial state")
            return 
        
        if target == None:
            print("Please set target")
            return

        if Num == len(ini_x) and Num==len(target):

            Collision=False

            
            for i in range(1,Num):
                for j in range(i):
                    
                    if(np.linalg.norm(ini_x[i]-ini_x[j])<r_min+0.02):
                        Collision=True
                        break 
                    
                if Collision:
                    break 
                

            if Collision:
                print("Initial position is in valid")
                return 

            
            Collision=False

            
            for i in range(1,Num):
                for j in range(i):
                    if(np.linalg.norm(target[i]-target[j])<r_min+2*epsilon):
                        Collision=True
                        break 
                if Collision:
                    break 
                 
            
            if Collision:
                print("Target is invalid")
                return  


            self.model2.moveToThread(self.thread)
            self.thread.started.connect(lambda: self.model2.solve(
                Num=Num, ini_x = ini_x,
                ini_v = ini_v, target = target,
                r_min=r_min,epsilon=epsilon,
                h=h,K=K,episodes=episodes))

            self.model2.finished.connect(self.thread.quit)
            # self.thread.finished.connect(self.thread.deleteLater)
            # self.model2.finished.connect(self.model2.deleteLater)
            self.thread.start()
        else:
            print("The number of robot is not equal to target or initial state")       
        
            
    def __init_widgets(self):
        self.start_btn.setDisabled(True)
        self.stop_btn.setDisabled(True)
        self.savevideo_btn.setDisabled(True)
        self.savetrack_btn.setDisabled(True)
        
    @pyqtSlot()
    def __reset_widgets(self):
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(True)
        self.savevideo_btn.setEnabled(True)
        self.savetrack_btn.setEnabled(True)
        
    @pyqtSlot()
    def run(self):

        if os.path.exists('savefig'):
            shutil.rmtree('savefig')
        os.mkdir('savefig')

        radius = self.r_min.value()/2.0
        self.panel_animation.fps = 1/self.h.value()*self.time.value()
        self.panel_animation.set_radius(radius)
        self.panel_animation.clear_videobuffer()
        
        self.__run2()
       
    
    def __run2(self):
        self.__drones = {}
        for i in range(self.model2.dronecount):
            start_point = self.model2.start_point(i)
            self.__drones[i] = Drone(start_point, 0.15)
            self.__drones[i].move(start_point)
        self.__set_goal(self.model2.goal[0:self.model2.dronecount])
        self.__set_legend(self.model2.dronecount)
        self.__thread = RunDrone2(self.__drones, self.model2, fps=1/self.h.value()*self.time.value())
        self.stop_btn.clicked.connect(self.__thread.stop)
        self.__thread.sig_drone.connect(self.__move_drone2)
        self.__thread.start()


class AnimationPanel(Panel):
    
    def __init__(self):
        super().__init__()
        self.__setup_ui()
        self.video_buffer = []
        self.__fps = 5
        
    @property
    def fps(self):
        return self.__fps
    
    @fps.setter
    def fps(self, value):
        self.__fps = value

    def set_radius(self, value):
        self.simulator.radius = value

    def __setup_ui(self):
        self.simulator = DroneSimulatorPlot()
        
        self._layout.addWidget(self.simulator)
                
    def setup_background(self, *data):
        self.simulator.paint_map(*data)
        self.__capture()
        
    def setup_goal(self, data):
        self.simulator.paint_goal(data)
        # self.__capture()
        
    def setup_legend(self, num):
        self.simulator.paint_legend(num)
        # self.__capture()
        
    def move_drone(self, pos, data):
        self.simulator.paint_drone(pos, data)
        self.__capture()
        
    def move_drone2(self, pos, trace_data, future_path, terminal_index,i):
        self.simulator.paint_drone2(pos, trace_data, future_path, terminal_index,i)
        self.__capture()
        
    def __capture(self):
        mat = self.simulator.frame
        mat = cv2.cvtColor(mat, cv2.COLOR_BGRA2RGB)
        self.video_buffer.append(mat)
        
    def clear_videobuffer(self):
        self.video_buffer = []
         
    def save_video(self, current_dir=None, ext=None):
        # with open('test.pkl', 'wb') as f:
        #     pickle.dump(self.video_buffer, f)
        
        
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')

        size=self.video_buffer[0].shape
        frameSize=(size[1],size[0])
        out = cv2.VideoWriter("result01.avi", fourcc, self.__fps, frameSize)
        # self.simulator.frame_size()
        for frame in self.video_buffer:
            out.write(frame)
        out.release()
        
        print("animation have been saved in \'result01.avi\'")

    

class BaseWidget(QWidget):
    def __init__(self):
        super().__init__()
        layout = QHBoxLayout()
        panel_animation = AnimationPanel()
        panel_controler = ControlPanel(panel_animation)

        layout.addWidget(panel_controler)
        layout.addWidget(panel_animation)
        
        self.setLayout(layout)


def main():
    app = QApplication(sys.argv)
    # apply_stylesheet(app, theme='light_blue.xml')
    window = QMainWindow()
    window.setCentralWidget(BaseWidget())
    window.show()
    sys.exit(app.exec_())



if __name__ == '__main__':

    multiprocessing.freeze_support()
    main()
    
