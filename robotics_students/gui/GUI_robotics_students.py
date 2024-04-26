# GUI_robotics_students.py 
'''
@author: Jesus Savage, UNAM-FI, 6-2024
'''
from MobileRobotSimulator import *
import os
import Tkinter as tk
import tkFileDialog
import tkMessageBox
from Tkinter import *
import math
from random import randrange, uniform
import time
import os
import numpy as np
from initial_behaviors import *
import time
#from PIL import Image
#from PIL import ImageDraw
 
 


#---------------------------------------------------------------------------------------
#	Global Variables

use_gui = True
gui = None
gui_planner = None
gui_example = None
debug = False
count = 0
DIM_CANVAS_X = 400
DIM_CANVAS_Y = 400
dim_x = 10.0
dim_y = 10.0
radio_robot = .03
pose_x = 4.0
pose_y = 3.0
pose_tetha = 0.0
mouse_1_x = 0.0
mouse_1_y = 0.0
mouse_2_x = 0.0
mouse_2_y = 0.0
mouse_3_x = 0.0
mouse_3_y = 0.0
num_pol = 0
polygons = []
flg_mov = 1
flg_sensor = 1
delay = .05
PATH = 'path'
File_Name = 'file_name'
File_Name_robot = 'file_name'
angle_robot = 0.0
sensor = "laser"
num_sensors = 2
flg_noise = 1
start_angle = -0.1
range_angle = 0.2
flg_execute = 1
robot_command = "../motion_planner/GoTo_State_Machine"
flg_plt = 1
vq = 0
#size_vq = 4
#pr_out = 0
flg_unk = 0
#number_unk = 0
#previous_num = 0
#previous_data = [0] * 2
number_steps_total = 100
varShowNodes   = False




#-------------------------------------------------------------------------------------------
#	TK Definitions

planner = tk.Tk()
planner.wm_title('PLANNER')
C = tk.Canvas(planner, bg="green", height=DIM_CANVAS_X, width=DIM_CANVAS_Y)


class PLANNER(object):
   

    def __init__(self):

	global C   
	global DIM_CANVAS_X 
	global DIM_CANVAS_Y
	global dim_x
	global dim_y
	global x
	global y
	global radio_robot
	global pose_x
	global pose_y
	global pose_tetha


 
	def callback_mouse_1(event):
		global mouse_1_x
		global mouse_1_y
		global angle_robot

		#print "clicked 1 at", event.x, event.y
		id = C.create_rectangle(event.x, event.y, event.x+1, event.y+1, fill= "black")
		x = (dim_x * event.x) / DIM_CANVAS_X
		y = (dim_y * (DIM_CANVAS_Y-event.y)) / DIM_CANVAS_Y
		print "left button x ", x, " y ", y
		mouse_1_x = x
		mouse_1_y = y
        	angle_robot = float(self.robot_angle.get())
  		#angle_robot = 0 

 
	def callback_mouse_2(event):
		global mouse_2_x
		global mouse_2_y

		#print "clicked 2 at", event.x, event.y
		id = C.create_rectangle(event.x, event.y, event.x+1, event.y+1, fill= "green", outline='yellow')
		x = (dim_x * event.x) / DIM_CANVAS_X
		y = (dim_y * (DIM_CANVAS_Y-event.y)) / DIM_CANVAS_Y
		print "middle button x ", x, " y ", y
		mouse_2_x = x
		mouse_2_y = y

    
	def callback_mouse_3(event):
		global mouse_3_x
		global mouse_3_y
		global flg_plt
		global BEHAVIOR



		#print "clicked 3 at", event.x, event.y
		id = C.create_rectangle(event.x, event.y, event.x+1, event.y+1, fill= "red", outline='red')
		x = (dim_x * event.x) / DIM_CANVAS_X
		y = (dim_y * (DIM_CANVAS_Y-event.y)) / DIM_CANVAS_Y
		print "right button x ", x, " y ", y
		mouse_3_x = x
		mouse_3_y = y

		if flg_execute == 1:
			if self.var_mov.get() == 0:
                                C.update_idletasks()

			flg_plt = 1

			self.togglePlotExecute(0,BEHAVIOR)
        
			PATH = self.path.get()
			#print 'Evaluate Robot PATH ',PATH
        		File_Name = self.file.get()
        		FILE = PATH + File_Name + '.raw'
        		#print 'Evaluate Robot File_Name ',FILE

                        File_Constants = PATH + 'Constants.txt'
                        partial_evaluation = self.readResultFile(FILE,File_Constants,0)
			print 'Evaluation ',partial_evaluation
			self.evaluation_ind.delete(0, END)
			self.evaluation_ind.insert ( 0, str(partial_evaluation))


	def initial(self):
       		global PATH
		global File_Name
		global File_Name_robot
       		global flg_mov
       		global flg_sensor
		global delay
 		global flg_plt
		global new_generation
		global flg_noise
		global BEHAVIOR
		global size_vq_ext
		global bh



		self.topLevelWindow = tk.Tk()
        	self.topLevelWindow.wm_title('GUI_ROBOTS')



#---------------------------------------------- Buttons Fields --------------------------------------------------------------------------------------------


		# Plot Robot button
        	self.RobotButton = tk.Button(self.topLevelWindow, width = 20, text = 'Plot Robot Behavior', bg = 'green', activebackground = 'green', command = self.togglePlotRobot)
		self.countRobot = 0
		# Plot Sensor button
        	self.ExecuteButton = tk.Button(self.topLevelWindow, width = 20, text = 'Execute Robot Command ', bg = 'green', activebackground = 'green', command = self.togglePlotExecute)
		self.countExecute = 0
		# Plot Map button
        	self.MapButton = tk.Button(self.topLevelWindow, width = 20, text = 'Plot Map', bg = 'green', activebackground = 'green', command = self.togglePlotMap)
		self.countMap = 0
		# Plot Path button
        	self.PathButton = tk.Button(self.topLevelWindow, width = 20, text = 'Plot Path', bg = 'green', activebackground = 'green', command = self.togglePlotPath)
		self.countPath = 0
      		# Path files entry 
		self.label_path = tk.Label(self.topLevelWindow,text =  'Path')
        	self.path = tk.Entry(self.topLevelWindow, width = 30, foreground='white',background='black')
		#self.path.insert ( 0, '/home/biorobotica/data/data_15/' )
		self.path.insert ( 0, INITIAL_PATH )
		PATH = self.path.get()
      		# World's File entry 
		self.label_file = tk.Label(self.topLevelWindow,text =  'World description')
        	self.file = tk.Entry(self.topLevelWindow, width = 30, foreground='white',background='black')
		#self.file.insert ( 0, 'room' )
		#self.file.insert ( 0, 'random' )
		self.file.insert ( 0, ENVIRONMENT )
		File_Name = self.file.get()
      		# Robot's File entry 
		self.label_file_robot = tk.Label(self.topLevelWindow,text =  'Robot Behavior File ')
        	self.file_robot = tk.Entry(self.topLevelWindow, width = 30, foreground='white',background='black')
		#self.file_robot.insert ( 0, 'random' )
		self.file_robot.insert ( 0, ENVIRONMENT)
		File_Name_robot = self.file_robot.get()
		# Check button movement
		self.var_mov = IntVar()
		def command_mov():
			#print "Checkbutton variable is", self.var_mov.get()
			if self.var_mov.get() == 0:
				self.Movement.select()
                		self.var_mov.set(1)
			else:
				self.Movement.deselect()
                		self.var_mov.set(0)
		self.Movement = tk.Checkbutton(self.topLevelWindow, text="Show robot movements", variable= self.var_mov,command=command_mov)
		#self.Movement.deselect()
		self.Movement.select()
		self.var_mov.set(1)

		# Check button sensor
                self.var_sensor = IntVar()
                def command_sensor():
                        if self.var_sensor.get() == 0:
                                self.sensor.select()
                                self.var_sensor.set(1)
                        else:
                                self.sensor.deselect()
                                self.var_sensor.set(0)

                self.sensor = tk.Checkbutton(self.topLevelWindow, text="Show sensors", variable= self.var_sensor,command=command_sensor)
                self.sensor.select()
                self.var_sensor.set(1)

	        # Check button add_noise
                self.add_noise = IntVar()
                def command_add_noise():
                        if self.add_noise.get() == 0:
                                self.noise.select()
                                self.add_noise.set(1)
                        else:
                                self.noise.deselect()
                                self.add_noise.set(0)

                self.noise = tk.Checkbutton(self.topLevelWindow, text="Add noise", variable= self.add_noise,command=command_add_noise)
                self.noise.deselect()
                self.add_noise.set(0)

		# Plot Topological map button
		#self.buttonPlotTopological= Button(self.rightMenu ,width = 20, text = "Plot Topological", foreground = self.buttonFontColor ,background = self.buttonColor, font = self.buttonFont ,command = self.print_topological_map  )
		self.buttonPlotTopological= Button(self.topLevelWindow ,width = 20, text = "Plot Topological", bg = 'green', activebackground = 'green',command = self.print_topological_map )

#---------------------------------------- Values' fields -----------------------------------------------------------------------------

      		# Number of sensors 
		self.label_num_sensors = tk.Label(self.topLevelWindow,text =  'Num. Sensors')
        	self.num_sensors = tk.Entry(self.topLevelWindow, width = 8, foreground='white',background='black')
		self.num_sensors.insert ( 0, '16' )
		num_sensors = self.num_sensors.get()
      		# Origen angle sensor 
		self.label_origen_angle = tk.Label(self.topLevelWindow,text =  'Origen angle sensor ')
        	self.origen_angle = tk.Entry(self.topLevelWindow, width = 8, foreground='white',background='black')
		self.origen_angle.insert ( 0, '-2.3561' )
		origen_angle = self.origen_angle.get()
      		# Range angle sensor 
		self.label_range_angle = tk.Label(self.topLevelWindow,text =  'Range angle sensor ')
        	self.range_angle = tk.Entry(self.topLevelWindow, width = 8, foreground='white',background='black')
		self.range_angle.insert ( 0, '4.7122' )
		range_angle = self.range_angle.get()
      		# Robot's magnitude advance  
		self.label_advance_robot = tk.Label(self.topLevelWindow,text =  "Robot's magnitude advance")
        	self.advance_robot = tk.Entry(self.topLevelWindow, width = 8, foreground='white',background='black')
		self.advance_robot.insert(0,'0.040')
		advance_robot = self.advance_robot.get()
		 # Robot's maximum angle  
                self.label_max_angle_robot = tk.Label(self.topLevelWindow,text =  "Robot's maximum turn angle")
                self.max_angle_robot = tk.Entry(self.topLevelWindow, width = 8, foreground='white',background='black')
                self.max_angle_robot.insert(0,'0.7854')
                max_angle_robot = self.max_angle_robot.get()
      		# Robot's radio  
		self.label_radio_robot = tk.Label(self.topLevelWindow,text =  "Robot's radio")
        	self.radio_robot = tk.Entry(self.topLevelWindow, width = 8, foreground='white',background='black')
		self.radio_robot.insert ( 0, '0.03' )
		radio_robot = self.radio_robot.get()
      		# Robot's pose x  
		self.label_robot_posex = tk.Label(self.topLevelWindow,text =  "Robot's pose x")
        	self.robot_posex = tk.Entry(self.topLevelWindow, width = 8, foreground='white',background='black')
		self.robot_posex.insert ( 0, '4.000' )
		pose_x = self.robot_posex.get()
      		# Robot's pose y  
		self.label_robot_posey = tk.Label(self.topLevelWindow,text =  "Robot's pose y")
        	self.robot_posey = tk.Entry(self.topLevelWindow, width = 8, foreground='white',background='black')
		self.robot_posey.insert ( 0, '5.000' )
		pose_y = self.robot_posey.get()
      		# Robot's angle  
		self.label_robot_angle = tk.Label(self.topLevelWindow,text =  "Robot's angle")
        	self.robot_angle = tk.Entry(self.topLevelWindow, width = 8, foreground='white',background='black')
		self.robot_angle.insert ( 0, '0.000' )
		pose_tetha = self.robot_angle.get()
      		# Robot's command  
		self.label_robot_command = tk.Label(self.topLevelWindow,text =  "Robot's command")
        	self.robot_command = tk.Entry(self.topLevelWindow, width = 40, foreground='white',background='black')
		self.robot_command.insert ( 0,"../motion_planner/GoTo_State_Machine")
		robot_command = self.robot_command.get()
      		# Number of steps  
		self.label_number_steps = tk.Label(self.topLevelWindow,text =  "Number of Steps")
        	self.number_steps = tk.Entry(self.topLevelWindow, width = 8, foreground='white',background='black')
		self.number_steps.insert ( 0, str(number_steps_total) )
		number_steps = self.number_steps.get()
      		# Selection of behavior  
		self.label_selection = tk.Label(self.topLevelWindow,text =  "Behavior Selection --------------------------->>>>")
        	self.selection = tk.Entry(self.topLevelWindow, width = 8, foreground='green',background='yellow')
		self.selection.insert ( 0,str(num_bh) )
		selection = self.selection.get()
      		# Largest value sensor  
		self.label_largest = tk.Label(self.topLevelWindow,text =  "Largest value sensor")
        	self.largest = tk.Entry(self.topLevelWindow, width = 8, foreground='white',background='black')
		self.largest.insert ( 0, '0.1' )
		largest = self.largest.get()
		# Evaluation Individual 
                self.label_evaluation_individual = tk.Label(self.topLevelWindow,text =  'Evaluation Individual')
                self.evaluation_ind = tk.Entry(self.topLevelWindow, width = 6, foreground='white',background='black')
                self.evaluation_ind.insert ( 0, '0' )
		# Noise percentage advance 
                self.label_noise_percentage_advance = tk.Label(self.topLevelWindow,text =  'Noise percentage advance')
                self.noise_percentage_advance = tk.Entry(self.topLevelWindow, width = 8, foreground='white',background='black')
                self.noise_percentage_advance.insert ( 0, '0.1' )
                noise_percentage_advance = self.noise_percentage_advance.get()
		# Noise percentage angle 
                self.label_noise_percentage_angle = tk.Label(self.topLevelWindow,text =  'Noise percentage angle')
                self.noise_percentage_angle = tk.Entry(self.topLevelWindow, width = 8, foreground='white',background='black')
                self.noise_percentage_angle.insert ( 0, '0.05' )
                noise_percentage_angle = self.noise_percentage_angle.get()
		# Noise percentage range 
                self.label_noise_percentage_lidar = tk.Label(self.topLevelWindow,text =  'Noise percentage lidar')
                self.noise_percentage_lidar = tk.Entry(self.topLevelWindow, width = 8, foreground='white',background='black')
                self.noise_percentage_lidar.insert ( 0, '0.01' )
                noise_percentage_lidar = self.noise_percentage_lidar.get()
		# Noise percentage range standard deviation 
                self.label_noise_percentage_lidar_sd = tk.Label(self.topLevelWindow,text =  'Noise percentage lidar sigma')
                self.noise_percentage_lidar_sd = tk.Entry(self.topLevelWindow, width = 8, foreground='white',background='black')
                self.noise_percentage_lidar_sd.insert ( 0, '0.001' )
                noise_percentage_lidar_sd = self.noise_percentage_lidar_sd.get()
		# Noise percentage light intensity 
                self.label_noise_percentage_light_intensity = tk.Label(self.topLevelWindow,text =  'Noise percentage light intensity')
                self.noise_percentage_light_intensity = tk.Entry(self.topLevelWindow, width = 8, foreground='white',background='black')
                self.noise_percentage_light_intensity.insert ( 0, '0.1' )
                noise_percentage_light_intensity = self.noise_percentage_light_intensity.get()
		# Noise percentage light quadrant 
                self.label_noise_percentage_light_quadrant = tk.Label(self.topLevelWindow,text =  'Noise percentage position light source')
                self.noise_percentage_light_quadrant = tk.Entry(self.topLevelWindow, width = 8, foreground='white',background='black')
                self.noise_percentage_light_quadrant.insert ( 0, '0.01' )
                noise_percentage_light_quadrant = self.noise_percentage_light_quadrant.get()
		# Number of movable obstacles  
                self.label_movable = tk.Label(self.topLevelWindow,text =  "Number of movable obstacles")
                self.movable = tk.Entry(self.topLevelWindow, width = 8, foreground='white',background='black')
                self.movable.insert ( 0, '0' )
                number_unk = self.movable.get()

		# Add noise
		flg_noise = self.add_noise.get()
		#print "noise ",str(flg_noise)

	

#--------------------------- G R I D S ------------------------------------------------------------- 


		self.label_robot_command.grid({'row':0, 'column': 0})        
		self.robot_command.grid({'row':0, 'column': 1})        
		self.label_path.grid({'row':1, 'column': 0})        
		self.path.grid({'row':1, 'column': 1})        
		self.label_file.grid({'row':2, 'column': 0})        
		self.file.grid({'row':2, 'column': 1})        
		self.label_file_robot.grid({'row':3, 'column': 0})        
		self.file_robot.grid({'row':3, 'column': 1})        
        	self.MapButton.grid({'row':5, 'column': 0})
        	self.RobotButton.grid({'row':6, 'column': 0})
		self.Movement.grid({'row':0, 'column': 2})
		self.sensor.grid({'row':1, 'column': 2})
		self.noise.grid({'row':2, 'column': 2})
		self.label_num_sensors.grid({'row':1, 'column': 3})        
		self.num_sensors.grid({'row':1, 'column': 4})        
		self.label_origen_angle.grid({'row':2, 'column': 3})        
		self.origen_angle.grid({'row':2, 'column': 4})        
		self.label_range_angle.grid({'row':3, 'column': 3})        
		self.range_angle.grid({'row':3, 'column': 4})        
		self.label_radio_robot.grid({'row':4, 'column': 3})        
		self.radio_robot.grid({'row':4, 'column': 4})        
		self.label_advance_robot.grid({'row':5, 'column': 3})        
		self.advance_robot.grid({'row':5, 'column': 4})        
		self.label_max_angle_robot.grid({'row':6, 'column': 3})        
		self.max_angle_robot.grid({'row':6, 'column': 4})        
		self.label_robot_posex.grid({'row':4, 'column': 1})        
		self.robot_posex.grid({'row':4, 'column': 2})        
		self.label_robot_posey.grid({'row':5, 'column': 1})        
		self.robot_posey.grid({'row':5, 'column': 2})        
		self.label_robot_angle.grid({'row':6, 'column': 1})        
		self.robot_angle.grid({'row':6, 'column': 2})        
		self.label_number_steps.grid({'row':7, 'column': 3})        
		self.number_steps.grid({'row':7, 'column': 4})        
		self.label_selection.grid({'row':7, 'column': 1})        
		self.selection.grid({'row':7, 'column': 2})        
		self.label_largest.grid({'row':6, 'column': 5})        
		self.largest.grid({'row':6, 'column': 6})        
		self.label_evaluation_individual.grid({'row':7, 'column': 5})        
		self.evaluation_ind.grid({'row':7, 'column': 6})        
		self.label_noise_percentage_advance.grid({'row':0, 'column': 5})        
		self.noise_percentage_advance.grid({'row':0, 'column': 6})        
		self.label_noise_percentage_angle.grid({'row':1, 'column': 5})        
		self.noise_percentage_angle.grid({'row':1, 'column': 6})        
		self.label_noise_percentage_lidar.grid({'row':2, 'column': 5})        
		self.noise_percentage_lidar.grid({'row':2, 'column': 6})        
		self.label_noise_percentage_lidar_sd.grid({'row':3, 'column': 5})        
		self.noise_percentage_lidar_sd.grid({'row':3, 'column': 6})        
		self.label_noise_percentage_light_intensity.grid({'row':4, 'column': 5})        
		self.noise_percentage_light_intensity.grid({'row':4, 'column': 6})        
		self.label_noise_percentage_light_quadrant.grid({'row':5, 'column': 5})        
		self.noise_percentage_light_quadrant.grid({'row':5, 'column': 6})        
		self.buttonPlotTopological.grid({'row':7, 'column': 0})
    
    		if num_behavior == 0:
			self.ButtonPath(1)
   
#___________________________________________________________________________________________________ 




    	#C.delete(polygon)
    	C.bind("<Button-1>", callback_mouse_1)
    	C.bind("<Button-2>", callback_mouse_2)
    	C.bind("<Button-3>", callback_mouse_3)
    	C.pack()
    	initial(self)
    	self.read_file_map(1)



    def plot_test(self):
	global C


	id = C.create_rectangle(0, 0, DIM_CANVAS_X, DIM_CANVAS_Y, fill= "blue")
    	coord = 10, 50, 40, 80
    	arc = C.create_arc(coord, start=0, extent=150, fill="red")
    	points = [150, 100, 200, 120, 240, 180, 210, 200, 150, 150, 100, 200]
    	polygon = C.create_polygon(points, outline='black', fill='red', width=1)
    	points = [1.50, 1.00, 2.00, 1.20, 2.40, 1.80, 2.10, 2.00, 1.50, 1.50, 1.00, 2.00]
    	self.plot_polygon(6, points)
    	oval = C.create_oval(300, 300, 380, 380, outline="black", fill="red", width=2)
    	line = C.create_line(30, 300, 100, 280,  fill="red", arrow="last")
    	self.plot_robot()


    def plot_polygon(self,num, data):
	global C

	XY= data
	#print "plot_polygon ",num
	for i in range(0, num):
		j=i*2 
		
		#print "data j ",j," x",data[j]," y ",data[j+1]
		X = ( DIM_CANVAS_X * data[j] ) / dim_x
		Y = DIM_CANVAS_Y - ( DIM_CANVAS_Y * data[j+1] ) / dim_y
		XY[j]=X;
		XY[j+1]=Y;
		#print "j ",j," X",XY[j]," Y ",XY[j+1]

	polygon = C.create_polygon(XY, outline='black', fill='red', width=1)
	return polygon


    def plot_polygon_green(self,num, data):
        global C

        XY= data
        #print "plot_polygon ",num
        for i in range(0, num):
                j=i*2

                #print "data j ",j," x",data[j]," y ",data[j+1]
                X = ( DIM_CANVAS_X * data[j] ) / dim_x
                Y = DIM_CANVAS_Y - ( DIM_CANVAS_Y * data[j+1] ) / dim_y
                XY[j]=X;
                XY[j+1]=Y;
                #print "j ",j," X",XY[j]," Y ",XY[j+1]

        polygon = C.create_polygon(XY, outline='green', fill='blue', width=1)
        return polygon




    def plot_line(self,x1,y1,x2,y2,color,flg):
        global C

        #print "plot_line "

        #print "x1 ",x1," y1 ",y1
        X1 = ( DIM_CANVAS_X * x1 ) / dim_x
        Y1 = DIM_CANVAS_Y - ( DIM_CANVAS_Y * y1 ) / dim_y
        #print "X1 ",X1," Y1 ",Y1

        #print "x2 ",x2," y2 ",y2
        X2 = ( DIM_CANVAS_X * x2 ) / dim_x
        Y2 = DIM_CANVAS_Y - ( DIM_CANVAS_Y * y2 ) / dim_y
        #print "X2 ",X2," Y2 ",Y2

	if flg == 1:
		line = C.create_line(X1,Y1,X2,Y2,fill=color, arrow="last")
	else:
		line = C.create_line(X1,Y1,X2,Y2,fill=color)
		id = C.create_rectangle(X2,Y2,X2+1,Y2+1, fill= "white", outline="white")


    def plot_oval(self,x,y):
        global C
        global radio_robot


        CNT=5.0
        #print "plot robot pose_tetha ",pose_tetha
        X = ( DIM_CANVAS_X * x ) / dim_x
        Y = DIM_CANVAS_Y - ( DIM_CANVAS_Y * y ) / dim_y
        ROBOT_radio = ( DIM_CANVAS_X * radio_robot ) / dim_x
        #X1 = X - ROBOT_radio/2
        #Y1 = Y - ROBOT_radio/2
        #X2 = X + ROBOT_radio/2
        #Y2 = Y + ROBOT_radio/2
        X1 = X - 1.5*ROBOT_radio
        Y1 = Y - 1.5*ROBOT_radio
        X2 = X + 1.5*ROBOT_radio
        Y2 = Y + 1.5*ROBOT_radio
        #print "X1 ", X1, " Y1 ", Y1
        #print "X2 ", X2, " Y2 ", Y2
        oval = C.create_oval(X1,Y1,X2,Y2, outline="black",fill="yellow", width=1)


    def plot_oval_green(self,x,y):
        global C
	global radio_robot


        CNT=5.0
        #print "plot robot pose_tetha ",pose_tetha
        X = ( DIM_CANVAS_X * x ) / dim_x
        Y = DIM_CANVAS_Y - ( DIM_CANVAS_Y * y ) / dim_y
        ROBOT_radio = ( DIM_CANVAS_X * radio_robot ) / dim_x
        #X1 = X - ROBOT_radio/2
        #Y1 = Y - ROBOT_radio/2
        #X2 = X + ROBOT_radio/2
        #Y2 = Y + ROBOT_radio/2
        X1 = X - 1.5*ROBOT_radio
        Y1 = Y - 1.5*ROBOT_radio
        X2 = X + 1.5*ROBOT_radio
        Y2 = Y + 1.5*ROBOT_radio
        #print "X1 ", X1, " Y1 ", Y1
        #print "X2 ", X2, " Y2 ", Y2
        oval = C.create_oval(X1,Y1,X2,Y2, outline="green",fill="green", width=1)


    def plot_robot(self):
        global C
	global pose_x
	global pose_y
	global radio_robot
	global pose_tetha


	CNT=5.0
	#print "plot robot pose_tetha ",pose_tetha
	X = ( DIM_CANVAS_X * pose_x ) / dim_x
	Y = DIM_CANVAS_Y - ( DIM_CANVAS_Y * pose_y ) / dim_y
	ROBOT_radio = ( DIM_CANVAS_X * radio_robot ) / dim_x
	X1 = X - ROBOT_radio/2
	Y1 = Y - ROBOT_radio/2
	X2 = X + ROBOT_radio/2
	Y2 = Y + ROBOT_radio/2
	#print "X1 ", X1, " Y1 ", Y1
	#print "X2 ", X2, " Y2 ", Y2
	#oval = C.create_oval(X1,Y1,X2,Y2, outline="black",fill="red", width=1)
	oval = C.create_oval(X1,Y1,X2,Y2, outline="black",fill="black", width=1)
	#oval = C.create_oval(X1,Y1,X2,Y2, outline="green",fill="green", width=1)
	#X1 = X 
	#Y1 = Y
	#Y2 = Y - CNT*ROBOT_radio*math.sin(pose_tetha)
	#print "sen ", pose_tetha, " = ",math.sin(pose_tetha)
	#line = C.create_line(X1,Y1,X2,Y2,fill="black", arrow="last")


	x1 = pose_x
        y1 = pose_y
	angle_robot = pose_tetha
        tetha = angle_robot + start_angle
        x2 = x1 + (dim_x/25)*math.cos(angle_robot)
        y2 = y1 + (dim_y/25)*math.sin(angle_robot)
        self.plot_line(x1,y1,x2,y2,"black",1)
        #self.plot_line(x1,y1,x2,y2,"green",1)

	mouse_3_x = pose_x
	mouse_3_y = pose_y



    def read_file_map(self,flg):
        global pose_x
        global pose_y
        global pose_tetha
        global C
        global num_pol
        global polygons
	global PATH
	global dim_x 
	global dim_y


        id = C.create_rectangle(0, 0, DIM_CANVAS_X, DIM_CANVAS_Y, fill= "green")

	PATH = self.path.get()
	File_Name = self.file.get()

	#print 'Path ',PATH 
	#print 'File ',File_Name

	FILE = PATH + File_Name + '.wrl'

        file = open(FILE, 'r')

        for line in file:
                #print line,
                words = line.split()
                data = words
                #print len(words)
                #for i in range(0, len(words)):
                        #print words[i]
		if len(words) > 1:
			if words[0] == ";(":
				j=0
			elif words[1] == "dimensions":
				dim_x = float(words[3])
				dim_y = float(words[4])
				#print 'dim_x ',dim_x, 'dim_y ',dim_y


                	elif words[1] == "polygon":
                        	j=0
                        	data = [0] * (len(words) - 5)
                        	for i in range(4, len(words) -1):
                                	#print words[i]
                                	data[j] = float(words[i])
                                	j=j+1

                        	j=j+1
                        	num = j / 2
				if flg == 1:
                        		polygons.append(self.plot_polygon(num,data))
                        		num_pol = num_pol + 1
				else:
					self.plot_polygon(num,data)



    def read_file(self,flg):
	global pose_x
	global pose_y
	global pose_tetha
	global C
	global num_pol
	global polygons
	global flg_mov
	global flg_sensor
	global delay
	global dim_x
	global dim_y
	global angle_robot
	global radio_robot
	global number_steps_total
	global flg_unk
	global previous_num  
	global previous_data
	global number_unk

	number_unk = int(self.movable.get())
	PATH = self.path.get()
        File_Name_robot = self.file_robot.get()
        FILE = PATH + File_Name_robot + '.raw'
        file = open(FILE, 'r')
	
	C.update_idletasks() # it updates the ide data
        delay = 0
	flg_mov = self.var_mov.get()
	flg_sensor = self.var_sensor.get()
	flg_plot_vq = 0
	number_steps = self.number_steps.get()
        number_steps_total = float(number_steps)
        num_steps = 1
	cnt_unk = 1
	flg_destination = 0
	
	for line in file:
    		#print line,
		words = line.split()
		data = words
		#print len(words)
		#for i in range(0, len(words)):
			#print words[i]
		if len(words) > 1:
		  if words[0] == ";(":
                                j=0

		  elif words[1] == "polygon":
			j=0
			data = [0] * (len(words) - 5)
                        for i in range(4, len(words) -1):
				#print words[i]
				data[j] = float(words[i])
				j=j+1 
				
			j=j+1 
			num = j / 2
			if flg == 1:
                        	polygons.append(self.plot_polygon(num,data))
                        	num_pol = num_pol + 1
                        else:
                        	self.plot_polygon(num,data)


		  elif words[1] == "dimensions":
                                dim_x = float(words[3])
                                dim_y = float(words[4])
                                #print 'dim_x ',dim_x, 'dim_y ',dim_y

		  elif words[1] == "radio_robot":
                                radio_robot = float(words[2])
                                #print 'radio robot ',radio_robot

    			
		  elif words[1] == "delete":
			for i in range(0,num_pol):
				C.delete(polygons[i])

		  elif words[1] == "destination":
                        x = float(words[2])
                        y = float(words[3])
			dest_x = x
			dest_y = y
			self.plot_oval(x,y)
			if flg_destination == 0:
				x_previous=x
				y_previous=y
				flg_destination=1
			else:
				self.plot_oval_green(x_previous,y_previous)
				x_previous=x
                                y_previous=y
                                time.sleep(0.1) # 0.1 delay seconds to see the plot of the destination

		  elif words[1] == "connection":                                  #to get polygons vertex
                                X1 = float (words[2]) * DIM_CANVAS_X / dim_x
                                Y1 = (dim_y - float (words[3])) * DIM_CANVAS_Y / dim_y
                                X2 = float (words[4]) * DIM_CANVAS_X / dim_x
                                Y2 = (dim_y - float (words[5])) * DIM_CANVAS_Y / dim_y
                                #print 'x1 ',X1, 'y1 ',Y2
                                #print 'x2 ',X2, 'y2 ',Y2
                                C.update()
                                id = C.create_rectangle(X1, Y1, X1+2, Y1+2, fill= "darkblue")
                                id = C.create_rectangle(X2, Y2, X2+2, Y2+2, fill= "darkblue")
				line = C.create_line(X1,Y1,X2,Y2,fill="darkblue", arrow="last")
                                C.update_idletasks()
                                time.sleep(0.001) # 0.001 delay seconds to see the plot of the lines



		  elif words[1] == "unknown":
				flg_unk = 1
                                j=0
                                data = [0] * (len(words) - 3)
                                for i in range(2, len(words) -1):
                                        #print words[i]
                                        data[j] = float(words[i])
                                        j=j+1

                                j=j+1
                                num = j / 2
				#value_s = input("Please enter a string:\n")
				if cnt_unk > number_unk:
					if flg_mov == 1:
						#print "clean screen"
						#value_s = input("Please enter a string:\n")
                                		id = C.create_rectangle(0, 0, DIM_CANVAS_X, DIM_CANVAS_Y, fill= "green")
                                		self.read_file_map(0)
					cnt_unk = 1

				#print "cnt_unk ",str(cnt_unk)," number_unk ",str(number_unk)
                                self.plot_polygon(num,data)
				cnt_unk = cnt_unk +1


		  elif words[1] == "robot":
			pose_x = float(words[3])
			pose_y = float(words[4])
			pose_tetha = float(words[5])
			angle_robot = pose_tetha
			str_angle = ("%3.4f" % angle_robot).strip()
			self.robot_angle.delete(0, END)
        		self.robot_angle.insert(0,str_angle)
			self.robot_posex.delete(0, END)
			self.robot_posex.insert (0, words[3] )
			self.robot_posey.delete(0, END)
			self.robot_posey.insert (0, words[4] )
			str_num_steps = ("%4d" % num_steps).strip()
			self.number_steps.delete(0, END)
        		self.number_steps.insert(0,str_num_steps)
			num_steps = num_steps + 1

			if flg_mov == 1:
				if flg_unk == 0:
					id = C.create_rectangle(0, 0, DIM_CANVAS_X, DIM_CANVAS_Y, fill= "green")
					self.read_file_map(0)

			self.plot_robot()
			self.plot_oval(dest_x,dest_y)
			if self.var_mov.get() == 0:
				C.update_idletasks()
			else:
				if flg_sensor == 0:
					C.update_idletasks()



			if flg_mov == 1:
				time.sleep(delay/2.0) # delays seconds

		  elif words[1] == "sensor":
		   if flg_plot_vq == 0:
		    if words[2] == "laser":
		     if flg_sensor == 1:
			if flg_mov == 1:
				if flg_unk == 0:
					id = C.create_rectangle(0, 0, DIM_CANVAS_X, DIM_CANVAS_Y, fill= "green")
					self.read_file_map(0)

			self.plot_robot()
			self.plot_oval(dest_x,dest_y)
                        #if flg_unk == 1:
				#self.plot_polygon(previous_num,previous_data)

			num = int(words[3])

			range_angle = float(words[4])
			start_angle = float(words[5])
			if num == 1:
				delta_angle = range_angle 
			else:
				delta_angle = range_angle / (num - 1)
			#print "num ", num, "Range Measurments ", range_angle
			#print "Start angle ", start_angle, " Delta Angle ", delta_angle

			x1 = pose_x 
       			y1 = pose_y
			tetha = angle_robot + start_angle
			x2 = x1 + (dim_x/10)*math.cos(angle_robot)
                        y2 = y1 + (dim_y/10)*math.sin(angle_robot)
                        #self.plot_line(x1,y1,x2,y2,"red",1)
			#print "sensor pose tetha ",pose_tetha
			#print "sensor angle_robot ",angle_robot
			#print "Origen Tetha ", tetha
			#print "sen ", tetha, " = ",math.sin(tetha)

			data = [0] * (len(words) - 5)
			j=0
                        for i in range(6, len(words) -1):
                                #print words[i]
				#print "Tetha ", j," ",tetha
                                data[j] = float(words[i])
        			x2 = x1 + data[j]*math.cos(tetha)
        			y2 = y1 + data[j]*math.sin(tetha)
				self.plot_line(x1,y1,x2,y2,"blue",0)
				#self.plot_line(x1,y1,x2,y2,"black",0)
                                j=j+1
				tetha = tetha + delta_angle
				#print "sen ", tetha, " = ",math.sin(tetha)


			C.update_idletasks()
			if flg_mov == 1:
				time.sleep(delay/2.0) # delays seconds

		  #elif words[1] == "sensor":
		   if flg_plot_vq == 1:
                    if words[2] == "vq_laser":
                     if flg_sensor == 1:
                        if flg_mov == 1:
				if flg_unk == 0:
					id = C.create_rectangle(0, 0, DIM_CANVAS_X, DIM_CANVAS_Y, fill= "green")
					self.read_file_map(0)

                        self.plot_robot()
			self.plot_oval(dest_x,dest_y)

                        num = int(words[3])

                        range_angle = float(words[4])
                        start_angle = float(words[5])
                        if num == 1:
                                delta_angle = range_angle
                        else:
                                delta_angle = range_angle / (num - 1)
                        #print "num ", num, "Range Measurments ", range_angle
                        #print "Start angle ", start_angle, " Delta Angle ", delta_angle

                        x1 = pose_x
                        y1 = pose_y
                        tetha = angle_robot + start_angle
                        x2 = x1 + (dim_x/10)*math.cos(angle_robot)
                        y2 = y1 + (dim_y/10)*math.sin(angle_robot)
                        #self.plot_line(x1,y1,x2,y2,"red",1)
                        #print "sensor pose tetha ",pose_tetha
                        #print "sensor angle_robot ",angle_robot
                        #print "Origen Tetha ", tetha
                        #print "sen ", tetha, " = ",math.sin(tetha)

                        data = [0] * (len(words) - 5)
                        j=0
                        for i in range(6, len(words) -1):
                                #print words[i]
                                #print "Tetha ", j," ",tetha
                                data[j] = float(words[i])
                                x2 = x1 + data[j]*math.cos(tetha)
                                y2 = y1 + data[j]*math.sin(tetha)
                                #self.plot_line(x1,y1,x2,y2,"black",0)
                                self.plot_line(x1,y1,x2,y2,"yellow",0)
                                #self.plot_line(x1,y1,x2,y2,"green",0)
                                j=j+1
                                tetha = tetha + delta_angle
                                #print "sen ", tetha, " = ",math.sin(tetha)


                        C.update_idletasks()
                        if flg_mov == 1:
                                time.sleep(delay/2.0) # delays seconds





	file.close()
	self.number_steps.delete(0, END)
       	self.number_steps.insert(0,number_steps)



#------------------------------------------------------------------------------------------

    def readResultFile(self,File_Results,File_Constants,num):

	      global advance_robot;
	      global number_steps_total
	      global num_steps


	      class Cnts:

                def __init__(self):

			self.K0 = 1.0
                        self.K1 = 10.0 * self.K0 #0.1
                        self.K2 = 2.0 * self.K0 #10.0  
                        self.K3 = 1.0 #0.0
                        self.K4 = 60.0 #60.0
                        self.K5 = 0.0 #0.1
                        self.K6 = 1.0 #1.0
                        self.K7 = 0.0 #.10
                        self.K8 = 11.0 * self.K0 #.10
                        self.K9 = 21.0 * self.K0 #.10




	      class Constants:

		def __init__(self, File_Constants):

			file_constants = open(File_Constants, 'r')
              		#print '***  File_Constants **** ',File_Constants


              		#evaluation = K1*(dif_o) + K2/(dif_d) + K3*dif_o_d/num_steps + 
              		#             K4/num_backwards + K5/num_turns + 
              		#             K6*num_steps_total/num_stops + K7*num_advance

              		for line in file_constants:
                        	#print line,
                        	words = line.split()
                        	#data = words
                        	#print len(words)
                        	#for i in range(0, len(words)):
                                	#print words[i]

			 	if len(words) > 1:
                                	if words[0] == "#":
                                        	xo = 0 
                                	elif words[0] == "K1":
                                        	self.K1 = float(words[1])
						#print 'K1 ', str(self.K1)
                                	elif words[0] == "K2":
                                        	self.K2 = float(words[1])
						#print 'K2 ',str(self.K2)
                                	elif words[0] == "K3":
                                        	self.K3 = float(words[1])
						#print 'K3 ',str(self.K3)
                                	elif words[0] == "K4":
                                        	self.K4 = float(words[1])
						#print 'K4 ',str(self.K4)
                                	elif words[0] == "K5":
                                        	self.K5 = float(words[1])
						#print 'K5 ',str(self.K5)
                                	elif words[0] == "K6":
                                        	self.K6 = float(words[1])
						#print 'K6 ',str(self.K6)
                                	elif words[0] == "K7":
                                        	self.K7 = float(words[1])
						#print 'K7 ',str(self.K7)

			file_constants.close()


	      Ct = Cnts()

	      #print 'num ',str(num)
	      #if num == 0:
	      	#Ct = Constants(File_Constants)
	


       	      file_results = open(File_Results, 'r')
	      #print 'File_Results ',File_Results

	      num_backwards = 1
	      num_advance = 1
	      num_turns = 1
	      num_stops = 1
	      num_st = 1
	      num_collisions = 1
	      previous_angle = 0.0
	      previous_advance = 0.0
	      previous_x = 0.0
	      previous_y = 0.0
	      THRESHOLD_MOVEMENT = 0.007
	      num_steps_total = float(self.number_steps.get())

	      threshold_noise = advance_robot/2
              threshold_angle = max_angle_robot/2
	      #print "threshold noise ",str(threshold_noise)
	      #print "threshold angle ",str(threshold_angle)
	      PATH = self.path.get()
              #print 'Evaluate Robot PATH ',PATH
              File_Name = self.file.get()

	      xx=[]
	      yy=[]


	      for line in file_results:
                	#print line,
                	words = line.split()
                	data = words
                	#print len(words)
                	#for i in range(0, len(words)):
                        	#print words[i]
			if len(words) > 1:
                  		if words[1] == "origen":
                                	xo = float(words[2])
                                	yo = float(words[3])
                                	zo = float(words[4])
                                	#print 'xo ',xo, ' yo ',yo,' zo ',zo
                  		elif words[1] == "robot":
                                	x= float(words[3])
					xx.append(x)
                                	y= float(words[4])
					yy.append(y)
                                	tetha= float(words[5])
              				#print 'robot x ',x, ' y ',y,' angle ',tetha
				        difx = (x-previous_x)	
				        dify = (y-previous_y)
					mag = math.sqrt(pow(difx,2)+pow(dify,2))	
					if mag < THRESHOLD_MOVEMENT:
						num_stops=num_stops + 1
					previous_x=x
					previous_y=y
					num_st = num_st+1
					#print "Num_steps ",str(num_st)," Magnitude ", str(mag)," num_stops ",str(num_stops)
                  		elif words[1] == "destination":
                                	xd= float(words[2])
                                	yd= float(words[3])
                                	#print 'xd ',xd, ' yd',yd
                  		elif words[1] == "collision":
                                	num_collisions = num_collisions + 1
                                	#print 'num.collisions ',str(num_collisions)
                  		elif words[1] == "distance":
                                	distance= float(words[2])
                                	#print 'distance ',distance
                  		elif words[1] == "num_steps":
					num_steps= float(words[2])-1
                                	#print 'num_steps ',num_steps
                  		elif words[1] == "movement":
                                	angle= float(words[2])
                                	advance= float(words[3])
                                	#print 'angle ',str(angle),' advance ',str(advance)
					if(advance < -threshold_noise):
						num_backwards=num_backwards + 1
                                		#print ' advance ',str(advance)

					if(abs(angle) >= threshold_angle and abs(previous_angle) >= threshold_angle):
                                                num_turns=num_turns + 1

                                        if(advance >= threshold_noise and previous_advance >= threshold_noise):
                                                num_advance=num_advance + 1
                                		#print ' advance ',str(advance),' prev. advance ',str(previous_advance)

					#if(angle == 0.0 and advance == 0.0):
						#num_stops=num_stops + 1

					previous_angle = angle
					previous_advance = advance		


	      sdx = np.std(xx)
	      sdy = np.std(yy)
	      sd = sdy + sdx
	      #print 'std ', str(sd)

	      #print "Num_stops ",str(num_stops)
	      #print 'Num.collisions ',str(num_collisions)
	      #print "Num_backwards ",str(num_backwards)
	
              #print 'final position x ',x, ' y ',y,' angle ',tetha
	      dif_o= math.sqrt( math.pow( (xo -x),2)+math.pow( (yo -y),2))
	      dif_d= math.sqrt(math.pow( (xd -x),2)+math.pow( (yd -y),2))

	      command = "../Dijkstra/Dijkstra -x " + str(x) + " -y " + str(y) + " -v " + str(xd) + " -z " + str(yd) + " -p " + PATH + " -e " + File_Name + " > " + PATH + "rslt_" + BEHAVIOR + ".dat"
              #print "Dijkstra command: ", command
              status = os.system(command)

	      #command = "tail rslt.dat"
              #status = os.system(command)
	      FILE = PATH + "rslt_" + BEHAVIOR + ".dat"
	      if(os.path.isfile(FILE)):
	      		file = open(FILE, 'r')
	      		dummy = file.readline()
              		distance_Dijkstra = float(file.readline())
	      		#print 'dif_d ',str(dif_d), ' distance_Dijkstra ',str(distance_Dijkstra)
	      		file.close()
	      else:
			distance_Dijkstra = 0
			#print "File does not exists " + FILE
	      dif_d = dif_d + distance_Dijkstra
	      #dif_d = dif_d + 60*distance_Dijkstra
	      dif_o_d= math.sqrt(math.pow( (xd -xo),2)+math.pow( (yd -yo),2))
	      #print 'dif_d ',str(dif_d)
	      ##print 'dif_o ',str(dif_o)
			

	      evaluation = Ct.K1*abs((number_steps_total - num_steps + 1)*dif_o) + abs(number_steps_total - num_steps + 1)/(Ct.K2*dif_d) + abs(dif_o_d/(Ct.K3*num_steps)) + abs(number_steps_total - num_steps+1)/(Ct.K4*num_backwards) + Ct.K8*sd + abs(number_steps_total - num_steps+1)/(Ct.K6*num_stops) + abs(number_steps_total - num_steps+1)/(Ct.K9*num_collisions)


	      file_results.close()

	      return evaluation


    def my_range(self,start, end, step):
    	while start <= end:
        	yield start
        	start += step


# Functions that handle the buttons

    def ButtonPath(self,option):
                        global INITIAL_PATH
                        global ORIGINAL_PATH
			global PREVIOUS_BEHAVIOR
			global PREVIOUS_BEST_BEHAVIOR 


                        self.TestButtonFSM ['bg'] = 'red'
                        self.TestButtonFSM ['activebackground'] = 'red'
                        self.path.delete(0, END)
			if option == 0:
                        	self.path.insert ( 0, ORIGINAL_PATH )
                        	self.DemoButtonPath ['bg'] = 'red'
                        	self.DemoButtonPath ['activebackground'] = 'red'
			else:
                        	self.path.insert ( 0, INITIAL_PATH )
			

    def toggleDemoButtonPath(self):

	self.ButtonPath(0)	



    def put_buttons_green(self):
	self.TestButtonFSM ['bg'] = 'green'
        self.TestButtonFSM ['activebackground'] = 'green'
	self.TestButtonFSMGEN ['bg'] = 'green'
        self.TestButtonFSMGEN ['activebackground'] = 'green'
	self.TestButtonFSMSTGEN ['bg'] = 'green'
        self.TestButtonFSMSTGEN ['activebackground'] = 'green'
        self.TestButtonREACTIVEGEN ['bg'] = 'green'
        self.TestButtonREACTIVEGEN['activebackground'] = 'green'
        self.TestButtonREACTIVESTGEN ['bg'] = 'green'
        self.TestButtonREACTIVESTGEN['activebackground'] = 'green'
        self.TestButtonPOTENTIALSGEN ['bg'] = 'green'
        self.TestButtonPOTENTIALSGEN['activebackground'] = 'green'
        self.TestButtonHMMGEN ['bg'] = 'green'
        self.TestButtonHMMGEN['activebackground'] = 'green'
        self.TestButtonMDPGEN ['bg'] = 'green'
        self.TestButtonMDPGEN['activebackground'] = 'green'
        self.TestButtonNNGEN ['bg'] = 'green'
        self.TestButtonNNGEN['activebackground'] = 'green'
        self.TestButtonNNSTGEN ['bg'] = 'green'
        self.TestButtonNNSTGEN['activebackground'] = 'green'
        self.TestButtonRNNGEN ['bg'] = 'green'
        self.TestButtonRNNGEN['activebackground'] = 'green'
        self.TestButtonRNNSTGEN ['bg'] = 'green'
        self.TestButtonRNNSTGEN['activebackground'] = 'green'
        self.DemoButtonPath ['bg'] = 'green'
        self.DemoButtonPath['activebackground'] = 'green'
	self.TestButtonMEALLYFSMGEN ['bg'] = 'green'
	self.TestButtonMOOREFSMGEN ['bg'] = 'green'
        self.TestButtonREACTIVEGEN['activebackground'] = 'green'



    def print_topological_map(self): # It plots  the topological map of the current map  and  show  "please wait" message
	#wait_bg=C.create_rectangle(DIM_CANVAS_X/2-30-120 ,DIM_CANVAS_Y/2-50 ,DIM_CANVAS_X/2-30+120 ,DIM_CANVAS_Y/2+50 ,fill="white")
	#wait = C.create_text(DIM_CANVAS_X/2-30,DIM_CANVAS_Y/2,fill="darkblue",font="Calibri 20 bold", text="PLEASE WAIT ...")
	C.update()
	self.buttonPlotTopological['bg'] = 'red'
       	self.buttonPlotTopological['activebackground'] = 'red'
	self.print_topological_map_lines()
	self.buttonPlotTopological['bg'] = 'green'
       	self.buttonPlotTopological['activebackground'] = 'green'
	#self.w.delete(wait)
	#self.w.delete(wait_bg)
	#self.plot_robot()



    def print_topological_map_lines(self): # It plots  the topological map of the current map  

                #self.clear_topological_map();
                self.varShowNodes = True

                #self.w.delete(self.nodes_image)        
                nodes_coords = []
                #image = Image.new('RGBA', (DIM_CANVAS_X,DIM_CANVAS_Y))
                #draw = ImageDraw.Draw(image)
		PATH = self.path.get()
	        File_Name = self.file.get()
        	#print 'Path ',PATH 
        	#print 'File ',File_Name
        	FILE = PATH + File_Name + '.top'
        	map_file = open(FILE, 'r')
                #map_file = open(self.rospack.get_path('simulator')+'/src/data/'+self.entryFile.get()+'/'+self.entryFile.get()+'.top','r')                  #Open file
                lines = map_file.readlines()                          #Split the file in lines
                for line in lines:                                                                        #To read line by line
                        words = line.split()                              #To separate  words 
                	#print len(words)
                	#for i in range(0, len(words)):
                        	#print words[i]
                        if words:                                                                                 #To avoid empty lines                                                 
                                if words[0] == "(":                                                       #To avoid coments
                                        if words[1] == "num":                     #To get world dimensions
                                                numNode = float (words[3])
                                        elif words[1] == "node":                                  #to get polygons vertex
                                                numNode = words[2]
						nodeXm = float (words[3]) * DIM_CANVAS_X / dim_x 
                				nodeYm = (dim_y - float (words[4])) * DIM_CANVAS_Y / dim_y 
						#print 'word[3] ' + words[3] + ' words[4] ' + words[4]
						#print 'Xm ' + str(nodeXm) + ' Ym ' + str(nodeYm)
						C.update()
						id = C.create_rectangle(nodeXm, nodeYm, nodeXm+2, nodeYm+2, fill= "darkblue")
                                                nodes_coords.append([nodeXm,nodeYm])
                                        elif words[1] == "connection":                            #to get polygons vertex
                                                c1 = int(words[2])
                                                c2 = int(words[3])
						#print 'Node C1 ' + str(c1) + ' C2 ' + str(c2)
						x1 = nodes_coords[c1][0]
						y1 = nodes_coords[c1][1]
						x2 = nodes_coords[c2][0]
						y2 = nodes_coords[c2][1]
						#print 'X1 ' + str(x1) + ' Y1 ' + str(y1)
						#print 'X2 ' + str(x2) + ' Y2 ' + str(y2)
						line = C.create_line(x1,y1,x2,y2,fill="darkblue", arrow="last")
        					C.update_idletasks()
						time.sleep(0.001) # 0.001 delay seconds to see the plot of the lines

		#print 'dim_x ' + str(dim_x) + ' dim_y ' + str(dim_y)

                map_file.close()




    def togglePlotRobot(self):
	global pose_x
	global pose_y
	global pose_tetha
	global PATH
        global File_Name
        global File_Name_robot
	global flg_mov
	global flg_sensor
	global delay

 
 
	self.RobotButton['bg'] = 'red'
       	self.RobotButton['activebackground'] = 'red'


	# uniform gives you a floating-point value
	#pose_x = uniform(0, dim_x)
	#pose_y = uniform(0, dim_y)
	#pose_tetha = uniform(0, 2*3.1416)
	#self.plot_robot()

        C.update_idletasks()

	PATH = self.path.get()
        #print 'Plot Robot PATH ',PATH

        File_Name = self.file.get()
        #print 'Plot Robot File_Name ',File_Name

        File_Name_robot = self.file_robot.get()
        #print 'Plot Robot File_Name_robot ',File_Name_robot

	flg_mov = self.var_mov.get()
	#print 'Plot Robot flg_mov ',flg_mov

	flg_sensor = self.var_sensor.get()
	#print 'Plot Robot flg_sensor ',flg_sensor

	#delay = float(self.delay.get())
	delay = 0.0 
	#print 'Plot Robot delay ',delay

	self.read_file_map(0)
	self.read_file(1)

    	self.RobotButton['bg'] = 'green'
       	self.RobotButton['activebackground'] = 'green'



    def togglePlotMap(self):
	global PATH
	global File_Name
 
 
       	self.MapButton['bg'] = 'red'
       	self.MapButton['activebackground'] = 'red'
	
	C.update_idletasks()

	PATH = self.path.get()
        #print 'Plot Map PATH ',PATH

        File_Name = self.file.get()
        #print 'Plot Map File_Name ',File_Name

	flg_mov = self.var_mov.get()
        #print 'Plot Robot flg_mov',flg_mov

	flg_sensor = self.var_sensor.get()
	#print 'Plot Robot flg_sensor ',flg_sensor

	self.read_file_map(0)

      	self.MapButton['bg'] = 'green'
       	self.MapButton['activebackground'] = 'green'

        str_angle = "0.00000"
        self.robot_angle.delete(0, END)
        self.robot_angle.insert(0,str_angle)





    def togglePlotExecute(self,flg_output,BEHAVIOR):
       
	global mouse_1_x
        global mouse_1_y
	global mouse_3_x
        global mouse_3_y
	global angle_robot
	global sensor
	global num_sensors 
	global flg_noise
	global start_angle 
	global range_angle
	global flg_execute
	global pose_x
	global pose_y
	global radio_robot
	global advance_robot
	global max_angle_robot
	global robot_command
	global flg_plt
	global number_unk


	flg_execute = 1 

       	self.ExecuteButton['bg'] = 'red'
       	self.ExecuteButton['activebackground'] = 'red'

	C.update_idletasks()

        flg_noise = self.add_noise.get()
	num_sensors = self.num_sensors.get()
        origen_angle = self.origen_angle.get()
        range_angle = self.range_angle.get()
        angle_robot = float(self.robot_angle.get())
        radio_robot = float(self.radio_robot.get())
        advance_robot = float(self.advance_robot.get())
        max_angle_robot = float(self.max_angle_robot.get())
	flg_sensor = self.var_sensor.get()
        number_steps = self.number_steps.get()
	#self.selection.delete(0, END)
	#self.selection.insert ( 0,str(num_bh) )
	selection = self.selection.get()
	largest = self.largest.get()
	PATH = self.path.get()
        #print 'Plot Robot PATH ',PATH
        File_Name = self.file.get()
        #print 'Plot Robot File_Name ',File_Name
        File_Name_robot = self.file_robot.get()
        #print 'Plot Robot File_Name_robot ',File_Name_robot
	#print 'selection ' + str(selection)
        vq = 0
        size_vq = 0
        pr_out = 0
        number_unk = 0
	nn_rec = 0

	if flg_output == 1:
		File_Output = File_Name + '_' + BEHAVIOR
	else:
		File_Output = File_Name

	#print "Behavior " + BEHAVIOR + " flg_out " + str(flg_output)

	robot_command = self.robot_command.get()
	origin = " -x " + str(mouse_1_x) + " -y " + str(mouse_1_y) + " -a " + str(angle_robot)
	destination = " -v " + str(mouse_3_x) + " -z " + str(mouse_3_y)
	rest = " -s " + sensor + " -n " + num_sensors + " -t " + origen_angle + " -r " + range_angle + " -radio " + str(radio_robot) + " -advance " + str(advance_robot) + " -MaxAngle " + str(max_angle_robot) + " -steps " + number_steps + " -selection " + selection + " -largest " + largest + " -p " + PATH + " -e " + File_Name + " -noise " + str(flg_noise) + " -vq " + str(vq) + " -size_vq " + str(size_vq) + " -pr_out " + str(pr_out) + " -nn_rec " + str(nn_rec)  
	command = robot_command + origin + destination + rest + " -out_file " + File_Output + " -nn_unk " + str(number_unk)  + " > " + PATH + "test_" + BEHAVIOR + ".dat"

	print "Robot Command: \n", command
	status = os.system(command)

	if flg_plt == 1:
		C.update_idletasks()

        	flg_mov = self.var_mov.get()
        	#print 'Plot Robot flg_mov ',flg_mov

        	flg_sensor = self.var_sensor.get()
        	#print 'Plot Robot flg_sensor ',flg_sensor

        	#delay = float(self.delay.get())
                delay = 0
        	#print 'Plot Robot delay ',delay

        	self.read_file_map(0)
        	self.read_file(1)


      		self.ExecuteButton['bg'] = 'green'
       		self.ExecuteButton['activebackground'] = 'green'

	mouse_1_x = pose_x
	mouse_1_y = pose_y



    def togglePlotPath(self):
       	global num_pol
	global polygons
 
       	if self.countPath == 1:
       		self.PathButton['bg'] = 'green'
       		self.PathButton['activebackground'] = 'green'
       	else:
       		self.PathButton['bg'] = 'red'
       		self.PathButton['activebackground'] = 'red'
		self.countPath = 0

	#print 'PlotPath num_pol ' + str(num_pol)

	for i in range(0,num_pol):
		C.delete(polygons[i])




    def togglePlotTest(self):
	self.plot_test()
        

#-----------------------------------------------------
#  MAIN


if __name__ == '__main__':
    gui_planner = PLANNER()

    tk.mainloop()

