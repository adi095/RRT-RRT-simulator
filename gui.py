from PyQt5 import QtCore, QtGui, QtWidgets

class graphics(object):
    def setupUi(self, planner):
        planner.setObjectName("planner")
        planner.resize(1005, 700) #window size

# Create a QGraphicsView widget for the simulation visualization 
        self.graphicsView = QtWidgets.QGraphicsView(planner)
        self.graphicsView.setGeometry(QtCore.QRect(20, 50, 600, 600))
        self.graphicsView.setObjectName("graphicsView")

# Create the "Start" button
        self.Start_simulation_pushButton = QtWidgets.QPushButton(planner)
        self.Start_simulation_pushButton.setGeometry(QtCore.QRect(690, 610, 120, 50))
        font = QtGui.QFont()
        font.setPointSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.Start_simulation_pushButton.setFont(font)
        self.Start_simulation_pushButton.setObjectName("Start_simulation_pushButton")

# Create the "Reset" button       
        self.Reset_pushButton = QtWidgets.QPushButton(planner)
        self.Reset_pushButton.setGeometry(QtCore.QRect(840, 610, 120, 50))
        font = QtGui.QFont()
        font.setPointSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.Reset_pushButton.setFont(font)
        self.Reset_pushButton.setObjectName("Reset_pushButton")

#Create main label "Visualization"
        self.label = QtWidgets.QLabel(planner)
        self.label.setGeometry(QtCore.QRect(212, 8, 300, 50))
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setObjectName("label")

#Create label "time step"
        self.Time_Step_label = QtWidgets.QLabel(planner)
        self.Time_Step_label.setGeometry(QtCore.QRect(640, 50, 250, 40))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.Time_Step_label.setFont(font)
        self.Time_Step_label.setObjectName("Time_Step_label")

#create groupbox for radio buttons (RRT and RRT star)
        self.groupBox = QtWidgets.QGroupBox(planner)
        self.groupBox.setGeometry(QtCore.QRect(640, 110, 350, 90))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.groupBox.setFont(font)
        self.groupBox.setObjectName("groupBox")

 #create RRT radio button
        self.RRT_radioButton = QtWidgets.QRadioButton(self.groupBox)
        self.RRT_radioButton.setGeometry(QtCore.QRect(40, 30, 200, 30))
        font = QtGui.QFont()
        font.setPointSize(24)
        self.RRT_radioButton.setFont(font)
        self.RRT_radioButton.setChecked(True)
        self.RRT_radioButton.setObjectName("RRT_radioButton")

 #create RRT* radio button
        self.RRT_star_radioButton = QtWidgets.QRadioButton(self.groupBox)
        self.RRT_star_radioButton.setGeometry(QtCore.QRect(170, 30, 200, 30))
        font = QtGui.QFont()
        font.setPointSize(24)
        self.RRT_star_radioButton.setFont(font)
        self.RRT_star_radioButton.setObjectName("RRT_star_radioButton")   

#create instruction Path cost label
        self.path_cost_label = QtWidgets.QLabel(planner)
        self.path_cost_label.setGeometry(QtCore.QRect(650, 460, 250, 40))
        font = QtGui.QFont()
        font.setPointSize(18)
        font.setBold(True)
        self.path_cost_label.setFont(font)
        self.path_cost_label.setObjectName("path_cost_label")
        self.path_cost_label.setText("Path Cost: 0.0")

#create SpinBox for Step size      
        self.stepsize_doubleSpinBox = QtWidgets.QDoubleSpinBox(planner)
        self.stepsize_doubleSpinBox.setGeometry(QtCore.QRect(910, 210, 80, 30))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.stepsize_doubleSpinBox.setFont(font)
        self.stepsize_doubleSpinBox.setMinimum(0.2)
        self.stepsize_doubleSpinBox.setMaximum(2.0)
        self.stepsize_doubleSpinBox.setSingleStep(0.1)
        self.stepsize_doubleSpinBox.setObjectName("stepsize_doubleSpinBox")

#create label for step size
        self.label_3 = QtWidgets.QLabel(planner)
        self.label_3.setGeometry(QtCore.QRect(650, 207, 220, 40))
        font = QtGui.QFont()
        font.setPointSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        self.goal_sampling_rate_spinBox = QtWidgets.QSpinBox(planner)

#create spinBox for goal sampling rate
        self.goal_sampling_rate_spinBox.setGeometry(QtCore.QRect(910, 250, 80, 30))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.goal_sampling_rate_spinBox.setFont(font)
        self.goal_sampling_rate_spinBox.setMinimum(1)
        self.goal_sampling_rate_spinBox.setObjectName("goal_sampling_rate_spinBox")

#create label for goal sampling rate
        self.label_4 = QtWidgets.QLabel(planner)
        self.label_4.setGeometry(QtCore.QRect(650, 250, 220, 40))
        font = QtGui.QFont()
        font.setPointSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.label_4.setFont(font)
        self.label_4.setObjectName("label_4")

#create label for goal position
        self.label_5 = QtWidgets.QLabel(planner)
        self.label_5.setGeometry(QtCore.QRect(650, 370, 220, 40))
        font = QtGui.QFont()
        font.setPointSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.label_5.setFont(font)
        self.label_5.setObjectName("label_5")

#create label for goal coordinate X
        self.label_6 = QtWidgets.QLabel(planner)
        self.label_6.setGeometry(QtCore.QRect(870, 370, 20, 30))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.label_6.setFont(font)
        self.label_6.setObjectName("label_6")

#create label for goal coordinate y
        self.label_7 = QtWidgets.QLabel(planner)
        self.label_7.setGeometry(QtCore.QRect(870, 410, 20, 30))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.label_7.setFont(font)
        self.label_7.setObjectName("label_7")

#create goal position spinboxes
#coordinate x
        self.Goal_X_doubleSpinBox = QtWidgets.QDoubleSpinBox(planner)
        self.Goal_X_doubleSpinBox.setGeometry(QtCore.QRect(910, 370, 80, 30))
        font = QtGui.QFont()
        font.setPointSize(18)
        font.setBold(False)
        font.setWeight(50)
        self.Goal_X_doubleSpinBox.setFont(font)
        self.Goal_X_doubleSpinBox.setMinimum(-15.0)
        self.Goal_X_doubleSpinBox.setMaximum(15.0)
        self.Goal_X_doubleSpinBox.setSingleStep(0.1)
        self.Goal_X_doubleSpinBox.setObjectName("Goal_X_doubleSpinBox")

#coordinate y
        self.Goal_Y_doubleSpinBox = QtWidgets.QDoubleSpinBox(planner)
        self.Goal_Y_doubleSpinBox.setGeometry(QtCore.QRect(910, 410, 80, 30))
        font = QtGui.QFont()
        font.setPointSize(18)
        font.setBold(False)
        font.setWeight(50)
        self.Goal_Y_doubleSpinBox.setFont(font)
        self.Goal_Y_doubleSpinBox.setMinimum(-15.0)
        self.Goal_Y_doubleSpinBox.setMaximum(15.0)
        self.Goal_Y_doubleSpinBox.setSingleStep(0.1)
        self.Goal_Y_doubleSpinBox.setObjectName("Goal_Y_doubleSpinBox")

#create label for planning constant
        self.label_8 = QtWidgets.QLabel(planner)
        self.label_8.setGeometry(QtCore.QRect(650, 293, 250, 40))
        font = QtGui.QFont()
        font.setPointSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.label_8.setFont(font)
        self.label_8.setObjectName("label_8")

#create planning constant spinbox
        self.planning_constant_SpinBox = QtWidgets.QDoubleSpinBox(planner)
        self.planning_constant_SpinBox.setGeometry(QtCore.QRect(910, 290, 80, 30))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.planning_constant_SpinBox.setFont(font)
        self.planning_constant_SpinBox.setMinimum(0.1)
        self.planning_constant_SpinBox.setMaximum(100.0)
        self.planning_constant_SpinBox.setSingleStep(0.1)
        self.planning_constant_SpinBox.setObjectName("planning_constant_SpinBox")

# Add new checkbox for obstacles
        self.obstacles_checkbox = QtWidgets.QCheckBox(planner)
        self.obstacles_checkbox.setGeometry(QtCore.QRect(650, 525, 250, 40))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.obstacles_checkbox.setFont(font)
        self.obstacles_checkbox.setChecked(True)  # Default: obstacles on
        self.obstacles_checkbox.setObjectName("obstacles_checkbox")

        self.retranslateUi(planner)
        self.Start_simulation_pushButton.clicked.connect(planner.Start_simulation)
        self.Reset_pushButton.clicked.connect(planner.reset)
        QtCore.QMetaObject.connectSlotsByName(planner)
        

    def retranslateUi(self, planner):
        _translate = QtCore.QCoreApplication.translate
        planner.setWindowTitle(_translate("planner", "RRT/RRT* Simulator"))
        self.Start_simulation_pushButton.setText(_translate("planner", "Start"))
        self.Reset_pushButton.setText(_translate("planner", "Reset"))
        self.label.setText(_translate("planner", "Visualization"))
        self.Time_Step_label.setText(_translate("planner", "Time Step : 0"))
        self.RRT_radioButton.setText(_translate("planner", "RRT"))
        self.RRT_star_radioButton.setText(_translate("planner", "RRT* "))
        self.path_cost_label.setText(_translate("planner", "Path cost"))
        self.label_3.setText(_translate("planner", "Step size"))
        self.label_4.setText(_translate("planner", "Goal Sampling Rate"))
        self.label_5.setText(_translate("planner", "Goal position"))
        self.label_6.setText(_translate("planner", "X"))
        self.label_7.setText(_translate("planner", "Y"))
        self.label_8.setText(_translate("planner", "Plannig constant"))
        self.obstacles_checkbox.setText(_translate("planner", "No Obstacles"))