'''Code executes RRT and RRT star with gui, with tunable parameters such as graph coordinates, step size and planning constant.
With and without obstacles involving euclidean distance as path cost.
To run command "python simulator.py"
Before running "start" press "reset" in GUI 
'''


from PyQt5.QtWidgets import QApplication, QDialog, QGraphicsScene
from PyQt5.QtGui import QBrush, QColor, QPen
from PyQt5.Qt import Qt
from PyQt5.QtCore import QTimer
from PyQt5.QtCore import QLineF


from gui import graphics

import math, random, sys, copy
import numpy as np
np.seterr(divide='ignore', invalid='ignore')


class RRT():
    

    def __init__(self, start_node, goal_node, obstacle, random_area, stepsize, sample_rate):
        self.start = Node(start_node[0], start_node[1])
        self.end = Node(goal_node[0], goal_node[1])
        self.area_min = random_area[0]
        self.area_max = random_area[1]
        self.stepsize = stepsize
        self.sample_rate = sample_rate
        self.obstacle = obstacle
        self.tree_nodes = [self.start]

    def Planning(self):
        random_point = self.get_random_point()

        # Find nearest node
        find = self.nearest_list(self.tree_nodes, random_point)
        nearestNode = self.tree_nodes[find]
        theta = math.atan2(random_point[1] - nearestNode.y, random_point[0] - nearestNode.x)

        newNode = copy.deepcopy(nearestNode)
        newNode.x += self.stepsize * math.cos(theta)
        newNode.y += self.stepsize * math.sin(theta)
        newNode.parent = find

        if self.is_collision_free(newNode, self.obstacle):
            self.tree_nodes.append(newNode)

            # Check goal
            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.stepsize:
                window.goal_flag = True

        path = [[self.end.x, self.end.y]]
        lastIndex = len(self.tree_nodes) - 1
        while self.tree_nodes[lastIndex].parent is not None:
            node = self.tree_nodes[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])


        window.draw(self.end, self.obstacle, self.tree_nodes)

        if window.goal_flag == True:
            # Calculate and display path cost
            path_cost = window.calculate_path_cost(path)
            window.update_path_cost_label(path_cost)
            window.show_final_path(path)

    def RRT_star_Planning(self):
        random_point = self.get_random_point()
        find = self.nearest_list(self.tree_nodes, random_point)
        newNode = self.steer(random_point, find)

        if self.is_collision_free(newNode, self.obstacle):
            nearinds = self.find_near_nodes(newNode)
            newNode = self.choose_parent(newNode, nearinds)
            self.tree_nodes.append(newNode)
            self.rewire(newNode, nearinds)

        dx_g = self.end.x - newNode.x
        dy_g = self.end.y - newNode.y
        d_g = math.sqrt(dx_g ** 2 + dy_g ** 2)
        if d_g <= self.stepsize:
            window.goal_flag = True

        window.draw(self.end, self.obstacle, self.tree_nodes)

        if window.goal_flag == True:
            lastIndex = self.best_last_index()
            path = self.gen_final_course(lastIndex)
            window.show_final_path(path)
            path_cost = window.calculate_path_cost(path)
            window.update_path_cost_label(path_cost)

    def choose_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
            return newNode

        dlist = []
        for i in nearinds:
            dx = newNode.x - self.tree_nodes[i].x
            dy = newNode.y - self.tree_nodes[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision(self.tree_nodes[i], theta, d):
                dlist.append(self.tree_nodes[i].cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            return newNode

        newNode.cost = mincost
        newNode.parent = minind
        return newNode

    def steer(self, random_point, find):
        # expand tree
        nearestNode = self.tree_nodes[find]
        theta = math.atan2(random_point[1] - nearestNode.y, random_point[0] - nearestNode.x)
        newNode = copy.deepcopy(nearestNode)
        newNode.x += self.stepsize * math.cos(theta)
        newNode.y += self.stepsize * math.sin(theta)

        newNode.cost += self.stepsize
        newNode.parent = find
        return newNode

    def get_random_point(self):
        if random.randint(0, 100) > self.sample_rate:
            random_point = [random.uniform(self.area_min, self.area_max), random.uniform(self.area_min, self.area_max)]
        else:  # goal point sampling
            random_point = [self.end.x, self.end.y]
        return random_point

    def nearest_list(self, tree_node, random_point):
        dlist = [(node.x - random_point[0]) ** 2 + (node.y - random_point[1]) ** 2 for node in tree_node]
        minind = dlist.index(min(dlist))
        return minind

    def best_last_index(self):
        disglist = [self.calc_dist_to_goal(node.x, node.y) for node in self.tree_nodes]
        goalinds = [disglist.index(i) for i in disglist if i <= self.stepsize]

        if len(goalinds) > 0:
            mincost = min([self.tree_nodes[i].cost for i in goalinds])
            for i in goalinds:
                if self.tree_nodes[i].cost == mincost:
                    return i
        return None

    def gen_final_course(self, goalind):
        path = [[self.end.x, self.end.y]]
        while self.tree_nodes[goalind].parent is not None:
            node = self.tree_nodes[goalind]
            path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, newNode):
        n_node = len(self.tree_nodes)
        r = float(window.ui.planning_constant_SpinBox.value()) * math.sqrt((math.log(n_node) / n_node))
        dlist = [(node.x - newNode.x) ** 2 + (node.y - newNode.y) ** 2 for node in self.tree_nodes]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds

    def is_collision_free(self, node, obstacleList):
        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= size:
                return False  # collision

        return True  # safe

    def rewire(self, newNode, nearinds):
        n_node = len(self.tree_nodes)
        for i in nearinds:
            nearNode = self.tree_nodes[i]

            dx = newNode.x - nearNode.x
            dy = newNode.y - nearNode.y
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = newNode.cost + d

            if nearNode.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collision(nearNode, theta, d):
                    nearNode.parent = n_node - 1
                    nearNode.cost = scost

    def check_collision(self, nearNode, theta, d):
        tmpNode = copy.deepcopy(nearNode)

        for i in range(int(d / self.stepsize)):
            tmpNode.x += self.stepsize * math.cos(theta)
            tmpNode.y += self.stepsize * math.sin(theta)
            if not self.is_collision_free(tmpNode, self.obstacle):
                return False
        return True


class Node():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None


class gui_window(QDialog):
    first_draw = True
    goal_flag = False
    time_step = 0
    register_pass = 0

    goal = [10, 10]
    start = [0.0, 0.0]
    randArea = [-15, 15]
    obstacleList = [(5, 5, 5), (-4, 6, 3), (3, -8, 6)]

    scale = 0.06  # 1pixel = 0.06m
    C = 1 / scale
    X_offset = 300
    Y_offset = 300

    timer = QTimer()

    def __init__(self, parent=None):
        super(gui_window, self).__init__(parent)
        self.ui = graphics()
        self.ui.setupUi(self)

        self.ui.stepsize_doubleSpinBox.setValue(1.0)
        self.ui.goal_sampling_rate_spinBox.setValue(5)
        self.ui.planning_constant_SpinBox.setValue(50)
        self.ui.Goal_X_doubleSpinBox.setValue(10)
        self.ui.Goal_Y_doubleSpinBox.setValue(10)

    def draw(self, end, obstacles, nodes):
        self.scene = QGraphicsScene()
        self.ui.graphicsView.setScene(self.scene)

        pen_axis = QPen(Qt.black)
        pen_axis.setWidth(2)
        self.scene.addLine(QLineF(-250 + self.X_offset, self.Y_offset, 250 + self.X_offset, self.Y_offset), pen_axis)
        self.scene.addLine(QLineF(self.X_offset, -250 + self.Y_offset, self.X_offset, 250 + self.Y_offset), pen_axis)
        pen_axis.setWidth(1)
        pen_axis.setStyle(Qt.DashLine)
        self.scene.addLine(QLineF(-250 + self.X_offset, 250 + self.Y_offset, -250 + self.X_offset, -250 + self.Y_offset), pen_axis)
        self.scene.addLine(QLineF(250 + self.X_offset, 250 + self.Y_offset, 250 + self.X_offset, -250 + self.Y_offset), pen_axis)

        # goal
        self.scene.addEllipse(self.C * end.x + self.X_offset - 14 / 2, -self.C * end.y + self.Y_offset - 14 / 2, 14, 14, QPen(Qt.red), QBrush(QColor(255,0,0)))

        # obstacles
        for k in range(len(obstacles)):
            self.scene.addEllipse(self.C * obstacles[k][0] + self.X_offset - self.C * obstacles[k][2] / 2,
                                  -self.C * obstacles[k][1] + self.Y_offset - self.C * obstacles[k][2] / 2,
                                  self.C * obstacles[k][2], self.C * obstacles[k][2], QPen(Qt.black), QBrush(QColor(0, 0, 255)))

        # path
        pen_path = QPen()
        pen_path.setWidth(2)
        pen_path.setColor(QColor(0, 255, 0))
        for k in range(len(nodes)):
            self.scene.addEllipse(self.C * nodes[k].x + self.X_offset - 6 / 2,
                                  -self.C * nodes[k].y + self.Y_offset - 6 / 2,
                                  6, 6, QPen(Qt.black), QBrush(QColor(0, 0, 0)))
            if nodes[k].parent is not None:
                self.scene.addLine(QLineF(self.C * nodes[k].x + self.X_offset, -self.C * nodes[k].y + self.Y_offset,
                                          self.C * nodes[nodes[k].parent].x + self.X_offset,
                                          -self.C * nodes[nodes[k].parent].y + self.Y_offset), pen_path)

    def show_final_path(self, final_pass):
        pen_final_path = QPen(Qt.red)
        pen_final_path.setWidth(3)
        for k in range(len(final_pass) - 1):
            self.scene.addLine(QLineF(self.C * final_pass[k][0] + self.X_offset,
                                      -self.C * final_pass[k][1] + self.Y_offset,
                                      self.C * final_pass[k + 1][0] + self.X_offset,
                                      -self.C * final_pass[k + 1][1] + self.Y_offset), pen_final_path)

    def calculate_path_cost(self, path):
        cost = 0.0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            cost += math.sqrt(dx**2 + dy**2)
        return cost

    def update_path_cost_label(self, path_cost):
        self.ui.path_cost_label.setText(f"Path Cost: {path_cost:.2f}")        

    def do_calculations(self):
        if self.goal_flag == False:
            if window.ui.RRT_radioButton.isChecked():
                self.rrt.Planning()
            else:
                self.rrt.RRT_star_Planning()
        else:
            self.timer.stop()

        self.time_step += 1
        self.ui.Time_Step_label.setText("Time Step : " + str(self.time_step))

    def pause(self):
        self.timer.stop()

    def reset(self):
        self.goal_flag = False
        self.time_step = 0
        self.ui.path_cost_label.setText("Path Cost: 0.0")

    def Start_simulation(self):
        self.goal[0] = float(self.ui.Goal_X_doubleSpinBox.value())
        self.goal[1] = float(self.ui.Goal_Y_doubleSpinBox.value())

        if self.ui.obstacles_checkbox.isChecked():
          obstacle_list = []  # Use default obstacles
        else:
          obstacle_list = self.obstacleList  # No obstacles

    # Use obstacle_list (not self.obstacleList) in the RRT constructor
        self.rrt = RRT(self.start, self.goal, obstacle_list, self.randArea,
                   float(self.ui.stepsize_doubleSpinBox.value()),
                   int(self.ui.goal_sampling_rate_spinBox.value()))

        self.timer.timeout.connect(self.do_calculations)
        self.timer.start(50)
        self.rrt.tree_nodes = [self.rrt.start]


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = gui_window()
    window.show()
    sys.exit(app.exec_())
