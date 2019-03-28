import sys
import os

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from PyQt5 import QtWidgets,QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

#from mymodules import fk_module
#from mymodules import Closed_form_ik
from kinematic_generator import Generator

import glob
import socket
import math

class Application(QtWidgets.QWidget):
    def __init__(self):
        super(Application,self).__init__()
        #Create kinematics Object and Get initial location of each motor
        self.initKinematics()
        # Initialize UI of qt
        self.initUI()
        # Initialize Figure&Menu
        #self.initFigure()
        self.initMenu()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(1)#update UI every 1 msec



    ##initialize the specification of the Arm
    def initKinematics(self):
        #self._L = self.arm.LINKS_LENGTH
        self.q_min= 90
        self.q_max= 90
        #self.kinematics = fk_module.Generater(L=self._L, q_max=self.q_max, q_min=self.q_min, eef_only=False)
        #self._initPos, self._initNOA = self.kinematics.fk([np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0)]) #(xyz)*5 + n_o_a

    ##initialize the UI
    def initUI(self):
        self._count = True
        # Create Widget for Figure/Menu
        self.FigureWidget = QtWidgets.QWidget(self)
        self.MenuWidget = QtWidgets.QWidget(self)
        # Add Layout to each Widget
        self.FigureLayout = QtWidgets.QVBoxLayout(self.FigureWidget)
        self.MenuLayout = QtWidgets.QVBoxLayout(self.MenuWidget)
        # Delete Margin of the layout
        self.FigureLayout.setContentsMargins(0,0,0,0)
        self.MenuLayout.setContentsMargins(0,0,0,0)
        # set Geometry
        self.setGeometry(0,0,900,600)
        self.FigureWidget.setGeometry(400,0,700,600)
        self.MenuWidget.setGeometry(0,0,200,600)

    ## initialize the Figure setting
    def initFigure(self):
        ## Creat the Figure to drow 3D plot
        self.Figure = plt.figure()
        # Add FigureCanvas to Figure
        self.FigureCanvas = FigureCanvas(self.Figure)
        # Addd FigureCanvas to Layout
        self.FigureLayout.addWidget(self.FigureCanvas)
        self.axis = self.Figure.add_subplot(121, projection='3d')
        self.axis.set_xlabel("X-axis")
        self.axis.set_ylabel("Y-axis")
        self.axis.set_zlabel("Z-axis")
        #set the limit of each axis
        self.axis.set_xlim3d(-0.7,0.2)
        self.axis.set_ylim3d(-0.2,0.7)
        self.axis.set_zlim3d(-0.7,0.2)

        #Drow every link of the arm
        self.drow_arm(self._initPos[:,0], self._initPos[:,1],self._initPos[:,2])
        #Drow the (shoulder-elbow-wrist-eeef) positions and coordinations
        self.drow_coordination(self._initPos,self._initNOA)

    def initMenu(self):
        #Create the label objects
        self.xlabel    = QtWidgets.QLabel(self)
        self.ylabel    = QtWidgets.QLabel(self)
        self.zlabel    = QtWidgets.QLabel(self)

        self.th0label  = QtWidgets.QLabel(self)
        self.th1label  = QtWidgets.QLabel(self)
        self.th2label  = QtWidgets.QLabel(self)
        self.th3label  = QtWidgets.QLabel(self)
        self.th4label  = QtWidgets.QLabel(self)
        self.th5label  = QtWidgets.QLabel(self)

        self.deg0label = QtWidgets.QLabel(self)
        self.deg1label = QtWidgets.QLabel(self)
        self.deg2label = QtWidgets.QLabel(self)
        self.deg3label = QtWidgets.QLabel(self)
        self.deg4label = QtWidgets.QLabel(self)
        self.deg5label = QtWidgets.QLabel(self)

        #Create the textbox objects
        self.xtextbox = QtWidgets.QLineEdit(self)
        self.ytextbox = QtWidgets.QLineEdit(self)
        self.ztextbox = QtWidgets.QLineEdit(self)

        self.deg0textbox = QtWidgets.QLineEdit(self)
        self.deg1textbox = QtWidgets.QLineEdit(self)
        self.deg2textbox = QtWidgets.QLineEdit(self)
        self.deg3textbox = QtWidgets.QLineEdit(self)
        self.deg4textbox = QtWidgets.QLineEdit(self)
        self.deg5textbox = QtWidgets.QLineEdit(self)
        #Create the button objects
        self.calcbutton = QtWidgets.QPushButton('calculate',self)
        self.drowbutton = QtWidgets.QPushButton('drow',self)
        #set the size of each textbox
        pos_size = [100,20]
        self.xtextbox.resize( pos_size[0],pos_size[1])
        self.ytextbox.resize( pos_size[0],pos_size[1])
        self.ztextbox.resize( pos_size[0],pos_size[1])

        noa_size = [40,20]

        th_size = [80,20]

        self.deg0textbox.resize(th_size[0],th_size[1])
        self.deg1textbox.resize(th_size[0],th_size[1])
        self.deg2textbox.resize(th_size[0],th_size[1])
        self.deg3textbox.resize(th_size[0],th_size[1])
        self.deg4textbox.resize(th_size[0],th_size[1])
        self.deg5textbox.resize(th_size[0],th_size[1])
        #set the size of each button object
        self.calcbutton.resize(80,30)
        self.drowbutton.resize(80,30)
        #set the name of each label
        self.xlabel.setText('x')
        self.ylabel.setText('y')
        self.zlabel.setText('z')

        self.th0label.setText('th0')
        self.th1label.setText('th1')
        self.th2label.setText('th2')
        self.th3label.setText('th3')
        self.th4label.setText('th4')
        self.th5label.setText('th5')

        self.deg0label.setText('[deg]')
        self.deg1label.setText('[deg]')
        self.deg2label.setText('[deg]')
        self.deg3label.setText('[deg]')
        self.deg4label.setText('[deg]')
        self.deg5label.setText('[deg]')
        #set the location of each labels
        pos = [15,  30, 50,  30, 200,  30, 255, 30]
        self.xlabel.move(   pos[0], pos[1]   )
        self.ylabel.move(   pos[0], pos[1]+30)
        self.zlabel.move(   pos[0], pos[1]+60)
        self.xtextbox.move( pos[2], pos[3]   )
        self.ytextbox.move( pos[2], pos[3]+30)
        self.ztextbox.move( pos[2], pos[3]+60)

        self.calcbutton.move(pos[2],pos[3]+90)

        ang = [15, 350, 55, 350, 135, 350, 200, 350, 280, 350]

        self.th0label.move(   pos[0], pos[4])
        self.th1label.move(   pos[0], pos[4]+30)
        self.th2label.move(   pos[0], pos[4]+60)
        self.th3label.move(   pos[0], pos[4]+90)
        self.th4label.move(   pos[0], pos[4]+120)
        self.th5label.move(   pos[0], pos[4]+150)

        self.deg0textbox.move(pos[2], pos[4])
        self.deg1textbox.move(pos[2], pos[4]+30)
        self.deg2textbox.move(pos[2], pos[4]+60)
        self.deg3textbox.move(pos[2], pos[4]+90)
        self.deg4textbox.move(pos[2], pos[4]+120)
        self.deg5textbox.move(pos[2], pos[4]+150)

        self.deg0label.move(  130, pos[4])
        self.deg1label.move(  130, pos[4]+30)
        self.deg2label.move(  130, pos[4]+60)
        self.deg3label.move(  130, pos[4]+90)
        self.deg4label.move(  130, pos[4]+120)
        self.deg5label.move(  130, pos[4]+150)
        
        self.drowbutton.move( pos[2],pos[4]+180)
        # connect the button to the click_action
        self.calcbutton.clicked.connect(self.on_calcclick)
        self.drowbutton.clicked.connect(self.on_drowclick)

    def drow_arm(self,x,y,z):
        self.axis.plot(x, y, z, color='orange', linewidth = 3.0)
        self.axis.scatter(x, y, z, linewidth = 3.0)
        self.FigureCanvas.draw()
        # length = np.concatenate((x.reshape(5,1),y.reshape(5,1),z.reshape(5,1)),axis=1)
        # print('0.245 = ',math.sqrt((length[0][0]-length[1][0])**2+(length[0][1]-length[1][1])**2+(length[0][2]-length[1][2])**2))
        # print('0.315 = ',math.sqrt((length[1][0]-length[2][0])**2+(length[1][1]-length[2][1])**2+(length[1][2]-length[2][2])**2))
        # print('0.230 = ',math.sqrt((length[2][0]-length[3][0])**2+(length[2][1]-length[3][1])**2+(length[2][2]-length[3][2])**2))
        # print('0.150 = ',math.sqrt((length[3][0]-length[4][0])**2+(length[3][1]-length[4][1])**2+(length[3][2]-length[4][2])**2))

    def drow_coordination(self,p,noa):
            ############################
            ###In my fk_module, 'Posion and NOA(vectors which consists the each axis)'  parameters are returned by following orders
            ### [[px,py,pz],[px,py,pz]....],
            ### [[[ox,oy,oz], [nx,ny,nz], [ax,ay,az]],[[ox,oy,oz], [nx,ny,nz], [ax,ay,az]]....]
            ###Then, need to transpose the vector_noa to make it to the usual orders.
            ### [[|ax|   [|ox|   [|ax|
            ###   |ay|    |oy|    |ay|
            ###   |az|],  |oz|],  |az|] ]
            ############################
        for k in range(0,5):
            self.plot_coordination(p[k,:],(15*p[k,:]+(noa[k*3:k*3+3,:].T).reshape(3,3))/15) #make the length of each axis to  1/15
            # self.plot_coordination(p[k,:],(3*p[k,:]+(noa[k*3:k*3+3,:].T).reshape(3,3))/3) ##make the length of each axis to  1/3

    def plot_coordination(self,centor,axis):
        x = np.concatenate((centor.reshape(1,3),axis[0,:].reshape(1,3)),axis=0)
        y = np.concatenate((centor.reshape(1,3),axis[1,:].reshape(1,3)),axis=0)
        z = np.concatenate((centor.reshape(1,3),axis[2,:].reshape(1,3)),axis=0)
        self.axis.plot(x[:,0],x[:,1],x[:,2],color="Red")
        self.axis.plot(y[:,0],y[:,1],y[:,2],color="Blue")
        self.axis.plot(z[:,0],z[:,1],z[:,2],color="Green")
        print('z location',z[:,0],z[:,1],z[:,2])

    def on_calcclick(self):
        x,y,z,nx,ny,nz,ox,oy,oz,ax,ay,az,alpha = self.get_input_val()
        #check if the input values are in-range of the arm workspace
        if (self.check(x,y,z,nx,ny,nz,ox,oy,oz,ax,ay,az)):
            #calculate the ik
            self.closed_form(x,y,z,nx,ny,nz,ox,oy,oz,ax,ay,az,alpha)
            # self.send(x,y,z,nx,ny,nz,ox,oy,oz,ax,ay,az)
        else:
            #ERROR Message
            QtWidgets.QMessageBox.question(self, 'ERROR', "You should input inrange values ", QtWidgets.QMessageBox.Ok, QtWidgets.QMessageBox.Ok)

    def on_drowclick(self):
        #get the parameters of the inputs
        th = self.get_output_val()
        th = self.saturation_th(th)
        th_deg = self.rad2deg(th)

        self.deg0textbox.setText(str(self.rounding(th_deg[0])))
        self.deg1textbox.setText(str(self.rounding(th_deg[1])))
        self.deg2textbox.setText(str(self.rounding(th_deg[2])))
        self.deg3textbox.setText(str(self.rounding(th_deg[3])))
        self.deg4textbox.setText(str(self.rounding(th_deg[4])))
        self.deg5textbox.setText(str(self.rounding(th_deg[5])))
        # calculate the fk
        p,noa = self.kinematics.fk([float(th[0]),float(th[1]),float(th[2]),float(th[3]),float(th[4]),float(th[5])])
        # update the arm drowing
        self.drow_arm(p[:,0], p[:,1],p[:,2])
        # update the arm coordination
        self.drow_coordination(p,noa)

        #output the locations of the eef
        self.pxtextbox.setText(str(self.rounding(p[-1,-3])))
        self.pytextbox.setText(str(self.rounding(p[-1,-2])))
        self.pztextbox.setText(str(self.rounding(p[-1,-1])))
        # print(p[-1,:])

    def get_input_val(self):
        x  = self.xtextbox.text()
        y  = self.ytextbox.text()
        z  = self.ztextbox.text()
        nx = self.nxtextbox.text()
        ny = self.nytextbox.text()
        nz = self.nztextbox.text()
        ox = self.oxtextbox.text()
        oy = self.oytextbox.text()
        oz = self.oztextbox.text()
        ax = self.axtextbox.text()
        ay = self.aytextbox.text()
        az = self.aztextbox.text()
        alpha = self.alphatextbox.text()
        return x,y,z,nx,ny,nz,ox,oy,oz,ax,ay,az,alpha

    def get_output_val(self):
        th0 = self.rad0textbox.text()
        th1 = self.rad1textbox.text()
        th2 = self.rad2textbox.text()
        th3 = self.rad3textbox.text()
        th4 = self.rad4textbox.text()
        th5 = self.rad5textbox.text()
        return th0,th1,th2,th3,th4,th5

    def check(self,x,y,z,nx,ny,nz,ox,oy,oz,ax,ay,az):
        L0_L1 = self._L[0] + self._L[1]
        L2_L3 = self._L[2] + self._L[3]

        _x,_y,_z = float(x),float(y),float(x)
        if (((_x+L0_L1)**2+_y**2+_z**2) < (L2_L3)**2 and float(y)>0.15):
            return True
        else:
            return False

    def saturation_th(self,q):
        ## q should be radians
        q_min = np.deg2rad(self.q_min)
        q_max = np.deg2rad(self.q_max)
        middle = ( q_max + (2*math.pi + q_min) )/2
        print('================================')
        for i in range(len(q)):
            if (float(q[i]) >= q_max[i]):
                print('theta[',i,'] is out of the limit')
                if ((float(q[i])-2*math.pi) > q_min[i]):
                    q[i] = float(q[i])-2*math.pi
                elif (q[i] <= middle[i]):
                    q[i] = q_max[i]
                else:
                    q[i] = q_min[i]
        print('================================')
        return q

    def rounding(self,k):
        return "{:.4f}".format(k)

    #def closed_form(self,x,y,z,nx,ny,nz,ox,oy,oz,ax,ay,az,alpha):
    #    #Create the instance of closed_form Module
    #    print('calculatiing ik...')
    #    cl_ik = Closed_form_ik.inv(L=self._L, q_max=[180,10,60,90,90,90], q_min=[-30,-90,-90,0,-90,0], eef_only=False)
    #
    #    #calculate the ik
    #    x_plot,y_plot,z_plot,q0,q1,q2,q3,q4,q5 = cl_ik.solve(x,y,z,nx,ny,nz,ox,oy,oz,ax,ay,az,alpha)
    #    p,noa = self.kinematics.fk([float(q0),float(q1),float(q2),float(q3),float(q4),float(q5)])
    #
    #    # Drow every link of the arm
    #    self.drow_arm(x_plot,y_plot,z_plot)  ##desired locarion & orientation
    #    self.drow_arm(p[:,0], p[:,1],p[:,2]) ##result of ik
    #
    #    #Drow the (shoulder-elbow-wrist-eeef) positions and coordinations
    #    self.drow_coordination(p,noa)
    #
    #    #update UI
    #    self.set_output_text(q0,q1,q2,q3,q4,q5,p[-1,-3],p[-1,-2],p[-1,-1])

    ##For TCP/IP communication
    #def send(self,x,y,z,nx,ny,nz,ox,oy,oz,ax,ay,az):
    #    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    #        # set the IP Address & Port number
    #        s.connect(('127.0.0.1', 50007))
    #        data = x + ',' + y + ',' + z + ',' + nx + ',' + ny + ',' + nz + ',' + ox + ',' + oy + ',' + oz + ',' + ax + ',' + ay + ',' + az
    #        s.sendall(data.encode())
    #
    #        #Plot the Target position
    #        self.axis.scatter(float(x), float(y), float(z))
    #        # The buffer size of the network is 1024: Get the strings from the server
    #        r_data = str(s.recv(1024).decode())
    #        r_data = r_data.split(',')
    #        # print(r_data)
    #        #Output the values on the textbox
    #        self.rad0textbox.setText(str(self.rounding(float(r_data[0]))))
    #        self.rad1textbox.setText(str(self.rounding(float(r_data[1]))))
    #        self.rad2textbox.setText(str(self.rounding(float(r_data[2]))))
    #        self.rad3textbox.setText(str(self.rounding(float(r_data[3]))))
    #        self.rad4textbox.setText(str(self.rounding(float(r_data[4]))))
    #        self.rad5textbox.setText(str(self.rounding(float(r_data[5]))))
    #        th0,th1,th2,th3,th4,th5 = self.get_output_val()
    #        self.deg0textbox.setText(str(self.rounding(np.rad2deg(float(th0)))))
    #        self.deg1textbox.setText(str(self.rounding(np.rad2deg(float(th1)))))
    #        self.deg2textbox.setText(str(self.rounding(np.rad2deg(float(th2)))))
    #        self.deg3textbox.setText(str(self.rounding(np.rad2deg(float(th3)))))
    #        self.deg4textbox.setText(str(self.rounding(np.rad2deg(float(th4)))))
    #        self.deg5textbox.setText(str(self.rounding(np.rad2deg(float(th5)))))
    #        # print('got result from server')

    def rad2deg(self,th):
        th_deg = np.empty(6)
        for i in range(len(th)):
            if(float(th[i]) < math.pi):
                th_deg[i] = np.rad2deg(float(th[i]))
            else:
                th_deg[i] = np.rad2deg(float(th[i])-2*math.pi)
        return th_deg

    def init_input_text(self):
        self.xtextbox.setText('')
        self.ytextbox.setText('')
        self.ztextbox.setText('')
        self.nxtextbox.setText('')
        self.nytextbox.setText('')
        self.nztextbox.setText('')
        self.oxtextbox.setText('')
        self.oytextbox.setText('')
        self.oztextbox.setText('')
        self.axtextbox.setText('')
        self.aytextbox.setText('')
        self.aztextbox.setText('')
        self.alphatextbox.setText('')

    def init_output_text(self):
        self.rad0textbox.setText('')
        self.rad1textbox.setText('')
        self.rad2textbox.setText('')
        self.rad3textbox.setText('')
        self.rad4textbox.setText('')
        self.rad5textbox.setText('')
        self.deg0textbox.setText('')
        self.deg1textbox.setText('')
        self.deg2textbox.setText('')
        self.deg3textbox.setText('')
        self.deg4textbox.setText('')
        self.deg5textbox.setText('')
        self.pxtextbox.setText('')
        self.pytextbox.setText('')
        self.pztextbox.setText('')

    def set_output_text(self,r0,r1,r2,r3,r4,r5,px,py,pz):
        self.rad0textbox.setText(str(self.rounding(float(r0))))
        self.rad1textbox.setText(str(self.rounding(float(r1))))
        self.rad2textbox.setText(str(self.rounding(float(r2))))
        self.rad3textbox.setText(str(self.rounding(float(r3))))
        self.rad4textbox.setText(str(self.rounding(float(r4))))
        self.rad5textbox.setText(str(self.rounding(float(r5))))
        self.deg0textbox.setText(str(self.rounding(float(np.rad2deg(r0)))))
        self.deg1textbox.setText(str(self.rounding(float(np.rad2deg(r1)))))
        self.deg2textbox.setText(str(self.rounding(float(np.rad2deg(r2)))))
        self.deg3textbox.setText(str(self.rounding(float(np.rad2deg(r3)))))
        self.deg4textbox.setText(str(self.rounding(float(np.rad2deg(r4)))))
        self.deg5textbox.setText(str(self.rounding(float(np.rad2deg(r5)))))
        self.pxtextbox.setText(str(self.rounding(px)))
        self.pytextbox.setText(str(self.rounding(py)))
        self.pztextbox.setText(str(self.rounding(pz)))

    # assign the action to KeyPressEvent
    def keyPressEvent(self, e):
        if e.key() == QtCore.Qt.Key_Escape:
            self.close()
        if e.key() == QtCore.Qt.Key_Return:
            print()


if __name__ == '__main__':
    QApp = QtWidgets.QApplication(sys.argv)
    app = Application()
    app.show()
    sys.exit(QApp.exec_())
