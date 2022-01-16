#!/usr/bin/env python

import rospy
import smach
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import sys
import cv2
from pyzbar import pyzbar
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int8

#Uso de la acción move_base en ROS para moverse a un punto determinado
#En ROS una acción es como una petición de un "cliente" a un "servidor"
#En este caso este código es el cliente y el servidor es ROS
#(en concreto el nodo de ROS 'move_base')
class ClienteMoveBase:
    def __init__(self):
        #creamos un cliente ROS para la acción, necesitamos el nombre del nodo 
        #y la clase Python que implementan la acción
        #Para mover al robot, estos valores son "move_base" y MoveBaseAction
        self.client =  actionlib.SimpleActionClient('move_base',MoveBaseAction)
        #esperamos hasta que el nodo 'move_base' esté activo`
        self.client.wait_for_server()

    def moveTo(self, x, y):
        #un MoveBaseGoal es un punto objetivo al que nos queremos mover
        goal = MoveBaseGoal()
        #sistema de referencia que estamos usando
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x   
        goal.target_pose.pose.position.y = y
        #La orientación es un quaternion. Tenemos que fijar alguno de sus componentes
        goal.target_pose.pose.orientation.w = 1.0

        #enviamos el goal 
        self.client.send_goal(goal)
        #vamos a comprobar cada cierto tiempo si se ha cumplido el goal
        #get_state obtiene el resultado de la acción 
        state = self.client.get_state()
        #ACTIVE es que está en ejecución, PENDING que todavía no ha empezado
        while state==GoalStatus.ACTIVE or state==GoalStatus.PENDING:
            rospy.Rate(10)   #esto nos da la oportunidad de escuchar mensajes de ROS
            state = self.client.get_state()
        return self.client.get_result()


# define state LeerQR
class LeerQR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['invalid', 'qr1', 'qr2', 'qr3'])
        self.lectura = 0
        self.subs = rospy.Subscriber('/qr', Int8, self.callback)
        
    def callback(self,data):
    	self.lectura = data.data

    def execute(self,userdata):
        #rospy.loginfo('Leyendo QR...')      
        if self.lectura == 1:
            return 'qr1'
        if self.lectura == 2:
            return 'qr2'
        if self.lectura == 3:
            return 'qr3'
        else:
            return 'invalid'
	
# define state Bar
class IrE1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['producto_entregado'])

    def execute(self, userdata):
        rospy.loginfo('Yendo a la estación 1')
        #moveto
        cliente = ClienteMoveBase()
        result = cliente.moveTo(1.03, 3.01)
        rospy.loginfo('Ha llegado a la estacion')
        return 'producto_entregado'

class IrE2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['producto_entregado'])

    def execute(self, userdata):
        rospy.loginfo('Yendo a la estación 2')
        #moveto
        cliente = ClienteMoveBase()
        result = cliente.moveTo(-3.3, 2.87)
        rospy.loginfo('Ha llegado a la estacion')
        return 'producto_entregado'

class IrE3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['producto_entregado'])

    def execute(self, userdata):
        rospy.loginfo('Yendo a la estación 3')
        #moveto
        cliente = ClienteMoveBase()
        result = cliente.moveTo(-7.16, 3.02)
        return 'producto_entregado'

class VolverHOME(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['home'])

    def execute(self, userdata):
        rospy.loginfo('Volviendo a posición de inicio')
        cliente = ClienteMoveBase()
        result = cliente.moveTo(5.7,2.49)
        return 'home'      
        
# main
def main():
    #rospy.init_node('camera_read', anonymous=False)
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine #que hay que poner de outcome aqui????
    sm = smach.StateMachine(outcomes=[]) 
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('VolverHOME', VolverHOME(), 
                               transitions={'home':'LeerQR'})
        smach.StateMachine.add('LeerQR', LeerQR(), 
                               transitions={'invalid':'LeerQR',                              
                                            'qr1':'IrE1',                
                                            'qr2':'IrE2', 
                                            'qr3':'IrE3'})
        smach.StateMachine.add('IrE1', IrE1(), 
                               transitions={'producto_entregado':'VolverHOME'})
        smach.StateMachine.add('IrE2', IrE2(), 
                               transitions={'producto_entregado':'VolverHOME'})
        smach.StateMachine.add('IrE3', IrE3(), 
                               transitions={'producto_entregado':'VolverHOME'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()

