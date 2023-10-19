import sim
import numpy as np
import sympy as sp

def connect(port):
# Establece la conexión a VREP
# port debe coincidir con el puerto de conexión en VREP
# retorna el número de cliente o -1 si no puede establecer conexión
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',port,True,True,2000,5) # Conectarse
    if clientID == 0: print("conectado a", port)
    else: print("no se pudo conectar")
    return clientID

# Conectarse al servidor de VREP
# *** ejecutar cada vez que se reinicia la simulación ***
clientID = connect(19999)

returnCode,handle=sim.simxGetObjectHandle(clientID,'Dummy',sim.simx_opmode_blocking)
dummy = handle
# Identificar las Articulaciones
ret,joint1=sim.simxGetObjectHandle(clientID,'Art_1',sim.simx_opmode_blocking)
ret,joint2=sim.simxGetObjectHandle(clientID,'Art_2',sim.simx_opmode_blocking)

def symTfromDH(theta, d, a, alpha):
    # theta y alpha en radianes
    # d y a en metros
    Rz = sp.Matrix([[sp.cos(theta), -sp.sin(theta), 0, 0],
                   [sp.sin(theta), sp.cos(theta), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    tz = sp.Matrix([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, d],
                   [0, 0, 0, 1]])
    ta = sp.Matrix([[1, 0, 0, a],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    Rx = sp.Matrix([[1, 0, 0, 0],
                   [0, sp.cos(alpha), -sp.sin(alpha), 0],
                   [0, sp.sin(alpha), sp.cos(alpha), 0],
                   [0, 0, 0, 1]])
    T = Rz*tz*ta*Rx
    return T

q1 = sp.symbols('q1')
q2 = sp.symbols('q2')

T = sp.simplify(symTfromDH(q1, 0, 0.2, 0)* symTfromDH(q2, 0, 0.2, 0))

returnCode,target=sim.simxGetObjectHandle(clientID,'Target',sim.simx_opmode_blocking)
returnCode,pos_target=sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_blocking)

# Ejemplo de nuevas coordenadas
a=pos_target[0]
b=pos_target[1]

# definimos las ecuaciones a resolver
ec1, ec2 = T[3]-a, T[7]-b
(ec1, ec2)

# ahora resolvemos la ecuación utilizando nsolve()
try:
    q = sp.nsolve((ec1, ec2),(q1,q2),(1,-1), prec=6)
except:
    print("no se pudo calcular")
    q = [0, 0, 0]
q
# enviamos los ángulos al robot
retCode = sim.simxSetJointTargetPosition(clientID, joint1, q[0], sim.simx_opmode_oneshot)
retCode = sim.simxSetJointTargetPosition(clientID, joint2, q[1], sim.simx_opmode_oneshot)