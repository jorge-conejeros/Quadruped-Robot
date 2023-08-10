import numpy as np
#import matplotlib.pyplot as plt
import sympy as sp
import pandas as pd

#-------------------------------------------------------------------------
#
#                       Funciones de la trayectoria
#
#-------------------------------------------------------------------------

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



def find_intersection_points(center, radius, line_slope, line_intercept):
    a = 1 + line_slope**2
    b = -2 * center[0] + 2 * line_slope * (line_intercept - center[1])
    c = center[0]**2 + (line_intercept - center[1])**2 - radius**2

    discriminant = b**2 - 4*a*c

    if discriminant < 0:
        return []  # No intersection points

    x1 = (-b + np.sqrt(discriminant)) / (2*a)
    x2 = (-b - np.sqrt(discriminant)) / (2*a)
    y1 = line_slope * x1 + line_intercept
    y2 = line_slope * x2 + line_intercept

    return [(x1, y1), (x2, y2)]

def create_oval_through_points(point1, point2, altura):
    x1, y1 = point1
    x2, y2 = point2
    
    # Calcular el ancho y la altura utilizando los puntos dados
    if x1 > x2:
        width = x1*2
    elif x1 < x2:
        width = x2*2
    height = altura
    
    # Calcular el centro del óvalo
    center_x = (x1 + x2) / 2
    center_y = (y1 + y2) / 2
    
    # Generar el óvalo utilizando las dimensiones calculadas
    theta = np.linspace(0, 2 * np.pi, 100)
    x = center_x + width/2 * np.cos(theta)
    y = center_y + height/2 * np.sin(theta)
    
    return x, y


#-------------------------------------------------------------------------
#
#                       Parametrización
#
#------------------------------------------------------------------------

# Descripción de Denavit-Hartenberg (a = largo de la extremidad (en metros))
#      theta     |      d      |      a      |    alpha
# ----------------------------------------------------------
#       q1             0             0.2            0
#       q2             0             0.2            0


largo_extremidad = 0.2
q1 = sp.symbols('q1')
q2 = sp.symbols('q2')

T = sp.simplify(symTfromDH(q1, 0, largo_extremidad, 0)* symTfromDH(q2, 0, largo_extremidad, 0))

# Parámetros de la circunferencia y la recta
center = (0, 0)
radius = largo_extremidad * 2
line_slope = 0
altura_actual = 0.3

# Encontrar puntos de intersección
intersection_points = find_intersection_points(center, radius, line_slope, altura_actual)

# Puntos que definen el óvalo
point1 = intersection_points[0]
point2 = intersection_points[1]

# Crear el óvalo que pasa por los puntos dados
#                                               , altura)
x, y = create_oval_through_points(point1, point2, altura_actual)

mov_x = []
mov_y = []
for i in range(len(y)):
    if y[i] <= altura_actual: # Si es menor que la distancia del robot al piso se puede elevar la pata
        mov_x.append(x[i])
        mov_y.append(y[i])

#--------------------------------------------------------
#
#       aGREGAR EL ARRASTE, LA LINEA RECTA
#
#--------------------------------------------------------

aux1 = [0.3 for i in range(len(mov_y))]
mov_y = mov_y + aux1
aux1 = np.linspace(mov_x[len(mov_x)-1], mov_x[1], len(mov_x))
mov_x = np.concatenate((mov_x,aux1))

df = pd.DataFrame(columns=['Muslo', 'Rodilla'])

# Levantamiento de pata hacia delante y Arrastre, LADO Derecho
for i in range(len(mov_y)):
    a=mov_y[i]
    b=mov_x[i]
    # definimos las ecuaciones a resolver
    ec1, ec2 = T[3]-a, T[7]-b
    (ec1, ec2)
    # ahora resolvemos la ecuación utilizando nsolve()
    try:
        q = sp.nsolve((ec1, ec2),(q1,q2),(1,-1), prec=6)
        df.loc[len(df)] = {'Muslo': q[0], 'Rodilla': q[1]}
    except:
        print(a,b)
        print("no se pudo calcular")
        q = [0, 0, 0]
        
