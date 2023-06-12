import numpy as np
import matplotlib.pyplot as plt


#-------------------------------------------------------------------------
#
#                       Funciones de la trayectoria
#
#-------------------------------------------------------------------------
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

# Parámetros de la circunferencia y la recta
center = (0, 0)
radius = 0.4
line_slope = 0
line_intercept = 0.3

# Encontrar puntos de intersección
intersection_points = find_intersection_points(center, radius, line_slope, line_intercept)

# Puntos que definen el óvalo
point1 = intersection_points[0]
point2 = intersection_points[1]

# Crear el óvalo que pasa por los puntos dados
#                                               , altura)
x, y = create_oval_through_points(point1, point2, 0.3)

mov_x = []
mov_y = []
for i in range(len(y)):
    if y[i] <= 0.3: # Si es menor que la distancia del robot al piso se puede elevar la pata
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