
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

#puertos que se usan
#MOTORES IZQ (D), DER(B)
#SENSORES LUZ IZQ (C), DER(A), FRENTE (E), ULTRASONICO (F)

hub = PrimeHub()

hub.speaker.volume(100)
#Calibracion de los sensores
#Valor sensor izquierdo centro:17
#Valor sensor izquierdo borde:48
#Valor sensor derecho centro:18
#Valor sensor derecho borde:48
#Valor sensor frontal centro:7
#Valor sensor fronta borde:27

#umbrales de negro
centro_negro = 25
cenntro_negro_frente= 24
borde_negro = 35

#umbral linea lateral
#sensor frontal E
negro_frontal = 9
#sensor derecha A
negro_derecha = 13
#sensor izquierda C
negro_izquierda = 13

# Inicializacion de los motores
motor_izquierdo = Motor(Port.D,Direction.COUNTERCLOCKWISE) #motor izquierdo sentido antihorario
motor_derecho = Motor(Port.B,Direction.CLOCKWISE) #motor derecho sentido horario

# Inicialización de los sensores
distance_sensor = UltrasonicSensor(Port.F)
color_sensor_l = ColorSensor(Port.C)
color_sensor_r = ColorSensor(Port.A)
color_sensor_f = ColorSensor(Port.E)

#funcion para avanzar-retroceder
def movermotores(velocidad_motor_izquierdo,velocidad_motor_derecho,rotacion):
    motor_izquierdo.run_angle(velocidad_motor_izquierdo,rotacion*360,wait=False)
    motor_derecho.run_angle(velocidad_motor_derecho,rotacion*360,wait=False)
    while not motor_izquierdo.done() or not motor_derecho.done():
        wait (10)


#funcion para mover motores de forma indefinida
def mover_motores_indefinido(velocidad_motor_izquierdo,velocidad_motor_derecho):
    motor_izquierdo.run(velocidad_motor_izquierdo)
    motor_derecho.run(velocidad_motor_derecho)

#funcion para parar motores
def parar_motores():
    motor_izquierdo.stop()
    motor_derecho.stop()


#lista de colores a usar, calibrar en cualquier pista
Color.BLACK = Color(h=180, s=20, v=27)
Color.GREEN = Color(h=166, s=68, v=34)
Color.WHITE = Color(h=204, s=17, v=93)
Color.SILVER = Color(h=0, s=0, v=99)


colores = (Color.BLACK, Color.GREEN,Color.NONE,Color.WHITE)
color_sensor_l.detectable_colors(colores)
color_sensor_r.detectable_colors(colores)
color_sensor_f.detectable_colors(colores)


# Función de actualización de los sensores
def update():
    global color_l, color_r, light_l, light_f, light_r, distance
    
    # Obtener la distancia del sensor de ultrasonido
    distance = distance_sensor.distance()
    
    # Obtener el color detectado por los sensores de color
    color_l = color_sensor_l.color()
    color_r = color_sensor_r.color()
    color_f = color_sensor_f.color()
    
    # Obtener la reflexión de luz de los sensores de color
    light_l = color_sensor_l.reflection()
    light_r = color_sensor_r.reflection()
    light_f = color_sensor_f.reflection()

#declaro las constantes a utilizar en mi logica de seguidor proporcional integrativo derivativo y luego las variables a utilizar
velocidad=20
#constante de proporcion
kp=1
#constante integral
ki=0
#constante derivativa
kd=0

error=10
error_anterior = 0
suma_errores = 0
P, I, D = 0,0,0
contador = 0.001
ultima_deteccion = StopWatch()

def seguidor_de_linea():
    global error, error_anterior, suma_errores, P, I, D, contador
    tiempo_actual = ultima_deteccion.time()
    #print(tiempo_actual)

    # Calcular el error entre los sensores izquierdo y derecho
    error = light_r - light_l

    # Comprobar si algún sensor detecta la línea negra
    if light_r < borde_negro or light_l < borde_negro:#  or light_f < borde_negro:
        ultima_deteccion.reset()


    # Si han pasado más de 3 segundos sin detectar la línea, retroceder
    if tiempo_actual > 6000:
        parar_motores()
        while True:
            mover_motores_indefinido(-70,-70)
            if color_sensor_r.reflection() < (borde_negro+5) or color_sensor_l.reflection() < (borde_negro+5) or color_sensor_f.reflection() < (negro_frontal+5): 
                parar_motores()
                ultima_deteccion.reset()
                break
               
    else:
        # Cálculo PID
        P = error
        I += error * contador
        D = (error - error_anterior) / contador
        correccion = int(P * kp + I * ki + D * kd)

        # Ajustar las potencias de los motores
        potencia_motor_izquierdo = velocidad - correccion
        potencia_motor_derecho = velocidad + correccion

        # Aplicar las potencias a los motores
        motor_izquierdo.dc(potencia_motor_izquierdo)
        motor_derecho.dc(potencia_motor_derecho)

        # Actualizar el error anterior
        error_anterior = error

def giro_90_grados_derecha():
    hub.imu.reset_heading(0)
    while hub.imu.heading() < 75:
        print(hub.imu.heading())
        mover_motores_indefinido(150,-150)
    parar_motores()
    wait(300)
        

def giro_90_grados_izquierda():
    hub.imu.reset_heading(0)
    while hub.imu.heading() >-65:
        print(hub.imu.heading())
        motor_izquierdo.run(-350)
        motor_derecho.run(350)
    parar_motores()
    wait(300)

def giro_180_grados_derecha():
    hub.imu.reset_heading(0)
    while hub.imu.heading() <177:
        print(hub.imu.heading())
        motor_izquierdo.run(300)
        motor_derecho.run(-300)
    parar_motores()

def obstaculo():
    if -1<distance<48:
        parar_motores()
        hub.speaker.beep(700,30)
        if -1<distance_sensor.distance()<48:
            #hub.display.text("obs")
            movermotores(-100,-100,0.1)
            hub.imu.reset_heading(0)
            while hub.imu.heading()>-70:
                mover_motores_indefinido(-100,100)
            parar_motores()
            movermotores(100,100,0.1)
            while color_sensor_l.reflection() > centro_negro:
                mover_motores_indefinido(285,100)
            parar_motores()
            movermotores(100,100,0.08)
            while color_sensor_r.reflection() > centro_negro:
                mover_motores_indefinido(-100,100)
            parar_motores()


def buscar_linea_derecha():
    parar_motores()
    #hub.display.text("BLD")
    hub.imu.reset_heading(0)
    while hub.imu.heading() < 45:
        mover_motores_indefinido(180,-180)
    parar_motores()

    while color_sensor_r.reflection() > (centro_negro+5):
        mover_motores_indefinido(-180,180)
    parar_motores()

    while color_sensor_l.reflection() > (centro_negro+5):
        mover_motores_indefinido(180,-180)
    parar_motores()



def buscar_linea_izquierda():
    parar_motores()
    #hub.display.text("BLI")
    hub.imu.reset_heading(0)
    while hub.imu.heading() > -45:
        mover_motores_indefinido(-180,180)
    parar_motores()

    while color_sensor_l.reflection() > (centro_negro+5):
        mover_motores_indefinido(180,-180)
    parar_motores()

    while color_sensor_r.reflection() > (centro_negro+5):
        mover_motores_indefinido(-100,100) #giro a la derecha para acomodarme a la linea con el sensor derecho
    parar_motores()

    """while color_sensor_f.reflection() > cenntro_negro_frente:
        mover_motores_indefinido(60,-60) #giro a la izquierda para acomodarme a l alinea con el sensor del frente
    parar_motores()"""

def doble_negro():
    if light_l < centro_negro+1 and light_r < centro_negro+1 and light_f < centro_negro+1:
        parar_motores()
        movermotores(70,70,0.04)
        if light_l < centro_negro+1 and light_r < centro_negro+1 and light_f < centro_negro+1:
            hub.display.text("DN")
            movermotores(70,70,0.1)


def deteccion_señal_verde():
    if color_r == Color.GREEN:
        motor_izquierdo.stop()
        motor_derecho.stop()
        movermotores(100,100,0.05)
        
        if color_sensor_l.color() == Color.GREEN:
            hub.display.text("DV")
            giro_180_grados_derecha()
            

        else:
            hub.display.text("VD")#verde derecha
            movermotores(150,150,0.15)
            giro_90_grados_derecha()
            movermotores(150,150,0.12)
            buscar_linea_derecha()

    if color_l == Color.GREEN:
        motor_izquierdo.stop()
        motor_derecho.stop()
        movermotores(100,100,0.05)
        

        if color_sensor_r.color() == Color.GREEN:
            hub.display.text("DV")
            giro_180_grados_derecha()
        
        
        elif color_l==  Color.GREEN:
            hub.display.text("VI")
            movermotores(150,150,0.15)
            giro_90_grados_izquierda()
            wait(10)
            movermotores(150,150,0.12)
            buscar_linea_izquierda()
        

def T_derecha():
    if light_r < negro_derecha+5 and light_f < negro_frontal+5:
        parar_motores()
        movermotores(100,100,0.03)
        if color_sensor_r.reflection() < negro_derecha+5 and color_sensor_f.reflection() < negro_frontal+5:
            parar_motores()
            wait(500)
            hub.display.text("TD")
            wait(1000)
            movermotores(70,70,0.22)


def T_izquierda():
        if light_l < negro_izquierda+5 and light_f < negro_frontal+5:
            parar_motores()
            movermotores(100,100,0.03)
        if color_sensor_l.reflection() < negro_izquierda+5 and color_sensor_f.reflection() < negro_frontal+5:
            parar_motores()
            wait(500)
            hub.display.text("TI")
            wait(1000)
            movermotores(70,70,0.22)


# Bucle principal
def main():
    while True:
        update()  # Llamada a la función de actualización de sensores
        obstaculo()
        deteccion_señal_verde()
        doble_negro()
        T_derecha()
        T_izquierda()        
        seguidor_de_linea()
        #print(f"izq,{light_l},derecho, {light_r}, frontal {light_f}")
        
# Ejecutar el bucle principal
main()

