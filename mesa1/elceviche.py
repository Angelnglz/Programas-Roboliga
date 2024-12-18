from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

#puertos que se usan
#MOTORES IZQ (D), DER(B)
#SENSORES LUZ IZQ (C), DER(A), FRENTE (E), ULTRASONICO (F)

hub = PrimeHub()
#definir base de robot 


hub.speaker.volume(100)
#Calibracion de los sensores
#Valor sensor izquierdo centro:16
#Valor sensor izquierdo borde:48
#Valor sensor derecho centro:15
#Valor sensor derecho borde:45
#Valor sensor frontal centro:7
#Valor sensor fronta borde:27

#umbrales de negro
centro_negro = 15
cenntro_negro_frente= 7 #15
borde_negro = 27 #35
"""
negro group atomo:
centro_negro = 22
cenntro_negro_frente= 4
borde_negro = 35
"""

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

base_robot = DriveBase(motor_izquierdo, motor_derecho, 56, 140)
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

def recorrer_proporcional(angulo, velocidad):
    lectura_actual = hub.imu.heading()
    kp = 5
    error = angulo - lectura_actual
    correccion_proporcional = error * kp

    base_robot.drive(velocidad, correccion_proporcional)

#funcion para avanzar distancia con correccion proporcional
def recorrer_distancia(cantidad_mm, angulo, velocidad):
    base_robot.reset()
    while True:
        recorrer_proporcional(angulo, velocidad)
        if abs(base_robot.distance()) >= abs(cantidad_mm) : 
            base_robot.brake()
            break

#funcion para parar motores de robot_base
def parar_motores_alt():
    base_robot.brake()

#funcion para parar motores
def parar_motores():
    motor_izquierdo.stop()
    motor_derecho.stop()


#lista de colores a usar, calibrar en cualquier pista
Color.BLACK = Color(h=180, s=20, v=27)
Color.GREEN = Color(h=166, s=68, v=34)
Color.SILVER = Color (h=0, s=0, v=99)

Color.SILVERFRONTAL = Color(h=0, s=0 , v=99)
Color.BLACKFRONTAL = Color(h=180, s=22, v=5)
Color.GREENFRONTAL = Color(h=165, s=68, v=19)
Color.WHITEFRONTAL = Color(h=202, s=19, v=79)

colores = (Color.BLACK, Color.GREEN,Color.NONE,Color.WHITE,Color.SILVER)
coloresf = (Color.BLACKFRONTAL, Color.GREENFRONTAL,Color.NONE,Color.WHITEFRONTAL, Color.SILVERFRONTAL)
color_sensor_l.detectable_colors(colores)
color_sensor_r.detectable_colors(colores)
color_sensor_f.detectable_colors(coloresf)

# Función de actualización de los sensores
def update():
    global color_l, color_r, color_f, light_l, light_f, light_r, distance
    
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
velocidad=38
#constante de proporcion
kp=4.2
#constante integral
ki=0.001   #0.001
#constante derivativa
kd=1.201  #1.201

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
    if tiempo_actual > 6500:
        parar_motores()
        while True:
            mover_motores_indefinido(-100,-100)
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
    while hub.imu.heading() < 65:
        print(hub.imu.heading())
        mover_motores_indefinido(100,-100)
    parar_motores()
        

def giro_90_grados_izquierda():
    hub.imu.reset_heading(0)
    while hub.imu.heading() >-65:
        print(hub.imu.heading())
        motor_izquierdo.run(-300)
        motor_derecho.run(300)
    parar_motores()

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
        if -1<distance_sensor.distance()<60:
            #hub.display.text("obs")
            movermotores(-100,-100,0.1)
            hub.imu.reset_heading(0)
            movermotores(-100,100,0.9)
            while hub.imu.heading()>-50: #revisar for star
                mover_motores_indefinido(-85,45)
            parar_motores()
            movermotores(100,100,0.3)
            while color_sensor_l.reflection() > centro_negro:
                mover_motores_indefinido(250,100)
            parar_motores()
            movermotores(100,100,0.2)
            while color_sensor_r.reflection() > centro_negro:
                mover_motores_indefinido(-100,100)
            parar_motores()


def buscar_linea_derecha():
    parar_motores()
    #hub.display.text("BLD")
    hub.imu.reset_heading(0)
    while hub.imu.heading() < 45:
        mover_motores_indefinido(100,-100)
    parar_motores()

    while color_sensor_r.reflection() > (centro_negro+5):
        mover_motores_indefinido(-100,100)
    parar_motores()


def buscar_linea_izquierda():
    parar_motores()
    #hub.display.text("BLI")
    hub.imu.reset_heading(0)
    while hub.imu.heading() > -45:
        mover_motores_indefinido(-100,100)
    parar_motores()

    while color_sensor_l.reflection() > (centro_negro+5):
        mover_motores_indefinido(100,-100)
    parar_motores()

    """while color_sensor_r.reflection() > (centro_negro+5):
        mover_motores_indefinido(-100,100) #giro a la derecha para acomodarme a la linea con el sensor derecho
    parar_motores()

    while color_sensor_f.reflection() > cenntro_negro_frente:
        mover_motores_indefinido(60,-60) #giro a la izquierda para acomodarme a l alinea con el sensor del frente
    parar_motores()"""

def doble_negro():
    if light_l < centro_negro+1 and light_r < centro_negro+1 and light_f < centro_negro+1:
        parar_motores()
        movermotores(-70,-70,0.05)
        movermotores(70,70,0.07)
        if light_l < centro_negro+1 and light_r < centro_negro+1 and light_f < centro_negro+1:
            hub.display.text("DN")
            movermotores(70,70,0.5)


def deteccion_señal_verde():
    if color_r == Color.GREEN:
        motor_izquierdo.stop()
        motor_derecho.stop()
        movermotores(100,100,0.03)
        wait(10)
        if color_sensor_l.color() == Color.GREEN:
            hub.display.text("DV")
            giro_180_grados_derecha()
            movermotores(100,100,0.2)
            

        else:
            hub.display.text("VD")
            movermotores(80,80,0.15)
            giro_90_grados_derecha()
            movermotores(80,80,0.10)
            buscar_linea_derecha()

    if color_l == Color.GREEN:
        motor_izquierdo.stop()
        motor_derecho.stop()
        movermotores(100,100,0.03)
        wait(10)
        if color_sensor_r.color() == Color.GREEN:
            hub.display.text("DV")
            giro_180_grados_derecha()
            movermotores(100,100,0.2)
            wait(10)
        
        elif color_l==  Color.GREEN:
            hub.display.text("VI")
            movermotores(80,80,0.15)
            giro_90_grados_izquierda()
            wait(10)
            movermotores(100,100,0.10)
            wait(10)
            buscar_linea_izquierda()
        

def T_derecha():
    if light_r < negro_derecha+5 and light_f < negro_frontal+5:
        parar_motores()
        movermotores(100,100,0.03)
        if color_sensor_r.reflection() < negro_derecha+5 and color_sensor_f.reflection() < negro_frontal+5:
            #hub.display.text("TD")
            movermotores(70,70,0.12)


def T_izquierda():
        if light_l < negro_izquierda+5 and light_f < negro_frontal+5:
            parar_motores()
            movermotores(100,100,0.03)
        if color_sensor_l.reflection() < negro_izquierda+5 and color_sensor_f.reflection() < negro_frontal+5:
            #hub.display.text("TI")
            movermotores(70,70,0.12)

"""
def buscar_pared():
    # Giro a la derecha 90 
    while hub.imu.heading() < 85:
        mover_motores_indefinido(300,-300)
    parar_motores()
    
    if distance_sensor.distance() < 500:  # Si encontré una pared a menos de 50 cm
        print(distance_sensor.distance())

        while distance_sensor.distance()>52:
            mover_motores_indefinido(150,150)
        parar_motores()
        
        # Vuelvo a la posición de entrada para girar hacia la izquierda
        while hub.imu.heading() > 2:
            mover_motores_indefinido(-300, 300)
        parar_motores()
        
        return "izquierda"  

    else:  # Si no encontré pared
        print(distance_sensor.distance())
        
        # Giro hacia la izquierda
        while hub.imu.heading() > -85:
            mover_motores_indefinido(-300, 300)
        parar_motores()

        while distance_sensor.distance()>52:
            mover_motores_indefinido(150,150)
        parar_motores()
        
        # Vuelvo a la posición de entrada para girar hacia la izquierda
        while hub.imu.heading() < 2:
            mover_motores_indefinido(300, -300)
        parar_motores()
        
        return "derecha"  # Retorno hacia la derecha

def rescate():
    esquina=0 #variable de control para la zona de rescate
    parar_motores()
    hub.display.text("R")
    hub.imu.reset_heading(0) #reinicio el giroscopio
    movermotores(200,200,0.8)
    orientacion=buscar_pared() #busco hacia que lado tengo la pared mas cercana para definir el recorrido
    print(orientacion)
    if orientacion == "izquierda":
        while True:
            while distance_sensor.distance()>59: #avance 
                recorrer_proporcional(0,300)
            parar_motores()
            if (color_sensor_l.color() == Color.BLACK or #si encontré negro en la primera pared salgo del bucle
                color_sensor_r.color() == Color.BLACK or 
                color_sensor_f.color() == Color.BLACK):
                esquina=1 #si la esquina es igual a 1 me preparo para hacer recorrido Largo-Corto y vuelvo en diagonal
                break

            while hub.imu.heading() > -88: #giro para la izquierda
                mover_motores_indefinido(-300, 300)
            parar_motores()
            wait(10)
            hub.imu.reset_heading(0) #reinicio el giroscopio
            recorrer_distancia(850,0,300) #hacer el recorrido largo (120 cm)

            if (color_sensor_l.color() == Color.BLACK or #si encontré negro en la segunda pared salgo del bucle
                color_sensor_r.color() == Color.BLACK or 
                color_sensor_f.color() == Color.BLACK):
                esquina=2 # si la esquina es igual 2 me preparo para hacer el recorrido Corto-Largo y vuelvo en diagonal
                break

            hub.imu.reset_heading(0)    
            while hub.imu.heading() > 85:
                mover_motores_indefinido(-300,300)
            parar_motores()
            hub.imu.reset_heading(0)
            while distance_sensor.distance()>60:
                recorrer_proporcional(0,300) #avance 
            parar_motores()
            esquina=3 #si la esquina es igual a 3 me preparo para hacer el recorrido Largo-Corto y vuelvo en diagonal
            break

        #aqui hago el programa para acomodarme para recorrer dos paredes mas
        #retrocede
        reiniciar_giroscopio(0)
        while distance_sensor.distance() < 128:
            recorrer_proporcional(0,-300)
        parar_motores()

        while hub.imu.heading() > -44:
            mover_motores_indefinido(-200,200)
        parar_motores()
        movermotores(300,300,1.4)#avanza para acercarme a la tercera pared

        while hub.imu.heading() > -89:
            mover_motores_indefinido(-200,200)
        parar_motores()


         #aqui pregunto en que esquina encontre el color negro y defino los avances para hacer dos paredes mas
        if esquina == 1 or esquina == 3: #hago reccorido largo-corto
            while distance_sensor.distance()>60:
                recorrer_proporcional(-90,300)
            parar_motores()

            while hub.imu.heading() > -178:#girar hasta la cuarta pared
                mover_motores_indefinido(-300,300)
            parar_motores()

            wait(100)
            reiniciar_giroscopio(0)
            while distance_sensor.distance()>60:
                recorrer_proporcional(0,300) #avance 
            parar_motores()

            while hub.imu.heading() > -125:#giro en diagonal
                mover_motores_indefinido(-300,300)
            parar_motores()    
        
            while distance_sensor.distance()>65:
                recorrer_proporcional(-125,300) #avance 
            parar_motores()

                
        if esquina == 2: #hago recorrido corto-largo
            while distance_sensor.distance()>60:
                recorrer_proporcional(-90,300)
            parar_motores()

            while hub.imu.heading() > -178:#girar hasta la cuarta pared
                mover_motores_indefinido(-300,300)
            parar_motores()

            wait(100)
            reiniciar_giroscopio(0)
            while distance_sensor.distance()>60:
                recorrer_proporcional(0,300) #avance 
            parar_motores()

            while hub.imu.heading() > -135:#giro en diagonal
                mover_motores_indefinido(-300,300)
            parar_motores()    
        
            while distance_sensor.distance()>65:
                recorrer_proporcional(-135,300) #avance 
            parar_motores()

    else: #aca debo hacer el recorrido girando derecha
        while True:
            while distance_sensor.distance()>59:
                recorrer_proporcional(0,300)
            parar_motores()
            if (color_sensor_l.color() == Color.BLACK or #si encontré negro en la primera pared salgo del bucle
                color_sensor_r.color() == Color.BLACK or 
                color_sensor_f.color() == Color.BLACK):
                esquina=1 #si la esquina es igual a 1 me preparo para hacer recorrido Largo-Corto y vuelvo en diagonal
                break

            while hub.imu.heading() < 88: #giro para la derecha y hago el recorrido largo segundo pared
                    mover_motores_indefinido(300,-300)
            parar_motores()
            wait(10)
            hub.imu.reset_heading(0) #reinicio el giroscopio
            recorrer_distancia(850,0,300) #hacer el recorrido largo (120 cm)

            if (color_sensor_l.color() == Color.BLACK or #si encontré negro en la segunda pared salgo del bucle
                color_sensor_r.color() == Color.BLACK or 
                color_sensor_f.color() == Color.BLACK):
                esquina=2 # si la esquina es igual 2 me preparo para hacer el recorrido Corto-Largo y vuelvo en diagonal
                break

            hub.imu.reset_heading(0)    
            while hub.imu.heading() < 85:
                    mover_motores_indefinido(300,-300)
            parar_motores()
            hub.imu.reset_heading(0)

            while distance_sensor.distance()>60:
                recorrer_proporcional(0,300) #avance 
            parar_motores()
            esquina=3 #si la esquina es igual a 3 me preparo para hacer el recorrido Largo-Corto y vuelvo en diagonal
            break

        #aqui hago el programa para acomodarme para recorrer dos paredes mas
        #retrocede
        reiniciar_giroscopio(0)
        while distance_sensor.distance() < 128:
            recorrer_proporcional(0,-300)
        parar_motores()

        while hub.imu.heading() < 44:
            mover_motores_indefinido(200,-200)
        parar_motores()
        movermotores(300,300,1.4)#avanza para acercarme a la tercera pared

        while hub.imu.heading() < 89:
            mover_motores_indefinido(200,-200)
        parar_motores()


         #aqui pregunto en que esquina encontre el color negro y defino los avances para hacer dos paredes mas
        if esquina == 1 or esquina == 3: #hago reccorido largo-corto
            while distance_sensor.distance()>60:
                recorrer_proporcional(90,300)
            parar_motores()
            while hub.imu.heading() < 179:#girar hasta la cuarta pared
                mover_motores_indefinido(300,-300)
            parar_motores()
            while distance_sensor.distance()>60:
                recorrer_proporcional(180,300) #avance 
            parar_motores()

            reiniciar_giroscopio(0)
            while hub.imu.heading() < 125:#giro en diagonal
                mover_motores_indefinido(300,-300)
            parar_motores()    
        
            while distance_sensor.distance()>65:
                recorrer_proporcional(125,300) #avance 
            parar_motores()
                
        if esquina == 2: #hago recorrido corto-largo
            while distance_sensor.distance()>60:
                recorrer_proporcional(90,300)
            parar_motores()
            while hub.imu.heading() < 179:#girar hasta la cuarta pared
                mover_motores_indefinido(300,-300)
            parar_motores()
            while distance_sensor.distance()>60:
                recorrer_proporcional(180,300) #avance 
            parar_motores()

            while hub.imu.heading() < 320:#giro en diagonal
                mover_motores_indefinido(300,-300)
            parar_motores()    
        
            while distance_sensor.distance()>65:
                recorrer_proporcional(320,300) #avance 
            parar_motores()


"""
""""""
def buscar_pared():

    # Giro a la derecha 90 
    while hub.imu.heading() < 85:
        mover_motores_indefinido(300,-300)
    parar_motores()
    
    if distance_sensor.distance() < 500:  # Si encontré una pared a menos de 50 cm
        print(distance_sensor.distance())

        while distance_sensor.distance()>100:
            mover_motores_indefinido(150,150)
        parar_motores()
        
        # Vuelvo a la posición de entrada para girar hacia la izquierda
        while hub.imu.heading() > 2:
            mover_motores_indefinido(-300, 300)
        parar_motores()
        
        return "izquierda"  

    else:  # Si no encontré pared
        print(distance_sensor.distance())
        
        # Giro hacia la izquierda
        while hub.imu.heading() > -85:
            mover_motores_indefinido(-300, 300)
        parar_motores()

        while distance_sensor.distance()>100:
            mover_motores_indefinido(150,150)
        parar_motores()
        
        # Vuelvo a la posición de entrada para girar hacia la izquierda
        while hub.imu.heading() < 2:
            mover_motores_indefinido(300, -300)
        parar_motores()
        
        return "derecha"  # Retorno hacia la derecha

def rescate():
    esquina=0 #variable de control para la zona de rescate
    parar_motores()
    hub.display.text("r")
    hub.imu.reset_heading(0) #reinicio el giroscopio
    movermotores(200,200,0.8)
    orientacion=buscar_pared() #busco hacia que lado tengo la pared mas cercana para definir el recorrido
    print(orientacion)
    if orientacion == "izquierda":
        while True:
            while distance_sensor.distance()>100: #avance 
                recorrer_proporcional(0,300)
            parar_motores()
            if (color_sensor_l.color() == Color.BLACK or #si encontré negro en la primera pared salgo del bucle
                color_sensor_r.color() == Color.BLACK or 
                color_sensor_f.color() == Color.BLACK):
                esquina=1 #si la esquina es igual a 1 me preparo para hacer recorrido Largo-Corto y vuelvo en diagonal
                break

            while hub.imu.heading() > -88: #giro para la izquierda
                mover_motores_indefinido(-300, 300)
            parar_motores()
            wait(10)
            hub.imu.reset_heading(0) #reinicio el giroscopio
            recorrer_distancia(850,0,300) #hacer el recorrido largo (120 cm)

            if (color_sensor_l.color() == Color.BLACK or #si encontré negro en la segunda pared salgo del bucle
                color_sensor_r.color() == Color.BLACK or 
                color_sensor_f.color() == Color.BLACK):
                esquina=2 # si la esquina es igual 2 me preparo para hacer el recorrido Corto-Largo y vuelvo en diagonal
                break

            hub.imu.reset_heading(0)    
            while hub.imu.heading() > 85:
                mover_motores_indefinido(-300,300)
            parar_motores()
            hub.imu.reset_heading(0)
            while distance_sensor.distance()>70:
                recorrer_proporcional(0,300) #avance 
            parar_motores()
            esquina=3 #si la esquina es igual a 3 me preparo para hacer el recorrido Largo-Corto y vuelvo en diagonal
            break

        #aqui hago el programa para acomodarme para recorrer dos paredes mas
        #retrocede
        reiniciar_giroscopio(0)
        while distance_sensor.distance() < 128:
            recorrer_proporcional(0,-300)
        parar_motores()

        while hub.imu.heading() > -44:
            mover_motores_indefinido(-200,200)
        parar_motores()
        movermotores(300,300,1.4)#avanza para acercarme a la tercera pared

        while hub.imu.heading() > -89:
            mover_motores_indefinido(-200,200)
        parar_motores()


         #aqui pregunto en que esquina encontre el color negro y defino los avances para hacer dos paredes mas
        if esquina == 1 or esquina == 3: #hago reccorido largo-corto
            while distance_sensor.distance()>60:
                recorrer_proporcional(-90,300)
            parar_motores()

            while hub.imu.heading() > -178:#girar hasta la cuarta pared
                mover_motores_indefinido(-300,300)
            parar_motores()

            wait(100)
            reiniciar_giroscopio(0)
            while distance_sensor.distance()>70:
                recorrer_proporcional(0,300) #avance 
            parar_motores()

            while hub.imu.heading() > -125:#giro en diagonal
                mover_motores_indefinido(-300,300)
            parar_motores()    
        
            while distance_sensor.distance()>65:
                recorrer_proporcional(-125,300) #avance 
            parar_motores()

                
        if esquina == 2: #hago recorrido corto-largo
            while distance_sensor.distance()>70:
                recorrer_proporcional(-90,300)
            parar_motores()

            while hub.imu.heading() > -178:#girar hasta la cuarta pared
                mover_motores_indefinido(-300,300)
            parar_motores()

            wait(100)
            reiniciar_giroscopio(0)
            while distance_sensor.distance()>70:
                recorrer_proporcional(0,300) #avance 
            parar_motores()

            while hub.imu.heading() > -135:#giro en diagonal
                mover_motores_indefinido(-300,300)
            parar_motores()    
        
            while distance_sensor.distance()>75:
                recorrer_proporcional(-135,300) #avance 
            parar_motores()

    else: #aca debo hacer el recorrido girando derecha
        while True:
            while distance_sensor.distance()>100:
                recorrer_proporcional(0,300)
            parar_motores()
            if (color_sensor_l.color() == Color.BLACK or #si encontré negro en la primera pared salgo del bucle
                color_sensor_r.color() == Color.BLACK or 
                color_sensor_f.color() == Color.BLACK):
                esquina=1 #si la esquina es igual a 1 me preparo para hacer recorrido Largo-Corto y vuelvo en diagonal
                break

            while hub.imu.heading() < 88: #giro para la derecha y hago el recorrido largo segundo pared
                    mover_motores_indefinido(300,-300)
            parar_motores()
            wait(10)
            hub.imu.reset_heading(0) #reinicio el giroscopio
            recorrer_distancia(850,0,300) #hacer el recorrido largo (120 cm)

            if (color_sensor_l.color() == Color.BLACK or #si encontré negro en la segunda pared salgo del bucle
                color_sensor_r.color() == Color.BLACK or 
                color_sensor_f.color() == Color.BLACK):
                esquina=2 # si la esquina es igual 2 me preparo para hacer el recorrido Corto-Largo y vuelvo en diagonal
                break

            hub.imu.reset_heading(0)    
            while hub.imu.heading() < 85:
                    mover_motores_indefinido(300,-300)
            parar_motores()
            hub.imu.reset_heading(0)

            while distance_sensor.distance()>60:
                recorrer_proporcional(0,300) #avance 
            parar_motores()
            esquina=3 #si la esquina es igual a 3 me preparo para hacer el recorrido Largo-Corto y vuelvo en diagonal
            break

        #aqui hago el programa para acomodarme para recorrer dos paredes mas
        #retrocede
        reiniciar_giroscopio(0)
        while distance_sensor.distance() < 128:
            recorrer_proporcional(0,-300)
        parar_motores()

        while hub.imu.heading() < 44:
            mover_motores_indefinido(200,-200)
        parar_motores()
        movermotores(300,300,1.4)#avanza para acercarme a la tercera pared

        while hub.imu.heading() < 89:
            mover_motores_indefinido(200,-200)
        parar_motores()


         #aqui pregunto en que esquina encontre el color negro y defino los avances para hacer dos paredes mas
        if esquina == 1 or esquina == 3: #hago reccorido largo-corto
            while distance_sensor.distance()>60:
                recorrer_proporcional(90,300)
            parar_motores()
            while hub.imu.heading() < 179:#girar hasta la cuarta pared
                mover_motores_indefinido(300,-300)
            parar_motores()
            while distance_sensor.distance()>60:
                recorrer_proporcional(180,300) #avance 
            parar_motores()

            reiniciar_giroscopio(0)
            while hub.imu.heading() < 125:#giro en diagonal
                mover_motores_indefinido(300,-300)
            parar_motores()    
        
            while distance_sensor.distance()>65:
                recorrer_proporcional(125,300) #avance 
            parar_motores()
                
        if esquina == 2: #hago recorrido corto-largo
            while distance_sensor.distance()>60:
                recorrer_proporcional(90,300)
            parar_motores()
            while hub.imu.heading() < 179:#girar hasta la cuarta pared
                mover_motores_indefinido(300,-300)
            parar_motores()
            while distance_sensor.distance()>60:
                recorrer_proporcional(180,300) #avance 
            parar_motores()

            while hub.imu.heading() < 320:#giro en diagonal
                mover_motores_indefinido(300,-300)
            parar_motores()    
        
            while distance_sensor.distance()>65:
                recorrer_proporcional(320,300) #avance 
            parar_motores()

        
def reiniciar_giroscopio(angulo):
    hub.imu.reset_heading(angulo)
# Bucle principal
modo = None

# Bucle principal
def main():
    global modo
    while True:
        update()  # Llamada a la función de actualización de sensores
        
        if color_l == Color.SILVER and color_r == Color.SILVER and color_f == Color.SILVERFRONTAL: # si detecto el valor del color plateado con los 3 sensores paro todo
            parar_motores()
            modo = "rescate" #asigno a la variable modo el texto "rescate"

        obstaculo() #Llamada a la funcion de obstaculo

        deteccion_señal_verde() #Llamada a la funcion de señales verdes

        doble_negro() #Llamada a la funcion de doble negro

        T_derecha() #Llamada a la funcion T derecha

        T_izquierda() #Llamada a la funcion T izquierda  

        if modo == "rescate": #si la variable modo es igual a rescate llamo a la funcion
            rescate()
            break # Llamada a la funcion rescate

        seguidor_de_linea() # Lamada a la funcion seguidor de linea
# Ejecutar el bucle principal
main()
"""while True:

    update()
    print(f"izq,{light_l},derecho, {light_r}, frontal {light_f}")
"""
"""reiniciar_giroscopio(0)

while distance_sensor.distance() > 70:
    recorrer_proporcional(0,300)
parar_motores()"""
