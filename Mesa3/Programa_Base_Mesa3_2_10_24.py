from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

#puertos que se usan
#MOTORES IZQ (D), DER(B)
#SENSORES LUZ IZQ (C), DER(A), FRENTE (E), ULTRASONICO (F)

#inicializo el HUB y le digo cuales van a ser los canales de transmision y recepcion
hub = PrimeHub(broadcast_channel=4, observe_channels=[5])

hub.speaker.volume(100)

#Calibracion de los sensores
#Valor sensor izquierdo centro:22
#Valor sensor izquierdo borde:47
#Valor sensor derecho centro:22
#Valor sensor derecho borde:47
#Valor sensor frontal centro:24
#Valor sensor fronta borde:47

#umbrales de negro
centro_negro = 25
cenntro_negro_frente= 15
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

#funcion para obtener los valores HSV, COLOR y LUZ de cualquiera de los 3 sensores para calibrarlos
def valores_HSV_color(sensor):
    # Obtener los valores de los sensores
    if sensor == 'derecho':
        hsv = color_sensor_r.hsv()
        color = color_sensor_r.color()
        luz = color_sensor_r.reflection()
        print(f"Derecho - HSV: {hsv}, Color: {color}, Luz: {luz}")

    elif sensor == 'izquierdo':
        hsv = color_sensor_l.hsv()
        color = color_sensor_l.color()
        luz = color_sensor_l.reflection()
        print(f"Izquierdo - HSV: {hsv}, Color: {color}, Luz: {luz}")

    elif sensor == 'frontal':
        hsv = color_sensor_f.hsv()
        color = color_sensor_f.color()
        luz = color_sensor_f.reflection()
        print(f"Frontal - HSV: {hsv}, Color: {color}, Luz: {luz}")

#Aqui se calibran los colores a utilizar en la pista en modo HSV con el sensor IZQUIERDO/DERECHO
Color.BLACK = Color(h=180, s=20, v=27)
Color.GREEN = Color(h=166, s=68, v=34)
Color.WHITE = Color(h=204, s=17, v=93)
Color.SILVER = Color(h=0, s=0, v=99)

#Aqui se calibran los colores a utilizar en la pista en modo HSV con el sensor FRONTAL
Color.WHITEFRONTAL = Color(h=207, s=22, v=51)
Color.SILVERFRONTAL = Color(h=232, s=11, v=97)

#lista de colores a utilizar con el sensor, si no se utiliza algun color creo que es mejor no ponerlo y se pueden agregar o inventar los colores que se deseen
colores = (Color.BLACK, Color.GREEN, Color.NONE, Color.WHITEFRONTAL, Color.WHITE, Color.SILVER, Color.SILVERFRONTAL)

#aqui le digo a los sensores que solo van a trabajar con la lista de colores que se creo anteriormente
color_sensor_l.detectable_colors(colores)
color_sensor_r.detectable_colors(colores)
color_sensor_f.detectable_colors(colores)


# Función de actualización de los valores que miden los sensores
def update():
    global color_l, color_r, color_f, light_l, light_r, light_f, distance #variables para los sensores
    
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
velocidad=35
#constante de proporcion
kp=4
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

    if motor_derecho.stalled() and motor_izquierdo.stalled():
        movermotores(300,300,0.1)

    # Comprobar si algún sensor detecta la línea negra
    if light_r < borde_negro or light_l < borde_negro:#  or light_f < borde_negro:
        ultima_deteccion.reset()


    # Si han pasado más de 6 segundos sin detectar la línea, retroceder
    if tiempo_actual > 6000:
        parar_motores()
        while True:
            mover_motores_indefinido(-150,-150)
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
        motor_izquierdo.run(300)
        motor_derecho.run(-300)
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
        motor_izquierdo.run(150)
        motor_derecho.run(-150)
    parar_motores()

def obstaculo():
    #si detecte algo paro todo
    if -1<distance<48: 
        parar_motores()
        hub.speaker.beep(700,30)

        if -1<distance_sensor.distance()<50: #confirmo si sigo detectando 
            hub.display.text("obs")
            movermotores(-100,-100,0.08) #retrocedo un poco para alejarme 

            hub.imu.reset_heading(0)
            while hub.imu.heading()>-60:
                mover_motores_indefinido(-100,100) #giro hacia la izquierda
            parar_motores()

            movermotores(100,100,0.1) #avanzo un poco para salir de la linea

            while color_sensor_l.reflection() > centro_negro:
                mover_motores_indefinido(425,100) #giro con direccion derecha hasta encontrar el centro de la linea con el sensor izquierdo
            parar_motores()


            movermotores(100,100,0.2) # avanzo un poco para salir de la linea

            while color_sensor_f.reflection() > cenntro_negro_frente:
                mover_motores_indefinido(-100,100) # giro hacia hata encontrar el centro de la linea con el sensor del frente para alinearme a la linea
            parar_motores()

            """while color_sensor_r.reflection() > centro_negro:
                mover_motores_indefinido(-100,100) # giro hacia la izquierda hasta encontrar el centro de la linea con el sensor derecho
            parar_motores()

            while color_sensor_l.reflection() > centro_negro:
                mover_motores_indefinido(100,-100) #giro hacia la derecha hasta encontrar el centro de la linea con el sensor izquierdo
            parar_motores()"""

def buscar_linea_derecha():
    parar_motores()
    hub.display.text("BLD")
    hub.imu.reset_heading(0) # reseteo el giroscopio al valor 0
    while hub.imu.heading() < 45:   
        mover_motores_indefinido(100,-100) #giro para la derecha mientras el valor sea menor a 45
    parar_motores()


    while color_sensor_r.reflection() > (centro_negro+5): #mientras el sensor derecho no sea el valor de la linea
        mover_motores_indefinido(-100,100) #giro para la derecha para acomodarme a la linea con el sensor derecho
    parar_motores()

    while color_sensor_l.reflection() > (centro_negro+5):
        mover_motores_indefinido(100,-100) #giro para la izquierda para acomodarme a la linea con el sensor izquierdo
    parar_motores()

    """while color_sensor_f.reflection() > cenntro_negro_frente:
        mover_motores_indefinido(-100,100) #giro para la derecha para acomodarme a la linea con el sensor del frente
    parar_motores()"""

def buscar_linea_izquierda():
    parar_motores()
    hub.display.text("BLI")
    hub.imu.reset_heading(0)
    while hub.imu.heading() > -45:
        mover_motores_indefinido(-100,100)
    parar_motores()

    while color_sensor_l.reflection() > (centro_negro+5)  :
        mover_motores_indefinido(100,-100) #giro a la izquierda para acomodarme a la linea con el sensor izquierdo
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
        if color_sensor_l.reflection() < centro_negro+1 and color_sensor_r.reflection() < centro_negro+1 and color_sensor_f.reflection() < centro_negro+1:
            hub.display.text("DN")
            movermotores(70,70,0.1)

def deteccion_señal_verde():
    #Si detecto un posible verde a la derecha paro todo
    if color_r == Color.GREEN:
        parar_motores()
        movermotores(100,105,0.02) #muevo un poco los motores y hago una verificacion
        #wait(10) #una pequeña espera para estar seguro que el robot esta completamente quieto

        #verifico si es verde
        if color_sensor_r.color() == Color.GREEN: 
            hub.speaker.beep(500,50)
            hub.display.text("CVD")
            movermotores(-100,-105,0.02) #vuelvo a la posicion inicial si confirme un verde
            
            if color_sensor_l.color() == Color.GREEN: # aqui verifico si el sensor contrario esta tambien en un verde
                hub.display.text("DV")
                giro_180_grados_derecha()
                movermotores(100,100,0.2)
                buscar_linea_derecha()
            

            else:
                # aqui es cuando detecte solo verde derecho
                hub.display.text("VD")
                movermotores(80,80,0.25)
                giro_90_grados_derecha()
                movermotores(80,80,0.35)
                buscar_linea_derecha()

    #Si detecto un posible verde a la izquierda paro todo
    if color_l == Color.GREEN:
        parar_motores()
        movermotores(105,100,0.02) #muevo un poco los motores y hago una verificacion
        #wait(10) #una pequeña espera para estar seguro que el robot esta completamente quieto

        #verifico si es verde
        if color_sensor_l.color()==Color.GREEN:
            hub.speaker.beep(500,50)
            hub.display.text("CVI")
            movermotores(-105,100,0.02) #vuelvo a la posicion inicial si confirme un verde
            

            if color_sensor_r.color() == Color.GREEN: # aqui verifico si el sensor contrario esta tambien en un verde
                hub.display.text("DV")
                giro_180_grados_derecha()
                movermotores(100,100,0.35)
                buscar_linea_derecha()
        
            else:
                # aqui es cuando detecte solo verde izquierdo
                hub.display.text("VI")
                movermotores(80,80,0.25)
                giro_90_grados_izquierda()
                movermotores(80,80,0.35)
                buscar_linea_izquierda()
              
def T_derecha():
    if light_r < negro_derecha+5 and light_f < negro_frontal+5: #pregunto si el sensor derecho y el frontal ven un valor cercano al color negro
        parar_motores()
        movermotores(100,100,0.03) #me muevo un poquito

        if color_sensor_r.reflection() < negro_derecha+5 and color_sensor_f.reflection() < negro_frontal+5: #vuelvo a preguntar si sigo en la misma situacion, el sensor derecho y el frontal en un valor cercano al negro
            
            if color_sensor_f.color() == Color.GREEN or color_sensor_r.color() == Color.GREEN: #aqui pregunto en modo color si alguno de los dos sensores detecto color verde ya que los valores de luz reflejada de verde y negro me dan un valor parecido algunas veces
                hub.speaker.beep(500,50)
                hub.display.text("X")
                movermotores(-100,-100,0.1) #si alguno fue verde retrocedo

            else: # si no, confirmo que es una T de lado izquierdo y avanzo
                hub.display.text("TD")
                movermotores(70,70,0.12)

def T_izquierda():
        if light_l < negro_izquierda+5 and light_f < negro_frontal+5: #pregunto si el sensor izquierdo y el frontal ven un valor cercano al color negro
            parar_motores()
            movermotores(100,100,0.03) #me muevo un poquito

        if color_sensor_l.reflection() < negro_izquierda+5 and color_sensor_f.reflection() < negro_frontal+5: #vuelvo a preguntar si sigo en la misma situacion, el sensor izquierdo y el frontal en un valor cercano al negro
            
            if color_sensor_l.color() == Color.GREEN or color_sensor_f.color() == Color.GREEN: #aqui pregunto en modo color si alguno de los dos sensores detecto color verde ya que los valores de luz reflejada de verde y negro me dan un valor parecido algunas veces
                hub.speaker.beep(500,50)
                hub.display.text("X")
                movermotores(-100,-100,0.1) #si alguno fue verde retrocedo
            else: # si no, confirmo que es una T de lado izquierdo y avanzo
                hub.display.text("TI")
                movermotores(70,70,0.12)

#funciones para la parte de rescate
def buscar_pared():
    # Giro a la derecha 90 grados
    while hub.imu.heading() < 85:
        mover_motores_indefinido(100, -100)
    parar_motores()
    
    if distance_sensor.distance() < 500:  # Si encontré una pared a menos de 50 cm
        print(distance_sensor.distance())
        
        # Vuelvo a la posición de entrada para girar hacia la izquierda
        while hub.imu.heading() > 1:
            mover_motores_indefinido(-100, 100)
        parar_motores()
        
        return "izquierda"  

    else:  # Si no encontré pared
        print(distance_sensor.distance())
        
        # Giro hacia la izquierda
        while hub.imu.heading() > -85:
            mover_motores_indefinido(-100, 100)
        parar_motores()
        
        return "derecha"  # Retorno hacia la derecha

def avanzar_hasta_tocar_pared():
    while True:
        mover_motores_indefinido(100,100) #muevo los motores hacia adelante
        """velocidad_motor_derecho=motor_derecho.speed()
        velocidad_motor_izquierdo=motor_izquierdo.speed()
        print(velocidad_motor_izquierdo,velocidad_motor_derecho)"""
        if motor_derecho.stalled() or motor_izquierdo.stalled(): #si alguno de los dos motores se detuvieron 
            parar_motores() #paro los motores
            break

def encontre_zona_de_evacuacion(): #si los sensores de color encuentran negro la funcion se vuelve verdadera, de lo contrario se vuelve falsa
    return color_sensor_l.color() == Color.BLACK or color_sensor_r.color() == Color.BLACK or color_sensor_f.color() == Color.BLACK

    

#Funciones para subir o bajar el motor
#S es para subir
#B es para bajar
def subir():
    mensaje = "S" #asigno a mensaje rescate el mensaje a enviar
    data_send = [mensaje] # lo almaceno en una lista
    hub.ble.broadcast(data_send) # comando para enviarlo 
    print(f"Enviado {data_send}")  #imprimo el mensaje que envie

def bajar():
    mensaje = "B" #asigno a mensaje rescate el mensaje a enviar
    data_send = [mensaje] # lo almaceno en una lista
    hub.ble.broadcast(data_send) # comando para enviarlo 
    print(f"Enviado {data_send}")  #imprimo el mensaje que envie

#funcion principal para realizar la zona de rescate
def rescate():
    #hub.display.text("rescate")
    hub.imu.reset_heading(0) #reinicio el giroscopio

    #movermotores(100,100,0.5) #avanzo un poco para entrar a la zona de rescate

    orientacion=buscar_pared() #busco hacia que lado tengo la pared mas cercana para definir el recorrido
    print(orientacion)
    
    if orientacion == "izquierda":#debo realizar el recorrido girando hacia la izquierda
        pass
    else:#debo hacer el recorrido girando hacia la derecha
        pass
        


    


#tiempo=StopWatch()
modo = None

# Bucle principal
def main():
    global modo
    while True:
        #tiempo.reset()
        #tiempo_inicial = tiempo.time()
        update()  # Llamada a la función de actualización de sensores
        
        if color_l == Color.SILVER and color_r == Color.SILVER and color_f == Color.SILVERFRONTAL or color_f == Color.SILVER: # si detecto el valor del color plateado con los 3 sensores paro todo
            parar_motores()
            modo = "rescate" #asigno a la variable modo el texto "rescate"

        obstaculo() #Llamada a la funcion de obstaculo

        deteccion_señal_verde() #Llamada a la funcion de señales verdes

        doble_negro() #Llamada a la funcion de doble negro

        T_derecha() #Llamada a la funcion T derecha

        T_izquierda() #Llamada a la funcion T izquierda  

        if modo == "rescate": #si la variable modo es igual a rescate llamo a la funcion
            parar_motores()
            rescate() # Llamada a la funcion rescate

        seguidor_de_linea() # Lamada a la funcion seguidor de linea
        #tiempo_final = tiempo.time()
        #print(tiempo_final)

# Ejecutar el bucle principal
main()
