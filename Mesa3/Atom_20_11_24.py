

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

def reiniciar_giroscopio(angulo):
    hub.imu.reset_heading(angulo)

hub.speaker.volume(100)

#Calibracion de los sensores
#Valor sensor izquierdo centro:22
#Valor sensor izquierdo borde:47
#Valor sensor derecho centro:22
#Valor sensor derecho borde:47
#Valor sensor frontal centro:24
#Valor sensor fronta borde:47

#umbrales de negro
centro_negro = 22
cenntro_negro_frente= 4
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

#definir robot base
base_robot = DriveBase(motor_izquierdo, motor_derecho, 56, 180)

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

def mover_motores_rescate(velocidad_motor_izq,rotacionizq,velocidad_motor_der,rotacionder):
    motor_izquierdo.run_angle(velocidad_motor_izq,rotacionizq*360,wait=False)
    motor_derecho.run_angle(velocidad_motor_der,rotacionder*360,wait=False)
    while not motor_izquierdo.done() or not motor_derecho.done():
        wait (10)
        if motor_derecho.stalled() or motor_izquierdo.stalled():
            hub.speaker.beep(500,60)
            parar_motores()
            break


#funcion para mover motores de forma indefinida
def mover_motores_indefinido(velocidad_motor_izquierdo,velocidad_motor_derecho):
    motor_izquierdo.run(velocidad_motor_izquierdo)
    motor_derecho.run(velocidad_motor_derecho)

def avance_con_correccion(angulo,distancia):
    #hub.imu.reset_heading(0)
    motor_derecho.reset_angle(0)
    while motor_derecho.angle()/360*17<distancia:
        if hub.imu.heading()==angulo:
            mover_motores_indefinido(500,500)
        if hub.imu.heading()>angulo:
            mover_motores_indefinido(250,500)
        if hub.imu.heading()<angulo:
            mover_motores_indefinido(500,250)

#funcion para correccion proporcional
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
    motor_derecho.stop()
    motor_izquierdo.stop()

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
"""Color.BLACK = Color(h=200, s=19, v=50)
Color.GREEN = Color(h=170, s=60, v=20)
Color.WHITE = Color(h=204, s=19, v=87)
Color.SILVER = Color(h=0, s=0, v=99)"""
#Aqui se calibran los colores a utilizar en la pista en modo HSV con el sensor IZQUIERDO/DERECHO
Color.BLACK = Color(h=180, s=20, v=27)
Color.GREEN = Color(h=166, s=68, v=34)
Color.WHITE = Color(h=204, s=17, v=93)
Color.SILVER = Color(h=0, s=0, v=99)
#Aqui se calibran los colores a utilizar en la pista en modo HSV con el sensor FRONTAL
Color.WHITEFRONTAL = Color(h=207, s=22, v=51)
Color.SILVERFRONTAL = Color(h=220, s=11, v=82)

#lista de colores a utilizar con el sensor, si no se utiliza algun color creo que es mejor no ponerlo y se pueden agregar o inventar los colores que se deseen
coloreslateral = (Color.BLACK, Color.GREEN, Color.NONE,  Color.WHITE, Color.SILVER )
coloresfrontal = (Color.BLACK,  Color.NONE, Color.WHITEFRONTAL,  Color.SILVERFRONTAL)

#aqui le digo a los sensores que solo van a trabajar con la lista de colores que se creo anteriormente
color_sensor_l.detectable_colors(coloreslateral)
color_sensor_r.detectable_colors(coloreslateral)
color_sensor_f.detectable_colors(coloresfrontal)


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
velocidad=100
#constante de proporcion
kp=12
#constante integral
ki=0.1
#constante derivativa
kd=3

error=0
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
    error = (light_r - light_l)

 

    # Comprobar si algún sensor detecta la línea negra
    if light_r < borde_negro or light_l < borde_negro:#  or light_f < borde_negro:
        ultima_deteccion.reset()


    # Si han pasado más de 2 segundos sin detectar la línea, retroceder
    if tiempo_actual > 2000:
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
        potencia_motor_derecho = velocidad + correccion
        potencia_motor_izquierdo = velocidad - correccion
        
        # Ajustar las velocidades de los motores 50
        motor_izquierdo.run(potencia_motor_izquierdo)
        motor_derecho.run(potencia_motor_derecho)
        #print ((potencia_motor_derecho), (potencia_motor_izquierdo))
        #print(f"light_r: {light_r}, light_l: {light_l}, error: {error}, correccion: {correccion}, potder{potencia_motor_derecho}, potizq {potencia_motor_izquierdo}")


        # Aplicar las potencias a los motores
        #motor_izquierdo.dc(potencia_motor_izquierdo)
        #motor_derecho.dc(potencia_motor_derecho)



        # Actualizar el error anterior
        error_anterior = error

def giro_90_grados_derecha():
    hub.imu.reset_heading(0)
    while hub.imu.heading() < 66:
        print(hub.imu.heading())
        motor_izquierdo.run(400)
        motor_derecho.run(-400)
    parar_motores()     

def giro_90_grados_izquierda():
    hub.imu.reset_heading(0)
    while hub.imu.heading() >-66:
        print(hub.imu.heading())
        motor_izquierdo.run(-400)
        motor_derecho.run(400)
    parar_motores()

def giro_180_grados_derecha():
    hub.imu.reset_heading(0)
    while hub.imu.heading() <177:
        print(hub.imu.heading())
        motor_izquierdo.run(450)
        motor_derecho.run(-450)
    parar_motores()

def obstaculo():
    #si detecte algo paro todo
    if -1<distance<48: 
        parar_motores()
        hub.speaker.beep(700,30)

        wait(500)
        if -1<distance_sensor.distance()<50: #confirmo si sigo detectando 
            hub.display.text("obs")
            movermotores(-100,-100,0.08) #retrocedo un poco para alejarme 

            hub.imu.reset_heading(0)
            while hub.imu.heading()>-80:
                mover_motores_indefinido(-100,100) #giro hacia la izquierda
            parar_motores()

            movermotores(100,100,0.1) #avanzo un poco para salir de la linea

            while color_sensor_r.reflection() > centro_negro:
                mover_motores_indefinido(450,100) #giro con direccion derecha hasta encontrar el centro de la linea con el sensor izquierdo
            parar_motores()


            movermotores(100,100,0.2) # avanzo un poco para salir de la linea

            """reiniciar_giroscopio(0)
            while hub.imu.heading()> -45:
                mover_motores_indefinido(-300,300)
            parar_motores()

            movermotores(-800,-800,0.5)"""

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
    while hub.imu.heading() < 20:   
        mover_motores_indefinido(300,-300) #giro para la derecha mientras el valor sea menor a 45
    parar_motores()


    while color_sensor_r.reflection() > (centro_negro+5): #mientras el sensor derecho no sea el valor de la linea
        mover_motores_indefinido(-300,300) #giro para la derecha para acomodarme a la linea con el sensor derecho
    parar_motores()

    while color_sensor_l.reflection() > (centro_negro+5):
        mover_motores_indefinido(300,-300) #giro para la izquierda para acomodarme a la linea con el sensor izquierdo
    parar_motores()

    """while color_sensor_f.reflection() > cenntro_negro_frente:
        mover_motores_indefinido(-100,100) #giro para la derecha para acomodarme a la linea con el sensor del frente
    parar_motores()"""

def buscar_linea_izquierda():
    parar_motores()
    hub.display.text("BLI")
    hub.imu.reset_heading(0)
    while hub.imu.heading() > -20:
        mover_motores_indefinido(-300,300)
    parar_motores()

    while color_sensor_l.reflection() > (centro_negro+5)  :
        mover_motores_indefinido(300,-300) #giro a la izquierda para acomodarme a la linea con el sensor izquierdo
    parar_motores()

    while color_sensor_r.reflection() > (centro_negro+5):
        mover_motores_indefinido(-300,300) #giro a la derecha para acomodarme a la linea con el sensor derecho
    parar_motores()

    """while color_sensor_f.reflection() > cenntro_negro_frente:
        mover_motores_indefinido(60,-60) #giro a la izquierda para acomodarme a l alinea con el sensor del frente
    parar_motores()"""

def doble_negro():
    if light_l < centro_negro+1 and light_r < centro_negro+1 and light_f < cenntro_negro_frente+1:
        parar_motores()
        movermotores(70,70,0.04)
        if color_sensor_l.reflection() < centro_negro+1 and color_sensor_r.reflection() < centro_negro+1 and color_sensor_f.reflection() < centro_negro+1:
            hub.display.text("DN")
            movermotores(70,70,0.3)

def deteccion_señal_verde():
    #Si detecto un posible verde a la derecha paro todo
    if color_r == Color.GREEN:
        parar_motores()
        movermotores(100,100,0.05) #muevo un poco los motores y hago una verificacion

        #verifico si es verde
        if color_sensor_r.color() == Color.GREEN: 
            hub.speaker.beep(500,50)
            hub.display.text("CVD")
            movermotores(-100,-100,0.05) #vuelvo a la posicion inicial si confirme un verde
            
            if color_sensor_l.color() == Color.GREEN: # aqui verifico si el sensor contrario esta tambien en un verde
                hub.display.text("DV")
                giro_180_grados_derecha()
                movermotores(100,100,0.35)
                buscar_linea_derecha()
            

            else:
                # aqui es cuando detecte solo verde derecho
                hub.display.text("VD")
                movermotores(80,80,0.2)
                giro_90_grados_derecha()
                movermotores(80,80,0.1)
                buscar_linea_derecha()

    #Si detecto un posible verde a la izquierda paro todo
    if color_l == Color.GREEN:
        parar_motores()
        movermotores(100,100,0.05) #muevo un poco los motores y hago una verificacion
        #wait(10) #una pequeña espera para estar seguro que el robot esta completamente quieto

        #verifico si es verde
        if color_sensor_l.color()==Color.GREEN:
            hub.speaker.beep(500,50)
            hub.display.text("CVI")
            movermotores(-100,100,0.05) #vuelvo a la posicion inicial si confirme un verde
            

            if color_sensor_r.color() == Color.GREEN: # aqui verifico si el sensor contrario esta tambien en un verde
                hub.display.text("DV")
                giro_180_grados_derecha()
                movermotores(100,100,0.35)
                buscar_linea_derecha()

            else:
                # aqui es cuando detecte solo verde izquierdo
                hub.display.text("VI")
                movermotores(80,80,0.2)
                giro_90_grados_izquierda()
                movermotores(80,80,0.1)
                buscar_linea_izquierda()
              
def T_derecha():
    if light_r < negro_derecha+5 and light_f < negro_frontal+5: #pregunto si el sensor derecho y el frontal ven un valor cercano al color negro
        parar_motores()
        movermotores(100,100,0.03) #me muevo un poquito

        if color_sensor_r.reflection() < negro_derecha+5 and color_sensor_f.reflection() < negro_frontal+5: #vuelvo a preguntar si sigo en la misma situacion, el sensor derecho y el frontal en un valor cercano al negro
            
            if color_sensor_f.color() == Color.GREEN or color_sensor_r.color() == Color.GREEN: #aqui pregunto en modo color si alguno de los dos sensores detecto color verde ya que los valores de luz reflejada de verde y negro me dan un valor parecido algunas veces
                hub.speaker.beep(500,50)
                hub.display.text("X")
                movermotores(-100,-100,0.2) #si alguno fue verde retrocedo

            else: # si no, confirmo que es una T de lado izquierdo y avanzo
                parar_motores()
                hub.display.text("TD")
                wait(1000)
                movermotores(70,70,0.3)

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
                parar_motores()
                hub.display.text("TI")
                wait(1000)
                movermotores(120,120,0.3)

#Funciones para subir o bajar el motor
#S es para subir
#B es para bajar
def buscar_pared():

    # Giro a la derecha 90 
    while hub.imu.heading() < 85:
        mover_motores_indefinido(300,-300)
    parar_motores()
    
    if distance_sensor.distance() < 500:  # Si encontré una pared a menos de 50 cm
        print(distance_sensor.distance())

        while distance_sensor.distance()>45:
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

        while distance_sensor.distance()>45:
            mover_motores_indefinido(150,150)
        parar_motores()
        
        # Vuelvo a la posición de entrada para girar hacia la izquierda
        while hub.imu.heading() < 2:
            mover_motores_indefinido(300, -300)
        parar_motores()
        
        return "derecha"  # Retorno hacia la derecha

def avanzar_hasta_tocar_pared_o_zona_rescate():
    while True:
        mover_motores_indefinido(100,100) #muevo los motores hacia adelante
        velocidad_motor_derecho=motor_derecho.speed()
        velocidad_motor_izquierdo=motor_izquierdo.speed()
        #print(velocidad_motor_izquierdo,velocidad_motor_derecho)
        if (motor_derecho.stalled() or
         motor_izquierdo.stalled() or 
         color_sensor_l.color() == Color.BLACK or 
         color_sensor_r.color() == Color.BLACK or 
         color_sensor_f.color() == Color.BLACK ): #si alguno de los dos motores se detuvieron 
            parar_motores() #paro los motores
            movermotores(-200,-200,1)
            break

def encontre_zona_de_evacuacion(): #si los sensores de color encuentran negro la funcion se vuelve verdadera, de lo contrario se vuelve falsa
    return color_sensor_l.color() == Color.BLACK or color_sensor_r.color() == Color.BLACK or color_sensor_f.color() == Color.BLACK
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

def rescate():
    esquina=0 #variable de control para la zona de rescate
    parar_motores()
    hub.display.text("r")
    #movermotores(300,-300,0.1) #correccion si la cinta esta arrugada y no detecta a la primera
    hub.imu.reset_heading(0) #reinicio el giroscopio
    recorrer_distancia(150,0,300)#avanza hacia la zona de rescate manteniendo el angulo 0 y avanza 15 cm

    orientacion=buscar_pared() #busco hacia que lado tengo la pared mas cercana para definir el recorrido
    print(orientacion)
    
    
    if orientacion == "izquierda":#debo realizar el recorrido girando hacia la izquierda
        wait(500)
        if distance_sensor.distance()  < 700:
            print(distance_sensor.distance())
            bajar()
            wait(1500)
            #recorrido corto-largo
            while True:
             
                recorrer_distancia(550,2,300) #hacer el recorrido corto (90cm)
                if (color_sensor_l.color() == Color.BLACK or #si encontré negro en la primera pared salgo del bucle
                color_sensor_r.color() == Color.BLACK or 
                color_sensor_f.color() == Color.BLACK):
                    esquina=1 #si la esquina es igual a 1 me preparo para hacer recorrido Largo-Corto y vuelvo en diagonal
                    break

                while hub.imu.heading() > -85: #giro para la izquierda y hago el recorrido largo segundo pared
                    mover_motores_indefinido(-300,300)
                parar_motores()
                hub.imu.reset_heading(0) #reinicio el giroscopio
                recorrer_distancia(910,0,300) #hacer el recorrido largo (120 cm)

                if (color_sensor_l.color() == Color.BLACK or #si encontré negro en la segunda pared salgo del bucle
                color_sensor_r.color() == Color.BLACK or 
                color_sensor_f.color() == Color.BLACK):
                    esquina=2 # si la esquina es igual 2 me preparo para hacer el recorrido Corto-Largo y vuelvo en diagonal
                    break

                hub.imu.reset_heading(0)    
                while hub.imu.heading() > -85:
                    mover_motores_indefinido(-300,300)
                parar_motores()
                hub.imu.reset_heading(0)
                recorrer_distancia(580,0,300)#hacer el recorrido corto (90cm)
                esquina=3 #si la esquina es igual a 3 me preparo para hacer el recorrido Largo-Corto y vuelvo en diagonal
                break

            #aqui hago el programa para dejar las pelotas y acomodarme para recorrer dos paredes mas       
            motor_derecho.run_angle(-900,330)
            movermotores(-900,-900,1)
            movermotores(300,300,1.5)#avanza para meter el corralito en el negro
            subir()
            movermotores(-600,-600,1)
            #retrocede
            hub.imu.reset_heading(0)#gira con un solo motor para acomodarse en diagonal
            while hub.imu.heading() > -88:
                motor_derecho.run(350)
            motor_derecho.stop()
            bajar()
            movermotores(300,300,0.6)#avanza para acercarme a la tercera pared

            hub.imu.reset_heading(0)#reinicio un girocospio
            while hub.imu.heading() > -45: #giro hasta alinearme con la tercera pared
                mover_motores_indefinido(-300,300)
            parar_motores()
            
            #aqui pregunto en que esquina encontre el color negro y defino los avances para hacer dos paredes mas
            if esquina == 1 or esquina == 3: #hago reccorido largo-corto
                hub.imu.reset_heading(0)#resetea giroscopo
                recorrer_distancia(750,0,300)#avanza 75cm largo
                while hub.imu.heading() > -87:#girar hasta la cuarta pared
                    mover_motores_indefinido(-300,300)
                parar_motores()
                hub.imu.reset_heading(0)
                recorrer_distancia(430,0,300)#avance 35cm corto
                while hub.imu.heading() > -120:#giro en diagonal
                    mover_motores_indefinido(-300,300)
                parar_motores()    
                recorrer_distancia(1050,-120,300)#avance hasta la zona de rescate
                subir()
                wait(4000)
                
                
            if esquina == 2: #hago recorrido corto-largo
                hub.imu.reset_heading(0)#resetea giroscopo
                recorrer_distancia(350,0,300)#avanza 45cm corto
                while hub.imu.heading() > -87:#girar hasta la cuarta pared
                    mover_motores_indefinido(-300,300)
                parar_motores()
                hub.imu.reset_heading(0)
                recorrer_distancia(790,0,300)#avance 80cm largo
                while hub.imu.heading() > -145:#giro en diagonal
                    mover_motores_indefinido(-300,300)
                parar_motores()    
                recorrer_distancia(1100,-145,300)#avance hasta la zona de rescate
                subir()
                wait(5000)

        else:
            #recorrido largo - corto
            print(distance_sensor.distance())
            bajar()

            while True:
                recorrer_distancia(830,2,300) #hacer el recorrido largo (120 cm)
                if (color_sensor_l.color() == Color.BLACK or #si encontré negro en la primera pared salgo del bucle
                color_sensor_r.color() == Color.BLACK or 
                color_sensor_f.color() == Color.BLACK):
                    esquina=1 #si la esquina es igual a 1 me preparo para hacer recorrido corto-largo y vuelvo en diagonal
                    break

                while hub.imu.heading() > -85: #giro para la izquierda y hago el recorrido largo segundo pared
                    mover_motores_indefinido(-300,300)
                parar_motores()
                hub.imu.reset_heading(0) #reinicio el giroscopio
                recorrer_distancia(630,0,300) #hacer el recorrido corto (90 cm)

                if (color_sensor_l.color() == Color.BLACK or #si encontré negro en la segunda pared salgo del bucle
                color_sensor_r.color() == Color.BLACK or 
                color_sensor_f.color() == Color.BLACK):
                    esquina=2 # si la esquina es igual 2 me preparo para hacer el recorrido largo-corto y vuelvo en diagonal
                    break

                hub.imu.reset_heading(0)    
                while hub.imu.heading() > -85:
                    mover_motores_indefinido(-300,300)
                parar_motores()
                hub.imu.reset_heading(0)
                recorrer_distancia(850,0,300)#hacer el recorrido largo (120cm)
                esquina=3 #si la esquina es igual a 3 me preparo para hacer el recorrido corto-largo y vuelvo en diagonal
                break

            #aqui hago el programa para dejar las pelotas y acomodarme para recorrer dos paredes mas
            wait(1000)       
            motor_derecho.run_angle(-900,330)
            movermotores(-900,-900,1)
            movermotores(300,300,1.2)#avanza para meter el corralito en el negro
            subir()
            wait(500)
            movermotores(-600,-600,1)#retrocede
            hub.imu.reset_heading(0)#gira con un solo motor para acomodarse en diagonal
            while hub.imu.heading() > -92:
                motor_derecho.run(400)
            motor_derecho.stop()
            bajar()

            movermotores(300,300,0.6)#avanza para acercarme a la tercera pared

            hub.imu.reset_heading(0)#reinicio un girocospio
            while hub.imu.heading() > -42: #giro hasta alinearme con la tercera pared
                mover_motores_indefinido(-300,300)
            parar_motores()
            
            #aqui pregunto en que esquina encontre el color negro y defino los avances para hacer dos paredes mas
            if esquina == 1 or esquina == 3: #hago reccorido corto-largo
                hub.imu.reset_heading(0)#resetea giroscopo
                recorrer_distancia(450,0,300)#avanza 45cm corto
                while hub.imu.heading() > -87:#girar hasta la cuarta pared
                    mover_motores_indefinido(-300,300)
                parar_motores()
                hub.imu.reset_heading(0)
                recorrer_distancia(720,0,300)#avance  largo
                while hub.imu.heading() > -138:#giro en diagonal
                    mover_motores_indefinido(-300,300)
                parar_motores()    
                recorrer_distancia(1100,-138,300)#avance hasta la zona de rescate
                subir()
                wait(4000)

            if esquina == 2: #hago recorrido largo-corto
                hub.imu.reset_heading(0)#resetea giroscopo
                recorrer_distancia(680,0,300)#avanza 45cm largo
                while hub.imu.heading() > -87:#girar hasta la cuarta pared
                    mover_motores_indefinido(-300,300)
                parar_motores()
                hub.imu.reset_heading(0)
                recorrer_distancia(480,0,300)#avance  corto
                while hub.imu.heading() > -120:#giro en diagonal
                    mover_motores_indefinido(-300,300)
                parar_motores()    
                recorrer_distancia(1100,-120,300)#avance hasta la zona de rescate
                subir()
                wait(4000)


    else: #debo realizar el recorrido girando hacia le derecha
        wait(500)
        if distance_sensor.distance()  < 700:
            wait(100)
            print(distance_sensor.distance())
            #recorrido corto-largo
            bajar()
            while True:
                
                recorrer_distancia(580,2,300) #hacer el recorrido corto (90cm)
                if (color_sensor_l.color() == Color.BLACK or #si encontré negro en la primera pared salgo del bucle
                color_sensor_r.color() == Color.BLACK or 
                color_sensor_f.color() == Color.BLACK):
                    esquina=1 #si la esquina es igual a 1 me preparo para hacer recorrido Largo-Corto y vuelvo en diagonal
                    break

                while hub.imu.heading() < 86: #giro para la derecha y hago el recorrido largo segundo pared
                    mover_motores_indefinido(300,-300)
                parar_motores()
                wait(10)
                hub.imu.reset_heading(0) #reinicio el giroscopio
                recorrer_distancia(9000,0,300) #hacer el recorrido largo (120 cm)

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
                recorrer_distancia(580,0,300)#hacer el recorrido corto (90cm)
                esquina=3 #si la esquina es igual a 3 me preparo para hacer el recorrido Largo-Corto y vuelvo en diagonal
                break

            #aqui hago el programa para dejar las pelotas y acomodarme para recorrer dos paredes mas
            wait(1000)       
            #mover_motores_rescate(-400,0.5,-800,1.3)#retrocede en curva
            motor_izquierdo.run_angle(-900,330)
            movermotores(-900,-900,1)
            movermotores(300,300,1.2)#avanza para meter el corralito en el negro
            subir()
            wait(1500)
            #retrocede
            movermotores(-600,-600,1)
            hub.imu.reset_heading(0)#gira con un solo motor para acomodarse en diagonal
            while hub.imu.heading() < 84:
                motor_izquierdo.run(350)
            motor_izquierdo.stop()
            movermotores(300,300,0.6)#avanza para acercarme a la tercera pared
            bajar()
            wait(1500)

            hub.imu.reset_heading(0)#reinicio un girocospio
            while hub.imu.heading() < 37: #giro hasta alinearme con la tercera pared
                mover_motores_indefinido(300,-300)
            parar_motores()
            
            #aqui pregunto en que esquina encontre el color negro y defino los avances para hacer dos paredes mas
            if esquina == 1 or esquina == 3: #hago reccorido largo-corto
                hub.imu.reset_heading(0)#resetea giroscopo
                recorrer_distancia(770,0,300)#avanza 75cm largo
                while hub.imu.heading() < 85:#girar hasta la cuarta pared
                    mover_motores_indefinido(300,-300)
                parar_motores()
                hub.imu.reset_heading(0)
                recorrer_distancia(480,0,300)#avance 35cm corto
                while hub.imu.heading() < 124:#giro en diagonal
                    mover_motores_indefinido(300,-300)
                parar_motores()    
                recorrer_distancia(1050,124,300)#avance hasta la zona de rescate

                
            if esquina == 2: #hago recorrido corto-largo
                hub.imu.reset_heading(0)#resetea giroscopo
                recorrer_distancia(300,0,300)#avanza 45cm corto
                while hub.imu.heading() < 89:#girar hasta la cuarta pared
                    mover_motores_indefinido(300,-300)
                parar_motores()
                hub.imu.reset_heading(0)
                recorrer_distancia(770,0,300)#avance 80cm largo
                while hub.imu.heading() < 144:#giro en diagonal
                    mover_motores_indefinido(300,-300)
                parar_motores()    
                recorrer_distancia(1000,144,300)#avance hasta la zona de rescate

        else:
            print(distance_sensor.distance())
            #recorrido largo - corto
        wait(500)
        if distance_sensor.distance()  < 700:
            wait(100)
            bajar()
            while True:
                
                recorrer_distancia(850,2,300) #hacer el recorrido largo (120)
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
                recorrer_distancia(580,0,300) #hacer el recorrido corto (90)

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
                recorrer_distancia(850,0,300)#hacer el recorrido largo (120)
                esquina=3 #si la esquina es igual a 3 me preparo para hacer el recorrido Largo-Corto y vuelvo en diagonal
                break

            #aqui hago el programa para dejar las pelotas y acomodarme para recorrer dos paredes mas
            wait(1000)       
            #mover_motores_rescate(-400,0.5,-800,1.3)#retrocede en curva
            motor_izquierdo.run_angle(-900,330)
            movermotores(-900,-900,1)
            movermotores(300,300,1.2)#avanza para meter el corralito en el negro
            subir()
            wait(1500)
            #retrocede
            movermotores(-600,-600,1)
            hub.imu.reset_heading(0)#gira con un solo motor para acomodarse en diagonal
            while hub.imu.heading() < 84:
                motor_izquierdo.run(350)
            motor_izquierdo.stop()
            movermotores(300,300,0.6)#avanza para acercarme a la tercera pared
            bajar()
            wait(1500)

            hub.imu.reset_heading(0)#reinicio un girocospio
            while hub.imu.heading() < 37: #giro hasta alinearme con la tercera pared
                mover_motores_indefinido(300,-300)
            parar_motores()
            
            #aqui pregunto en que esquina encontre el color negro y defino los avances para hacer dos paredes mas
            if esquina == 1 or esquina == 3: #hago reccorido largo-corto
                hub.imu.reset_heading(0)#resetea giroscopo
                recorrer_distancia(430,0,300)#avanza 35cm corto
                while hub.imu.heading() < 85:#girar hasta la cuarta pared
                    mover_motores_indefinido(300,-300)
                parar_motores()
                hub.imu.reset_heading(0)
                recorrer_distancia(790,0,300)#avanza 75cm largo
                while hub.imu.heading() < 140:#giro en diagonal
                    mover_motores_indefinido(300,-300)
                parar_motores()    
                recorrer_distancia(1050,140,300)#avance hasta la zona de rescate

                
            if esquina == 2: #hago recorrido corto-largo
                hub.imu.reset_heading(0)#resetea giroscopo
                recorrer_distancia(770,0,300)#avanza 80cm largo
                while hub.imu.heading() < 87:#girar hasta la cuarta pared
                    mover_motores_indefinido(300,-300)
                parar_motores()
                hub.imu.reset_heading(0)
                recorrer_distancia(770,0,300)#avance 80cm largo
                while hub.imu.heading() < 122:#giro en diagonal
                    mover_motores_indefinido(300,-300)
                parar_motores()    
                recorrer_distancia(1000,122,300)#avance hasta la zona de rescate
          


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
            rescate()
            break # Llamada a la funcion rescate

        seguidor_de_linea() # Lamada a la funcion seguidor de linea
        #tiempo_final = tiempo.time()
        #print(tiempo_final)


# Ejecutar el bucle principal 
#
subir()
wait(1000)
#
main()
#para ver los valores de los colores
"""while True:
    update()
    valores_HSV_color("frontal")"""

"""while True:
    print(distance_sensor.distance())"""

"""bajar()
wait(3000)
subir()
wait(3000)"""

"""while True:
    update()
    print(f"luz izquierda: {light_l} luz derecha {light_r} color fremte {light_f}")"""
    
