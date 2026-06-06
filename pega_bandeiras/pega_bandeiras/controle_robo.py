import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from scipy.spatial.transform import Rotation as R

from cv_bridge import CvBridge
import cv2
import numpy as np
from enum import Enum

# O ROBO TRANSITA ENTRE OS CINCO ESTADOS POSSIVEIS:
class ESTADOS(Enum):
    EXPLORANDO = 1
    BANDEIRA_ENCONTRADA = 2
    INDO_PARA_BANDEIRA = 3
    POSICIONANDO_NA_BANDEIRA = 4
    DESVIANDO = 5


class ControleRobo(Node):

    def __init__(self):
        super().__init__('controle_robo')

        # Publisher para comando de velocidade
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(
            Image, '/robot_cam/colored_map', self.camera_callback, 10)

        # Utilizado para converter imagens ROS -> OpenCV
        self.bridge = CvBridge()

        # Timer para enviar comandos continuamente
        self.timer = self.create_timer(0.1, self.move_robot)

        # Contador para evitar loop infinito
        self.maximo_loops = 90
        self.contador = self.maximo_loops

        # Estado interno
        self.obstaculo_a_frente = False             # Define se existe algo a menos de uma certa distancia usando o LIDAR
        self.direcao_obstaculo = 0                  # 1 -> direita e -1 -> esquerda
        self.percentual_bandeira = -1               # Diz a porcentagem que a bandeira ocupa na camera
        self.menor_distancia_esquerda = -1          # Angulo da menor distancia a esquerda
        self.menor_distancia_direita = -1           # Angulo da menor distancia a direita
        self.bandeira_a_frente = False              # Vira true quando a bandeira esta no centro da camera
        self.estado_atual = ESTADOS.EXPLORANDO      # Estado atual do robo na maquina de estados
        self.chegou_na_bandeira = False             # Vira true quando uma certa portagem da bandeira ocupa a tela
        self.giro_desvio = 0                        # Define se o robo esta girando para desvio de algum obstaculo

        self.const_dist_obstaculo = 0.8             # Constante que define a distancia minima para um obstaculo

        self.pos_x_bandeira = -1                    # Posicao horizontal da bandeira na tela
        self.pos_x_mastro = -1                      # Posicao horizontal do mastro da bandeira na tela
        self.ultima_posx_bandeira = -1              # Ultima posicao da bandeira vista na tela (caso saia da camera)
        self.centro_x = -1                          # Centro horizontal da imagem (tamanho horizontal dividido por 2)
        self.dx = 20                                # Tolerancia em pixels para considerar a bandeira no centro da imagem
        self.dx_mastro = 3                          # Tolerancia em pixels para considerar o mastro da bandeira no centro da imagem

    # FUNCAO QUE RECEBE OS DADOS DO LIDAR
    def scan_callback(self, msg: LaserScan):
        # Verifica uma faixa estreita ao redor de 0° (frente)
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        indices_frente_direita = list(range(330, 360))      # Indices que estao ate 30° a direita
        indices_frente_esquerda = list(range(0, 31))        # Indices que estao ate 30° a esquerda
        indices_direita = list(range(240, 360))             # Indices que estao ate 120° a direita
        indices_esquerda = list(range(0, 120))              # Indices que estao ate 120° a esquerda

        distancias_frente_esquerda = [msg.ranges[i] for i in indices_frente_esquerda]   # Distancias para ate 30° a esquerda
        distancias_frente_direita = [msg.ranges[i] for i in indices_frente_direita]     # Distancias para ate 30° a direita
        
        
        #Loop que calcula o angulo da menor distancia a direita
        dist_esq = msg.ranges[0]
        self.menor_distancia_esquerda = 0
        for i in range(1,120):
            if msg.ranges[i] < dist_esq:
                dist_esq = msg.ranges[i]
                self.menor_distancia_esquerda = i
        
        
        #Loop que calcula a menor distancia a esquerda
        dist_dir = msg.ranges[240]
        self.menor_distancia_direita = 240
        for i in range(240,360):
            if msg.ranges[i] < dist_dir:
                dist_dir = msg.ranges[i]
                self.menor_distancia_direita = i

        # Caso observe obstaculos em ambos os lados:
        if distancias_frente_direita and min(distancias_frente_direita) < self.const_dist_obstaculo and distancias_frente_esquerda and min(distancias_frente_esquerda) < self.const_dist_obstaculo:
            # Se a menor distancia estiver a esquerda:
            if min(distancias_frente_esquerda) < min(distancias_frente_direita):
                # Define que o obstaculo esta a esquerda
                self.direcao_obstaculo = -1
            # Caso a menor distancia estiver a direita
            else:
                # Define que o obstaculo esta a direita
                self.direcao_obstaculo = 1

            #Define que existe obstaculo a frente
            self.obstaculo_a_frente = True

        # Caso so hajam obstaculos a esquerda
        elif distancias_frente_esquerda and min(distancias_frente_esquerda) < self.const_dist_obstaculo:
            # Define que o obstaculo esta a esquerda
            self.obstaculo_a_frente = True
            self.direcao_obstaculo = -1

        # Caso so hajam obstaculos a direita
        elif distancias_frente_direita and min(distancias_frente_direita) < self.const_dist_obstaculo:
            # Define que o obstaculo esta a direita
            self.obstaculo_a_frente = True
            self.direcao_obstaculo = 1

        # Caso nao hajam obstaculos
        else:
            # Define a variavel que indica se existem obstaculos como False
            self.obstaculo_a_frente = False

    def imu_callback(self, msg: Imu):
        pass

    def odom_callback(self, msg: Odometry):
        # Mensagens de Odometria das rodas!
        pass

    # FUNCAO QUE RECEBE OS DADOS DO LIDAR
    def camera_callback(self, msg: Image):
        # Converte mensagem ROS para imagem OpenCV (BGR)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Obtem o tamanho da imagem e determina o centro horizontal da imagem
        h, w = frame.shape[:2]
        self.centro_x = w // 2

        # Cor da bandeira (BGR)
        target_color = np.array([227, 73, 0])

        # Máscara
        mask = cv2.inRange(frame, target_color, target_color)

        # Quantos pixels da imagem pertencem à bandeira
        pixels_bandeira = cv2.countNonZero(mask)
        area_imagem = h * w
        self.percentual_bandeira = pixels_bandeira / area_imagem

        # Detecta contornos
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # Define a variavel bandeira_a_frente como True caso haja algum contorno da cor da bandeira na imagem
        self.bandeira_a_frente = len(contours) > 0

        # Valores padrão
        if self.pos_x_bandeira != None: self.ultima_posx_bandeira = self.pos_x_bandeira
        self.pos_x_bandeira = None
        self.area_bandeira = self.percentual_bandeira

        # Caso exista algum contorno da bandeira
        if contours:
            # Seleciona apenas o maior blob
            maior_contorno = max(contours, key=cv2.contourArea)

            M = cv2.moments(maior_contorno)

            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # Define a coordenada x da bandeira sendo o centro do maior blob
                self.pos_x_bandeira = cx

            # Calcula o quanto a bandeira ocupa na imagem
            area_blob = cv2.contourArea(maior_contorno)

            # Define que chegou na bandeira caso essa ocupe mais de 2,5% da imagem
            self.chegou_na_bandeira = self.percentual_bandeira > 0.025
        
        # Caso esteja no estado POSICIONANDO_NA_BANDEIRA
        # Calcula a coordenada x do mastro da bandeira considerando apenas a parte inferior da imagem
        if self.estado_atual == ESTADOS.POSICIONANDO_NA_BANDEIRA:
            # Define limite inferior (apenas a parte inferior da imagem)
            y_limite = int(h * 2/3)

            # Cria uma máscara apenas da região inferior
            mask_inferior = np.zeros_like(mask)
            mask_inferior[y_limite:h, :] = mask[y_limite:h, :]

            # Quantos pixels da imagem pertencem à bandeira na região inferior
            pixels_bandeira = cv2.countNonZero(mask_inferior)
            area_imagem_inferior = (h - y_limite) * w
            self.percentual_bandeira = pixels_bandeira / area_imagem_inferior

            # Detecta contornos apenas na região inferior
            contours, _ = cv2.findContours(
                mask_inferior,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            if contours:
                # Seleciona apenas o maior blob
                maior_contorno = max(contours, key=cv2.contourArea)

                # Metade da altura da imagem
                y_meio = h // 2

                # Mantém apenas os pontos do contorno na metade inferior
                pontos_inferiores = maior_contorno[maior_contorno[:, 0, 1] >= y_meio]

                if len(pontos_inferiores) > 2:  # necessário para formar um contorno
                    M = cv2.moments(pontos_inferiores)

                    if M['m00'] != 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])

                        # Define a coordenada x do mastro sendo o centro do maior blob
                        self.pos_x_mastro = cx
    
    # FUNCAO QUE DEFINE A MAQUINA DE ESTADOS DO ROBO E A SUA MOVIMENTACAO PARA CADA ESTADO
    def move_robot(self):

        # Printa o estado atual do robo no terminal
        if self.estado_atual == ESTADOS.EXPLORANDO: self.get_logger().info("EXPLORANDO")
        if self.estado_atual == ESTADOS.BANDEIRA_ENCONTRADA: self.get_logger().info("BANDEIRA_ENCONTRADA") 
        if self.estado_atual == ESTADOS.INDO_PARA_BANDEIRA: self.get_logger().info("INDO_PARA_BANDEIRA") 
        if self.estado_atual == ESTADOS.POSICIONANDO_NA_BANDEIRA: self.get_logger().info("POSICIONANDO_NA_BANDEIRA")
        if self.estado_atual == ESTADOS.DESVIANDO: self.get_logger().info("DESVIANDO")

        twist = Twist()
        
        # Muda para o estado "POSICIONANDO_NA_BANDEIRA" quando percebe que chegou na bandeira
        if self.chegou_na_bandeira and self.obstaculo_a_frente:
            self.estado_atual = ESTADOS.POSICIONANDO_NA_BANDEIRA


        # CASO ESTEJA NO ESTADO "EXPLORANDO"
        # Nao sabe onde a bandeira esta, e anda aleatoriamente desviando dos obstaculos ate encontrar
        if self.estado_atual == ESTADOS.EXPLORANDO:
            # Anda pra frente enquanto nao encontrar obstaculos
            if not self.obstaculo_a_frente:
                twist.linear.x = 0.5  # Move para frente
            # Quando encontra, gira para desviar
            else:
                twist.angular.z = self.direcao_obstaculo * 0.3  # Gira em torno do proprio eixo

            # Muda para o estado "BANDEIRA_ENCONTRADA" quando reconhece a bandeira
            if self.bandeira_a_frente:
                self.estado_atual = ESTADOS.BANDEIRA_ENCONTRADA


        # CASO ESTEJA NO ESTADO "BANDEIRA_ENCONTRADA"
        # Percebeu visualmente na camera algum pixel da bandeira
        elif self.estado_atual == ESTADOS.BANDEIRA_ENCONTRADA:
            # Volta para o estado "EXPLORANDO" caso perca a bandeira no processo
            if self.pos_x_bandeira == None:
                self.estado_atual = ESTADOS.EXPLORANDO

            # Vira para a direita caso a bandeira esteja sendo vista na esquerda do centro da imagem
            elif self.pos_x_bandeira < self.centro_x - self.dx:
                twist.angular.z = 0.3  # Gira em torno do proprio eixo
                twist.linear.x = 0.1
            # Vira para a esquerda caso a bandeira esteja sendo vista na direita do centro da imagem
            elif self.pos_x_bandeira > self.centro_x + self.dx:
                twist.angular.z = -0.3  # Gira em torno do proprio eixo
                twist.linear.x = 0.1
            # Caso a bandeira ja esteja no centro da image, vai para o estado "INDO_PARA_BANDEIRA"
            elif self.pos_x_bandeira <= self.centro_x + self.dx and self.pos_x_bandeira >= self.centro_x - self.dx:
                self.estado_atual = ESTADOS.INDO_PARA_BANDEIRA

        
        # CASO ESTEJA NO ESTADO "INDO_PARA_BANDEIRA"
        # A bandeira esta em vista e ele vai reto em direcao a bandeira
        elif self.estado_atual == ESTADOS.INDO_PARA_BANDEIRA:

            # Caso nao tenham obstaculos a frente
            if not self.obstaculo_a_frente:
                # Caso a bandeira saia do centro da imagem, volta para o estado "BANDEIRA_ENCONTRADA"
                if self.pos_x_bandeira != None and not (self.pos_x_bandeira < self.centro_x + self.dx and self.pos_x_bandeira > self.centro_x - self.dx):
                    self.estado_atual = ESTADOS.BANDEIRA_ENCONTRADA
                    self.giro_desvio = 0                # Tambem define o giro como zero
                    self.contador = self.maximo_loops   # Reseta o contador de loop infinito
                # Caso giro_desvio seja diferente de zero, esta vindo do estado "DESVIANDO"
                # Portanto, deve continuar girando na direcao contraria ao desvio ate enxergar a bandeira novamente
                if self.giro_desvio != 0:
                    # Gira ate encontrar a bandeira novamente e decremente o contador
                    twist.angular.z = self.giro_desvio * 0.3
                    self.contador = self.contador - 1

                    # Caso o contador zere, volta para o estado de "EXPLORANDO"
                    if self.contador == 0:
                        self.giro_desvio = 0
                        self.estado_atual = ESTADOS.EXPLORANDO
                        self.contador = self.maximo_loops

                # Caso contrario, continua andando reto ate a bandeira
                else:
                    self.giro_desvio = 0
                    twist.linear.x = 0.5
            # Caso tenham obstaculos a frente, entra no estado "DESVIANDO"
            else:
                self.estado_atual = ESTADOS.DESVIANDO 

            # Caso nao tenha a bandeira em vista e tambem nao esteja voltando do desviando, volta para explorando
            if not self.bandeira_a_frente and self.giro_desvio == 0:
                self.estado_atual = ESTADOS.EXPLORANDO
        
        
        # CASO ESTEJA NO ESTADO "DESVIANDO"
        # O robo detectou um obstaculo a frente e precisa desviar
        elif self.estado_atual == ESTADOS.DESVIANDO:
            
            # Caso o obstaculo nao esteja mais a frente
            if not self.obstaculo_a_frente:

                # Caso o obstaculo esteja a esquerda
                if self.direcao_obstaculo == -1:
                    # Se a bandeira estiver a vista a direita, volta para o estado de "INDO_PARA_BANDEIRA"
                    if self.pos_x_bandeira != None and self.pos_x_bandeira > self.centro_x + self.dx :
                        self.estado_atual = ESTADOS.INDO_PARA_BANDEIRA
                    # Avalia se a menor distancia a esquerda, ainda esta a menos de 90°, vai pra frente
                    elif self.menor_distancia_esquerda < 90:
                        twist.linear.x = 0.5
                    # Caso ja esteja a mais de 90°
                    else:
                        self.direcao_obstaculo = 0          # Define que nao existe mais obstaculo
                        self.giro_desvio = 1                # Define o giro de desvio para a direita
                        twist.angular.z = 0.3               # Comeca a girar
                        self.estado_atual = ESTADOS.INDO_PARA_BANDEIRA      # Volta para o estado "INDO_PARA_BANDEIRA"

                # Caso o obstaculo esteja a direita
                elif self.direcao_obstaculo == 1:
                    # Se a bandeira estiver a vista a esquerda, volta para o estado de "INDO_PARA_BANDEIRA"
                    if self.pos_x_bandeira != None and self.pos_x_bandeira < self.centro_x - self.dx :
                        self.estado_atual = ESTADOS.INDO_PARA_BANDEIRA
                    # Avalia se a menor distancia a direita, ainda esta a menos de 90°, vai pra frente
                    elif self.menor_distancia_direita > 270:
                        twist.linear.x = 0.5
                    # Caso ja esteja a mais de -90°
                    else:
                        self.direcao_obstaculo = 0          # Define que nao existe mais obstaculo
                        self.giro_desvio = -1               # Define o giro de desvio para a esquerda
                        twist.angular.z = -0.3              # Comeca a girar
                        self.estado_atual = ESTADOS.INDO_PARA_BANDEIRA      # Volta para o estado "INDO_PARA_BANDEIRA"
                        
            # Caso o obstaculo ainda esteja a frente
            else:
                # Gira para o lado contrario ao lado que o obstaculo esta
                if self.direcao_obstaculo == -1:
                    twist.angular.z = -0.3
                elif self.direcao_obstaculo == 1:
                    twist.angular.z = 0.3
                    

        # CASO ESTEJA NO ESTADO "POSICIONANDO_NA_BANDEIRA"
        # O robo esta na frente da bandeira e ajeita para pega-la
        elif self.estado_atual == ESTADOS.POSICIONANDO_NA_BANDEIRA:

            # Caso aconteca de perder o mastro de vista, volta para o estado "EXPLORANDO"
            if self.pos_x_mastro == None:
                self.estado_atual = ESTADOS.EXPLORANDO
            # Caso o mastro esteja a esquerda do centro da imagem, gira para a direita ate alinhar
            elif self.pos_x_mastro < self.centro_x - self.dx_mastro:
                twist.angular.z = 0.2  # Gira em torno do proprio eixo
            # Caso o mastro esteja a direita do centro da imagem, gira para a esquerda ate alinhar
            elif self.pos_x_mastro > self.centro_x + self.dx_mastro:
                twist.angular.z = -0.2  # Gira em torno do proprio eixo
            # Caso o mastro esteja no centro da imagem, fica parado
            else:
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0


        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()