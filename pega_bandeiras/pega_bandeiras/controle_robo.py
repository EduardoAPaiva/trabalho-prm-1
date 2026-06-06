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

        # Estado interno
        self.obstaculo_a_frente = False
        self.direcao_obstaculo = 0   #1 -> direita e -1 -> esquerda
        self.percentual_bandeira = -1
        self.menor_distancia_esquerda = -1
        self.menor_distancia_direita = -1
        self.bandeira_a_frente = False
        self.estado_atual = ESTADOS.EXPLORANDO
        self.chegou_na_bandeira = False
        self.giro_desvio = 0

        self.const_dist_obstaculo = 0.7

        self.pos_x_bandeira = -1
        self.pos_x_mastro = -1
        self.ultima_posx_bandeira = -1
        self.centro_x = -1
        self.dx = 20
        self.dx_mastro = 5
        self.distancia_frente = float('inf')

    def scan_callback(self, msg: LaserScan):
        # Verifica uma faixa estreita ao redor de 0° (frente)
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        # Índices de -30° a +30° (equivalente a 330 até 30)
        indices_frente_direita = list(range(330, 360))
        indices_frente_esquerda = list(range(0, 31))
        indices_direita = list(range(240, 360))
        indices_esquerda = list(range(0, 120))

        # Filtra distancias 
        distancias_frente_esquerda = [msg.ranges[i] for i in indices_frente_esquerda]
        distancias_frente_direita = [msg.ranges[i] for i in indices_frente_direita]
        
        dist_esq = msg.ranges[0]
        self.menor_distancia_esquerda = 0
        for i in range(1,120):
            if msg.ranges[i] < dist_esq:
                dist_esq = msg.ranges[i]
                self.menor_distancia_esquerda = i
        
        dist_dir = msg.ranges[240]
        self.menor_distancia_direita = 240
        for i in range(240,360):
            if msg.ranges[i] < dist_dir:
                dist_dir = msg.ranges[i]
                self.menor_distancia_direita = i

        self.distancia_frente = msg.ranges[0]
        
        if distancias_frente_esquerda and min(distancias_frente_esquerda) < self.const_dist_obstaculo:
            self.obstaculo_a_frente = True
            self.direcao_obstaculo = -1
        elif distancias_frente_direita and min(distancias_frente_direita) < self.const_dist_obstaculo:
            self.obstaculo_a_frente = True
            self.direcao_obstaculo = 1
        elif distancias_frente_direita and min(distancias_frente_direita) < self.const_dist_obstaculo and distancias_frente_esquerda and min(distancias_frente_esquerda) < self.const_dist_obstaculo:
            if min(distancias_frente_esquerda) < min(distancias_frente_direita):
                self.direcao_obstaculo = -1
            else:
                self.direcao_obstaculo = 1
            self.obstaculo_a_frente = True
        else:
            self.obstaculo_a_frente = False

    def imu_callback(self, msg: Imu):
        pass

    def odom_callback(self, msg: Odometry):
        # Mensagens de Odometria das rodas!
        pass

    def camera_callback(self, msg: Image):
        # Converte mensagem ROS para imagem OpenCV (BGR)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

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

        self.bandeira_a_frente = len(contours) > 0

        # Valores padrão
        self.ultima_posx_bandeira = self.pos_x_bandeira
        self.pos_x_bandeira = None
        self.area_bandeira = self.percentual_bandeira

        if contours:
            # Seleciona apenas o maior blob
            maior_contorno = max(contours, key=cv2.contourArea)

            M = cv2.moments(maior_contorno)

            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                self.pos_x_bandeira = cx

            area_blob = cv2.contourArea(maior_contorno)

            # self.get_logger().info(
            #     f'Bandeira: {self.percentual_bandeira*100:.1f}% da imagem'
            # )

            # Ajuste esse limiar experimentalmente
            self.chegou_na_bandeira = self.percentual_bandeira > 0.025
        
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

                        self.pos_x_mastro = cx

    def move_robot(self):

        if self.estado_atual == ESTADOS.EXPLORANDO: print("EXPLORANDO")
        if self.estado_atual == ESTADOS.BANDEIRA_ENCONTRADA: print("BANDEIRA_ENCONTRADA")
        if self.estado_atual == ESTADOS.INDO_PARA_BANDEIRA: print("INDO_PARA_BANDEIRA")
        if self.estado_atual == ESTADOS.POSICIONANDO_NA_BANDEIRA: print("POSICIONANDO_NA_BANDEIRA")
        if self.estado_atual == ESTADOS.DESVIANDO: print("DESVIANDO")

        twist = Twist()

        if self.chegou_na_bandeira and self.obstaculo_a_frente:
            self.estado_atual = ESTADOS.POSICIONANDO_NA_BANDEIRA

        if self.estado_atual == ESTADOS.EXPLORANDO:
            if not self.obstaculo_a_frente:
                twist.linear.x = 0.5  # Move para frente
            else:
                twist.angular.z = self.direcao_obstaculo * 0.3  # Gira em torno do proprio eixo

            if self.bandeira_a_frente:
                self.estado_atual = ESTADOS.BANDEIRA_ENCONTRADA

        elif self.estado_atual == ESTADOS.BANDEIRA_ENCONTRADA:

            if self.pos_x_bandeira == None:
                self.estado_atual = ESTADOS.EXPLORANDO

            elif self.pos_x_bandeira < self.centro_x - self.dx:
                twist.angular.z = 0.3  # Gira em torno do proprio eixo
            elif self.pos_x_bandeira > self.centro_x + self.dx:
                twist.angular.z = -0.3  # Gira em torno do proprio eixo
            elif self.pos_x_bandeira <= self.centro_x + self.dx and self.pos_x_bandeira >= self.centro_x - self.dx:
                self.estado_atual = ESTADOS.INDO_PARA_BANDEIRA
            else:
                self.estado_atual = ESTADOS.EXPLORANDO
        

        elif self.estado_atual == ESTADOS.INDO_PARA_BANDEIRA:

            print(self.percentual_bandeira)

            if not self.obstaculo_a_frente:
                if self.pos_x_bandeira != None and not (self.pos_x_bandeira < self.centro_x + self.dx and self.pos_x_bandeira > self.centro_x - self.dx):
                    self.estado_atual = ESTADOS.BANDEIRA_ENCONTRADA
                    self.giro_desvio = 0
                if self.giro_desvio != 0:
                    twist.angular.z = self.giro_desvio * 0.3
                else:
                    self.giro_desvio = 0
                    twist.linear.x = 0.5
            else:
                self.estado_atual = ESTADOS.DESVIANDO 

            if not self.bandeira_a_frente and self.giro_desvio == 0:
                self.estado_atual = ESTADOS.EXPLORANDO
        
        elif self.estado_atual == ESTADOS.DESVIANDO:
            if not self.obstaculo_a_frente:
                if self.direcao_obstaculo == -1:
                    if self.pos_x_bandeira != None and self.pos_x_bandeira > self.centro_x + self.dx :
                        self.estado_atual = ESTADOS.INDO_PARA_BANDEIRA
                    elif self.menor_distancia_esquerda < 90:
                        twist.linear.x = 0.5
                    else:
                        self.direcao_obstaculo = 0
                        self.giro_desvio = 1
                        twist.angular.z = 0.3
                        self.estado_atual = ESTADOS.INDO_PARA_BANDEIRA
                elif self.direcao_obstaculo == 1:
                    if self.pos_x_bandeira != None and self.pos_x_bandeira < self.centro_x - self.dx :
                        self.estado_atual = ESTADOS.INDO_PARA_BANDEIRA
                    elif self.menor_distancia_direita > 270:
                        twist.linear.x = 0.5
                    else:
                        self.direcao_obstaculo = 0
                        self.giro_desvio = -1
                        twist.angular.z = -0.3
                        self.estado_atual = ESTADOS.INDO_PARA_BANDEIRA
            else:   
                if self.direcao_obstaculo == -1:
                    twist.angular.z = -0.3
                elif self.direcao_obstaculo == 1:
                    twist.angular.z = 0.3

        elif self.estado_atual == ESTADOS.POSICIONANDO_NA_BANDEIRA:
            print(self.pos_x_bandeira, self.pos_x_mastro)
            if self.pos_x_mastro < self.centro_x - self.dx_mastro:
                twist.angular.z = 0.3  # Gira em torno do proprio eixo
            elif self.pos_x_mastro > self.centro_x + self.dx_mastro:
                twist.angular.z = -0.3  # Gira em torno do proprio eixo
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