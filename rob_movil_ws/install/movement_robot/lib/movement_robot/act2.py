import rclpy
from rclpy.node import Node
 
# Msj Imagen
from sensor_msgs.msg import Image
# Msj Vector
from geometry_msgs.msg import Vector3
 
import cv2
from cv_bridge import CvBridge
import numpy as np

class  ObstacleDetectorNode(Node): 
    def __init__(self):
        super().__init__("obstacle_detector") 
        # Crear subscriber
        self.depth_sub = self.create_subscription(Image, "/camera/depth/image_raw", self.callback_depth, 10)
        # Crear bridge
        self.bridge = CvBridge()
        self.current_cv_depth_image = None
        # Crear publisher
        self.obstacle_pub = self.create_publisher(Vector3, "/occupancy_state", 10)
        
    def callback_depth(self, data):
        # Se extrae la imagen en formato CV2 y su forma
        self.current_cv_depth_image = self.bridge.imgmsg_to_cv2(data)
        img_shape = self.current_cv_depth_image.shape
    
        # Filtro Median para eliminar ruido
        # - Primero se copia la imagen actual y se le quitan los valores infinitos
        clean_img = np.copy(self.current_cv_depth_image)
        clean_img[np.isinf(clean_img)] = 10.0
        clean_img[np.isnan(clean_img)] = 0.0
        
        # - Ahora se aplica el filtro 
        self.median_filter_img = cv2.medianBlur(clean_img.astype(np.float32), 5)

        # Dividir la imagen filtrada en tercios
        left_third   =   self.median_filter_img[:, :img_shape[1] // 3]
        center_third =   self.median_filter_img[:, img_shape[1] // 3: 2 * img_shape[1] // 3]
        right_third  =   self.median_filter_img[:, 2 * img_shape[1] // 3:]
                
        # Se revisa cada tercio 
        left_obstacle = self.check_region(left_third)
        center_obstacle = self.check_region(center_third)
        right_obstacle = self.check_region(right_third)
        
        # Se publica el mensaje
        self.publish_occupancy([left_obstacle, center_obstacle, right_obstacle])
        
        #print(str(left_obstacle),str(center_obstacle),str(right_obstacle))
        
    def publish_occupancy(self, states):
        # Crear mensaje
        msg = Vector3()
        msg.x = 1.0 if states[0] else 0.0
        msg.y = 1.0 if states[1] else 0.0
        msg.z = 1.0 if states[2] else 0.0
        # Publicar mensaje
        self.obstacle_pub.publish(msg)
        
    def print_maxmin(self, img, name=""):
        # print humilde para chequear los minimos y máximos
        min_val = np.nanmin(img)
        max_val = np.nanmax(img)
        
        print("Valor mínimo "+ name+ f": {min_val}")
        print("Valor máximo "+ name+ f": {max_val}")
        
    def check_region(self, region):
        # Básicamente, si es que hay algún pixel en la región que esté entre 0.5 y 0.0
        # - Como eliminamos el ruido, nos aseguramos que esto es verdad. 
        return np.any((region <= 0.5) & (region >= 0.0))

        
def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()