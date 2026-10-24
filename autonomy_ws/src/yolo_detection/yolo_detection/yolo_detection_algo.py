import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from PIL import Image as PILImage
from autonomy_core.base_drive_node import BaseDriveNode
from cv_bridge import CvBridge, CvBridgeError
import subprocess
from sensor_msgs.msg import Image 
from ultralytics.utils.plotting import Annotator
import os
import csv
from datetime import datetime
import numpy as np
import cv2
from ament_index_python.packages import get_package_share_directory




class YOLODetectionNode(Node):

    def __init__(self):

        super().__init__('yolo_detection_node')

        #PARAMETERS DECLARATION FOR YOLO INFERENCE
        self.declare_parameter('use_rosbags', False) #update this if there's no real-time input and the data is coming from rosbags (testing)
        self.declare_parameter('rosbag_dir_name', "rosbags/rosbag4") #update this with the directory name

        self.declare_parameter('count', 0) #counter to count the frame
        
        
        self.declare_parameter('model_name', "blaze_racer/autonomy_ws/src/yolo_detection/yolo_detection/25_08_runs_ep150_sz640_adamW_lr0.001_lrf_0.1_wd0.0005_Fz10_GS50_noauto_hsv_translate0.1_scale0.3_Coslr_fromtheinitialweight.onnx")
        self.model_name= self.get_parameter('model_name').value
        #model_path = os.path.join(
        #get_package_share_directory('yolo_detection'),
        #'models', self.get_parameter('model_name').value)  # just the filename
        
        
        self.model=YOLO(self.model_name, task='segment')
        self.bridge= CvBridge()

        self.image_sub= self.create_subscription(Image, "/camera/camera/color/image_raw", self.rosbag_to_cv, 10)
        self.detection_pub= self.create_publisher(Image, "/yolo/annotated", 10)
        self.mask_pub=self.create_publisher(Image, "/yolo/drivable_mask", 10)

        #VARIABLES FOR MODEL TESTING
        self.car_count=0
        self.total_detections=0
        self.wall_count=0
        self.frames_with_wall=0
    

        #VARIABLES FOR MASK DETECTION
        self.target_class={0,1}
        self.count=self.get_parameter('count').value 
    
        self.data_stream_selection()
      
      

    def data_stream_selection(self):

        """
        This functions selects the test mode : Use_rosbags=True and the real time mode: Use_rosbags=False
        """
        use_rosbags= self.get_parameter('use_rosbags').value

        if use_rosbags==True:
             self.rosbag_dir_name= self.get_parameter('rosbag_dir_name').value
             self.get_logger().info("Playing back rosbag")
             subprocess.Popen(['ros2', 'bag', 'play', self.rosbag_dir_name, '--rate', '0.1'])
           
    
             

        else:
            self.rosbag_dir_name= self.get_parameter('rosbag_dir_name').value
            self.get_logger().info("Real-time data stream selected")


    def rosbag_to_cv(self, msg:Image):
        try:
            self.get_logger().info("Image callback triggered")
            self.get_logger().info(f"Image size: {msg.width} x {msg.height}")
            self.get_logger().info(f"Encoding: {msg.encoding}")
            self.get_logger().info(f"Data length: {len(msg.data)}")
            cv_image=self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.yolo_inference(cv_image)
            #cv2.imshow('camera_image',cv_image)
         
        except CvBridgeError as e:
            self.get_logger().info(e)

    
    def yolo_inference(self, cv_image):

         
        results = self.model.track(cv_image, conf=0.05, seed=42)
       
        today = datetime.now().strftime("%Y-%m-%d")
        self.save_directory= f"autonomy_ws/src/yolo_detection/{today}/output_results_{self.model_name}_1"
        os.makedirs(self.save_directory, exist_ok=True)

        #saving them in image format
        self.count+=1
        for r in results:
            
           im_array=r.plot(  masks=True,
                             boxes=True,
                             labels=False,
                             conf=True,)
           
           #self.get_logger().info(f"YOLO plot output shape: {im_array}")
           im=PILImage.fromarray(im_array[...,::-1])

          
           #img_msg= cv2.imread(im)
           
           #image_path= os.path.join(self.save_directory, f"output_results_{self.count}.jpg")
           #im.save(image_path)
           mask = self.binary_mask(r)
           if mask is None:
            
            self.get_logger().info("No mask detected this frame, skipping")
            continue 

           #im_array_np = np.ascontiguousarray(im_array).astype(np.uint8)
           #rgb_image = cv2.cvtColor(im_array_np, cv2.COLOR_BGR2RGB)
           #img_msg=self.bridge.cv2_to_imgmsg(rgb_image, encoding="rgb8")   
           #self.detection_pub.publish(img_msg)    
           
           #creating image msg
           msg = Image()
           msg.header.stamp = self.get_clock().now().to_msg()
           msg.header.frame_id = "camera_color_optical_frame"

           msg.height = im_array.shape[0]
           msg.width = im_array.shape[1]

        # Use rgb8 first because your original camera topic was rgb8
           msg.encoding = "bgr8"

           msg.is_bigendian = 0
           msg.step = msg.width * 3
           msg.data = im_array.tobytes()

           self.detection_pub.publish(msg)
           self.get_logger().info("Published YOLO annotated image")

        #creating mask msg
           
           msg2=Image()
           msg2.header.stamp=self.get_clock().now().to_msg()
           msg2.header.frame_id="yolo_drivable_mask"

           msg2.height = mask.shape[0]
           msg2.width = mask.shape[1]

           msg2.encoding= 'mono8'

           msg2.is_bigendian=0
           msg2.step= msg2.width * 1

           msg2.data=mask.tobytes()

           self.mask_pub.publish(msg2)
           self.get_logger().info("Published mask")

        #self.extracting_details(results)

    def binary_mask(self, r):
        
            #initializing a blank binary mask matching the original dimension
            img_h, img_w = r.orig_shape
            combined_mask = np.zeros((img_h, img_w), dtype=np.uint8) 

            #check if any masks were detected
            if r.masks is not None:

                clss= r.boxes.cls.cpu().numpy() #class IDs

                masks= r.masks.data.cpu().numpy() #shape: (masks, h, w)


                #Loop through detections and merge classes
                for idx, clss_id in enumerate(clss):

                    if int(clss_id) in self.target_class:
                        mask=masks[idx]

                        if mask.shape[:2]!=(img_h, img_w):


                            mask= cv2.resize(mask, (img_w, img_h), interpolation=cv2.INTER_NEAREST)


                        
                    
                        #combine using logical or  
                        combined_mask= np.logical_or(combined_mask, mask).astype(np.uint8)
                        
                # Clean small holes and random dots.
                kernel1 =cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))

              
                
                cleaned_mask = cv2.morphologyEx(
                    combined_mask,
                    cv2.MORPH_CLOSE,
                    kernel1,
                    iterations=1
                )
                                 
                
                # Remove very small blobs.
                num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
                        cleaned_mask,
                        connectivity=8
                    )

                self.cleaned = np.zeros_like(cleaned_mask)

                min_area = 3000
                component_areas=[]
                
                for label in range(1, num_labels):
                    area = stats[label, cv2.CC_STAT_AREA]

                    """
                    component_areas.append((label,area))
                    

                    #write an if statement and correction                           
                    
                    print(
                        "connected component areas",
                        component_areas
                    )
                    
                    """
                    if area >= min_area:
                      self.cleaned[labels == label] = 255

                
                filepath= os.path.join(self.save_directory, 'connected_components.csv')
                with open(filepath, 'a', newline="",encoding='utf-8') as file:
                                writer=csv.writer(file)
                    
                                writer.writerow(f"connected component")
                                writer.writerow(component_areas)
                                writer.writerow("*******************")


                kernel2 =cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (12,12))

                self.cleaned = cv2.morphologyEx( self.cleaned, cv2.MORPH_DILATE, kernel2,iterations=1)


                ##### finding the connected component again after dilation
                num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
                                        self.cleaned,
                                        connectivity=8
                                    )
                
                component_areas=[]
                                
                for label in range(1, num_labels):
                    area = stats[label, cv2.CC_STAT_AREA]
                
                    component_areas.append((label,area))
                                    
                
                    #write an if statement and correction                           
                                    
                    print(
                        "connected component areas",
                        component_areas
                        )
                
                #################
                                 
    
                self.fixing_holes(labels,component_areas, stats)

                kernel2 =cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (12,12))
                self.cleaned = cv2.morphologyEx( self.cleaned, cv2.MORPH_DILATE, kernel2,iterations=1)
                
                #combined_image = os.path.join(self.save_directory,  f"combined_image_{self.count}.jpg")

                    #combined_mask*=255
                            
                #cv2.imwrite(combined_image,self.cleaned)
                mask=self.drivable_mask(self.cleaned)
                self.get_logger().info("Drivable mask created and saved")
                return mask
            else:
                return None
            

    def drivable_mask(self, cleaned_wall_mask):

        start_point=(320, 479)

        fill_value=128

        h, w=cleaned_wall_mask.shape[:2]
        mask=np.zeros((h+2, w+2), np.uint8)

        

        cv2.floodFill(cleaned_wall_mask, mask, seedPoint=start_point, newVal=fill_value, loDiff= 0, upDiff= 0, flags=4)

        #drivable_image= os.path.join(
         #   self.save_directory,
          #  f"drivable_image_{self.count}.jpg"
        #)
        #cv2.imwrite(drivable_image, cleaned_wall_mask)
        return cleaned_wall_mask
        #self.birds_eye_view(cleaned_wall_mask)

    def fixing_holes(self, label, component_areas, stats):

        largest_label, largest_area= max(component_areas, key=lambda item: item[1])
         

        x=stats[largest_label, cv2.CC_STAT_LEFT]
        y=stats[largest_label, cv2.CC_STAT_TOP]
        w=stats[largest_label, cv2.CC_STAT_WIDTH]
        h=stats[largest_label, cv2.CC_STAT_HEIGHT]
        
        touches_left   = (x == 0)
        touches_right  = (x + w == 640)
        touches_top    = (y == 0)
        touches_bottom = (y + h == 480)

        component=(label==largest_label)

        if not touches_left:

            component_rows = np.where(component.any(axis=1))[0]

            for row in component_rows:
                component_xs = np.flatnonzero(component[row])
                leftmost_x = component_xs[0]
                if leftmost_x<=320: 
                  component[row, :leftmost_x + 1] = True


            self.cleaned = component.astype(np.uint8) * 255


        
        if not touches_right:
        
            component_rows_r = np.where(component.any(axis=1))[0]
        
            for row in component_rows_r:
                component_xs_r = np.flatnonzero(component[row])

                
                rightmost_x = component_xs_r[-1]
                if rightmost_x>=320: 
                    print("right_most", rightmost_x)
                    
                    component[row,rightmost_x + 1:] = True
            self.cleaned = component.astype(np.uint8) * 255         
        
        


    def birds_eye_view(self, cleaned_wall_mask):
        #Four points
        srcpoints=np.array([[180, 340], [70,472], [438,340], [538, 472]], dtype=np.float32)

        dstpoints=np.array([[0,0], [0,480], [640,0], [640,480]], dtype=np.float32) #size of the entire image

        matrix= cv2.getPerspectiveTransform(srcpoints, dstpoints) #get the transformation matrix

        #INTER_NEAREST prevents blending: no blending due to binary image
        transformed_frame= cv2.warpPerspective( cleaned_wall_mask, matrix,(640, 480), flags=cv2.INTER_NEAREST, borderValue=0) 

        
        transformed_image= os.path.join(
                    self.save_directory,
                    f"birds_eye_view_{self.count}.jpg"
                )
        
                
        cv2.imwrite(transformed_image, transformed_frame)

        

    def extracting_details(self, results):
        
        filepath= os.path.join(self.save_directory, 'summary.csv')
        
        class_names= {
           0: 'car',
           1: 'wall'
        }

        wall_found=False
        with open(filepath, 'a', newline="",encoding='utf-8') as file:
            writer=csv.writer(file)
            for r in results:
                print(r.boxes)
                
                if r.boxes is None or len(r.boxes)==0:
                    writer.writerow(["no detection", "", ""])
                    self.total_detections +=1
                    continue

                class_id = r.boxes.cls.int()
                confidence=r.boxes.conf

                for class_id, confidence in zip(class_id, confidence):
                    class_id=int(class_id.item())
                    confidence=float(confidence.item())
                    class_name= class_names.get(
                        class_id, f"Unkown Class {class_id}"
                    )

                    self.total_detections +=1
                    if class_id==0:
                        self.car_count+=1

                    if class_id==1:
                        self.wall_count+=1
                        wall_found=True

                    writer.writerow([f"image_number:{self.count}",class_id, class_name, confidence])
                
                if wall_found:
                    self.frames_with_wall+=1      
    def save_counts(self):
        filepath= os.path.join(self.save_directory, 'summary.csv')
        with open(filepath, 'a', newline="",encoding='utf-8') as file:
            writer=csv.writer(file)

            writer.writerow([f"counts",
                            f"car_counts: {self.car_count}",
                            f"wall_count: {self.wall_count}",
                            f"total_detections:  {self.total_detections}"])

            total_walls_detected= self.frames_with_wall/self.count #count is the total number of frames
            writer.writerow([f"percentage_of_walls_detected: {total_walls_detected}"])
        self.get_logger().info("Final_Detections_Saved")

def main(args=None):
    """Spin the Follow-the-Gap node until interrupted."""
    rclpy.init(args=args)
    node =YOLODetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        #node.save_counts()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
