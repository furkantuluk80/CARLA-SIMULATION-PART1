import glob
import os
import sys
import time
import random
import numpy as np
import cv2
import math
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


SHOW_PREVIEW= False
IM_HEIGHT = 640
IM_WIDTH =480
SECONDS_PER_EPISODE = 10

class CarEnv:
    SHOW_CAM= SHOW_PREVIEW 
    STEER_AMT=1.0
    im_width=IM_WIDTH
    im_height=IM_HEIGHT
    front_camera=None

    def __init__(self):
        self.client=carla.Client("localhost",2000)
        self.client.set_timeout(10.0)
        self.world=client.get_world()
        self.blueprint_library = world.get_blueprint_library()
        self.model3=blueprint_library.filter("model3")[0]

    def reset(self):
        self.collision_hist = []
        self.actor_list= []

        self.transform = random.choice(self.world.get_map().get_spawn_points()) #Haritayı alma ve doğuş(yaratılış) konumlarını alma
        self.vehicle = self.world.spawn_actor(self.model_3, self.transform)

        self.actor_list.append(self.vehicle)

        self.rgb_cam = blueprint_library.find("sensor.camera.rgb")
        self.rgb.set_attribute("image_size_x",f"{im_width}")
        self.rgb.set_attribute("image_size_y",f"{im_height}")
        self.rgb.set_attribute("fov","110") #kamera ayarı balıkgözü gibi görüş alanı ayarlanır

        transform=carla.Transform(carla.Location(x=2.5, z=0.7 )) #Kamerayı arabaya takma yeri x,y,z deneme yanılma ile bulunacak
        self.sensor=self.world.spawn_actor(self.rgb_cam, transform,attach_to=self.vehicle) #sensör oluştururdu ve alındı
        self.actor_list.append(self.sensor)
        self.sensor.listen(lambda data: self.process_img(data)) #verileri alıyoruz lambda veri işleme mimarisidir,gerçek zamanlı akış işlemeyi kullanır                                                         #x ileri geri y sol sağ z yükseklik

        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0))
        time.sleep(4)

        #2 veya daha fazla nesnenin kesişimini saptamak için
        colsensor=self.blueprint_library.find("sensor.other.collision")
        self.colsensor = self.world.spawn_actor(colsensor,transform,attach_to=self.vehicle) #colsensor yakınlık sensörü yani herhangi bir fiziksel
                                                                                            #temas olmadan yakındaki nesnelerin varlığını algılayabilen bir sensördür. 
        self.self.actor_list.append(self.colsensor) #oluşturulan aktorler listeye eklenir.
        self.colsensor.listen(lambda event: self.collision_data(event)) 

        while self.front_camera is None:

            time.sleep(0.01)
        self.episode_start = time.time()

        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0))

        return self.front_camera

    def collision_data(self, event):
        self.collision_hist.append(event) #eğer bi event varsa o eklenir collision hist arrayine

     
    def process_img(image):  #
        i=np.array(image.raw_data) #datalar resmin data şekli, 

        print(i.shape) #shape diziyi düzenler
        i2=i.reshape((self.im_height, self.im_width, 4))
        i3= i2[:, :, :3]  #RGB yi almanın hızlı yolu
        if self.SHOW_CAM
            cv2.imshow("", i3) #görüntüyü pencerede gösterir
            cv2.waitKey(1)  #kamerayı açıyor
        self.front_camera= i3

    def step(self, action): #Eylemleri gerçekleştirmek için dönüşleri sol sağ düz
        if action == 0:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=-1*self.STEER_AMT))
        elif action == 1:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0))
        elif action == 2:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=1*self.STEER_AMT))

        v = self.vehicle.get_velocity() #arabanın anlık hızı alınır. 
        kmh= int (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)) # 1.0m/s=3.6km/h'dır. Arabanın hızı dönüştürülür.

        if len(self.collision_hist) != 0:
            done = True
            reward = -200
        elif kmh < 50:
            done = False 
            reward = -1
        else:
            done = False
            reward = 1

        if self.episode_start + SECONDS_PER_EPISODE < time.time():
            done = True

        return self.front_camera, reward, done, None























