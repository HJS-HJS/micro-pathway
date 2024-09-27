import cv2
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from scipy.stats import mode

class ImageManager():
    def __init__(self, video_path):
        self.video = cv2.VideoCapture(video_path)
        self.image = None
        self.target = (0, 0)
        self.target_r = 0
        self.pushers = []
        self.goal = (0, 0)
        self.obs = []

    def get_image(self):
        ret, frame = self.video.read()
        self.image = frame
    
    def get_gray_image(self):
        ret, frame = self.video.read()
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
    
    def onMouseGoal(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN :
            self.goal = (x, y)
        elif event == cv2.EVENT_RBUTTONUP :
            self.goal = (x, y)
    
    def onMouseObs(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN :
            self.obs.append((x, y, 3))
        elif event == cv2.EVENT_RBUTTONUP :
            self.obs.append((x, y, 3))

    def get_target(self):
        target_seg = ImageManager.color_extraction(ImageManager.hsv_segment(self.image), (100, 150, 150), (140, 255, 255))
        mass_x, mass_y = np.where(target_seg >= 255)
        cent_x = np.average(mass_x).astype(int)
        cent_y = np.average(mass_y).astype(int)
        self.target = (cent_y, cent_x)
        self.target_r = int((np.max(mass_x) - np.min(mass_x)) / 2)

    def get_pushers(self, num):
        self.pushers = []
        pushers_seg = ImageManager.color_extraction(self.image, (0, 0, 0), (150, 150, 150))
        for i in range(num):
            seg = ImageManager.segment_outlier(pushers_seg)
            pushers_seg = (1 - seg) * pushers_seg
            
            mass_x, mass_y = np.where(seg >= 1)
            cent_x = np.average(mass_x).astype(int)
            cent_y = np.average(mass_y).astype(int)
            if (cent_x < 0) or (cent_x > self.map[0]): continue
            self.pushers.append((cent_y, cent_x))

    def set_goal(self):
        cv2.imshow('image', self.image)
        cv2.setMouseCallback('image', self.onMouseGoal)
        cv2.waitKey()

    def set_obstacle(self):
        cv2.circle(self.image, self.goal, 3, (0,0,255), -1)
        cv2.imshow('image', self.image)
        self.obs = []
        cv2.setMouseCallback('image', self.onMouseObs)
        cv2.waitKey()

    def show_system(self):
        cv2.circle(self.image, self.goal, 3, (0,0,255), -1)
        cv2.circle(self.image, self.target, self.target_r, (0,100,255), -1)

        for obs in self.obs:
            cv2.circle(self.image, obs[0:2], 3, (0,255,0), -1)

        for pusher in self.pushers:
            cv2.circle(self.image, pusher, 3, (255,0,0), -1)

        cv2.imshow('image', self.image)
        cv2.waitKey()

    def show_path(self, path, *args):
        cv2.circle(self.image, self.goal, 3, (0,0,255), -1)
        cv2.circle(self.image, self.target, self.target_r, (0,100,255), -1)

        for obs in self.obs:
            cv2.circle(self.image, obs[0:2], 3, (0,255,0), -1)

        for pusher in self.pushers:
            cv2.circle(self.image, pusher, 3, (255,0,0), -1)
        _xy = path[0]

        for idx in range(len(path) - 1):
            cv2.line(self.image,(int(_xy[0]), int(_xy[1])),(int(path[idx + 1][0]), int(path[idx + 1][1])),(0,0,255),3)
            _xy = path[idx + 1]

        for pusher_path in args:
            _xy = pusher_path[0]
            for idx in range(len(pusher_path) - 1):
                cv2.line(self.image,(int(_xy[0]), int(_xy[1])),(int(pusher_path[idx + 1][0]), int(pusher_path[idx + 1][1])),(255,0,0),1)
                _xy = pusher_path[idx + 1]

        cv2.imshow('image', self.image)
        cv2.waitKey()

    @property
    def map(self):
        return self.image.shape[0], self.image.shape[1]

    @staticmethod
    def color_extraction(image, lowerb, upperb):
        return cv2.inRange(image, lowerb, upperb)

    @staticmethod
    def hsv_segment(image):
        return cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    @staticmethod
    def segment_outlier(segment):
        mask = cv2.connectedComponents(segment)[1]
        mask_1D = mask.reshape(-1,)
        delet_mask_1D = mask_1D[np.where(mask_1D > 0)]
        n = mode(delet_mask_1D , keepdims=True)[0][0]
        mask = np.where(mask!=n,0,mask) / n
        return mask.reshape(segment.shape).astype(np.uint8)

if __name__ == "__main__":
    print("Get image from video")
    image = ImageManager("../../video/Pushing_bead_two_swimmers 1.mp4")
    image.get_image()
    image.get_target()
    image.get_pushers(2)
    image.set_goal()
    image.show_system()
