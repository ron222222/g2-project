#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
16_product_detector_opencv.py
G2右手相机产品检测
检测黑色长条产品，输出中心点、角度、长宽
"""
import time
import cv2
import numpy as np
import agibot_gdk

MIN_AREA=3000

class ProductDetector:
    def __init__(self):
        self.camera=agibot_gdk.Camera()
        time.sleep(2)

    def detect(self,frame):
        gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        blur=cv2.GaussianBlur(gray,(5,5),0)
        _,th=cv2.threshold(blur,90,255,cv2.THRESH_BINARY_INV)
        kernel=np.ones((3,3),np.uint8)
        th=cv2.morphologyEx(th,cv2.MORPH_OPEN,kernel)
        cnts,_=cv2.findContours(th,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        best=None
        best_area=0
        for c in cnts:
            area=cv2.contourArea(c)
            if area<MIN_AREA:
                continue
            if area>best_area:
                best=c
                best_area=area
        if best is None:
            return frame,None
        rect=cv2.minAreaRect(best)
        (cx,cy),(w,h),angle=rect
        box=cv2.boxPoints(rect)
        box=np.int32(box)
        cv2.drawContours(frame,[box],0,(0,255,0),2)
        cv2.circle(frame,(int(cx),int(cy)),5,(0,0,255),-1)
        txt=f'Center=({int(cx)},{int(cy)}) Angle={angle:.1f}'
        cv2.putText(frame,txt,(20,40),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,255,0),2)
        return frame,{'cx':cx,'cy':cy,'angle':angle,'w':w,'h':h}

    def run(self):
        while True:
            image=self.camera.get_latest_image(agibot_gdk.CameraType.kHandRightColor,1000.0)
            if image is None:
                continue
            frame=cv2.imdecode(np.frombuffer(image.data,np.uint8),cv2.IMREAD_COLOR)
            if frame is None:
                continue
            frame,res=self.detect(frame)
            cv2.imshow('Product Detector',frame)
            key=cv2.waitKey(1)&0xFF
            if key==27:
                break
            if key==ord('s') and res:
                print(res)
        cv2.destroyAllWindows()

def main():
    if agibot_gdk.gdk_init()!=agibot_gdk.GDKRes.kSuccess:
        print('GDK初始化失败')
        return
    try:
        ProductDetector().run()
    finally:
        agibot_gdk.gdk_release()

if __name__=='__main__':
    main()
