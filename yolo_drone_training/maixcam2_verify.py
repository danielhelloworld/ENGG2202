"""Run on MaixCAM2 with MaixPy; verify the converted drone model visually."""
from maix import camera,display,nn,app,image

MODEL='/root/models/drone_v2/best.mud'  # Keep the converter's actual filename.
detector=nn.YOLO11(model=MODEL,dual_buff=True)
cam=camera.Camera(detector.input_width(),detector.input_height(),detector.input_format())
screen=display.Display()
while not app.need_exit():
    frame=cam.read()
    objects=detector.detect(frame,conf_th=.25,iou_th=.45)
    for obj in objects:
        if detector.labels[obj.class_id]!='drone':continue
        frame.draw_rect(obj.x,obj.y,obj.w,obj.h,color=image.COLOR_RED)
        frame.draw_string(obj.x,obj.y,f'drone {obj.score:.2f}',color=image.COLOR_RED)
        # Normalized image coordinates for the existing teammate tracking/fusion
        # interface; confirm its axis signs and message format before connecting.
        cx=(obj.x+obj.w/2)/frame.width()
        cy=(obj.y+obj.h/2)/frame.height()
        print(f'DRONE,{cx:.6f},{cy:.6f},{obj.score:.4f}')
    screen.show(frame)
