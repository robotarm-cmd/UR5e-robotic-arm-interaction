"""Render a real opaque MuJoCo frame with DH axes and origin labels."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont

folder=Path(__file__).resolve().parent / 'model'
out=Path(__file__).resolve().parent / 'renders'
out.mkdir(exist_ok=True)
scene=ET.parse(folder/'scene.xml')
root=scene.getroot()
root.find('include').set('file','ur5e_dh.xml')
root.find('./visual/global').set('offwidth','1400')
root.find('./visual/global').set('offheight','1100')
root.find('./asset/texture[@name="groundplane"]').set('rgb1','0.19 0.19 0.19')
root.find('./asset/texture[@name="groundplane"]').set('rgb2','0.13 0.13 0.13')
root.find('./asset/material[@name="groundplane"]').set('reflectance','0')
scene.write(folder/'scene_dh.xml',encoding='unicode')
model=mujoco.MjModel.from_xml_path(str(folder/'scene_dh.xml'))
data=mujoco.MjData(model)
q=[0,-90,90,-90,-90,0]
data.qpos[:]=np.deg2rad(q);mujoco.mj_forward(model,data)
Ts=[np.eye(4)]
for t,a,d,al in zip(data.qpos,[0,-.425,-.3922,0,0,0],[.1625,0,0,.1333,.0997,.0996],[np.pi/2,0,0,np.pi/2,-np.pi/2,0]):
    c,s,ca,sa=np.cos(t),np.sin(t),np.cos(al),np.sin(al)
    Ts.append(Ts[-1]@np.array([[c,-s*ca,s*sa,a*c],[s,c*ca,-c*sa,a*s],[0,sa,ca,d],[0,0,0,1]]))
cam=mujoco.MjvCamera()
cam.type=mujoco.mjtCamera.mjCAMERA_FREE
cam.lookat[:]=[-.18,-.08,.32]
cam.distance=1.65;cam.azimuth=35;cam.elevation=-22
opt=mujoco.MjvOption();opt.geomgroup[3]=0;opt.sitegroup[:]=0
w,h=1200,950
renderer=mujoco.Renderer(model,height=h,width=w)
renderer.update_scene(data,camera=cam,scene_option=opt)
colors=[np.array([1,.7,.1,1]),np.array([1,.28,.65,1]),np.array([.2,.85,.35,1])]
pixels=renderer.render()
image=Image.fromarray(pixels)
draw=ImageDraw.Draw(image)
fontpath='C:/Windows/Fonts/msyh.ttc'
font=ImageFont.truetype(fontpath,29)
small=ImageFont.truetype(fontpath,24)
glcam=renderer.scene.camera[0]
pos=np.mean([renderer.scene.camera[0].pos,renderer.scene.camera[1].pos],axis=0)
forward=np.array(glcam.forward);up=np.array(glcam.up);right=np.cross(forward,up)
def project(p):
    rel=p-pos;depth=np.dot(rel,forward)
    half=np.tan(np.deg2rad(model.vis.global_.fovy)/2)
    return (w/2+np.dot(rel,right)/depth/half*h/2,h/2-np.dot(rel,up)/depth/half*h/2)
def text(xy,s,fill=(255,255,255),f=font):
    draw.text(xy,s,font=f,fill=fill,stroke_width=3,stroke_fill=(20,20,20))
text((32,23),'UR5e · 不透明实体 + 标准 DH 坐标系')
text((32,69),'q = (0, −90, 90, −90, −90, 0)°',f=small)
boxes=[]
def label(s,p,fill=(255,255,255),offset=(15,-20)):
    bounds=draw.textbbox((0,0),s,font=small,stroke_width=3)
    tw,th=bounds[2]-bounds[0],bounds[3]-bounds[1]
    candidates=[offset,(20,-42),(-55,-43),(22,20),(-55,24),(0,-70),(0,50),(60,-15),(-80,-10),(50,50),(-85,50)]
    for dx,dy in candidates:
        x=np.clip(p[0]+dx,15,w-tw-15);y=np.clip(p[1]+dy,110,h-th-110)
        box=(x,y,x+tw,y+th)
        if not any(box[0]<b[2]+9 and box[2]+9>b[0] and box[1]<b[3]+9 and box[3]+9>b[1] for b in boxes):break
    boxes.append(box)
    if np.linalg.norm(np.array([x+tw/2,y+th/2])-p)>40:
        draw.line([p,(x+tw/2,y+th/2)],fill=(190,190,190),width=1)
    text((x,y),s,fill,f=small)
for f in [0,4]:
    p=Ts[f][:3,3]
    for k in range(3):
        start=np.array(project(p));end=np.array(project(p+Ts[f][:3,k]*.14))
        color=tuple(np.rint(colors[k][:3]*255).astype(int))
        v=end-start;length=np.linalg.norm(v)
        depth=np.dot(Ts[f][:3,k],-forward)
        if length>10:
            direction=v/length
            if depth<-.04:
                for t in np.arange(0,length-10,13):
                    draw.line([tuple(start+direction*t),tuple(start+direction*min(t+8,length-10))],fill=color,width=4)
            else:draw.line([tuple(start),tuple(end)],fill=color,width=4)
            side=np.array([-direction[1],direction[0]])
            draw.polygon([tuple(end),tuple(end-direction*15+side*6),tuple(end-direction*15-side*6)],fill=color)
        label('xyz'[k]+str(f),tuple(end),color,offset=(int(v[0]/max(length,1)*15)-10,int(v[1]/max(length,1)*15)-10))
for i,T in enumerate(Ts):
    p=project(T[:3,3]);offs={0:(15,14),1:(-58,0),2:(-25,-40),3:(12,-37),4:(-55,-33),5:(18,-3),6:(17,18)}[i]
    draw.ellipse((p[0]-4,p[1]-4,p[0]+4,p[1]+4),fill=(245,245,245),outline=(20,20,20),width=1)
    label('O'+str(i),p,offset=offs)
text((32,h-90),'x4 = −z0     y4 = −y0     z4 = −x0',f=small)
text((32,h-50),'虚线：朝屏幕里   ·   网格来源：MuJoCo Menagerie；骨架：UR 官方 DH',f=small)
path=out/'UR5e-MuJoCo-DH彩图.png'
image.save(path)
renderer.close()
print('Rendered',path,'size',image.size)
