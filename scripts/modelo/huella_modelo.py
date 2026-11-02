#!/usr/bin/env python3
"""Mide la huella del robot con las mallas del modelo MuJoCo y comprueba la de nav2.yaml.
Uso: python3 -I scripts/modelo/huella_modelo.py <carpeta del modelo, p. ej. ~/Downloads/modelo_robot_29oct>
Coordenadas de base_link: x adelante, y izquierda, z arriba, origen en el eje de las ruedas. No mueve nada."""
import math
import os
import re
import struct
import sys

import xml.etree.ElementTree as ET
import numpy as np
d=os.path.expanduser(sys.argv[1])
r=ET.parse(os.path.join(d,'robot_corregido.xml')).getroot()
meshes={os.path.splitext(m.get('file'))[0]:os.path.join(d,'assets',m.get('file')) for m in r.find('asset').findall('mesh')}
def stl(p):
    b=open(p,'rb').read(); n=struct.unpack('<I',b[80:84])[0]
    a=np.frombuffer(b,dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]),offset=84,count=n)
    return a['v'].reshape(-1,3,3).astype(float)
def Rq(q):
    w,x,y,z=np.array(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])
def pose(e):
    return np.array([float(x) for x in (e.get('pos') or '0 0 0').split()]),Rq([float(x) for x in (e.get('quat') or '1 0 0 0').split()])
def collect(q):
    out=[]
    def walk(b,P,R):
        p,Q=pose(b); P2=P+R@p; R2=R@Q
        for j in b.findall('joint'):
            n=j.get('name')
            if n in q:
                ax=np.array([float(x) for x in j.get('axis').split()])
                if j.get('type')=='slide': P2=P2+R2@(ax*q[n])
                else:
                    t=q[n]/2; R2=R2@Rq([math.cos(t),*(ax*math.sin(t))])
        for g in b.findall('geom'):
            if g.get('mesh') and g.get('class')=='visual':
                gp,gq=pose(g); v=stl(meshes[g.get('mesh')])
                out.append((b.get('name'),g.get('mesh'),(R2@((gq@v.reshape(-1,3).T).T+gp).T).T+P2))
        for c in b.findall('body'): walk(c,P2,R2)
    for b in r.find('worldbody').findall('body'): walk(b,np.zeros(3),np.eye(3))
    return out
ay=-0.00325611; az=0.01125; cx=(0.130218-0.130182)/2
def tob(w): return np.c_[-(w[:,1]-ay),w[:,0]-cx,w[:,2]-az]
def sample(t,h=0.003):
    t=t.reshape(-1,3,3); pts=[t.reshape(-1,3)]
    e=np.maximum(np.linalg.norm(t[:,1]-t[:,0],axis=1),np.linalg.norm(t[:,2]-t[:,0],axis=1))
    for tri,l in zip(t[e>h],e[e>h]):
        n=int(l/h)+1; u,v=np.meshgrid(np.linspace(0,1,n+1),np.linspace(0,1,n+1)); m=u+v<=1
        pts.append(tri[0]+np.outer(u[m],tri[1]-tri[0])+np.outer(v[m],tri[2]-tri[0]))
    return np.vstack(pts)
def env(P,lab):
    a=np.degrees(np.arctan2(P[:,1],P[:,0])); rr=np.hypot(P[:,0],P[:,1])
    print(lab,'bbox',P.min(0).round(3),P.max(0).round(3),'R',rr.max().round(3))
    print(' polar',' '.join(f'{x}:{rr[(a>=x-7.5)&(a<x+7.5)].max():.3f}' for x in range(-180,180,30)))
POSES={'cerrado':{'dedo_a':-0.2,'dedo_b':0.2},'neutro':{'dedo_a':0,'dedo_b':0},'abierto':{'dedo_a':1,'dedo_b':-1}}
def inside(P,x,y):
    d=np.zeros(len(x),bool); n=len(P)
    for i in range(n):
        x1,y1=P[i];x2,y2=P[(i+1)%n]
        c=((y1>y)!=(y2>y))&(x<x1+(y-y1)*(x2-x1)/(y2-y1+1e-12))
        d^=c
    return d
def edge(P,x,y):
    b=np.full(len(x),9.0);n=len(P)
    for i in range(n):
        x1,y1=P[i];x2,y2=P[(i+1)%n];dx,dy=x2-x1,y2-y1
        t=np.clip(((x-x1)*dx+(y-y1)*dy)/(dx*dx+dy*dy),0,1)
        b=np.minimum(b,np.hypot(x-x1-t*dx,y-y1-t*dy))
    return b
raiz=os.path.join(os.path.dirname(os.path.abspath(__file__)),'..','..')
txt=open(os.path.join(raiz,'ros/wallens_nav/config/nav2.yaml'),encoding='utf8').read()
P=[(float(x),float(y)) for x,y in re.findall(r'\[(-?[\d.]+), (-?[\d.]+)\]',re.search(r'footprint: "(.*)"',txt).group(1))]
for lab,q in POSES.items():
    A=sample(np.vstack([tob(w).reshape(-1,3) for n,m,w in collect(q)]))
    env(A,lab)
    out=A[~inside(P,A[:,0],A[:,1])]
    print(f' fuera de la huella de nav2.yaml (sin padding): {edge(P,out[:,0],out[:,1]).max()*1000:.1f} mm' if len(out) else ' dentro de la huella')
