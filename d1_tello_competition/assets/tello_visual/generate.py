"""Original procedural display meshes, metres; Apache-2.0. No physics geometry."""
from pathlib import Path
import math
import xml.etree.ElementTree as E
import numpy as np
OUT=Path(__file__).resolve().parent

def mesh(name,verts,faces):
 v=np.asarray(verts);f=np.asarray(faces);n=np.zeros_like(v)
 fn=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
 for j in range(3):np.add.at(n,f[:,j],fn)
 n/=np.maximum(np.linalg.norm(n,axis=1,keepdims=True),1e-15)
 c=E.Element('COLLADA',xmlns='http://www.collada.org/2005/11/COLLADASchema',version='1.4.1');a=E.SubElement(c,'asset');E.SubElement(a,'unit',name='meter',meter='1');E.SubElement(a,'up_axis').text='Z_UP'
 m=E.SubElement(E.SubElement(E.SubElement(c,'library_geometries'),'geometry',id='surface'),'mesh')
 for key,arr in [('positions',v),('normals',n)]:
  s=E.SubElement(m,'source',id=key);E.SubElement(s,'float_array',id=key+'-array',count=str(arr.size)).text=' '.join('%.8g'%x for x in arr.ravel());acc=E.SubElement(E.SubElement(s,'technique_common'),'accessor',source='#'+key+'-array',count=str(len(arr)),stride='3')
  for axis in 'XYZ':E.SubElement(acc,'param',name=axis,type='float')
 E.SubElement(E.SubElement(m,'vertices',id='verts'),'input',semantic='POSITION',source='#positions');t=E.SubElement(m,'triangles',count=str(len(f)));E.SubElement(t,'input',semantic='VERTEX',source='#verts',offset='0');E.SubElement(t,'input',semantic='NORMAL',source='#normals',offset='1');E.SubElement(t,'p').text=' '.join(map(str,np.repeat(f.ravel(),2)))
 vs=E.SubElement(E.SubElement(c,'library_visual_scenes'),'visual_scene',id='Scene');E.SubElement(E.SubElement(vs,'node',id='part'),'instance_geometry',url='#surface');E.SubElement(E.SubElement(c,'scene'),'instance_visual_scene',url='#Scene');E.ElementTree(c).write(OUT/(name+'.dae'),encoding='utf-8',xml_declaration=True)

def loft(name,rings):
 v=[];f=[];N=96
 for x,y,r,z in rings:
  for corner in range(4):
   a=corner*math.pi/2;cx=(x-r)*(1 if corner in (0,3) else -1);cy=(y-r)*(1 if corner in (0,1) else -1)
   for j in range(24):
    t=a+j*math.pi/48;v.append((cx+r*math.cos(t),cy+r*math.sin(t),z))
 for k in range(len(rings)-1):
  for j in range(N):
   a=k*N+j;b=k*N+(j+1)%N;f.extend([(a,b,b+N),(a,b+N,a+N)])
 for k,reverse in [(0,True),(len(rings)-1,False)]:
  center=len(v);v.append((0,0,rings[k][3]))
  for j in range(N):
   tri=(center,k*N+j,k*N+(j+1)%N);f.append(tri[::-1] if reverse else tri)
 mesh(name,v,f)

loft('cover',[(.034,.021,.007,.009),(.036,.023,.008,.010),(.0365,.0235,.009,.012),(.036,.023,.009,.0145),(.0345,.0215,.009,.0165),(.031,.018,.008,.018),(.023,.012,.006,.0188),(.012,.006,.003,.019)])
loft('body',[(.028,.015,.006,-.0185),(.033,.0185,.007,-.017),(.0355,.0205,.007,-.013),(.0355,.0205,.007,.005),(.0345,.020,.007,.0095)])
# Continuous guard strips with rounded cross section, replacing 80 individual boxes.
for i,(x,y) in enumerate([(.0425,-.05),(-.0425,.05),(.0425,.05),(-.0425,-.05)]):
 v=[];f=[];center=math.atan2(y,x);N=96;M=12
 for j in range(N+1):
  a=center-2.042+j*4.084/N
  for k in range(M):
   b=k*2*math.pi/M;rr=.0445+.0015*math.cos(b);z=.018+.0045*math.sin(b)
   v.append((x+rr*math.cos(a),y+rr*math.sin(a),z))
 for j in range(N):
  for k in range(M):
   a=j*M+k;b=j*M+(k+1)%M;f.extend([(a,a+M,b+M),(a,b+M,b)])
 mesh('guard_'+str(i),v,f)
