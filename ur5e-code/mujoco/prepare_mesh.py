"""Preserve Menagerie visual meshes and align its skeleton to nominal UR DH."""
from pathlib import Path
import base64
import gzip
import json
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import trimesh

folder = Path(__file__).resolve().parent / 'model'
tree = ET.parse(folder / 'ur5e.xml')
root = tree.getroot()
changes = {'shoulder_link':'0 0 0.1625', 'wrist_1_link':'0 0 0.3922',
           'wrist_2_link':'0 0.1263 0', 'wrist_3_link':'0 0 0.0997'}
for name, value in changes.items():
    root.find(f'.//body[@name="{name}"]').set('pos',value)
root.find('.//site[@name="attachment_site"]').set('pos','0 0.0996 0')
tree.write(folder / 'ur5e_dh.xml',encoding='unicode')

model = mujoco.MjModel.from_xml_path(str(folder / 'ur5e_dh.xml'))
data = mujoco.MjData(model)
def fk(q):
    T = np.eye(4)
    result = [T.copy()]
    for t,a,d,al in zip(q,[0,-.425,-.3922,0,0,0],[.1625,0,0,.1333,.0997,.0996],[np.pi/2,0,0,np.pi/2,-np.pi/2,0]):
        c,s,ca,sa = np.cos(t),np.sin(t),np.cos(al),np.sin(al)
        T = T @ np.array([[c,-s*ca,s*sa,a*c],[s,c*ca,-c*sa,a*s],[0,sa,ca,d],[0,0,0,1]])
        result.append(T.copy())
    return result

# The attachment site must agree with the exact standard-DH frame 6.
site = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,'attachment_site')
body_frames = {'base':0,'shoulder_link':1,'upper_arm_link':2,'forearm_link':3,
               'wrist_1_link':4,'wrist_2_link':5,'wrist_3_link':6}
zeroTs = fk(np.zeros(6))
body_constants = {}
def body_matrix(i):
    B=np.eye(4); B[:3,:3]=data.xmat[i].reshape(3,3); B[:3,3]=data.xpos[i]
    return B
mujoco.mj_forward(model,data)
for name, frame in body_frames.items():
    i=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,name)
    body_constants[name]=np.linalg.inv(zeroTs[frame]) @ body_matrix(i)
rng=np.random.default_rng(9845)
max_error=0
for q in [np.zeros(6),np.deg2rad([30,-60,90,-30,45,60]),*rng.uniform(-np.pi,np.pi,(80,6))]:
    data.qpos[:]=q; mujoco.mj_forward(model,data)
    Ts=fk(q)
    actual=np.eye(4);actual[:3,:3]=data.site_xmat[site].reshape(3,3);actual[:3,3]=data.site_xpos[site]
    max_error=max(max_error,float(np.max(np.abs(actual-Ts[6]))))
    for name,frame in body_frames.items():
        i=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,name)
        max_error=max(max_error,float(np.max(np.abs(body_matrix(i)-Ts[frame] @ body_constants[name]))))
assert max_error<1e-10, max_error
data.qpos[:]=0;mujoco.mj_forward(model,data)

materials={e.attrib['name']:[float(x) for x in e.attrib['rgba'].split()] for e in root.findall('./asset/material')}
meshes=[]
original_faces=0
for body in root.findall('.//body'):
    name=body.attrib.get('name')
    if name not in body_frames:continue
    for geom in body.findall('./geom'):
        if geom.attrib.get('class')!='visual':continue
        mesh_name=geom.attrib['mesh']
        mesh=trimesh.load(folder / 'assets' / (mesh_name+'.obj'),force='mesh',process=True)
        mesh.merge_vertices(merge_tex=True,merge_norm=True)
        original_faces+=len(mesh.faces)
        # Keep curved silhouettes and mechanical details while fitting inline size.
        target=min(len(mesh.faces),max(400,int(len(mesh.faces)*.62)))
        if len(mesh.faces)>target:
            mesh=mesh.simplify_quadric_decimation(face_count=target,aggression=5)
        C=body_constants[name]
        verts=np.asarray(mesh.vertices) @ C[:3,:3].T + C[:3,3]
        low,high=verts.min(axis=0),verts.max(axis=0)
        quant=np.rint((verts-low)/np.maximum(high-low,1e-10)*65535).astype('<u2')
        indices=np.asarray(mesh.faces,dtype='<u2')
        assert len(verts)<65536
        meshes.append({'name':mesh_name,'frame':body_frames[name],
            'material':materials[geom.attrib['material']],
            'min':low.round(8).tolist(),'max':high.round(8).tolist(),
            'vertices':base64.b64encode(quant.tobytes()).decode(),
            'indices':base64.b64encode(indices.tobytes()).decode()})
out=Path(__file__).resolve().parents[1] / 'src' / 'mesh-data.json'
out.write_text(json.dumps(meshes,separators=(',',':')),encoding='utf-8')
compressed=gzip.compress(out.read_bytes(),compresslevel=9,mtime=0)
(out.parent/'ur5e-mesh-data.gz.b64').write_text(base64.b64encode(compressed).decode(),encoding='ascii')
print('Original faces:',original_faces,'reduced:',sum(len(base64.b64decode(m['indices']))//6 for m in meshes))
print('Mesh data bytes:',out.stat().st_size,'max FK/body error:',max_error)
print('Compressed base64 bytes:',len(base64.b64encode(compressed)))
(folder / 'kinematics-verification.json').write_text(json.dumps({'samples':82,'max_matrix_error':max_error,'changes':changes,'attachment_site_pos':'0 0.0996 0'},indent=2),encoding='utf-8')
