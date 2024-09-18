import bpy 
import math 
from mathutils import Matrix
import numpy as np
from numpy import genfromtxt
### IMPORT OF 3D Models
#Import stl files and chage origin of each body to CG.
stl_names = ["Base","Jib_Concrete","Jib_Steel","Trolley","Load"]
GC_Models = [[0.3513, 1.3800, 24.9501],
             [-12.0440, 1.3862, 43.9635],
             [8.1592, 1.5052, 43.9357],
             [29.5160, 1.4246, 43.6045],
             [29.4569, 1.3992, 39.6491]]
for model_name,CG in zip(stl_names,GC_Models):
    #Previous version. STL addon required.
    #bpy.ops.import_mesh.stl(filepath=bpy.path.abspath("//stl//"+model_name+".stl"))
    #New version: Built feature to import STL models.
    #https://blender.stackexchange.com/questions/321366/correct-way-of-using-bpy-ops-import-mesh-stl-bpy-function
    bpy.ops.wm.stl_import(filepath=bpy.path.abspath("//stl//"+model_name+".stl"))
    bpy.context.scene.cursor.location = CG
    Body_Object = bpy.data.objects[model_name]
    Body_Object.select_set(True)
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
    Body_Object.select_set(False)
    
bpy.context.scene.cursor.location = [0,0,0]

Base_Obj = bpy.data.objects["Base"]
Trolley_Obj = bpy.data.objects["Trolley"]
Load_Obj = bpy.data.objects["Load"]

#Join the Jib Bodies
j_1 = bpy.data.objects["Jib_Concrete"]  # or whatever object you want
j_2 = bpy.data.objects["Jib_Steel"]
selected_objects = [
    j_1, 
    j_2
    ]
j_1.select_set(True)
j_2.select_set(True)
with bpy.context.temp_override(active_object=j_1, selected_objects=selected_objects):
    bpy.ops.object.join()
Jib_Obj = Jib_Steel_Obj = bpy.data.objects["Jib_Concrete"]
#Change Name to Jib and origin to CG.
Jib_Obj.name = 'Jib'
CG_Jib = [-1.9424, 1.4457, 43.9496]
Jib_Obj.select_set(True)
bpy.context.scene.cursor.location = CG_Jib
bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
Jib_Obj.select_set(False)

bpy.context.scene.cursor.location = [0,0,0]

### SIMULATION


T_Jib_sim = genfromtxt(bpy.path.abspath("//csv//Jib.csv"), delimiter=',')
T_Trolley_sim = genfromtxt(bpy.path.abspath("//csv//Trolley.csv"), delimiter=',')
T_Load_sim = genfromtxt(bpy.path.abspath("//csv//Load.csv"), delimiter=',')

T_Jib_sim_Tensor = np.zeros((4,4,int(len(T_Jib_sim)/4)))
T_Trolley_sim_Tensor = np.zeros((4,4,int(len(T_Trolley_sim)/4)))
T_Load_sim_Tensor = np.zeros((4,4,int(len(T_Load_sim)/4)))

for i in range(int(len(T_Jib_sim)/4)):
    T_Jib_sim_Tensor[:,:,i] = T_Jib_sim[4*i:4*i+4,0:4]
for i in range(int(len(T_Trolley_sim)/4)):
    T_Trolley_sim_Tensor[:,:,i] = T_Trolley_sim[4*i:4*i+4,0:4]
for i in range(int(len(T_Load_sim)/4)):
    T_Load_sim_Tensor[:,:,i] = T_Load_sim[4*i:4*i+4,0:4]
    
    
for nT in enumerate(np.rollaxis(T_Jib_sim_Tensor, 2)):
    n_frame = nT[0]+1
    Jib_Obj.matrix_world =  Matrix(nT[1])
    bpy.context.view_layer.update()
    Jib_Obj.keyframe_insert("location",frame=n_frame)
    Jib_Obj.keyframe_insert("rotation_euler", frame=n_frame)
    

for nT in enumerate(np.rollaxis(T_Trolley_sim_Tensor, 2)):
    n_frame = nT[0]+1
    Trolley_Obj.matrix_world = Matrix(nT[1])
    bpy.context.view_layer.update()
    Trolley_Obj.keyframe_insert("location",frame=n_frame)
    Trolley_Obj.keyframe_insert("rotation_euler", frame=n_frame)
    
for nT in enumerate(np.rollaxis(T_Load_sim_Tensor, 2)):
    n_frame = nT[0]+1
    Load_Obj.matrix_world = Matrix(nT[1])
    bpy.context.view_layer.update()
    Load_Obj.keyframe_insert("location",frame=n_frame)
    Load_Obj.keyframe_insert("rotation_euler", frame=n_frame)