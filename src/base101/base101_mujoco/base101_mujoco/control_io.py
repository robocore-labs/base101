"""ABI shared with base101_sim_control; only the Python process steps physics."""
import mmap
import struct
import time
import numpy as np

class Exchange:
    def __init__(self,path,model,data,joints):
        self.file=open(path,'w+b');self.file.truncate(1592)
        self.memory=mmap.mmap(self.file.fileno(),1592)
        self.memory[:]=bytes(1592)
        self.model=model;self.data=data;self.joints=joints;self.sequence=0
        self.hold={name:float(data.qpos[model.joint(name).qposadr[0]]) for name,kind in joints if kind=='position'}
        self.was_active=False
        self.update();struct.pack_into('<QI',self.memory,0,0x584C45524F424F54,len(joints))
    def update(self):
        self.sequence+=1;struct.pack_into('<Q',self.memory,16,self.sequence)
        for i,(name,_) in enumerate(self.joints):
            j=self.model.joint(name)
            struct.pack_into('<d',self.memory,56+8*i,self.data.qpos[j.qposadr[0]])
            struct.pack_into('<d',self.memory,568+8*i,self.data.qvel[j.dofadr[0]])
        struct.pack_into('<d',self.memory,32,time.time())
        self.sequence+=1;struct.pack_into('<Q',self.memory,16,self.sequence)
    def apply(self):
        active=False;commands=None
        for _ in range(10):
            seq=struct.unpack_from('<Q',self.memory,24)[0]
            if seq&1:continue
            age=time.time()-struct.unpack_from('<d',self.memory,40)[0]
            enabled=struct.unpack_from('<Q',self.memory,48)[0]
            commands=np.frombuffer(self.memory,dtype='<f8',count=len(self.joints),offset=1080).copy()
            if seq!=struct.unpack_from('<Q',self.memory,24)[0]:continue
            active=bool(seq and enabled and age<.5 and np.isfinite(commands).all());break
        if self.was_active and not active:
            for name in self.hold:self.hold[name]=float(self.data.qpos[self.model.joint(name).qposadr[0]])
        for i,(name,kind) in enumerate(self.joints):
            self.data.ctrl[self.model.actuator(name).id]=commands[i] if active else (0. if kind=='velocity' else self.hold[name])
        self.was_active=active
    def close(self):self.memory.close();self.file.close()
