"""Executable DLL contract tests; no claim of nonlinear OpenFAST validation."""
import ctypes as ct
import json
import re
import shutil
import tempfile
from pathlib import Path
import numpy as np
import pandas as pd

ROOT=Path(__file__).resolve().parents[2]
DLL=ROOT/'rosco/controller/build-fracture-mpc/Release/discon.dll'
BASE=ROOT/'rosco/lib/discon.dll'
DATA=Path(__file__).parent/'IEA10'


def replace(text,key,v):
    return re.sub(rf'^.*?(!\s*{re.escape(key)}\b.*)$',lambda m:f'{v} {m.group(1)}',text,flags=re.M)


class Driver:
    def __init__(self,dll,config):
        self.folder=Path(tempfile.mkdtemp(prefix='rosco_fmpc_'))
        shutil.copy2(dll,self.folder/'controller.dll')
        for p in DATA.iterdir():
            if p.is_file(): shutil.copy2(p,self.folder/p.name)
        (self.folder/'DISCON.IN').write_text(config)
        self.lib=ct.CDLL(str(self.folder/'controller.dll'))
        self.fun=self.lib.DISCON
        self.fun.argtypes=[ct.POINTER(ct.c_float),ct.POINTER(ct.c_int),ct.c_char_p,ct.c_char_p,ct.c_char_p]
        self.swap=np.zeros(3300,np.float32)
        self.fail=ct.c_int(0); self.msg=ct.create_string_buffer(2000)
        self.infile=str(self.folder/'DISCON.IN').encode();self.root=str(self.folder/'run').encode()
        self.started=False

    def step(self,t,omega=.8,beta=.30,wind=10,extent=3300,packet=0,torque=1e6,fracture_s=.3,
             drivetrain_inertia=1.558017e8):
        s=self.swap;s[0]=0 if not self.started else 1;s[1]=t;s[2]=.01
        s[3]=beta;s[32:34]=beta;s[19]=omega;s[20]=omega;s[22]=torque
        s[13]=omega*torque;s[14]=s[13]*.95;s[26]=wind
        s[48]=2000;s[49]=len(self.infile);s[50]=len(self.root)
        s[59]=.9+omega*t;s[60]=3;s[128]=extent
        s[1018]=0;s[1020]=0;s[1022]=1.52e8;s[1024]=fracture_s;s[1025]=.02;s[1026]=packet
        s[1027]=drivetrain_inertia
        self.fun(s.ctypes.data_as(ct.POINTER(ct.c_float)),ct.byref(self.fail),self.infile,self.root,self.msg)
        self.started=True
        if self.fail.value<0:raise RuntimeError(self.msg.value.decode(errors='replace'))
        return np.array([s[46],s[44]],float)

    def finish(self):
        self.swap[0]=-1
        self.fun(self.swap.ctypes.data_as(ct.POINTER(ct.c_float)),ct.byref(self.fail),self.infile,self.root,self.msg)


def main():
    # Independent constrained QP: unconstrained optimum (2,2), sum <= 1 -> (.5,.5).
    lib=ct.CDLL(str(DLL));qp=lib.fmpc_qp_solve
    pointer=ct.POINTER(ct.c_double)
    qp.argtypes=[ct.c_int,ct.c_int]+[pointer]*7+[ct.c_int,pointer,pointer]
    qp.restype=ct.c_int
    arrays=[np.asfortranarray(v,dtype=np.float64) for v in
            [2*np.eye(2),[-4,-4],[[1,1]],[0,0],[10,10],[-1e20],[1]]]
    sol=np.zeros(2);violation=ct.c_double()
    rc=qp(2,1,*[v.ctypes.data_as(pointer) for v in arrays],100,sol.ctypes.data_as(pointer),ct.byref(violation))
    assert rc==0 and violation.value<1e-6
    np.testing.assert_allclose(sol,[.5,.5],atol=1e-6)
    text=(DATA/'DISCON.IN').read_text()
    off=replace(text,'FMPC_Mode',0)
    on=replace(replace(text,'FMPC_Trigger',2),'FMPC_TriggerTime',.5)
    a=Driver(BASE,off);b=Driver(DLL,on)
    errors=[]
    for k in range(50):
        x=a.step(k*.01);y=b.step(k*.01);errors.append(float(np.max(abs(x-y))))
    assert max(errors)==0, max(errors)
    a.finish()
    # Same-timestamp calls must not advance MPC/actuator state.
    y0=b.step(.5);y1=b.step(.5);np.testing.assert_array_equal(y0,y1)
    outputs=[y0]
    for k in range(51,250):outputs.append(b.step(k*.01))
    b.finish();outputs=np.asarray(outputs)
    assert np.isfinite(outputs).all()
    assert np.all((outputs[:,1]>=0)&(outputs[:,1]<=1.5701))
    assert np.all(outputs[:,0]>=0)
    assert np.max(np.abs(np.diff(outputs[:,1])))<=.05236*.01+1e-6
    tr=pd.read_csv(b.folder/'run.FMPC.csv')
    assert (tr.status==1).any(),tr.status.value_counts().to_dict()
    assert (tr.loc[tr.status==1,'qp_status']==0).all()
    # At low speed leave the trained gain domain and monotonically feather/unload.
    c=Driver(DLL,on)
    for k in range(80):c.step(k*.01,omega=.08)
    c.finish();low=pd.read_csv(c.folder/'run.FMPC.csv');assert (low.status==5).all()
    # Out-of-domain lookup cannot silently extrapolate.
    d=Driver(DLL,on)
    for k in range(80):d.step(k*.01,wind=100)
    d.finish();far=pd.read_csv(d.folder/'run.FMPC.csv');assert (far.status==3).all()
    # Packet-mode healthy operation must not read extended slots when extent is short.
    e=Driver(DLL,text)
    for k in range(80):e.step(k*.01,extent=129,packet=2)
    e.finish();no_packet=pd.read_csv(e.folder/'run.FMPC.csv');assert no_packet.empty
    # Exercise staged fracture payload: ramp guard then MPC and latch despite status loss.
    f=Driver(DLL,text)
    for k in range(120):f.step(k*.01,packet=0 if k<50 else (1 if k<65 else (2 if k<100 else 0)))
    f.finish();pkt=pd.read_csv(f.folder/'run.FMPC.csv')
    assert (pkt.status==2).any() and (pkt.status==1).any() and (pkt.status==7).any(),pkt.status.value_counts()
    np.testing.assert_allclose(pkt.inertia_kgm2,1.558017e8,rtol=2e-7)
    invalid_inertia=Driver(DLL,text)
    for k in range(80):invalid_inertia.step(k*.01,packet=0 if k<50 else 2,drivetrain_inertia=3e8)
    invalid_inertia.finish();bad_inertia=pd.read_csv(invalid_inertia.folder/'run.FMPC.csv')
    assert (bad_inertia.status==7).all()
    limited=Driver(DLL,replace(on,'FMPC_Iterations',1))
    for k in range(80):limited.step(k*.01)
    limited.finish();failure=pd.read_csv(limited.folder/'run.FMPC.csv')
    assert (failure.status==4).any(),failure.status.value_counts()
    mismatch=Driver(DLL,text)
    for k in range(80):mismatch.step(k*.01,packet=0 if k<50 else 2,fracture_s=.4)
    mismatch.finish();bad=pd.read_csv(mismatch.folder/'run.FMPC.csv');assert (bad.status==7).all()
    sd=on
    settings={'SD_Mode':1,'SD_TimeActivate':0,'SD_EnablePitch':0,'SD_EnableYawError':0,
              'SD_EnableGenSpeed':0,'SD_EnableTime':1,'SD_MaxPit':1.5,'SD_PitchCornerFreq':1,
              'SD_MaxYawError':1,'SD_YawErrorCornerFreq':1,'SD_MaxGenSpd':2,'SD_GenSpdCornerFreq':1,
              'SD_Time':.4,'SD_Method':1,'SD_Stage_N':1,'SD_StageTime':1000,'SD_StagePitch':1.57,
              'SD_MaxTorqueRate':225000,'SD_MaxPitchRate':.0044}
    for key,v in settings.items():
        if re.search(rf'!\s*{key}\b',sd):sd=replace(sd,key,v)
        else:sd+=f'\n{v} ! {key}\n'
    sd_base=Driver(BASE,replace(sd,'FMPC_Mode',0));sd_mpc=Driver(DLL,sd)
    for k in range(100):np.testing.assert_array_equal(sd_base.step(k*.01),sd_mpc.step(k*.01))
    sd_base.finish();sd_mpc.finish()
    priority=pd.read_csv(sd_mpc.folder/'run.FMPC.csv');assert (priority.status==6).all()
    report={'healthy_command_max_difference':max(errors),'mpc_rows':int((tr.status==1).sum()),
            'mpc_statuses':{str(k):int(v) for k,v in tr.status.value_counts().items()},
            'duplicate_timestamp_exact':True,'rate_and_absolute_limits':True,
            'low_speed_fallback':True,'out_of_domain_fallback':True,'short_swap_guard':True,
            'fracture_ramp_final_and_latch':True,'runtime_inertia_update':True,
            'invalid_runtime_inertia_fallback':True,'qp_failure_fallback':True,
            'independent_qp_known_solution':True,'fault_metadata_mismatch_fallback':True,
            'native_shutdown_exact_priority':True,
            'trace_example':str(b.folder/'run.FMPC.csv')}
    (Path(__file__).parent/'dll_test_report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
