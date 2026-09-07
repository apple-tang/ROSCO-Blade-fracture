"""Isolated actual-fracture smoke run using the modified OpenFAST executable."""
import json
import re
import shutil
import subprocess
from pathlib import Path
import pandas as pd
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT.parent/'IEA-10.0-198-RWT-master/openfast_5MWProgram'
DEST=ROOT/'Examples/FractureMPC/simulation_test_out'


def set_label(text,key,val):
    pattern=rf'^.*?(\s{re.escape(key)}\s.*)$'
    text,n=re.subn(pattern,lambda m:f'{val:<22} {m.group(1).lstrip()}',text,count=1,flags=re.M)
    if n!=1:raise ValueError(key)
    return text


def main():
    DEST.mkdir(exist_ok=True)
    for src in SOURCE.iterdir():
        if src.is_dir() and src.name=='Airfoils':shutil.copytree(src,DEST/src.name,dirs_exist_ok=True)
        elif src.suffix.lower() in ['.dat','.fst','.in','.txt']:shutil.copy2(src,DEST/src.name)
    for src in (ROOT/'Examples/FractureMPC/IEA10').iterdir():
        if src.name in ['DISCON.IN','FMPC_Gains.txt']:shutil.copy2(src,DEST/src.name)
    ed=DEST/'IEA-10.0-198-RWT_ElastoDyn.dat'
    edtext=set_label(ed.read_text(),'NTwGages','1')
    edtext=set_label(edtext,'TwrGagNd','20')
    edtext=edtext.replace('"RotSpeed"','"RotSpeed"\n"TwHt1TPxi, TwHt1TPyi, TwrBsMyt"')
    ed.write_text(edtext)
    for name,prefix in [('IEA-10.0-198-RWT_ServoDyn.dat','servo')]:
        path=DEST/name;txt=path.read_text()
        txt=set_label(txt,'DLL_FileName','"'+str(ROOT/'rosco/controller/build-fracture-mpc/Release/discon.dll')+'"')
        txt=set_label(txt,'DLL_InFile','"DISCON.IN"');path.write_text(txt)
    fst=DEST/'IEA-10.0-198-RWT.fst';text=fst.read_text()
    for key,val in [('TMax','12'),('CompSeaSt','0'),('CompHydro','0'),('Linearize','False')]:text=set_label(text,key,val)
    block='''---------------------- BLADE FRACTURE ------------------------------------------
True                   BladeFracture
1                      BladeFrac_Blade
0.30                   BladeFrac_s
5.0                    BladeFrac_t
0.02                   BladeFrac_StiffReduc
2000                   BladeFrac_Dt_Ramp
3                      Ramp_Function
False                  EnergyFracture
'''
    text=text.replace('---------------------- ENVIRONMENTAL CONDITIONS',block+'---------------------- ENVIRONMENTAL CONDITIONS')
    fst.write_text(text)
    exe=ROOT.parent/'5MW_WindTurbine/OpenFAST_Release.exe'
    result=subprocess.run([str(exe),fst.name],cwd=DEST,capture_output=True,text=True,errors='replace',timeout=1800)
    log=result.stdout+result.stderr;(DEST/'run.log').write_text(log)
    summary={'normal_termination':'OpenFAST terminated normally' in log,'returncode':result.returncode,
             'fracture_time_s':5,'fracture_ramp_s':2,'tmax_s':12,'log':str(DEST/'run.log')}
    if summary['normal_termination']:
        trace=pd.read_csv(DEST/'IEA-10.0-198-RWT.FMPC.csv')
        assert np.isfinite(trace.to_numpy(float)).all()
        assert (trace.status==1).any() and (trace.status==2).any()
        assert trace.tower_x_m.abs().max()>1e-6
        assert trace.loc[trace.status==1,'qp_status'].eq(0).all()
        summary.update(mpc_solves=int((trace.status==1).sum()),ramp_guard_rows=int((trace.status==2).sum()),
                       first_activation_s=float(trace.time.min()),first_mpc_s=float(trace.loc[trace.status==1,'time'].min()),
                       maximum_qp_violation=float(trace.qp_violation.max()),
                       final_rotor_speed_rpm=float(trace.omega_radps.iloc[-1]*30/np.pi),
                       final_pitch_deg=float(trace.pitch_applied_rad.iloc[-1]*180/np.pi),
                       nonzero_tower_displacement=True)
    print(log[-5000:]);print(json.dumps(summary,indent=2))
    (ROOT/'Examples/FractureMPC/openfast_smoke_report.json').write_text(json.dumps(summary,indent=2))


if __name__=='__main__':main()
