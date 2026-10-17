"""Export a turbine-specific ROSCO FMPC case; no turbine constants live in the solver."""
import argparse
import json
import re
import shutil
from pathlib import Path
import numpy as np
import pandas as pd


def value(path, label):
    for line in path.read_text().splitlines():
        if line.lstrip().startswith(('!','#')):
            continue
        if re.search(rf'(?<![\w]){re.escape(label)}(?![\w])',line):
            try:
                return float(line.split()[0])
            except ValueError:
                pass
    raise ValueError(f'{label} missing in {path}')


def blade_mass(path, length, hub):
    lines=path.read_text().splitlines()
    begin=next(i for i,s in enumerate(lines) if 'DISTRIBUTED PROPERTIES' in s)+1
    stations=[]; densities=[]; i=begin
    while i<len(lines):
        if not lines[i].strip(): i+=1; continue
        span=float(lines[i].strip()); matrix=[]; i+=1
        while i<len(lines) and len(matrix)<12:
            if lines[i].strip(): matrix.append([float(x) for x in lines[i].split()])
            i+=1
        if len(matrix)!=12: raise ValueError('Incomplete BeamDyn matrix block')
        stations.append(span); densities.append(matrix[6][0])
    z=np.asarray(stations)*length; m=np.asarray(densities)
    return float(np.trapz(m,z)),float(np.trapz(m*(hub+z)**2,z))


def main():
    p=argparse.ArgumentParser()
    for key in ['gain-csv','base-discon','ed','tower','bd-healthy','bd-fractured','out']:
        p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--rated-rpm',type=float,required=True)
    p.add_argument('--blade',type=int,default=1)
    p.add_argument('--fracture-s',type=float,default=.3)
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    df=pd.read_csv(a.gain_csv)
    coords=['Omega_radps','beta_rad','wind_speed_mps','alpha_radps2','azimuth_sin','azimuth_cos']
    gains=[f'G_{load}_{axis}' for load in ['T','F'] for axis in ['Omega','beta','U','alpha','S','C']]
    numbers=df[coords+gains].to_numpy(float)
    if not len(numbers) or not np.isfinite(numbers).all(): raise ValueError('Invalid gain rows')
    if not np.allclose(np.sum(numbers[:,4:6]**2,axis=1),1,atol=1e-6): raise ValueError('Invalid phase coordinates')
    if not df.case_name.str.contains(f'-b{a.blade}-s{a.fracture_s:.2f}'.replace('.','p'),regex=False).all():
        raise ValueError('Gain table fault metadata mismatch')
    with (a.out/'FMPC_Gains.txt').open('w') as f:
        f.write(f'ROSCO_FMPC_TABLE_V1\n{len(numbers)} {a.blade} {a.fracture_s} {a.rated_rpm}\n')
        np.savetxt(f,numbers,fmt='%.16e')
    hub=value(a.ed,'HubRad');length=value(a.ed,'TipRad')-hub
    healthy_mass,healthy_j=blade_mass(a.bd_healthy,length,hub)
    broken_mass,broken_j=blade_mass(a.bd_fractured,length,hub)
    gear=value(a.ed,'GBRatio');gen=value(a.ed,'GenIner')
    rated_j=value(a.base_discon,'WE_Jtot')-gen*gear**2
    hub_j=value(a.ed,'HubIner')
    jrf=hub_j+(rated_j-hub_j)*(2*healthy_j+broken_j)/(3*healthy_j)
    top_mass=sum(value(a.ed,n) for n in ['HubMass','NacMass','YawBrMass'])+2*healthy_mass+broken_mass
    L=value(a.ed,'TowerHt')-value(a.ed,'TowerBsHt')
    props=[]
    for line in a.tower.read_text().splitlines():
        try:
            row=[float(x) for x in line.split()]
            if len(row)==4 and 0<=row[0]<=1: props.append(row)
        except ValueError: pass
    props=np.asarray(props);s=np.linspace(0,1,2001)
    coeff={n:value(a.tower,f'TwFAM1Sh({n})') for n in range(2,7)}
    phi=sum(v*s**n for n,v in coeff.items());dphi=sum(n*v*s**(n-1) for n,v in coeff.items())
    ddphi=sum(n*(n-1)*v*s**(n-2) for n,v in coeff.items())
    density=np.interp(s,props[:,0],props[:,1]);ei=np.interp(s,props[:,0],props[:,2])
    mass=float(top_mass*phi[-1]**2+L*np.trapz(density*phi**2,s))
    stiffness=float(np.trapz(ei*ddphi**2,s)/L**3)
    zeta=value(a.tower,'TwrFADmp(1)')/100
    damping=2*zeta*np.sqrt(mass*stiffness)
    cfg={'FMPC_Mode':1,'FMPC_Trigger':1,'FMPC_ExtendedMotion':1,'FMPC_TriggerTime':30,
         'FMPC_Blade':a.blade,'FMPC_FractureS':a.fracture_s,'FMPC_DT':.05,'FMPC_NPred':40,
         'FMPC_NControl':10,'FMPC_Iterations':200,'FMPC_TowerMass':mass,
         'FMPC_TowerDamping':damping,'FMPC_TowerStiffness':stiffness,'FMPC_PitchTau':.25,
         'FMPC_TorqueTau':.25,'FMPC_FilterTau':.2,'FMPC_Decel':.02,'FMPC_MinSpeed':np.pi/30,
         'FMPC_MaxDistance':3,'FMPC_TableFile':'"FMPC_Gains.txt"',
         'FMPC_Q':'30 40 80 0.1 10','FMPC_R':'1 60',
         'FMPC_StateScale':f'{a.rated_rpm*np.pi/30:.9g} 5 2 1 1.570796327',
         'FMPC_LookupScale':'0.104719755 0.087266463 2 0.2 1 1'}
    comments={
        'FMPC_Mode':'Fracture MPC mode {0: disabled, 1: enabled}',
        'FMPC_Trigger':'Activation source {1: modified OpenFAST fracture packet, 2: configured trigger time}',
        'FMPC_ExtendedMotion':'Tower feedback {0: leaky acceleration observer, 1: packet displacement/velocity}',
        'FMPC_TriggerTime':'Timed-test activation time [s], used only when FMPC_Trigger = 2',
        'FMPC_Blade':'Fractured blade ID {1, 2, 3}; must match the gain-table metadata',
        'FMPC_FractureS':'Nondimensional span where the fractured segment begins [-]',
        'FMPC_DT':'MPC optimization update period [s]',
        'FMPC_NPred':'Number of prediction intervals [-]',
        'FMPC_NControl':'Number of optimized control intervals [-]',
        'FMPC_Iterations':'Maximum qpOASES working-set iterations [-]',
        'FMPC_TowerMass':'Effective first fore-aft tower-mode mass [kg]',
        'FMPC_TowerDamping':'Effective first fore-aft tower-mode damping [N s/m]',
        'FMPC_TowerStiffness':'Effective first fore-aft tower-mode stiffness [N/m]',
        'FMPC_PitchTau':'Post-fracture first-order pitch-actuator time constant [s]',
        'FMPC_TorqueTau':'Post-fracture first-order torque-actuator time constant [s]',
        'FMPC_FilterTau':'Rotor/tower acceleration low-pass filter time constant [s]',
        'FMPC_Decel':'Post-fracture rotor-speed reference deceleration [rad/s^2]',
        'FMPC_MinSpeed':'Minimum rotor speed for MPC operation [rad/s]',
        'FMPC_MaxDistance':'Maximum normalized gain-table lookup distance [-]',
        'FMPC_TableFile':'Gain-table path, relative to DISCON.IN unless absolute [-]',
        'FMPC_Q':'State weights [Omega, tower_x, tower_v, normalized_torque, pitch]',
        'FMPC_R':'Command regularization weights [normalized_torque, pitch]',
        'FMPC_StateScale':'State scales [Omega(rad/s), tower_x(m), tower_v(m/s), torque(-), pitch(rad)]',
        'FMPC_LookupScale':'Lookup scales [Omega(rad/s), beta(rad), U(m/s), alpha(rad/s^2), sin(psi), cos(psi)]',
    }
    text=a.base_discon.read_text()
    text+='\n!------- FRACTURE MPC --------------------------------------------------\n'
    text+=''.join(f'{v!s:<35} ! {k:<22} - {comments[k]}\n' for k,v in cfg.items())
    (a.out/'DISCON.IN').write_text('\n'.join(line.rstrip() for line in text.splitlines()).rstrip()+'\n')
    perf=re.search(r'"([^"]+)"\s*!\s*PerfFileName',text)
    if perf:
        src=a.base_discon.parent/perf.group(1)
        (a.out/src.name).write_text('\n'.join(line.rstrip() for line in src.read_text().splitlines()).rstrip()+'\n')
    report={'gain_source':str(a.gain_csv.resolve()),'rows':len(numbers),'rated_rpm':a.rated_rpm,
            'ed_source':str(a.ed.resolve()),'derived_fractured_rotor_inertia_reference':jrf,
            'derived_fractured_drivetrain_inertia_reference':jrf+gen*gear**2,'tower_modal_mass':mass,
            'tower_modal_stiffness':stiffness,'tower_modal_damping':damping,
            'inertia_source':'Runtime total equivalent drivetrain inertia is read from modified OpenFAST avrSWAP(1028); derived values are provenance only.',
            'approximation':'Rigid-base first-tower-mode Rayleigh estimate; no SubDyn/soil or gravity-stiffening calibration.'}
    (a.out/'turbine_provenance.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__': main()
