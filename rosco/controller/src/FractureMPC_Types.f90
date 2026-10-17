! SPDX-License-Identifier: Apache-2.0
MODULE FractureMPC_Types
    USE Constants, ONLY: DbKi, IntKi
    IMPLICIT NONE
    INTEGER, PARAMETER :: FMPC_NX=6, FMPC_MAX_U=80
    TYPE :: FMPC_Parameters
        INTEGER(IntKi) :: Mode=0, Trigger=1, ExtendedMotion=1, Blade=1
        INTEGER(IntKi) :: NPred=40, NControl=10, Iterations=100
        REAL(DbKi) :: DT=0.05_DbKi, TriggerTime=9999, FractureS=0.3_DbKi
        REAL(DbKi) :: TowerMass=0, TowerDamping=0, TowerStiffness=0
        REAL(DbKi) :: PitchTau=0.25_DbKi, TorqueTau=0.25_DbKi, FilterTau=0.2_DbKi
        REAL(DbKi) :: Decel=0.02_DbKi, MinSpeed=0.104719755_DbKi, MaxDistance=3
        REAL(DbKi) :: Q(5)=[30.0_DbKi,40.0_DbKi,80.0_DbKi,0.1_DbKi,10.0_DbKi]
        REAL(DbKi) :: R(2)=[1.0_DbKi,60.0_DbKi]
        REAL(DbKi) :: StateScale(5)=[1.0_DbKi,5.0_DbKi,2.0_DbKi,1.0_DbKi,1.0_DbKi]
        REAL(DbKi) :: LookupScale(6)=[0.10472_DbKi,0.0872665_DbKi,2.0_DbKi,0.2_DbKi,1.0_DbKi,1.0_DbKi]
        CHARACTER(1024) :: TableFile='unused'
    END TYPE
    TYPE :: FMPC_Table
        REAL(DbKi), ALLOCATABLE :: Z(:,:), G(:,:)
        INTEGER :: Count=0, Blade=0
        REAL(DbKi) :: S=0, RatedRPM=0
    END TYPE
    TYPE :: FMPC_State
        LOGICAL :: Initialized=.FALSE., Active=.FALSE.
        REAL(DbKi) :: LastTime=-1, LastSolve=-1, StartTime=0, StartOmega=0
        REAL(DbKi) :: LastOmega=0, Alpha=0, TowerX=0, TowerV=0, TowerAcc=0
        REAL(DbKi) :: Command(2)=0, Applied(2)=0, LastPitch(3)=0, Inertia=0
        REAL(DbKi) :: Distance=0, LastQPResidual=0
        REAL(DbKi) :: AppliedPitchRate=0
        INTEGER :: Status=0, QPStatus=0, Solves=0, Fallbacks=0, TraceUnit=0
    END TYPE
END MODULE
