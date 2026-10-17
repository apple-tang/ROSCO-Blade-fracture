! SPDX-License-Identifier: Apache-2.0
MODULE FractureMPC
    USE FractureMPC_Types
    USE FractureMPC_Table
    USE FractureMPC_Model
    USE ROSCO_Types, ONLY: ControlParameters,LocalVariables,ErrorVariables
    USE ROSCO_Helpers, ONLY: ParseInput,ParseAry,FindLine,MaxLineLength,GetPath
    USE Constants, ONLY: PI,ReKi
    USE, INTRINSIC :: IEEE_ARITHMETIC
    IMPLICIT NONE
CONTAINS
    SUBROUTINE FMPC_Init(P,T,S,V,C,Root,E)
        TYPE(FMPC_Parameters), INTENT(INOUT) :: P
        TYPE(FMPC_Table), INTENT(INOUT) :: T
        TYPE(FMPC_State), INTENT(INOUT) :: S
        TYPE(LocalVariables), INTENT(IN) :: V
        TYPE(ControlParameters), INTENT(IN) :: C
        CHARACTER(*), INTENT(IN) :: Root
        TYPE(ErrorVariables), INTENT(INOUT) :: E
        CHARACTER(1024) :: File,Directory
        CHARACTER(MaxLineLength), ALLOCATABLE :: Lines(:)
        CHARACTER(MaxLineLength) :: Line
        REAL(DbKi), ALLOCATABLE :: Values(:)
        INTEGER :: I,U,IOS,N,LineNumber
        LOGICAL :: Found
        File=''
        DO I=1,MIN(V%ACC_INFILE_SIZE,1023)
            IF (IACHAR(V%ACC_INFILE(I))==0) EXIT
            File(I:I)=V%ACC_INFILE(I)
        ENDDO
        OPEN(NEWUNIT=U,FILE=TRIM(File),STATUS='OLD',ACTION='READ',IOSTAT=IOS)
        IF (IOS/=0) THEN
            E%aviFAIL=-1; E%ErrMsg='FMPC: cannot read DISCON input'; RETURN
        ENDIF
        N=0
        DO
            READ(U,'(A)',IOSTAT=IOS) Line
            IF (IOS/=0) EXIT
            N=N+1
        ENDDO
        ALLOCATE(Lines(N)); REWIND(U)
        DO I=1,N
            READ(U,'(A)') Lines(I)
        ENDDO
        CLOSE(U)
        CALL FindLine(Lines,'FMPC_Mode',Found,Line,LineNumber)
        IF (.NOT.Found) RETURN
        CALL ParseInput(Lines,'FMPC_Mode',P%Mode,File,E,.FALSE.)
        IF (P%Mode==0 .OR. E%aviFAIL<0) RETURN
        CALL ParseInput(Lines,'FMPC_Trigger',P%Trigger,File,E,.FALSE.)
        CALL ParseInput(Lines,'FMPC_ExtendedMotion',P%ExtendedMotion,File,E,.FALSE.)
        CALL ParseInput(Lines,'FMPC_TriggerTime',P%TriggerTime,File,E,.FALSE.)
        CALL ParseInput(Lines,'FMPC_Blade',P%Blade,File,E,.FALSE.)
        CALL ParseInput(Lines,'FMPC_FractureS',P%FractureS,File,E,.FALSE.)
        CALL ParseInput(Lines,'FMPC_DT',P%DT,File,E,.FALSE.)
        CALL ParseInput(Lines,'FMPC_NPred',P%NPred,File,E,.FALSE.)
        CALL ParseInput(Lines,'FMPC_NControl',P%NControl,File,E,.FALSE.)
        CALL ParseInput(Lines,'FMPC_Iterations',P%Iterations,File,E,.FALSE.)
        CALL ParseInput(Lines,'FMPC_TowerMass',P%TowerMass,File,E,.FALSE.)
        CALL ParseInput(Lines,'FMPC_TowerDamping',P%TowerDamping,File,E,.FALSE.)
        CALL ParseInput(Lines,'FMPC_TowerStiffness',P%TowerStiffness,File,E,.FALSE.)
        CALL ParseInput(Lines,'FMPC_PitchTau',P%PitchTau,File,E,.FALSE.)
        CALL ParseInput(Lines,'FMPC_TorqueTau',P%TorqueTau,File,E,.FALSE.)
        CALL ParseInput(Lines,'FMPC_FilterTau',P%FilterTau,File,E,.FALSE.)
        CALL ParseInput(Lines,'FMPC_Decel',P%Decel,File,E,.FALSE.)
        CALL ParseInput(Lines,'FMPC_MinSpeed',P%MinSpeed,File,E,.FALSE.)
        CALL ParseInput(Lines,'FMPC_MaxDistance',P%MaxDistance,File,E,.FALSE.)
        CALL ParseInput(Lines,'FMPC_TableFile',P%TableFile,File,E,.FALSE.)
        CALL ParseAry(Lines,'FMPC_Q',Values,5,File,E,.FALSE.); IF (E%aviFAIL>=0) P%Q=Values
        IF (ALLOCATED(Values)) DEALLOCATE(Values)
        CALL ParseAry(Lines,'FMPC_R',Values,2,File,E,.FALSE.); IF (E%aviFAIL>=0) P%R=Values
        IF (ALLOCATED(Values)) DEALLOCATE(Values)
        CALL ParseAry(Lines,'FMPC_StateScale',Values,5,File,E,.FALSE.); IF (E%aviFAIL>=0) P%StateScale=Values
        IF (ALLOCATED(Values)) DEALLOCATE(Values)
        CALL ParseAry(Lines,'FMPC_LookupScale',Values,6,File,E,.FALSE.); IF (E%aviFAIL>=0) P%LookupScale=Values
        IF (E%aviFAIL<0) RETURN
        IF (.NOT.ALL(IEEE_IS_FINITE([P%DT,P%TriggerTime,P%FractureS,C%WE_Jtot, &
            C%WE_GearboxRatio,P%TowerMass,P%TowerDamping,P%TowerStiffness,P%PitchTau,P%TorqueTau,P%FilterTau, &
            P%Decel,P%MinSpeed,P%MaxDistance])) .OR. .NOT.ALL(IEEE_IS_FINITE(P%Q)) .OR. &
            .NOT.ALL(IEEE_IS_FINITE(P%R)) .OR. .NOT.ALL(IEEE_IS_FINITE(P%StateScale)) .OR. &
            .NOT.ALL(IEEE_IS_FINITE(P%LookupScale))) THEN
            E%aviFAIL=-1; E%ErrMsg='FMPC: parameters must be finite'; RETURN
        ENDIF
        IF (P%Mode/=1 .OR. P%Trigger<1 .OR. P%Trigger>2 .OR. P%Blade<1 .OR. P%Blade>3 .OR. &
            P%ExtendedMotion<0 .OR. P%ExtendedMotion>1 .OR. P%DT<V%DT .OR. P%DT>0.2_DbKi .OR. &
            P%NControl<1 .OR. 2*P%NControl>FMPC_MAX_U .OR. P%NPred<P%NControl .OR. P%NPred>160 .OR. &
            P%Iterations<1 .OR. C%WE_GearboxRatio<1 .OR. &
            P%TowerMass<=0 .OR. P%TowerStiffness<=0 .OR. P%TowerDamping<0 .OR. &
            P%PitchTau<P%DT .OR. P%TorqueTau<P%DT .OR. P%FilterTau<=0 .OR. P%Decel<=0 .OR. &
            P%MinSpeed<=0 .OR. P%MaxDistance<=0 .OR. ANY(P%StateScale<=0) .OR. ANY(P%LookupScale<=0) .OR. &
            ANY(P%Q<0) .OR. ANY(P%R<=0) .OR. C%WE_Jtot<=0 .OR. C%VS_MaxTq<=0 .OR. &
            C%VS_MaxRat<=0 .OR. C%PC_MaxRat<=0) THEN
            E%aviFAIL=-1; E%ErrMsg='FMPC: invalid mode, physical parameters, horizon, sample time or limits'; RETURN
        ENDIF
        CALL GetPath(File,Directory)
        IF (P%TableFile(1:1)/='/' .AND. P%TableFile(2:2)/=':') P%TableFile=TRIM(Directory)//TRIM(P%TableFile)
        CALL FMPC_LoadTable(P,T,E)
        IF (E%aviFAIL<0) RETURN
        ! ROSCO already owns the intact equivalent drivetrain inertia. The
        ! modified OpenFAST packet replaces its rotor component after fracture.
        S%Inertia=C%WE_Jtot
        S%LastOmega=V%RotSpeed; S%Initialized=.TRUE.
        OPEN(NEWUNIT=S%TraceUnit,FILE=TRIM(Root)//'.FMPC.csv',STATUS='REPLACE',ACTION='WRITE',IOSTAT=IOS)
        IF (IOS/=0) THEN
            S%TraceUnit=0; E%aviFAIL=-1; E%ErrMsg='FMPC: cannot open controller trace'; RETURN
        ENDIF
        WRITE(S%TraceUnit,'(A)') 'time,status,qp_status,omega_radps,alpha_radps2,wind_mps,pitch_rad,tower_x_m,tower_v_mps,tq_command_Nm,pitch_command_rad,tq_applied_Nm,pitch_applied_rad,lookup_distance,qp_violation,inertia_kgm2'
    END SUBROUTINE

    SUBROUTINE FMPC_WriteCommands(S,V,Swap)
        TYPE(FMPC_State), INTENT(IN) :: S
        TYPE(LocalVariables), INTENT(INOUT) :: V
        REAL(ReKi), INTENT(INOUT) :: Swap(*)
        V%PitCom=S%Applied(2); V%PitComAct=S%Applied(2)
        V%PC_PitComT=S%Applied(2); V%PC_PitComT_Last=S%Applied(2)
        V%GenTq=S%Applied(1); V%VS_LastGenTrq=S%Applied(1)
        Swap(42:45)=REAL(S%Applied(2),ReKi); Swap(47)=REAL(S%Applied(1),ReKi)
        Swap(46)=REAL(S%AppliedPitchRate,ReKi)
        Swap(55)=0; Swap(56)=0
    END SUBROUTINE

    SUBROUTINE FMPC_Step(P,T,S,C,V,Swap,E)
        TYPE(FMPC_Parameters), INTENT(IN) :: P
        TYPE(FMPC_Table), INTENT(IN) :: T
        TYPE(FMPC_State), INTENT(INOUT) :: S
        TYPE(ControlParameters), INTENT(IN) :: C
        TYPE(LocalVariables), INTENT(INOUT) :: V
        REAL(ReKi), INTENT(INOUT) :: Swap(*)
        TYPE(ErrorVariables), INTENT(INOUT) :: E
        REAL(DbKi) :: Dt,Pitch,Wind,Omega,Z(6),G(12),X0(6),AFilter,NewApplied(2),Limits(2),PreviousPitch,NewCommand(2),QPResidual
        REAL(DbKi) :: PacketDrivetrainInertia
        INTEGER :: Extent,FaultStatus,QPCode
        LOGICAL :: Triggered,OK,PacketOK,Emergency,LogNow
        IF (P%Mode==0 .OR. .NOT.S%Initialized) RETURN
        IF (V%iStatus==-1) THEN
            IF (S%TraceUnit/=0) CLOSE(S%TraceUnit)
            S%TraceUnit=0; RETURN
        ENDIF
        IF (V%iStatus<=-8) THEN
            E%aviFAIL=-1; E%ErrMsg='FMPC: restart/checkpoint is not supported while FMPC is enabled'; RETURN
        ENDIF
        IF (V%Time<S%LastTime-1e-7_DbKi) THEN
            E%aviFAIL=-1; E%ErrMsg='FMPC: time moved backwards without a supported restart'; RETURN
        ENDIF
        IF (V%Time<=S%LastTime) THEN
            IF (S%Active) CALL FMPC_WriteCommands(S,V,Swap)
            RETURN
        ENDIF
        Dt=V%DT
        IF (S%LastTime>=0) Dt=V%Time-S%LastTime
        Omega=V%RotSpeed; Pitch=(REAL(Swap(4),DbKi)+REAL(Swap(33),DbKi)+REAL(Swap(34),DbKi))/3
        Wind=V%HorWindV_F
        IF (.NOT.IEEE_IS_FINITE(Wind) .OR. Wind<=0) Wind=V%HorWindV
        IF (.NOT.ALL(IEEE_IS_FINITE([Omega,Pitch,Wind,V%FA_Acc_TT])) .OR. Dt<=0) THEN
            E%aviFAIL=-1; E%ErrMsg='FMPC: invalid measured signals or elapsed time'; RETURN
        ENDIF
        AFilter=1-EXP(-Dt/P%FilterTau)
        IF (S%LastTime>=0) S%Alpha=S%Alpha+AFilter*((Omega-S%LastOmega)/Dt-S%Alpha)
        S%TowerAcc=S%TowerAcc+AFilter*(V%FA_Acc_TT-S%TowerAcc)
        S%LastOmega=Omega; S%LastTime=V%Time
        ! Record 129 advertises the allocated extent in the OpenFAST Bladed interface.
        Extent=0
        IF (IEEE_IS_FINITE(REAL(Swap(129),DbKi))) Extent=NINT(Swap(129))
        PacketOK=.FALSE.; FaultStatus=0; PacketDrivetrainInertia=0
        IF (Extent>=1028) THEN
            IF (IEEE_IS_FINITE(REAL(Swap(1027),DbKi))) FaultStatus=NINT(Swap(1027))
            PacketDrivetrainInertia=REAL(Swap(1028),DbKi)
            PacketOK=FaultStatus>=1 .AND. FaultStatus<=2 .AND. IEEE_IS_FINITE(PacketDrivetrainInertia) &
                .AND. PacketDrivetrainInertia>0.01_DbKi*C%WE_Jtot &
                .AND. PacketDrivetrainInertia<=1.05_DbKi*C%WE_Jtot &
                .AND. ABS(Swap(1025)-P%FractureS)<1e-4_DbKi
        ENDIF
        Triggered=.FALSE.
        IF (P%Trigger==1) Triggered=FaultStatus>=1 .AND. FaultStatus<=2
        IF (P%Trigger==2) Triggered=V%Time>=P%TriggerTime
        IF (P%ExtendedMotion==1 .AND. Extent>=1027) THEN
            S%TowerX=Swap(1021); S%TowerV=Swap(1019)
        ELSE
            ! Bounded, leaky acceleration observer; extended displacement/velocity are preferred.
            S%TowerV=EXP(-0.2_DbKi*Dt)*S%TowerV+Dt*S%TowerAcc
            S%TowerX=EXP(-0.1_DbKi*Dt)*S%TowerX+Dt*S%TowerV
        ENDIF
        IF (.NOT.ALL(IEEE_IS_FINITE([S%TowerX,S%TowerV]))) THEN
            E%aviFAIL=-1; E%ErrMsg='FMPC: invalid tower feedback'; RETURN
        ENDIF
        IF (.NOT.S%Active) THEN
            S%Command=[REAL(Swap(47),DbKi),REAL(Swap(45),DbKi)]
            S%Applied=[REAL(Swap(23),DbKi),Pitch]
            IF (.NOT.Triggered) RETURN
            S%Active=.TRUE.; S%StartTime=V%Time; S%StartOmega=Omega
            S%Command=S%Applied; S%LastSolve=-1
        ENDIF
        IF (PacketOK) S%Inertia=PacketDrivetrainInertia
        Emergency=.FALSE.
        IF (C%SD_Mode>0) Emergency=V%SD_Trigger>0
        LogNow=.FALSE.
        IF (S%LastSolve<0 .OR. V%Time-S%LastSolve>=P%DT-1e-7_DbKi) THEN
            S%LastSolve=V%Time; LogNow=.TRUE.; S%QPStatus=0; S%LastQPResidual=0
            Z=[Omega,Pitch,Wind,S%Alpha,SIN(V%Azimuth+(P%Blade-1)*2*PI/3),COS(V%Azimuth+(P%Blade-1)*2*PI/3)]
            CALL FMPC_Lookup(P,T,Z,G,S%Distance,OK)
            S%Status=1
            IF (.NOT.OK) S%Status=3
            IF (P%Trigger==1 .AND. .NOT.PacketOK) S%Status=7
            IF (P%Trigger==1 .AND. FaultStatus==1) S%Status=2
            IF (P%ExtendedMotion==1 .AND. Extent<1027) S%Status=7
            IF (Omega<=P%MinSpeed) S%Status=5
            IF (Emergency) S%Status=6
            IF (S%Status==1) THEN
                X0=[Omega,S%TowerX,S%TowerV,REAL(Swap(23),DbKi)/C%VS_MaxTq,Pitch,0.0_DbKi]
                CALL FMPC_Optimize(P,S,G,Z,X0,V%Time,C%VS_MaxTq,C%WE_GearboxRatio,C%VS_MaxRat, &
                                   C%PC_MinPit,C%PC_MaxPit,C%PC_MaxRat, &
                                   NewCommand,QPCode,QPResidual)
                S%QPStatus=QPCode; S%LastQPResidual=QPResidual
                IF (S%QPStatus==0) S%Command=NewCommand
                S%Solves=S%Solves+1
                IF (S%QPStatus/=0) S%Status=4
            ENDIF
            IF (S%Status/=1 .AND. .NOT.Emergency) THEN
                S%Fallbacks=S%Fallbacks+1
                S%Command=[MAX(0.0_DbKi,S%Command(1)-C%VS_MaxRat*P%DT), &
                    MIN(C%PC_MaxPit,S%Command(2)+C%PC_MaxRat*P%DT)]
            ENDIF
        ENDIF
        PreviousPitch=S%Applied(2)
        IF (Emergency) THEN
            ! Native ROSCO shutdown owns the commands and actuator handling here.
            S%Command=[V%GenTq,V%PitCom(1)]
            S%Applied=[REAL(Swap(47),DbKi),REAL(Swap(45),DbKi)]
            S%AppliedPitchRate=(S%Applied(2)-PreviousPitch)/Dt
        ELSE
            NewApplied(1)=S%Applied(1)+(1-EXP(-Dt/P%TorqueTau))*(S%Command(1)-S%Applied(1))
            NewApplied(2)=S%Applied(2)+(1-EXP(-Dt/P%PitchTau))*(S%Command(2)-S%Applied(2))
            Limits=[C%VS_MaxRat,C%PC_MaxRat]*Dt
            S%Applied=S%Applied+MAX(-Limits,MIN(Limits,NewApplied-S%Applied))
            S%Applied(1)=MAX(0.0_DbKi,MIN(C%VS_MaxTq,S%Applied(1)))
            S%Applied(2)=MAX(C%PC_MinPit,MIN(C%PC_MaxPit,S%Applied(2)))
            S%AppliedPitchRate=(S%Applied(2)-PreviousPitch)/Dt
            CALL FMPC_WriteCommands(S,V,Swap)
        ENDIF
        IF (LogNow .AND. S%TraceUnit/=0) THEN
            WRITE(S%TraceUnit,'(*(G0,:,","))') V%Time,S%Status,S%QPStatus,Omega,S%Alpha,Wind,Pitch,S%TowerX,S%TowerV, &
                S%Command,S%Applied,S%Distance,S%LastQPResidual,S%Inertia
            FLUSH(S%TraceUnit)
        ENDIF
    END SUBROUTINE
END MODULE
