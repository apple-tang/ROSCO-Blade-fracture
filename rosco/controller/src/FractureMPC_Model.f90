! SPDX-License-Identifier: Apache-2.0
MODULE FractureMPC_Model
    USE, INTRINSIC :: ISO_C_BINDING
    USE, INTRINSIC :: IEEE_ARITHMETIC
    USE FractureMPC_Types
    IMPLICIT NONE
    INTERFACE
        INTEGER(C_INT) FUNCTION SolveQP(N,M,H,G,A,LB,UB,LA,UA,Iterations,X,Violation) BIND(C,NAME='fmpc_qp_solve')
            IMPORT C_INT,C_DOUBLE
            INTEGER(C_INT), VALUE :: N,M,Iterations
            REAL(C_DOUBLE), INTENT(IN) :: H(*),G(*),A(*),LB(*),UB(*),LA(*),UA(*)
            REAL(C_DOUBLE), INTENT(OUT) :: X(*),Violation
        END FUNCTION
    END INTERFACE
CONTAINS
    SUBROUTINE FMPC_Optimize(P,S,G,Z,X0,Now,QScale,GearRatio,TqRate,PitchMin,PitchMax,PitchRate,Command,Code,Residual)
        TYPE(FMPC_Parameters), INTENT(IN) :: P
        TYPE(FMPC_State), INTENT(IN) :: S
        REAL(DbKi), INTENT(IN) :: G(12),Z(6),X0(6),Now,QScale,GearRatio,TqRate,PitchMin,PitchMax,PitchRate
        REAL(DbKi), INTENT(OUT) :: Command(2),Residual
        INTEGER, INTENT(OUT) :: Code
        REAL(DbKi) :: AC(6,6),BC(6,2),AD(6,6),BD(6,2),Integral(6,6),Identity(6,6),FC(6)
        REAL(DbKi) :: H(2*P%NControl,2*P%NControl),Grad(2*P%NControl)
        REAL(DbKi) :: Con(2*P%NControl+P%NPred,2*P%NControl)
        REAL(DbKi) :: LB(2*P%NControl),UB(2*P%NControl),LA(2*P%NControl+P%NPred),UA(2*P%NControl+P%NPred)
        REAL(DbKi) :: Sol(2*P%NControl),Sensitivity(6,2*P%NControl),Free(6),Ref(6),Bias(6),Weights(5)
        REAL(DbKi) :: Jeff,Psi,GTpsi,GFpsi,PhaseT,PhaseF,Last(2),Rates(2),AlphaCoupling
        INTEGER :: N,M,I,J,K,L,Block,Column
        N=2*P%NControl; M=N+P%NPred; Code=-10; Command=S%Command; Residual=HUGE(1.0_DbKi)
        Jeff=S%Inertia-G(4)
        IF (.NOT.IEEE_IS_FINITE(Jeff) .OR. Jeff<0.1_DbKi*S%Inertia) RETURN
        AC=0; BC=0; Identity=0
        DO I=1,6
            Identity(I,I)=1
        ENDDO
        Psi=ATAN2(Z(5),Z(6)); GTpsi=G(5)*COS(Psi)-G(6)*SIN(Psi)
        GFpsi=G(11)*COS(Psi)-G(12)*SIN(Psi)
        AC(1,1)=G(1)/Jeff; AC(1,3)=-G(3)/Jeff
        AC(1,4)=-GearRatio*QScale/Jeff; AC(1,5)=G(2)/Jeff; AC(1,6)=GTpsi/Jeff
        AC(2,3)=1
        AlphaCoupling=G(10)/P%TowerMass
        AC(3,:)=AlphaCoupling*AC(1,:)
        AC(3,1)=AC(3,1)+G(7)/P%TowerMass
        AC(3,2)=-P%TowerStiffness/P%TowerMass
        AC(3,3)=AC(3,3)-(P%TowerDamping+G(9))/P%TowerMass
        AC(3,5)=AC(3,5)+G(8)/P%TowerMass; AC(3,6)=AC(3,6)+GFpsi/P%TowerMass
        AC(4,4)=-1/P%TorqueTau; BC(4,1)=1/P%TorqueTau
        AC(5,5)=-1/P%PitchTau; BC(5,2)=1/P%PitchTau
        AC(6,1)=1
        FC=0; FC(1)=S%Alpha-DOT_PRODUCT(AC(1,:),X0)
        FC(3)=S%TowerAcc-DOT_PRODUCT(AC(3,:),X0); FC(6)=-X0(1)
        ! Second-order discretization; input and measured-load bias are held per MPC interval.
        AD=Identity+P%DT*AC+0.5_DbKi*P%DT**2*MATMUL(AC,AC)
        Integral=P%DT*Identity+0.5_DbKi*P%DT**2*AC; BD=MATMUL(Integral,BC)
        Weights=P%Q/P%StateScale**2
        H=0; Grad=0; Con=0; LB=0; UB=0; LA=-1e20_DbKi; UA=1e20_DbKi
        Last=[S%Command(1)/QScale,S%Command(2)]
        Rates=[TqRate*P%DT/QScale,PitchRate*P%DT]
        DO I=1,P%NControl
            DO J=1,2
                Column=2*(I-1)+J
                H(Column,Column)=2*P%R(J); Grad(Column)=-2*P%R(J)*Last(J)
                LB(Column)=0; UB(Column)=1
                IF (J==2) THEN
                    LB(Column)=MAX(PitchMin,Last(2)); UB(Column)=PitchMax
                ENDIF
                Con(Column,Column)=1
                LA(Column)=-Rates(J); UA(Column)=Rates(J)
                IF (J==2) LA(Column)=0
                IF (I==1) THEN
                    LA(Column)=LA(Column)+Last(J); UA(Column)=UA(Column)+Last(J)
                ELSE
                    Con(Column,Column-2)=-1
                ENDIF
            ENDDO
        ENDDO
        Free=X0; Sensitivity=0
        DO K=1,P%NPred
            Bias=FC
            PhaseT=(G(5)*(SIN(Psi+X0(1)*K*P%DT)-SIN(Psi))+G(6)*(COS(Psi+X0(1)*K*P%DT)-COS(Psi)))/Jeff
            PhaseF=(G(11)*(SIN(Psi+X0(1)*K*P%DT)-SIN(Psi))+G(12)*(COS(Psi+X0(1)*K*P%DT)-COS(Psi)))/P%TowerMass
            Bias(1)=Bias(1)+PhaseT; Bias(3)=Bias(3)+PhaseF+AlphaCoupling*PhaseT
            Free=MATMUL(AD,Free)+MATMUL(Integral,Bias)
            Sensitivity=MATMUL(AD,Sensitivity)
            Block=MIN(K,P%NControl)
            Sensitivity(:,2*Block-1:2*Block)=Sensitivity(:,2*Block-1:2*Block)+BD
            Ref=0; Ref(1)=MAX(0.0_DbKi,S%StartOmega-P%Decel*(Now-S%StartTime+K*P%DT)); Ref(5)=PitchMax
            DO I=1,N
                DO L=1,5
                    Grad(I)=Grad(I)+2*Weights(L)*Sensitivity(L,I)*(Free(L)-Ref(L))
                ENDDO
                DO J=1,N
                    H(I,J)=H(I,J)+2*SUM(Weights*Sensitivity(1:5,I)*Sensitivity(1:5,J))
                ENDDO
            ENDDO
            Con(N+K,:)=Sensitivity(1,:); LA(N+K)=-Free(1)
        ENDDO
        IF (.NOT.ALL(IEEE_IS_FINITE(H)) .OR. .NOT.ALL(IEEE_IS_FINITE(Grad))) RETURN
        Code=SolveQP(N,M,H,Grad,Con,LB,UB,LA,UA,P%Iterations,Sol,Residual)
        IF (Code==0) Command=[Sol(1)*QScale,Sol(2)]
    END SUBROUTINE
END MODULE
