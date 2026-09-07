! SPDX-License-Identifier: Apache-2.0
MODULE FractureMPC_Table
    USE FractureMPC_Types
    USE ROSCO_Types, ONLY: ErrorVariables
    USE, INTRINSIC :: IEEE_ARITHMETIC
    IMPLICIT NONE
CONTAINS
    SUBROUTINE FMPC_LoadTable(P,T,E)
        TYPE(FMPC_Parameters), INTENT(IN) :: P
        TYPE(FMPC_Table), INTENT(INOUT) :: T
        TYPE(ErrorVariables), INTENT(INOUT) :: E
        INTEGER :: U,IOS,I
        CHARACTER(128) :: Header
        IF (ALLOCATED(T%Z)) DEALLOCATE(T%Z,T%G)
        OPEN(NEWUNIT=U,FILE=TRIM(P%TableFile),STATUS='OLD',ACTION='READ',IOSTAT=IOS)
        IF (IOS/=0) THEN
            E%aviFAIL=-1; E%ErrMsg='FMPC: cannot open gain table '//TRIM(P%TableFile); RETURN
        ENDIF
        READ(U,'(A)',IOSTAT=IOS) Header
        IF (IOS==0 .AND. TRIM(Header)=='ROSCO_FMPC_TABLE_V1') THEN
            READ(U,*,IOSTAT=IOS) T%Count,T%Blade,T%S,T%RatedRPM
        ELSE
            IOS=1
        ENDIF
        IF (IOS==0) THEN
            IF (T%Count<1 .OR. T%Count>1000000 .OR. T%Blade/=P%Blade .OR. ABS(T%S-P%FractureS)>1e-6_DbKi) IOS=1
        ENDIF
        IF (IOS==0) THEN
            ALLOCATE(T%Z(6,T%Count),T%G(12,T%Count))
            DO I=1,T%Count
                READ(U,*,IOSTAT=IOS) T%Z(:,I),T%G(:,I)
                IF (IOS/=0) EXIT
                IF (.NOT.ALL(IEEE_IS_FINITE(T%Z(:,I))) .OR. .NOT.ALL(IEEE_IS_FINITE(T%G(:,I)))) THEN
                    IOS=1; EXIT
                ENDIF
                IF (ABS(SUM(T%Z(5:6,I)**2)-1)>1e-5_DbKi) THEN
                    IOS=1; EXIT
                ENDIF
            ENDDO
        ENDIF
        CLOSE(U)
        IF (IOS/=0) THEN
            E%aviFAIL=-1; E%ErrMsg='FMPC: invalid table version, dimensions, fault metadata or numeric data'
        ENDIF
    END SUBROUTINE

    SUBROUTINE FMPC_Lookup(P,T,Z,G,Distance,OK)
        TYPE(FMPC_Parameters), INTENT(IN) :: P
        TYPE(FMPC_Table), INTENT(IN) :: T
        REAL(DbKi), INTENT(IN) :: Z(6)
        REAL(DbKi), INTENT(OUT) :: G(12),Distance
        LOGICAL, INTENT(OUT) :: OK
        REAL(DbKi) :: Best(4),D,W(4)
        INTEGER :: Idx(4),I,J,K,N
        Best=HUGE(1.0_DbKi); Idx=0; G=0; Distance=HUGE(1.0_DbKi); OK=.FALSE.
        IF (.NOT.ALL(IEEE_IS_FINITE(Z))) RETURN
        DO I=1,T%Count
            D=SUM(((Z-T%Z(:,I))/P%LookupScale)**2)
            DO J=1,4
                IF (D<Best(J)) THEN
                    DO K=4,J+1,-1
                        Best(K)=Best(K-1); Idx(K)=Idx(K-1)
                    ENDDO
                    Best(J)=D; Idx(J)=I; EXIT
                ENDIF
            ENDDO
        ENDDO
        Distance=SQRT(Best(1))
        IF (Distance>P%MaxDistance) RETURN
        N=MIN(4,T%Count)
        IF (Best(1)<1e-16_DbKi) THEN
            G=T%G(:,Idx(1))
        ELSE
            W(1:N)=1/MAX(Best(1:N),1e-12_DbKi); W(1:N)=W(1:N)/SUM(W(1:N))
            DO I=1,N
                G=G+W(I)*T%G(:,Idx(I))
            ENDDO
        ENDIF
        OK=ALL(IEEE_IS_FINITE(G))
    END SUBROUTINE
END MODULE
