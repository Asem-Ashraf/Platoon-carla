#!/usr/bin/env python3.7

import numpy as np
class MPC():
    def __init__(self):
        # Q, S, R are the cost matrices of the bicycle model
        self.Q = np.matrix('1 0;0 1')
        self.S = np.matrix('30 0;0 30')
        self.R = np.matrix('1')

    def mpc_simplification(self, Ad, Bd, Cd, Dd, hz):
        '''This function creates the compact matrices for Model Predictive
        Control'''
        # db - double bar
        # dbt - double bar transpose
        # dc - double circumflex

        A_aug=np.concatenate((Ad,Bd),axis=1)
        # print('Ad:',Ad)
        # print('Bd:',Bd)
        # print('A_aug:',A_aug)
        temp1=np.zeros((np.size(Bd,1),np.size(Ad,1)))
        temp2=np.identity(np.size(Bd,1))
        temp=np.concatenate((temp1,temp2),axis=1)
        # print('temp:',temp)

        A_aug=np.concatenate((A_aug,temp),axis=0)
        # print('A_aug:',A_aug)
        B_aug=np.concatenate((Bd,np.identity(np.size(Bd,1))),axis=0)
        # print('B_aug:',B_aug)
        C_aug=np.concatenate((Cd,np.zeros((np.size(Cd,0),np.size(Bd,1)))),axis=1)
        # print('C_aug:',C_aug)
        D_aug=Dd
        # print('D_aug:',D_aug)


        CQC=np.matmul(np.transpose(C_aug),self.Q)
        CQC=np.matmul(CQC,C_aug)
        # print('CQC:',CQC)

        CSC=np.matmul(np.transpose(C_aug),self.S)
        CSC=np.matmul(CSC,C_aug)
        # print('CSC:',CSC)

        QC=np.matmul(self.Q,C_aug)
        # print('QC:',QC)
        SC=np.matmul(self.S,C_aug)
        # print('SC:',SC)

        Qdb=np.zeros((np.size(CQC,0)*hz,np.size(CQC,1)*hz))
        # print('Qdb:',Qdb)
        Tdb=np.zeros((np.size(QC,0)*hz,np.size(QC,1)*hz))
        # print('Tdb:',Tdb)
        Rdb=np.zeros((np.size(self.R,0)*hz,np.size(self.R,1)*hz))
        # print('Rdb:',Rdb)
        Cdb=np.zeros((np.size(B_aug,0)*hz,np.size(B_aug,1)*hz))
        # print('Cdb:',Cdb)
        Adc=np.zeros((np.size(A_aug,0)*hz,np.size(A_aug,1)))
        # print('Adc:',Adc)

        for i in range(0,hz):
            if i == hz-1:
                Qdb[np.size(CSC,0)*i:np.size(CSC,0)*i+CSC.shape[0],np.size(CSC,1)*i:np.size(CSC,1)*i+CSC.shape[1]]=CSC
                Tdb[np.size(SC,0)*i:np.size(SC,0)*i+SC.shape[0],np.size(SC,1)*i:np.size(SC,1)*i+SC.shape[1]]=SC
            else:
                Qdb[np.size(CQC,0)*i:np.size(CQC,0)*i+CQC.shape[0],np.size(CQC,1)*i:np.size(CQC,1)*i+CQC.shape[1]]=CQC
                Tdb[np.size(QC,0)*i:np.size(QC,0)*i+QC.shape[0],np.size(QC,1)*i:np.size(QC,1)*i+QC.shape[1]]=QC

            Rdb[np.size(self.R,0)*i:np.size(self.R,0)*i+self.R.shape[0],np.size(self.R,1)*i:np.size(self.R,1)*i+self.R.shape[1]]=self.R

            for j in range(0,hz):
                if j<=i:
                    Cdb[np.size(B_aug,0)*i:np.size(B_aug,0)*i+B_aug.shape[0],np.size(B_aug,1)*j:np.size(B_aug,1)*j+B_aug.shape[1]]=np.matmul(np.linalg.matrix_power(A_aug,((i+1)-(j+1))),B_aug)

            Adc[np.size(A_aug,0)*i:np.size(A_aug,0)*i+A_aug.shape[0],0:0+A_aug.shape[1]]=np.linalg.matrix_power(A_aug,i+1)

        Hdb=np.matmul(np.transpose(Cdb),Qdb)
        Hdb=np.matmul(Hdb,Cdb)+Rdb

        temp=np.matmul(np.transpose(Adc),Qdb)
        temp=np.matmul(temp,Cdb)

        temp2=np.matmul(-Tdb,Cdb)
        Fdbt=np.concatenate((temp,temp2),axis=0)

        return Hdb,Fdbt,Cdb,Adc
