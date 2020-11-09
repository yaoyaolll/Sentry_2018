#include "main.h"

/**
  * @brief  λ��ʽPID�����㷨
  * @param  PID_Struct *P  PID�����ṹ��
  *         ActualValue    PID���㷴��������ǰ��ʵ���ֵ��
  * @retval PID�����������ֵ
  */
float PID_Calc(Pid_Typedef *P, float ActualValue)
{
		P->PreError = P->SetPoint - ActualValue;
		P->dError = P->PreError - P->LastError;
	
		P->SumError += P->PreError;
		P->LastError = P->PreError;
		
		if(P->SumError >= P->IMax)
			P->SumError = P->IMax;
		else if(P->SumError <= -P->IMax)
			P->SumError = -P->IMax;
		
		P->POut = P->P * P->PreError;
		P->IOut = P->I * P->SumError;
		P->DOut = P->D * P->dError;
		
		return P->POut+P->IOut+P->DOut;
}

