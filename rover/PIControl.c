#include <stdio.h>

 // PI����Q�C��
#define GAIN_P 5.0 // ���޲�
#define GAIN_I 1.0 // �ϕ��޲�
// �^�����f��
#define K_POS 1.0 // �o�l��
#define K_VEL 2.0 // �_���s���O
#define MASS 1.0 // ����
#define INT_TIME 0.1 // ���Խï��

//PI����
double PIctrl(double dCommand, double dVal)
 {
 static double s_dErrIntg = 0;
 double dErr;
 double dRet;

 // �덷
dErr = dCommand - dVal;

 // �덷�ϕ�
s_dErrIntg += dErr * INT_TIME;

 // �������
dRet = GAIN_P * dErr + GAIN_I * s_dErrIntg;

 return (dRet);
 }
 //�����܂�

// ���ؗp�v���O����
void main()
 {
 double dF, dFctrl;
 double dAcc, dVel, dPos, dTime;
 int i;

 // �����l
dVel = 0; // ���x
dPos = 0; // �ʒu
dTime = 0; // ����

for (i = 0; i < 500; i++) {
 // �R���g���[��
dFctrl = PIctrl(10, dPos);

 // �O��
dF = dFctrl - (K_POS * dPos + K_VEL * dVel);

 // �����x
dAcc = dF / MASS;

 // ���x
dVel += dAcc * INT_TIME;

 // �ʒu
dPos += dVel * INT_TIME;

 // ����
dTime += INT_TIME;

 printf("TIME = %f, POS = %f\n", dTime, dPos);
 }
 }