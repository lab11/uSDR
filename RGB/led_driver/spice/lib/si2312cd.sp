*May 03, 2010
*Doc. ID: 66696, ECN S10-1084, Rev. A
*This document is intended as a SPICE modeling guideline and does not
*constitute a commercial product data sheet. Designers should refer to the
*appropriate data sheet of the same number for guaranteed specification
*limits.
.SUBCKT si2312cds D G S 
M1 3 GX S S NMOS W= 754648u L= 0.25u 
M2 S GX S D PMOS W= 754648u L= 1.803e-07 
R1 D 3 2.304e-02 3.899e-03 9.007e-06 
CGS GX S 6.613e-10 
CGD GX D 3.143e-11 
RG G GY 2.4 
RTCV 100 S 1e6 1.191e-04 2.831e-06 
ETCV GX GY 100 200 1 
ITCV S 100 1u 
VTCV 200 S 1 
DBD S D DBD 
**************************************************************** 
.MODEL NMOS NMOS ( LEVEL = 3 TOX = 1.7e-8 
+ RS = 5.000e-04 KP = 3.501e-05 NSUB = 1.427e+17 
+ KAPPA = 1.000e-06 ETA = 8.728e-04 NFS = 8.000e+09 
+ LD = 0 IS = 0 TPG = 1) 
*************************************************************** 
.MODEL PMOS PMOS ( LEVEL = 3 TOX = 1.7e-8 
+NSUB = 2.718e+16 IS = 0 TPG = -1 ) 
**************************************************************** 
.MODEL DBD D ( 
+FC = 0.1 TT = 9.818e-09 TREF = 25 BV = 22 
+RS = 4.617e-02 N = 1.144e+00 IS = 5.104e-09 
+EG = 8.357e-01 XTI = 2.002e-01 TRS = 1.367e-03 
+CJO = 2.376e-10 VJ = 3.000e-01 M = 2.981e-01 ) 
.ENDS 
