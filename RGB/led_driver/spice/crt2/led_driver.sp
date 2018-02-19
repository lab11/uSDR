
.protect
.include '../lib/si2312cd.sp'
.include '../lib/si2301cd.sp'
.include '../lib/adcmp601.sp'
.include '../lib/MCE.sp'
.include '../lib/ddz9678.sp'
.hdl '../lib/or.va'
.unprotect

.PARAM vsupply=5
.GLOBAL vss vdd

vdd vdd 0 vsupply
vss vss 0 0

.option post
.option ingold=0

.option method = gear
.option runlvl = 0
.option converge=1

*D1 vdd p1 MCERed
D1 vdd p1 MCEGreen
*D1 vdd p1 MCEBlue
x1 p1 p2 p3 si2312cds
r1 p3 vss 0.9

x2 p2 p3 vss si2312cds
r2 vdd p2 100k

x3 p2 vpwm vss si2312cds
x4 p2 vref vdd vdd vss p8 ADCMP601
x5 vpwm p8 vcharge vdd vss orgate 
x6 p2 vcharge p7 si2301cds 
x7 vss p2 DDZ9678
r3 vdd p7 100
v3 vref vss 1.263

*r1 p3 vss rval

*vp vpwm vss 0
vp vpwm vss pulse(0 5 10u 100p 100p 5u 10u)
*.ic v(p3)=0 v(p2)=vsupply
.ic v(p2)=1.52
.tran 1n 30u uic

*.dc vsupply 4 6 0.001
.print i(D1)

.END

