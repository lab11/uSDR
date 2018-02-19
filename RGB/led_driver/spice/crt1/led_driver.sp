
.protect
.include '../lib/si2312cd.sp'
.include '../lib/si2301cd.sp'
.include '../lib/MCE.sp'
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

*D1 vdd p1 MCEGreen 
*D1 vdd p1 MCEBlue
D1 vdd p1 MCEGreen
x1 p1 p2 p3 si2312cds

x2 p2 p3 vss si2312cds
r2 vdd p2 100k

x3 p2 vpwm vss si2312cds
*r1 p3 vss rval
r1 p3 vss 0.75

*vp vpwm vss 0
vp vpwm vss pulse(0 3.3 10u 100p 100p 150u 300u)
.ic v(p3)=0 v(p2)=vsupply
.tran 10n 500u uic

*.dc vsupply 4 6 0.001
.print i(D1)

.END

