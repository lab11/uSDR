
.protect
.include '../lib/si2312cd.sp'
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
D1 vpwm p1 MCEBlue
r1 p1 vss 2.15

*vp vpwm vss vsupply
vp vpwm vss pulse(0 vsupply 10u 100p 100p 300u 600u)
*.ic v(p3)=0 v(p2)=vsupply
.tran 10u 500u uic

*.dc vsupply 4 6 0.001
.print i(D1)

.EN

