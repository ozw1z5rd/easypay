CIRCAD Version 4.0 -- Data File.

Setup parameters:
-gc0,0,0,0,0,0,0,GND,EARTH,0,0,2,7,8,10,11
-gd.1,FFFFFF,909090,FFFFFF,FFFF00
-gm3,2,3,0
-gp0,0,1.0,0,0,0,0,0
-gs0.100",0,10,100,.1mm
-gx-1.0,-1.0,11.0,8.5
-gz0,0
-gtt0,0,0,0,0,0,0,0,0
-gtb0,0,0,0,0,0,0,0,0
-ga0
-gfC

Item parameters:
-il.01
-iv2,.04,0
-ip0,0,.01,.01,0
-it.1
-ix0

Layer/color/plot/type/sequence/name table:
-y2
-y0:0000AA00,00FFFFFF,0,0,Master
-y1:00000000,00FFFFFF,4,1,Symbols
-y2:000000AA,00FFFFFF,6,2,Signals
-y3:00AA0000,00FFFFFF,7,3,Busses
-y4:00555555,00FFFFFF,4,4,Pin names
-y5:00000000,00FFFFFF,4,5,Pin numbers

Gerber parameters:
-dp0,0,0,0,0
-dy

Component libraries:
-b
-bTTL-als.LIB
-bTTL-as.LIB
-bTTL-f.LIB
-bTTL-hc.LIB
-bTTL-hct.LIB
-bTTL-ls.LIB
-bTTL-s.LIB
-bECL.LIB
-bCMOS.LIB
-bPLD.LIB
-bMICRO.LIB
-bMEMORY.LIB
-bANALOG.LIB
-bDISCRETE.LIB
-bSCH.LIB
-bTTL-hc.lib
-bTHD.LIB/p
-bSMD.LIB/p

Primary Data File:
Component: 6 0 0 X1 Y1 X2 Y2 Parameters
Text: 4 layer height X0 Y0 length angle anchor font data
Pad: 3 layer type X0 Y0 width height hole angle pin signal
Arc: 2 layer width X0 Y0 radius start sweep ellipse angle signal
Line: 1 layer width X1 Y1 X2 Y2 signal
6 4 0 0 0 10.0 7.5 $$BOX
1 1 .01 0 0 10.0 0
1 1 .01 10.0 0 10.0 7.5
1 1 .01 10.0 7.5 0 7.5
1 1 .01 0 7.5 0 0
0 6 0 0 0
6 0 0 2.525 3.65 2.675 3.704 $$GND
3 2 0 2.6 3.7 .009 .009 0 0 0 GND
1 1 .01 2.525 3.7 2.675 3.7
1 1 .01 2.55 3.675 2.65 3.675
1 1 .01 2.575 3.65 2.625 3.65
0 6 0 2.6 3.7
6 0 0 1.425 5.85 1.575 5.904 $$GND
3 2 0 1.5 5.9 .009 .009 0 0 0 GND
1 1 .01 1.425 5.9 1.575 5.9
1 1 .01 1.45 5.875 1.55 5.875
1 1 .01 1.475 5.85 1.525 5.85
0 6 0 1.5 5.9
6 0 0 1.925 3.45 2.075 3.505 $$GND
3 2 0 2.0 3.5 .009 .009 0 0 0 GND
1 1 .01 1.925 3.5 2.075 3.5
1 1 .01 1.95 3.475 2.05 3.475
1 1 .01 1.975 3.45 2.025 3.45
0 6 0 2.0 3.5
6 0 0 2.58 6.195 2.62 6.25 $$PWR
4 1 .06 2.6 6.31 .15 0 5 0 &@
3 2 0 2.6 6.2 .009 .009 0 0 0 +5V
1 1 .01 2.6 6.2 2.6 6.25
1 1 .01 2.58 6.21 2.6 6.25
1 1 .01 2.62 6.21 2.6 6.25
0 6 0 2.6 6.2
6 0 0 1.88 6.995 1.92 7.05 $$PWR
4 1 .06 1.9 7.11 .15 0 5 0 &@
3 2 0 1.9 7.0 .009 .009 0 0 0 +5V
1 1 .01 1.9 7.0 1.9 7.05
1 1 .01 1.88 7.01 1.9 7.05
1 1 .01 1.92 7.01 1.9 7.05
0 6 0 1.9 7.0
6 0 0 .8 4.455 1.505 4.545 $$SIG
4 1 .06 .825 4.5 .55 0 4 0 &@
3 2 0 1.5 4.5 .009 .009 0 0 0 INTERRUPT1
1 1 .01 1.5 4.5 1.455 4.545
1 1 .01 1.455 4.545 .8 4.545
1 1 .01 .8 4.545 .8 4.455
1 1 .01 .8 4.455 1.455 4.455
1 1 .01 1.455 4.455 1.5 4.5
0 6 0 1.5 4.5
6 0 0 1.0 4.055 1.505 4.145 $$SIG
4 1 .06 1.025 4.1 .39 0 4 0 &@
3 2 0 1.5 4.1 .009 .009 0 0 0 JUMPER5
1 1 .01 1.5 4.1 1.455 4.145
1 1 .01 1.455 4.145 1.0 4.145
1 1 .01 1.0 4.145 1.0 4.055
1 1 .01 1.0 4.055 1.455 4.055
1 1 .01 1.455 4.055 1.5 4.1
0 6 0 1.5 4.1
6 0 0 1.0 4.155 1.505 4.245 $$SIG
4 1 .06 1.025 4.2 .39 0 4 0 &@
3 2 0 1.5 4.2 .009 .009 0 0 0 JUMPER4
1 1 .01 1.5 4.2 1.455 4.245
1 1 .01 1.455 4.245 1.0 4.245
1 1 .01 1.0 4.245 1.0 4.155
1 1 .01 1.0 4.155 1.455 4.155
1 1 .01 1.455 4.155 1.5 4.2
0 6 0 1.5 4.2
6 0 0 1.0 4.255 1.505 4.345 $$SIG
4 1 .06 1.025 4.3 .39 0 4 0 &@
3 2 0 1.5 4.3 .009 .009 0 0 0 JUMPER3
1 1 .01 1.5 4.3 1.455 4.345
1 1 .01 1.455 4.345 1.0 4.345
1 1 .01 1.0 4.345 1.0 4.255
1 1 .01 1.0 4.255 1.455 4.255
1 1 .01 1.455 4.255 1.5 4.3
0 6 0 1.5 4.3
6 0 0 1.0 4.355 1.505 4.445 $$SIG
4 1 .06 1.025 4.4 .39 0 4 0 &@
3 2 0 1.5 4.4 .009 .009 0 0 0 JUMPER2
1 1 .01 1.5 4.4 1.455 4.445
1 1 .01 1.455 4.445 1.0 4.445
1 1 .01 1.0 4.445 1.0 4.355
1 1 .01 1.0 4.355 1.455 4.355
1 1 .01 1.455 4.355 1.5 4.4
0 6 0 1.5 4.4
6 0 0 1.0 4.555 1.505 4.645 $$SIG
4 1 .06 1.025 4.6 .39 0 4 0 &@
3 2 0 1.5 4.6 .009 .009 0 0 0 JUMPER1
1 1 .01 1.5 4.6 1.455 4.645
1 1 .01 1.455 4.645 1.0 4.645
1 1 .01 1.0 4.645 1.0 4.555
1 1 .01 1.0 4.555 1.455 4.555
1 1 .01 1.455 4.555 1.5 4.6
0 6 0 1.5 4.6
6 0 180 3.495 5.155 3.8 5.245 $$SIG
4 1 .06 3.775 5.2 .22 0 6 0 &@
3 2 0 3.5 5.2 .009 .009 0 180 0 /RCP
1 1 .01 3.5 5.2 3.545 5.155
1 1 .01 3.545 5.155 3.8 5.155
1 1 .01 3.8 5.155 3.8 5.245
1 1 .01 3.8 5.245 3.545 5.245
1 1 .01 3.545 5.245 3.5 5.2
0 6 0 3.5 5.2
6 0 180 3.495 4.955 3.9 5.045 $$SIG
4 1 .06 3.875 5.0 .32 0 6 0 &@
3 2 0 3.5 5.0 .009 .009 0 180 0 KEY_IN
1 1 .01 3.5 5.0 3.545 4.955
1 1 .01 3.545 4.955 3.9 4.955
1 1 .01 3.9 4.955 3.9 5.045
1 1 .01 3.9 5.045 3.545 5.045
1 1 .01 3.545 5.045 3.5 5.0
0 6 0 3.5 5.0
6 0 180 3.495 5.255 3.8 5.345 $$SIG
4 1 .06 3.775 5.3 .34 0 6 0   /WCP
3 2 0 3.5 5.3 .009 .009 0 180 0 /RDT
1 1 .01 3.5 5.3 3.545 5.255
1 1 .01 3.545 5.255 3.8 5.255
1 1 .01 3.8 5.255 3.8 5.345
1 1 .01 3.8 5.345 3.545 5.345
1 1 .01 3.545 5.345 3.5 5.3
0 6 0 3.5 5.3
6 0 0 .7 6.355 1.505 6.445 $$SIG
4 1 .06 .725 6.4 .68 0 4 0 &@
3 2 0 1.5 6.4 .009 .009 0 0 0 /MICRO_RESET
1 1 .01 1.455 6.445 .7 6.445
1 1 .01 .7 6.445 .7 6.355
1 1 .01 .7 6.355 1.455 6.355
0 6 0 1.5 6.4
6 0 180 3.495 5.355 3.8 5.445 $$SIG
4 1 .06 3.775 5.4 .15 0 6 0 &@
3 2 0 3.5 5.4 .009 .009 0 180 0 /S4
1 1 .01 3.5 5.4 3.545 5.355
1 1 .01 3.545 5.355 3.8 5.355
1 1 .01 3.8 5.355 3.8 5.445
1 1 .01 3.8 5.445 3.545 5.445
1 1 .01 3.545 5.445 3.5 5.4
0 6 0 3.5 5.4
6 0 180 3.495 5.455 3.8 5.545 $$SIG
4 1 .06 3.775 5.5 .15 0 6 0 &@
3 2 0 3.5 5.5 .009 .009 0 180 0 /S3
1 1 .01 3.5 5.5 3.545 5.455
1 1 .01 3.545 5.455 3.8 5.455
1 1 .01 3.8 5.455 3.8 5.545
1 1 .01 3.8 5.545 3.545 5.545
1 1 .01 3.545 5.545 3.5 5.5
0 6 0 3.5 5.5
6 0 180 3.495 5.555 3.8 5.645 $$SIG
4 1 .06 3.775 5.6 .15 0 6 0 &@
3 2 0 3.5 5.6 .009 .009 0 180 0 /S1
1 1 .01 3.5 5.6 3.545 5.555
1 1 .01 3.545 5.555 3.8 5.555
1 1 .01 3.8 5.555 3.8 5.645
1 1 .01 3.8 5.645 3.545 5.645
1 1 .01 3.545 5.645 3.5 5.6
0 6 0 3.5 5.6
6 0 180 3.495 5.055 3.8 5.145 $$SIG
4 1 .06 3.775 5.1 .22 0 6 0 /RDT
3 2 0 3.5 5.1 .009 .009 0 180 0 /RCP
1 1 .01 3.5 5.1 3.545 5.055
1 1 .01 3.545 5.055 3.8 5.055
1 1 .01 3.8 5.055 3.8 5.145
1 1 .01 3.8 5.145 3.545 5.145
1 1 .01 3.545 5.145 3.5 5.1
0 6 0 3.5 5.1
6 4 0 6.5 0 10.0 1.25 $$TTL
4 1 .06 6.55 1.1 .13 0 0 0 Co:
4 1 .06 6.55 .85 .24 0 0 0 Title:
4 1 .06 6.55 .6 .31 0 0 0 Board:
4 1 .06 6.55 .35 .36 0 0 0 Author:
4 1 .06 6.55 .1 .25 0 0 0 Date:
4 1 .126 6.9 1.1 3.086 0 0 0 Know How & Technologies sas
4 1 .126 7.0 .825 .84 0 0 0 CONTROL
4 1 .126 7.0 .575 .966 0 0 0 TCONTROL
4 1 .126 7.0 .325 1.848 0 0 0 Ing. Alessio Palma
4 1 .126 7.0 .075 1.302 0 0 0 10 Sept 2003
4 1 .06 9.25 .6 .4 0 0 0 Revision:
4 1 .06 9.25 .35 .21 0 0 0 Size:
4 1 .06 9.25 .1 .28 0 0 0 Sheet
4 1 .108 9.575 .075 .054 0 0 0 1
4 1 .126 9.6 .325 .084 0 0 0 A
4 1 .06 9.7 .1 .1 0 0 0 of
4 1 .126 9.775 .575 .084 0 0 0 A
4 1 .108 9.85 .075 .054 0 0 0 1
1 1 .01 6.5 0 6.5 1.25
1 1 .01 6.5 1.25 10.0 1.25
1 1 .01 6.5 1.0 10.0 1.0
1 1 .01 6.5 .75 10.0 .75
1 1 .01 10.0 .5 6.5 .5
1 1 .01 6.5 .25 10.0 .25
1 1 .01 6.5 0 10.0 0
1 1 .01 10.0 0 10.0 1.25
0 6 0 10.0 0
6 0 0 1.595 5.918 1.805 6.082 C'C10.1 �FC*
4 1 .06 1.7 6.2 .09 0 1 0 &1
4 1 .06 1.7 6.11 .28 0 1 0 &2
3 0 0 1.6 6.0 .01 .01 0 0 1
3 0 0 1.8 6.0 .01 .01 0 0 2
2 1 .01 1.485 6.0 .2 247.50 45 0 90
2 1 .01 1.915 6.0 .2 67.50 45 0 90
1 1 .01 1.685 6.0 1.6 6.0
1 1 .01 1.715 6.0 1.8 6.0
0 6 0 1.7 6.0
6 0 0 1.875 6.495 1.925 6.905 RR1100K�R*
4 1 .06 1.95 6.72 .09 0 0 0 &1
4 1 .06 1.95 6.62 .25 0 0 0 &2
3 0 0 1.9 6.9 .01 .01 0 0 1
3 0 0 1.9 6.5 .01 .01 0 0 2
1 1 .01 1.9 6.9 1.9 6.85
1 1 .01 1.9 6.85 1.875 6.825
1 1 .01 1.875 6.825 1.925 6.775
1 1 .01 1.925 6.775 1.875 6.725
1 1 .01 1.875 6.725 1.925 6.675
1 1 .01 1.925 6.675 1.875 6.625
1 1 .01 1.875 6.625 1.925 6.575
1 1 .01 1.925 6.575 1.9 6.55
1 1 .01 1.9 6.55 1.9 6.5
0 6 0 1.9 6.7
6 0 0 1.695 3.895 3.105 5.805 AT90LS8535-4PCU2ATMEGA16LDIP40
4 1 .06 3.1 5.8 .09 0 0 0 &1
4 1 .06 3.1 3.9 .5 0 8 0 &2
4 4 .06 1.77 5.6 .15 0 4 0 PB0
4 4 .06 1.77 5.5 .15 0 4 0 PB1
4 4 .06 1.77 5.4 .15 0 4 0 PB2
4 4 .06 1.77 5.3 .15 0 4 0 PB3
4 4 .06 1.77 5.2 .15 0 4 0 PB4
4 4 .06 1.77 5.1 .15 0 4 0 PB5
4 4 .06 1.77 5.0 .15 0 4 0 PB6
4 4 .06 1.77 4.9 .15 0 4 0 PB7
4 4 .06 1.77 4.8 .15 0 4 0 PD0
4 4 .06 1.77 4.7 .15 0 4 0 PD1
4 4 .06 1.77 4.6 .15 0 4 0 PD2
4 4 .06 1.77 4.5 .15 0 4 0 PD3
4 4 .06 1.77 4.4 .15 0 4 0 PD4
4 4 .06 1.77 4.3 .15 0 4 0 PD5
4 4 .06 1.77 4.2 .15 0 4 0 PD6
4 4 .06 1.77 4.1 .15 0 4 0 PD7
4 4 .06 3.03 5.6 .15 0 6 0 PA0
4 4 .06 3.03 5.5 .15 0 6 0 PA1
4 4 .06 3.03 5.4 .15 0 6 0 PA2
4 4 .06 3.03 5.3 .15 0 6 0 PA3
4 4 .06 3.03 5.2 .15 0 6 0 PA4
4 4 .06 3.03 5.1 .15 0 6 0 PA5
4 4 .06 3.03 5.0 .15 0 6 0 PA6
4 4 .06 3.03 4.9 .15 0 6 0 PA7
4 4 .06 3.03 4.8 .15 0 6 0 PC7
4 4 .06 3.03 4.7 .15 0 6 0 PC6
4 4 .06 3.03 4.6 .15 0 6 0 PC5
4 4 .06 3.03 4.5 .15 0 6 0 PC4
4 4 .06 3.03 4.4 .15 0 6 0 PC3
4 4 .06 3.03 4.3 .15 0 6 0 PC2
4 4 .06 3.03 4.2 .15 0 6 0 PC1
4 4 .06 3.03 4.1 .15 0 6 0 PC0
4 4 .06 2.3 3.97 .16 0 1 0 GND
4 4 .06 2.6 3.97 .22 0 1 0 AGND
4 4 .06 1.9 5.73 .22 0 9 0 /RST
4 4 .06 1.9 3.97 .15 0 1 0 XT2
4 4 .06 2.1 3.97 .15 0 1 0 XT1
4 4 .06 2.3 5.73 .16 0 9 0 VCC
4 4 .06 2.6 5.73 .22 0 9 0 AVCC
4 4 .06 2.9 5.73 .22 0 9 0 AREF
4 5 .06 1.72 5.6 .03 0 14 0 1
4 5 .06 1.72 5.5 .03 0 14 0 2
4 5 .06 1.72 5.4 .03 0 14 0 3
4 5 .06 1.72 5.3 .03 0 14 0 4
4 5 .06 1.72 5.2 .03 0 14 0 5
4 5 .06 1.72 5.1 .03 0 14 0 6
4 5 .06 1.72 5.0 .03 0 14 0 7
4 5 .06 1.72 4.9 .03 0 14 0 8
4 5 .06 1.72 4.8 .08 0 14 0 14
4 5 .06 1.72 4.7 .08 0 14 0 15
4 5 .06 1.72 4.6 .08 0 14 0 16
4 5 .06 1.72 4.5 .08 0 14 0 17
4 5 .06 1.72 4.4 .08 0 14 0 18
4 5 .06 1.72 4.3 .08 0 14 0 19
4 5 .06 1.72 4.2 .08 0 14 0 20
4 5 .06 1.72 4.1 .08 0 14 0 21
4 5 .06 3.08 5.6 .08 0 12 0 40
4 5 .06 3.08 5.5 .08 0 12 0 39
4 5 .06 3.08 5.4 .08 0 12 0 38
4 5 .06 3.08 5.3 .08 0 12 0 37
4 5 .06 3.08 5.2 .08 0 12 0 36
4 5 .06 3.08 5.1 .08 0 12 0 35
4 5 .06 3.08 5.0 .08 0 12 0 34
4 5 .06 3.08 4.9 .08 0 12 0 33
4 5 .06 3.08 4.8 .08 0 12 0 29
4 5 .06 3.08 4.7 .08 0 12 0 28
4 5 .06 3.08 4.6 .08 0 12 0 27
4 5 .06 3.08 4.5 .08 0 12 0 26
4 5 .06 3.08 4.4 .08 0 12 0 25
4 5 .06 3.08 4.3 .08 0 12 0 24
4 5 .06 3.08 4.2 .08 0 12 0 23
4 5 .06 3.08 4.1 .08 0 12 0 22
4 5 .06 2.28 3.93 .08 0 10 0 11
4 5 .06 2.58 3.93 .08 0 10 0 31
4 5 .06 1.88 5.77 .03 0 2 0 9
4 5 .06 1.88 3.93 .08 0 10 0 12
4 5 .06 2.08 3.93 .08 0 10 0 13
4 5 .06 2.28 5.77 .08 0 2 0 10
4 5 .06 2.58 5.77 .08 0 2 0 30
4 5 .06 2.88 5.77 .08 0 2 0 32
3 0 0 1.7 5.6 .01 .01 0 0 1
3 0 0 1.7 5.5 .01 .01 0 0 2
3 0 0 1.7 5.4 .01 .01 0 0 3
3 0 0 1.7 5.3 .01 .01 0 0 4
3 0 0 1.7 5.2 .01 .01 0 0 5
3 0 0 1.7 5.1 .01 .01 0 0 6
3 0 0 1.7 5.0 .01 .01 0 0 7
3 0 0 1.7 4.9 .01 .01 0 0 8
3 0 0 1.7 4.8 .01 .01 0 0 14
3 0 0 1.7 4.7 .01 .01 0 0 15
3 0 0 1.7 4.6 .01 .01 0 0 16
3 0 0 1.7 4.5 .01 .01 0 0 17
3 0 0 1.7 4.4 .01 .01 0 0 18
3 0 0 1.7 4.3 .01 .01 0 0 19
3 0 0 1.7 4.2 .01 .01 0 0 20
3 0 0 1.7 4.1 .01 .01 0 0 21
3 0 0 3.1 5.6 .01 .01 0 0 40
3 0 0 3.1 5.5 .01 .01 0 0 39
3 0 0 3.1 5.4 .01 .01 0 0 38
3 0 0 3.1 5.3 .01 .01 0 0 37
3 0 0 3.1 5.2 .01 .01 0 0 36
3 0 0 3.1 5.1 .01 .01 0 0 35
3 0 0 3.1 5.0 .01 .01 0 0 34
3 0 0 3.1 4.9 .01 .01 0 0 33
3 0 0 3.1 4.8 .01 .01 0 0 29
3 0 0 3.1 4.7 .01 .01 0 0 28
3 0 0 3.1 4.6 .01 .01 0 0 27
3 0 0 3.1 4.5 .01 .01 0 0 26
3 0 0 3.1 4.4 .01 .01 0 0 25
3 0 0 3.1 4.3 .01 .01 0 0 24
3 0 0 3.1 4.2 .01 .01 0 0 23
3 0 0 3.1 4.1 .01 .01 0 0 22
3 0 0 2.3 3.9 .01 .01 0 0 11
3 0 0 2.6 3.9 .01 .01 0 0 31
3 0 0 1.9 5.8 .01 .01 0 0 9
3 0 0 1.9 3.9 .01 .01 0 0 12
3 0 0 2.1 3.9 .01 .01 0 0 13
3 0 0 2.3 5.8 .01 .01 0 0 10
3 0 0 2.6 5.8 .01 .01 0 0 30
3 0 0 2.9 5.8 .01 .01 0 0 32
1 1 .01 1.75 5.6 1.7 5.6
1 1 .01 1.75 5.5 1.7 5.5
1 1 .01 1.75 5.4 1.7 5.4
1 1 .01 1.75 5.3 1.7 5.3
1 1 .01 1.75 5.2 1.7 5.2
1 1 .01 1.75 5.1 1.7 5.1
1 1 .01 1.75 5.0 1.7 5.0
1 1 .01 1.75 4.9 1.7 4.9
1 1 .01 1.75 4.8 1.7 4.8
1 1 .01 1.75 4.7 1.7 4.7
1 1 .01 1.75 4.6 1.7 4.6
1 1 .01 1.75 4.5 1.7 4.5
1 1 .01 1.75 4.4 1.7 4.4
1 1 .01 1.75 4.3 1.7 4.3
1 1 .01 1.75 4.2 1.7 4.2
1 1 .01 1.75 4.1 1.7 4.1
1 1 .01 3.05 5.6 3.1 5.6
1 1 .01 3.05 5.5 3.1 5.5
1 1 .01 3.05 5.4 3.1 5.4
1 1 .01 3.05 5.3 3.1 5.3
1 1 .01 3.05 5.2 3.1 5.2
1 1 .01 3.05 5.1 3.1 5.1
1 1 .01 3.05 5.0 3.1 5.0
1 1 .01 3.05 4.9 3.1 4.9
1 1 .01 3.05 4.8 3.1 4.8
1 1 .01 3.05 4.7 3.1 4.7
1 1 .01 3.05 4.6 3.1 4.6
1 1 .01 3.05 4.5 3.1 4.5
1 1 .01 3.05 4.4 3.1 4.4
1 1 .01 3.05 4.3 3.1 4.3
1 1 .01 3.05 4.2 3.1 4.2
1 1 .01 3.05 4.1 3.1 4.1
1 1 .01 2.3 3.95 2.3 3.9
1 1 .01 2.6 3.95 2.6 3.9
1 1 .01 1.9 5.75 1.9 5.8
1 1 .01 1.9 3.95 1.9 3.9
1 1 .01 2.1 3.95 2.1 3.9
1 1 .01 2.3 5.75 2.3 5.8
1 1 .01 2.6 5.75 2.6 5.8
1 1 .01 2.9 5.75 2.9 5.8
1 1 .01 1.75 3.95 3.05 3.95
1 1 .01 3.05 3.95 3.05 5.75
1 1 .01 3.05 5.75 1.75 5.75
1 1 .01 1.75 5.75 1.75 3.95
0 6 0 2.4 4.9
6 0 270 1.861 3.495 2.139 3.905 XTALPAKX1?MHz?PX400
4 1 .06 2.26 3.8 .09 90 10 0 &1
4 1 .06 2.18 3.8 .21 90 10 0 &2
3 0 0 2.1 3.9 .01 .01 0 270 1
3 0 0 2.0 3.5 .01 .01 0 270 2
3 0 0 1.9 3.9 .01 .01 0 270 3
3 1 3 2.1 3.8 .04 .04 0 270 0
3 1 3 1.9 3.8 .04 .04 0 270 0
3 1 3 2.0 3.6 .04 .04 0 270 0
2 1 .01 2.1 3.8 .09 247.50 45 0 0
2 1 .01 2.1 3.6 .09 67.50 45 0 0
2 1 .01 1.9 3.6 .09 67.50 45 0 0
2 1 .01 1.9 3.8 .09 247.50 45 0 0
1 1 .01 2.02 3.84 2.02 3.76
1 1 .01 2.02 3.76 1.98 3.76
1 1 .01 1.98 3.76 1.98 3.84
1 1 .01 1.98 3.84 2.02 3.84
1 1 .01 2.04 3.84 2.04 3.76
1 1 .01 1.96 3.76 1.96 3.84
1 1 .01 1.96 3.8 1.9 3.8
1 1 .01 1.9 3.8 1.9 3.71
1 1 .01 2.04 3.8 2.1 3.8
1 1 .01 2.1 3.8 2.1 3.71
1 1 .01 2.1 3.69 2.1 3.6
1 1 .01 2.1 3.6 1.9 3.6
1 1 .01 1.9 3.6 1.9 3.69
1 1 .01 2.0 3.6 2.0 3.5
1 1 .01 2.1 3.8 2.1 3.9
1 1 .01 1.9 3.8 1.9 3.9
0 6 0 2.0 3.7
1 2 .01 1.9 6.5 1.9 5.8
1 2 .01 1.9 5.8 1.9 6.0
1 2 .01 1.9 6.0 1.8 6.0
1 2 .01 1.6 6.0 1.5 6.0
1 2 .01 1.5 6.0 1.5 5.9
1 2 .01 1.9 7.0 1.9 6.9
1 2 .01 2.9 5.9 2.9 5.8
1 2 .01 2.3 5.8 2.3 5.9
1 2 .01 2.6 5.8 2.6 6.2
1 2 .01 2.6 3.8 2.6 3.9
1 2 .01 2.3 3.9 2.3 3.8
1 2 .01 2.6 3.8 2.6 3.7
1 2 .01 3.1 5.6 3.5 5.6
1 2 .01 3.1 5.5 3.5 5.5
1 2 .01 3.1 5.4 3.5 5.4
1 2 .01 3.1 5.3 3.5 5.3
1 2 .01 3.1 5.2 3.5 5.2
1 2 .01 3.1 5.1 3.5 5.1
1 2 .01 1.7 5.1 1.5 5.1
1 2 .01 1.7 5.0 1.5 5.0
1 2 .01 1.7 4.9 1.5 4.9
1 2 .01 2.6 3.8 2.3 3.8
1 2 .01 1.7 5.3 1.5 5.3
1 2 .01 1.5 5.4 1.7 5.4
1 2 .01 2.3 5.9 2.9 5.9
1 2 .01 3.5 5.0 3.1 5.0
1 2 .01 1.5 5.6 1.7 5.6
1 2 .01 1.7 5.5 1.5 5.5
1 2 .01 1.5 4.1 1.7 4.1
1 2 .01 1.7 4.2 1.5 4.2
1 2 .01 1.5 4.3 1.7 4.3
1 2 .01 1.7 4.4 1.5 4.4
1 2 .01 1.5 4.6 1.7 4.6
1 2 .01 1.7 5.2 1.5 5.2
1 2 .01 1.7 4.5 1.5 4.5
1 2 .01 1.5 6.4 1.9 6.4

END OF FILE