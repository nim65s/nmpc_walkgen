period = 0.005
T = 0.1

set size 1,1
set origin 0.0,0.0

wf = "walkForward2m_sInterpolation.dat"
ws = "walkSideward2m_sInterpolation.dat"

set terminal postscript eps enhanced colour
set output "/home/mnaveau/devel/Walking-Pattern-Generator-Prototype/tests/data/ReferenceTrajectories.eps"

set multiplot
set title 'Reference trajectory from Andrei Herdt algorithm'

set size 1,0.5
set origin 0.0,0.5

set xlabel 'time (s)'
set ylabel 'Walking forward'
set yrange [-0.2:3.0]
plot \
 wf u 1:2 w l t "CoM x (in m)",\
 wf u 1:14 w l t "ZMP x",\
 wf u 1:16 w l t "LF x",\
 wf u 1:28 w l t "RF x",\
 wf u 1:3 w l lc 7 t "CoM y",\
 wf u 1:15 w l lc 8 t "ZMP y",\
 wf u 1:17 w l lc 9 t "LF y",\
 wf u 1:29 w l lc 10 t "RF y"


set origin 0.0,0.0

set xlabel 'time (s)'
set ylabel 'Walking sideward'
set yrange [-0.2:3.0]
plot \
 ws u 1:2 w l t "CoM x (in m)",\
 ws u 1:14 w l t "ZMP x",\
 ws u 1:16 w l t "LF x",\
 ws u 1:28 w l t "RF x",\
 ws u 1:3 w l lc 7 t "CoM y",\
 ws u 1:15 w l lc 8 t "ZMP y",\
 ws u 1:17 w l lc 9 t "LF y",\
 ws u 1:29 w l lc 10 t "RF y"

unset multiplot
