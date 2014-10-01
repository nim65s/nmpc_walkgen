r="./wieber2010python.csv"
l="./NMPCpython.csv"
plot \
r u ($0/200):1 w l t "com y", \
r u ($0/200):14 w l t "RF y", \
r u ($0/200):18 w l t "LF y", \
l u ($0/200):1 w l t "com y", \
l u ($0/200):14 w l t "RF y", \
l u ($0/200):18 w l t "LF y"
