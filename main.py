import os, sys
from walking_generator.base import BaseGenerator
from walking_generator.classic import ClassicGenerator
from walking_generator.interpolation import Interpolation

if __name__ == '__main__':
    gen = ClassicGenerator()
    gen.simulate()
    gen.update()
    gen.solve()

    inter = Interpolation()


    sys.exit()
    for i in range(gen.N*2):
        print 'iteration: ', i
        gen.update()
        raw_input('press return to continue: ')


