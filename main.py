import os, sys
from walking_generator.base import BaseGenerator
from walking_generator.classic import ClassicGenerator

if __name__ == '__main__':
    gen = ClassicGenerator()
    gen.simulate()
    gen.update()
    gen.solve()

    sys.exit()
    for i in range(gen.N*2):
        print 'iteration: ', i
        gen.update()
        raw_input('press return to continue: ')
