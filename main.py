from walking_generator.base import BaseGenerator
from walking_generator.classic import ClassicGenerator

if __name__ == '__main__':
    gen = BaseGenerator()
    gen.simulate()

    try:
        gen.solve()
    except:
        print 'Exception!'

    gen = ClassicGenerator()
    gen.simulate()
    gen.solve()

