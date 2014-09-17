from walking_generator.classic import ClassicGenerator
from walking_generator.helper import Plotter

gen = ClassicGenerator()
show_canvas=True
save2file=False

plotter = Plotter(gen, show_canvas, save2file)
plotter.load_from_file('./data.json')
#plotter.create_waterfall_plot()
plotter.create_reference_plot()

if __name__=='__main__':
    raw_input('press key: ')
