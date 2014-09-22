from walking_generator.classic import ClassicGenerator
from walking_generator.visualization import Plotter

gen = ClassicGenerator()
gen = None
show_canvas=True
save_to_file=False

plotter = Plotter(generator=gen, show_canvas=show_canvas, save_to_file=save_to_file)
plotter.load_from_file('./data.json')
plotter.update()
plotter.create_reference_plot()

if __name__=='__main__':
    raw_input('press key: ')
