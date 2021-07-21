import pickle
import matplotlib.pyplot as plt
import config

figure_name = config.save_figs_location + '/pickle/Joint 1, data 1: Position.pickle'
fig = pickle.load(open(figure_name,'rb'))
plt.show()

data = fig.axes[0].lines[0].get_data()
print(data)

