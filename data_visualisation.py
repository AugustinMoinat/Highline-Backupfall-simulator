import pandas as pd
import matplotlib.pyplot as plt


def data_visual(data, folder, segments, save, show):
    plt.style.use('Solarize_Light2')
    plt.rcParams["figure.figsize"] = (10, 6)
    plt.rcParams["figure.autolayout"] = True
    fig, ax = plt.subplots()
    plt.ylabel("Force in kN")
    plt.xlabel("Seconds")
    y = data['max_force']
    y2 = data['leash']
    x = data['time']
    plt.plot(x, y, color='#595959', linewidth=0.4)
    plt.plot(x, y2, color='r', linewidth=0.3)
    for i in range(segments):
        if i != 0:
            plt.plot(x, data['split ' + str(i-1)], color='c', linewidth=0.2)
        plt.plot(x, data['main ' + str(i)], color='b', linewidth=0.2)
        plt.plot(x, data['backup ' + str(i)], color='g', linewidth=0.2)
    props = dict(boxstyle='round', facecolor='#fdf6e4', alpha=0.5)
    ax.text(0.8, 0.95, 'main', transform=ax.transAxes, fontsize=12,
            verticalalignment='top', color='b', bbox=props)
    ax.text(0.88, 0.95, 'backup', transform=ax.transAxes, fontsize=12,
            verticalalignment='top', color='g', bbox=props)
    ax.text(0.72, 0.95, 'leash', transform=ax.transAxes, fontsize=12,
            verticalalignment='top', color='r', bbox=props)
    if save:
        plt.savefig(folder + 'graph.svg', dpi=300)
    if show:
        plt.show()


if __name__ == "__main__":
    example = pd.read_csv('22-10-27-22-36-21-TEST.csv')
    data_visual(example, 'test', True, True)