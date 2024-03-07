import numpy as np
import matplotlib.pyplot as plt

sa_counts = np.array([28, 29, 38, 38, 40])
category_names = [
    'NA',
    'FSC',
    'FSA',
    'VSC',
    'VSA'
]

import matplotlib.colors as mcolors
# Get the RGBA value of gray
rgba_gray = mcolors.to_rgba('gray')
print(rgba_gray)
# Get the colormap "Set2"
cmap = plt.get_cmap("Paired")
# Extract the first five colors
colors = [cmap(i) for i in range(4)]
colors.insert(0, rgba_gray)

plt.bar(category_names, sa_counts, color=colors, align='center', alpha=0.5, ecolor='black')
plt.title('Situation Awareness')
plt.ylabel('Total number of obstacles reported')
plt.xlabel('Conditions')

plt.show()