import matplotlib.pyplot as plt
import numpy as np

# # 模擬數據
# years = np.linspace(1800, 2020, 100)
# population = np.sin(np.linspace(0, 10, 100)) * 10 + 50  # 人口變化的模擬數據

# # 畫出有粗細變化的線條
# fig, ax = plt.subplots()
# for i in range(len(years) - 1):
#     x = [years[i], years[i + 1]]
#     y = [population[i], population[i + 1]]
#     width = population[i] / 2  # 將人口數縮放為線條寬度
#     ax.plot(x, y, linewidth=width, color='blue')

# # 添加標籤
# ax.set_xlabel('Year')
# ax.set_ylabel('Population (Millions)')
# ax.set_title('Historical Population Over Time')

# plt.show()

import numpy as np
from matplotlib.collections import LineCollection
import matplotlib.pyplot as plt
x = np.linspace(0,4*np.pi,10000)
y = np.cos(x)
lwidths=1+x[:-1]
points = np.array([x, y]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)
lc = LineCollection(segments, linewidths=lwidths,color='blue')
fig,a = plt.subplots()
a.add_collection(lc)
a.set_xlim(0,4*np.pi)
a.set_ylim(-1.1,1.1)
plt.show()