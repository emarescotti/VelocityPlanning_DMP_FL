# SPDX-License-Identifier: AGPL-3.0-or-later

import numpy as np
import math
import matplotlib.pyplot as plt
from genStraight import funStraight
from genSawTooth import funSawtooth
from genCurve import funCurve 

# SQUARE SPIRAL

# line 1
X_pts = [0,150,150,0,0,135,135,15,15,120,120,30,30,105,105,45,45,90,90,60,60,75]
Y_pts = [0,0,-150,-150,-15,-15,-135,-135,-30,-30,-120,-120,-45,-45,-105,-105,-60,-60,-90,-90,-75,-75]

# final plotting
plt.figure(1)
plt.grid()
plt.title('Generic Path [dim: mm]')
plt.plot(X_pts,Y_pts,'o-')
plt.get_current_fig_manager().window.showMaximized()
plt.show()

np.savez('test1',[X_pts,Y_pts])