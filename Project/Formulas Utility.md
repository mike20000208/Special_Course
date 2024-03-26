# Formulas

For a plane represented as $ax + by + cz + d = 0$, the distance between arbitrary point $P(x_{0}, y_{0}, z_{0})$ and the plane will be: 


$$
\begin{align}
\begin{split}

d & = \frac{|ax_{0} + by_{0} + cz_{0} + d|}{\sqrt{a^2 + b^2 + c^2}}

\end{split}
\end{align}
$$


Assume that we have a destination represented by a vector $\vec{v_{1}}$, which points out the direction from the origin to the destination, and the center of mass of a slice is represented by another vector $\vec{v_{2}}$, which is the vector from the origin to the center of mass. In this case, I define the angle between the slice and the destination is: 

$$
\begin{align}
angle & = acos(\frac{\vec{v_{1}} \cdot \vec{v_{2}}}{|\vec{v_{1}}||\vec{v_{1}}|})
\end{align}
$$


