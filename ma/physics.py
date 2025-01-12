from manim import *
import numpy as np
import math
from typing import Callable

MAX_U = 3 * PI
SKIP_SECTIONS = False


class Physics(ThreeDScene):
    def construct(self) -> None:
        # inital scene
        self.next_section(skip_animations=SKIP_SECTIONS)
        self.set_camera_orientation(phi=60 * DEGREES, theta=-60 * DEGREES)

        xr = [0, MAX_U]
        yr = [-5, 5]
        zr = [-4, 4]
        xl = xr[1] - xr[0]
        yl = yr[1] - yr[0]
        zl = zr[1] - zr[0]
        self.axes_3d = ThreeDAxes(
            x_range=xr, y_range=yr, z_range=zr, x_length=xl, y_length=yl, z_length=zl
        ).move_to(np.array([0, 0, -1]))

        constant = Text("Constant").to_corner(UR)
        self.add_fixed_in_frame_mobjects(constant)
        varies = Text("Varies").to_edge(RIGHT)
        self.add_fixed_in_frame_mobjects(varies)
        labels = self.axes_3d.get_axis_labels()
        self.add(labels)
        self.add(self.axes_3d)

        # show hl curve
        self.next_section(skip_animations=SKIP_SECTIONS)

        curve = ParametricFunction(f(self.axes_3d), t_range=np.array([0, MAX_U]))
        self.add(curve)
        curve_label = (
            MathTex(r"\vec{r}")
            .add_background_rectangle(color=BLACK)
            .move_to(f(self.axes_3d)(MAX_U) + np.array([0, 0, 0]))
        )
        self.add_fixed_orientation_mobjects(curve_label)
        vec_r = MathTex(r"\vec{r}").next_to(constant, DOWN, aligned_edge=LEFT)
        self.add_fixed_in_frame_mobjects(
            vec_r
        )
        self.play(Create(curve_label), Create(curve), Create(vec_r))

        # parameterize by u
        self.next_section()

        self.u: float = 1.2
        self.u_label = MathTex(r"\vec{r}(u)")
        self.add_fixed_orientation_mobjects(self.u_label)
        self.hl_pos_dot = Dot3D().move_to(f(self.axes_3d)(self.u))
        self.add(self.hl_pos_dot)
        self.play(Create(self.hl_pos_dot))
        self.hl_pos_dot.add_updater(self.update_hl_pos)
        self.u_label.add_updater(
            self.update_u_label
        )
        self.add_updater(self.increase_u)

        self.wait(5, frozen_frame=False)
        #self.u_label.a
        #self.play(self.u_label.animate.opacity(0.2))

    def increase_u(self, dt: float):
        self.u += dt
    
    def update_u_label(self, m: Mobject) -> None:
        if isinstance(m, MathTex):
            m.next_to(self.hl_pos_dot, UP)
        else:
            raise TypeError

    def update_hl_pos(self, m: Mobject) -> None:
        #print(self.u)
        #self.u += dt
        m.move_to(f(self.axes_3d)(self.u))


def f(axes: ThreeDAxes) -> Callable[[float], np.ndarray]:
    def f_(t: float) -> np.ndarray:
        p = np.array([t, 0, 2 - t / 3 + math.sin(t / 3)])
        return axes.c2p(*p)
    return f_
