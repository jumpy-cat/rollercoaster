from manim import *
import numpy as np
import math
from typing import Callable

MAX_U: float = 3 * PI
SKIP_SECTIONS: bool = True

u_label = None
axes_3d = None
u = 1.2
hl_pos_dot = None


class Physics(ThreeDScene):
    def construct(self) -> None:
        global axes_3d, u_label, u, hl_pos_dot
        # inital scene
        self.next_section(skip_animations=SKIP_SECTIONS)
        self.set_camera_orientation(phi=60 * DEGREES, theta=-60 * DEGREES)

        xr = [0, MAX_U]
        yr = [-5, 5]
        zr = [-4, 4]
        xl = xr[1] - xr[0]
        yl = yr[1] - yr[0]
        zl = zr[1] - zr[0]
        axes_3d = ThreeDAxes(
            x_range=xr, y_range=yr, z_range=zr, x_length=xl, y_length=yl, z_length=zl
        ).move_to(np.array([0, 0, -1]))

        constant = Text("Constant").to_corner(UR)
        self.add_fixed_in_frame_mobjects(constant)
        varies = Text("Varies").to_edge(RIGHT)
        self.add_fixed_in_frame_mobjects(varies)
        labels = axes_3d.get_axis_labels()
        self.add(labels)
        self.add(axes_3d)

        # show hl curve
        self.next_section(skip_animations=SKIP_SECTIONS)

        curve = ParametricFunction(r, t_range=np.array([0, MAX_U]))
        self.add(curve)
        curve_label = (
            MathTex(r"\vec{r}")
            .add_background_rectangle(color=BLACK)
            .move_to(r(MAX_U) + np.array([0, 0, 0]))
        )
        self.add_fixed_orientation_mobjects(curve_label)
        vec_r = MathTex(r"\vec{r}").next_to(constant, DOWN, aligned_edge=LEFT)
        self.add_fixed_in_frame_mobjects(
            vec_r
        )
        self.play(Create(curve_label), Create(curve), Create(vec_r))

        # parameterize by u
        self.next_section()

        u_label = MathTex(r"\vec{r}(u)")
        self.add_fixed_orientation_mobjects(u_label)
        hl_pos_dot = Dot3D().move_to(r(u))
        self.add(hl_pos_dot)
        self.play(Create(hl_pos_dot))
        hl_pos_dot.add_updater(update_hl_pos)
        u_label.add_updater(
            update_u_label
        )
        self.add_updater(increase_u)

        self.wait(1)
        self.remove_updater(increase_u)
        self.play(u_label.animate.set_opacity(0.3))
        self.wait()
        
        # com pos
        self.next_section()
        up_vec = dr(u)
        #up_vec = np.array([0,0,1])
        #up_vec = normalize(make_ortho_to(np.array([0, 0, 1]), dr(u)))
        #up = Line3D(up_vec).put_start_and_end_on(r(u), r(u) + up_vec)
        up = Line3D(r(u), r(u)+up_vec)
        print(r(u))
        print(up_vec)
        self.add(up)
        self.wait(1)

        self.interactive_embed()


def vec_proj(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.dot(a, b) / np.linalg.norm(b) * b / np.linalg.norm(b)


def vec_normalize(a: np.ndarray) -> np.ndarray:
    return a / np.linalg.norm(a)


def make_ortho_to(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return a - vec_proj(a, b)

    
def update_u_label(m: Mobject) -> None:
    if isinstance(m, MathTex):
        if hl_pos_dot is None:
            raise ValueError
        m.next_to(hl_pos_dot, UP)
    else:
        raise TypeError


def increase_u(dt: float):
    global u
    u += dt


def update_hl_pos(m: Mobject) -> None:
    #print(self.u)
    #self.u += dt
    if axes_3d is None:
        raise ValueError
    m.move_to(r(u))


def r(t: float) -> np.ndarray:
    p = np.array([t, 0, 2 - t / 3 + math.sin(t / 3)])
    if axes_3d is None:
        raise ValueError
    return axes_3d.c2p(*p)


def dr(t: float) -> np.ndarray:
    # Calculate the derivative of the position vector r(t)
    dx = 1  # Derivative of x with respect to t
    dy = 0  # Derivative of y with respect to t
    dz = -1 / 3 + (1 / 3) * math.cos(t / 3)  # Corrected derivative of z with respect to t
    d = np.array([dx, dy, dz])
    if axes_3d is None:
        raise ValueError
    return axes_3d.c2p(*d)
