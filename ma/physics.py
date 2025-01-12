from manim import *
import numpy as np
import math
from typing import Callable

O = 1.0
MAX_U: float = 3 * PI
SKIP_SECTIONS: bool = True

u_label = None
axes_3d = None
u = 1.2
u2 = 0.0
hl_pos_dot = None

up_theta = 0 * DEGREES


class Physics(ThreeDScene):
    def construct(self) -> None:
        global axes_3d, u_label, u, hl_pos_dot, up_theta, u2
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

        # constant = Text("Constant").to_corner(UR)
        # self.add_fixed_in_frame_mobjects(constant)
        # varies = Text("Varies").to_edge(RIGHT)
        # self.add_fixed_in_frame_mobjects(varies)
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
        # vec_r = MathTex(r"\vec{r}").next_to(constant, DOWN, aligned_edge=LEFT)
        # self.add_fixed_in_frame_mobjects(vec_r)
        self.play(Create(curve_label), Create(curve))  # , Create(vec_r))

        # parameterize by u
        self.next_section(skip_animations=SKIP_SECTIONS)

        u_label = MathTex(r"\vec{r}(u)")
        self.add_fixed_orientation_mobjects(u_label)
        hl_pos_dot = Dot3D().move_to(r(u))
        self.add(hl_pos_dot)
        self.play(Create(hl_pos_dot))

        hl_pos_dot.add_updater(update_hl_pos)
        u_label.add_updater(update_u_label)
        self.add_updater(increase_u)

        self.wait(1)

        self.remove_updater(increase_u)

        self.play(u_label.animate.set_opacity(0.3))
        self.wait()

        # com pos
        self.next_section(skip_animations=SKIP_SECTIONS)
        # up_vec = dr(u)

        up_vec = lambda: normalize(
            make_ortho_to(np.array([0, math.sin(up_theta), math.cos(up_theta)]), dr(u))
        )

        # up = always_redraw(lambda: Arrow3D(r(u), r(u) + up_vec()))

        up_label = MathTex(r"\vec{\mathrm{up}}")
        # up_label.next_to(up, np.array([0.2, 1.0, 0.0]))

        # self.add(up)
        self.add_fixed_orientation_mobjects(up_label.add_background_rectangle())
        self.wait(1)

        com_pos = lambda: r(u) - up_vec() * O
        # com_pos_dot = always_redraw(lambda: Dot3D(com_pos()))
        com_pos_label = MathTex(r"\vec{x}")
        # com_pos_label.next_to(com_pos_dot, np.array([0.0, 0.0, -1.5]))

        """
        up_theta = -45 * DEGREES
        self.add(com_pos_dot)
        self.add_fixed_orientation_mobjects(com_pos_label.add_background_rectangle())

        self.wait()

        # rotate up
        self.add_updater(rotate_up)

        self.wait_until(lambda: up_theta >= 0 * DEGREES)
        self.remove_updater(rotate_up)

        up_theta = 0

        self.wait()"""

        self.next_section(skip_animations=SKIP_SECTIONS)

        vel_upt = MathTex(r"v_{n+1}=v_n+a \Delta t").to_corner(UR)
        pos_upt = MathTex(r"x_{n+1}=x_n+ v_{n+1} \Delta t").next_to(
            vel_upt, DOWN, aligned_edge=RIGHT
        )

        self.add_fixed_in_frame_mobjects(vel_upt, pos_upt)
        self.play(Create(vel_upt), Create(pos_upt))

        vel_upt2 = MathTex(r"v_{n+1}=v_n + ( N + g ) \Delta t").to_corner(UR)
        self.add_fixed_in_frame_mobjects(vel_upt2)
        vel_upt2.next_to(vel_upt, DOWN, aligned_edge=RIGHT)
        self.play(
            Create(vel_upt2),
            pos_upt.animate.next_to(vel_upt2, DOWN, aligned_edge=RIGHT),
        )
        self.play(
            FadeOut(vel_upt),
            vel_upt2.animate.to_corner(UR),
            pos_upt.animate.next_to(vel_upt2, DOWN, aligned_edge=RIGHT),
        )

        vel_upt3 = MathTex(r"v_{n+1}=v_n+N \Delta t + g \Delta t").to_corner(UR)
        self.add_fixed_in_frame_mobjects(vel_upt3)
        vel_upt3.next_to(vel_upt2, DOWN, aligned_edge=RIGHT)
        self.play(
            Create(vel_upt3),
            pos_upt.animate.next_to(vel_upt3, DOWN, aligned_edge=RIGHT),
        )
        self.play(FadeOut(vel_upt2), vel_upt3.animate.to_corner(UR))

        vel_upt4 = MathTex(r"v_{n+1}=\mathrm{rot}(v_n) + g \Delta t").to_corner(UR)
        self.add_fixed_in_frame_mobjects(vel_upt4)
        vel_upt4.next_to(vel_upt3, DOWN, aligned_edge=RIGHT)
        self.play(
            Create(vel_upt4),
            pos_upt.animate.next_to(vel_upt4, DOWN, aligned_edge=RIGHT),
        )
        self.play(
            FadeOut(vel_upt3),
            vel_upt4.animate.to_corner(UR),
            pos_upt.animate.next_to(vel_upt3, DOWN, aligned_edge=RIGHT),
        )

        pos_upt2 = MathTex(
            r"x_{n+1}=\underset{\text{center}}{\boxed{x_n + g\Delta t^2}} + \mathrm{rot}(\underset{\text{radius}}{\boxed{v_n\Delta t}})"
        )
        self.add_fixed_in_frame_mobjects(pos_upt2)
        pos_upt2.next_to(pos_upt, DOWN, aligned_edge=RIGHT)
        self.play(Create(pos_upt2))
        self.wait()
        sphere = Sphere(com_pos() - np.array([0, 0, 0.1]), 0.2)
        sphere.z_index = 1
        self.play(
            FadeOut(pos_upt),
            FadeOut(vel_upt4),
            pos_upt2.animate.to_corner(UR),
            Create(sphere),
        )
        self.wait()

        self.next_section(skip_animations=False)#SKIP_SECTIONS)

        constr2 = MathTex(r"x_{n+1}-\vec{r}(u_{n+1})=\vec{V}")
        txt = MathTex(r"\text{where } ||\vec{V}||=o\text{ and }\vec{V}\cdot\vec{r'}(u_{n+1})=0")
        txt.scale(0.75)
        constr2.next_to(pos_upt2, DOWN, aligned_edge=RIGHT)
        txt.next_to(constr2, DOWN, aligned_edge=RIGHT)
        self.add_fixed_in_frame_mobjects(constr2, txt)
        self.play(Create(constr2), Create(txt))

        self.wait()

        u2 = u + 1.0
        circ = Circle(O)
        circ.save_state()
        def update_circ(_):
            global u2 
            circ.restore()
            circ.save_state()
            circ.rotate(PI/2+angle_between_vectors(dr(u2), RIGHT),UP)
            circ.move_to(r(u2))

        circ.add_updater(update_circ)
        circ.rotate(PI/2+angle_between_vectors(dr(u2), RIGHT),UP)
        circ.move_to(r(u2))
        self.play(Create(circ))
        def updt_u(dt):
            global u2 
            u2 -= dt
        self.add_updater(updt_u)

        self.wait()

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
    if axes_3d is None:
        raise ValueError
    m.move_to(r(u))


def r(t: float) -> np.ndarray:
    p = np.array([t, 0, 2 - t / 3 + math.sin(t / 3)])
    if axes_3d is None:
        raise ValueError
    return axes_3d.c2p(*p)


def dr(t: float) -> np.ndarray:
    dx = 1
    dy = 0
    dz = -1 / 3 + (1 / 3) * math.cos(t / 3)
    d = np.array([dx, dy, dz])
    return d


def rotate_up(t: float) -> None:
    global up_theta
    up_theta += t * 2 * PI
