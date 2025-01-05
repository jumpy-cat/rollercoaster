from manim import *


class CreateCircle(Scene):
    def construct(self):
        circle = Circle()  # create a circle
        circle.set_fill(PINK, opacity=0.5)  # set the color and transparency
        self.play(Create(circle))  # show the circle on screen

class SquareToCircle(Scene):
    def construct(self):
        circle = Circle()  # create a circle
        circle.set_fill(PINK, opacity=0.5)  # set color and transparency

        square = Square()  # create a square
        square.rotate(PI / 4)  # rotate a certain amount

        self.play(Create(square))  # animate the creation of the square
        self.play(Transform(square, circle))  # interpolate the square into the circle
        self.play(FadeOut(square))  # fade out animation


class Interp(Scene):
    def construct(self):

        p = (-2, 1)
        q = (3,2)

        p_dot = Dot(point=(*p, 0))
        pp
        q_dot = Dot(point=(q, 0))

        circle = Circle()
        circle.set_fill(PINK, opacity=0.5)
        eq = MathTex(r'\vec{r}(u)=Au^7+Bu^6+Cu^5+Du^4+Eu^3+Fu^2+Gu+H')
        mat = Matrix([
                [0,0,0,0,0,0,0,1],
                [0,0,0,0,0,0,1,0],
                [0,0,0,0,0,2,0,0],
                [0,0,0,0,6,0,0,0],
                [1]*8,
                [7-i for i in range(8)],
                [(7-i)*(6-i) for i in range(8)],
                [(7-i)*(6-i)*(5-i) for i in range(8)]
        ])
        mat.scale(0.5)
        mat.next_to(eq, UP)

        param_mat = Matrix([[ch] for ch in 'ABCDEFGH'])
        param_mat.scale(0.5)
        param_mat.next_to(mat, RIGHT)

        input_mat = Matrix([
            [x] for x in ['p', "p'", "p''", "p'''", "q", "q'", "q''", "q'''"]
        ])
        equals = MathTex('=')
        equals.next_to(param_mat, RIGHT)
        input_mat.scale(0.5)
        input_mat.next_to(equals)

        matinv = Matrix([
            [20,10,2,'1/6', -20, -10, -2,'1/6'],
            [-70,-36,'-15/2','-2/3',70,-34,'13/2','-1/2'],
            [84,45,10,1,-84,39,-7,'1/2'],
            [-35,-20,-5,'-2/3',35,-15,'5/2','-1/6'],
            [0,0,0,'1/6',0,0,0,0],
            [0,0,'1/2',0,0,0,0,0],
            [0,1,0,0,0,0,0,0],
            [1,0,0,0,0,0,0,0]
        ])
        matinv.scale(0.5)
        matinv.next_to(param_mat,LEFT)

        mat_equat = MathTex(r'MX=Y')
        mat_equat2 = MathTex(r'M^{-1}MX=M^{-1}Y')
        mat_equat3 = MathTex(r'IX=M^{-1}Y')
        mat_equat4 = MathTex(r'X=M^{-1}Y')
        mat_equat5 = MathTex(r'M^{-1}Y=X')

        mat_equat.next_to(eq, DOWN)
        mat_equat2.next_to(mat_equat, DOWN)
        mat_equat3.next_to(mat_equat2, DOWN)
        mat_equat4.next_to(mat_equat3, DOWN)
        mat_equat5.next_to(mat_equat3, DOWN)

        self.play(Create(p_dot))
        self.play(Create(q_dot))
        self.play(Create(eq))

        self.next_section()
        self.play(Create(mat), Create(param_mat), Create(equals), Create(input_mat))

        self.wait()
        self.play(Create(mat_equat))
        self.play(Create(mat_equat2))
        self.play(Create(mat_equat3))
        self.play(Create(mat_equat4))

        self.wait()
        self.play(
            Transform(mat, matinv),
            input_mat.animate.next_to(matinv),
            param_mat.animate.next_to(equals),
            Transform(mat_equat4, mat_equat5)
        )
        
        self.wait()

