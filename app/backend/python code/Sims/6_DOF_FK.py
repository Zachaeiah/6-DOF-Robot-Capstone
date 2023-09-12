import numpy as np
import sympy as sym
import matplotlib.pyplot as plt

from sympy import *
init_printing()

theta0, theta1, theta2, theta3, theta4, theta5 = sym.symbols("theta0 theta1 theta2 theta3 theta4 theta5", real=True) 
a0, a1, a2, a3, a4, a5, a6, a7 = sym.symbols("a0 a1 a2 a3 a4 a5 a6 a7", real=True) 

def ZRot(theta:sym.core.symbol.Symbol, matris: sym.matrices.dense.MutableDenseMatrix) ->sym.matrices.dense.MutableDenseMatrix:
    RotMatrix = Matrix([
                        [sym.cos(theta), -sym.sin(theta), 0, 0],
                        [sym.sin(theta), sym.cos(theta),  0, 0],
                        [0,              0,               0, 0],
                        [0,              0,               0, 1]])*matris
    
    return RotMatrix

def YRot(theta:sym.core.symbol.Symbol, matris: sym.matrices.dense.MutableDenseMatrix) ->sym.matrices.dense.MutableDenseMatrix:
    RotMatrix = Matrix([
                        [sym.cos(theta),  0, sym.sin(theta), 0],
                        [0,               1, 0,              0],
                        [-sym.sin(theta), 0, sym.cos(theta), 0],
                        [0,               0, 0,              1]
                        ])*matris
    
    return RotMatrix

def XRot(theta:sym.core.symbol.Symbol, matris: sym.matrices.dense.MutableDenseMatrix) ->sym.matrices.dense.MutableDenseMatrix:
    RotMatrix = Matrix([
                        [1, 0,              0,               0 ],
                        [0, sym.cos(theta), -sym.sin(theta), 0 ],
                        [0, sym.sin(theta), sym.cos(theta),  0 ],
                        [0, 0,              0,                1]])*matris
    
    return RotMatrix

def translat(dx: sym.core.symbol.Symbol, dy: sym.core.symbol.Symbol, dz: sym.core.symbol.Symbol, matris: sym.matrices.dense.MutableDenseMatrix) -> sym.matrices.dense.MutableDenseMatrix:
    transMatrix = Matrix([
                        [1, 0, 0, dx],
                        [0, 1, 0, dy],
                        [0, 0, 1, dz],
                        [0, 0, 0, 1]]) * matris
    
    return transMatrix

BACE = Matrix([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]])

Frame1 = translat(0, 0, a0+a1, BACE)*XRot(sym.pi, BACE)*ZRot(theta0, BACE)
Frame2 = translat(0, a2, 0, Frame1)*ZRot(theta1, Frame1)
Frame3 = translat(0, a4, -a3, Frame2)*XRot(-sym.pi, Frame2)*XRot(theta2, Frame2)
Frame4 = translat(0, 0, a5, Frame3)*XRot(sym.pi, Frame3)*ZRot(theta3, Frame3)
Frame5 = translat(0, a7, -a6, Frame4)*XRot(-sym.pi, Frame4)*ZRot(theta4, Frame4)
Frame6 = ZRot(theta5, Frame5)

Frame6.subs({a0: 118, a1:123, a2: 535, a3:114, a4:210, a5:212, a6:110, a7:246})


pprint(Frame6)






