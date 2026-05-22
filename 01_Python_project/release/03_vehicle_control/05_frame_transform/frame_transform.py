"""Frame Transform — Global → Local + 다항식 fit/evaluate.

과제 명세는 README.md 참조.
"""
from __future__ import annotations

import numpy as np


class Global2Local:
    def __init__(self, num_points: int):
        self.n = num_points
        self.local_points = np.zeros((num_points, 2))

    def convert(self, points: np.ndarray, yaw_ego: float,
                x_ego: float, y_ego: float) -> np.ndarray:
        theta = -yaw_ego
        R = np.array([[np.cos(theta), -np.sin(theta)],
                      [np.sin(theta), np.cos(theta)]])
        for i in range(self.n):
            global_pt = points[i] - np.array([x_ego, y_ego])
            local_pt = R @ global_pt
            self.local_points[i] = local_pt
            self.local_points = self.local_points.reshape(self.n, 2)
        return self.local_points
    

        # TODO: global frame points → local frame 변환.
        # 1) 회전각 θ = -yaw_ego  (ego 좌표계로 돌리려면 음수 부호)
        # 2) R(θ) = [[cosθ, -sinθ], [sinθ, cosθ]]
        # 3) 각 점에 대해: local_pt = R · (global_pt - [x_ego, y_ego])
        # 결과를 self.local_points 에 저장하고 반환
        raise NotImplementedError


class PolynomialFitting:
    def __init__(self, degree: int, num_points: int):
        self.degree = degree
        self.n = num_points
        self.coeff = np.zeros((degree + 1, 1))

    def fit(self, points: np.ndarray) -> np.ndarray:
        A_ij = np.zeros((self.n, self.degree + 1))
        b_i = np.zeros((self.n, 1))
        for i in range(self.n):
            for j in range(self.degree + 1):
                A_ij[i, j] = points[i][0] ** (self.degree - j)
            b_i[i] = points[i][1]
        A_T = A_ij.T
        A_T_A = A_T @ A_ij
        A_T_b = A_T @ b_i
        self.coeff = np.linalg.inv(A_T_A) @ A_T_b
        return self.coeff
            
        # TODO: least-squares 로 다항식 계수 fitting.
        # 1) A_ij = points[i][0] ** (degree - j)   (행 i, 열 j)
        # 2) b_i = points[i][1]
        # 3) coeff = (Aᵀ A)⁻¹ Aᵀ b   (정상방정식; np.linalg.inv 사용)
        # 결과를 self.coeff 에 저장하고 반환
        raise NotImplementedError


class PolynomialValue:
    def __init__(self, degree: int, num_x: int):
        self.degree = degree
        self.n = num_x
        self.y = np.zeros((num_x, 1))
        self.points = np.zeros((num_x, 2))

    def calculate(self, coeff: np.ndarray, x: np.ndarray) -> np.ndarray:
        for i in range(self.n):
            self.y[i] = 0.0
            for j in range(self.degree + 1):
                self.y[i] += coeff[j] * x[i] ** (self.degree - j)
                self.points[i] = np.array([x[i], self.y[i, 0]])
            
        self.y = self.y.reshape(-1, 1)
        self.points = self.points.reshape(self.n, 2)
        return self.y
        
        # TODO: 주어진 coeff (고차 → 저차 순서) 로 x 배열에서 y 값 평가.
        # 각 i 에 대해: y[i] = Σ_j coeff[j] · x[i] ** (degree - j)
        # 결과를 self.y, self.points 에 저장하고 self.y 반환
        raise NotImplementedError
