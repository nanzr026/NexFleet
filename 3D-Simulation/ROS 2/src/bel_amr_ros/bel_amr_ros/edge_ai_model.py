#!/usr/bin/env python3

"""
Edge-AI Local Intelligence & Neural Inference Module
for BEL AMR Fleet Coordination.

Computes real-time task suitability and route conflict risk scores locally
on each AMR using a lightweight multi-layer neural model.
"""

import math


class EdgeAIModel:

    def __init__(self):
        # Neural Network Weights (2-layer MLP: 5 inputs -> 8 hidden -> 2 outputs)
        # Inputs: [norm_distance, norm_battery, availability, workload, nearby_amr_density]
        # Outputs: [task_suitability_score, route_risk_score]

        self.w1 = [
            [-0.45,  0.35,  0.60, -0.15, -0.20],
            [-0.60,  0.40,  0.50, -0.10, -0.30],
            [-0.30,  0.25,  0.70, -0.05, -0.15],
            [-0.50,  0.50,  0.40, -0.20, -0.25],
            [ 0.20, -0.30, -0.50,  0.40,  0.60],
            [ 0.35, -0.20, -0.40,  0.30,  0.50],
            [-0.40,  0.30,  0.55, -0.10, -0.10],
            [ 0.15, -0.15, -0.30,  0.20,  0.45],
        ]

        self.b1 = [0.10, 0.05, 0.12, 0.08, -0.05, -0.02, 0.07, -0.04]

        self.w2 = [
            [ 0.40,  0.45,  0.50,  0.35, -0.40, -0.35,  0.30, -0.25],  # suitability
            [-0.20, -0.25, -0.15, -0.30,  0.55,  0.50, -0.20,  0.45],  # risk
        ]

        self.b2 = [0.05, 0.02]

    @staticmethod
    def relu(val):
        return max(0.0, val)

    @staticmethod
    def sigmoid(val):
        return 1.0 / (1.0 + math.exp(-max(-10.0, min(10.0, val))))

    def predict(self, distance, battery, available, workload=0, nearby_amrs=0):
        """
        Run local neural inference.
        Returns (task_suitability_score, route_risk_score)
        """
        norm_dist = max(0.0, 1.0 - (distance / 25.0))
        norm_batt = max(0.0, min(1.0, battery / 100.0))
        avail = 1.0 if available else 0.0
        norm_workload = min(1.0, workload / 10.0)
        norm_density = min(1.0, nearby_amrs / 5.0)

        inputs = [norm_dist, norm_batt, avail, norm_workload, norm_density]

        # Hidden layer
        hidden = []
        for h_idx in range(8):
            acc = self.b1[h_idx]
            for i_idx in range(5):
                acc += self.w1[h_idx][i_idx] * inputs[i_idx]
            hidden.append(self.relu(acc))

        # Output layer
        out_suitability_acc = self.b2[0]
        out_risk_acc = self.b2[1]

        for h_idx in range(8):
            out_suitability_acc += self.w2[0][h_idx] * hidden[h_idx]
            out_risk_acc += self.w2[1][h_idx] * hidden[h_idx]

        suitability = self.sigmoid(out_suitability_acc)
        risk = self.sigmoid(out_risk_acc)

        return round(suitability, 4), round(risk, 4)
