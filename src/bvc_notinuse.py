#!/usr/bin/env python3

import numpy as np
from sklearn.neighbors import NearestNeighbors

def calculate_bvc(points, safety_distance):
    # calculate Voronoi diagram
    nn = NearestNeighbors(n_neighbors=len(points))
    nn.fit(points)
    distances, indices = nn.kneighbors(points)
    vc = []
    for i in range(len(points)):
        vc.append(calc_voronoi_cell(points[i], points[indices[i][1:]], distances[i][1:]))
    
    # calculate buffered Voronoi cell
    bvc = []
    for i in range(len(points)):
        bvc.append(calc_buffered_voronoi_cell(vc[i], points[i], safety_distance))
    
    return bvc

def calc_voronoi_cell(p, neighbors, distances):
    vc = []
    for j in range(len(neighbors)):
        a = neighbors[j] - p
        b = neighbors[(j+1) % len(neighbors)] - p
        n = np.cross(a, b)
        n /= np.linalg.norm(n)
        d = np.dot(n, p)
        vc.append((n, d))
    return vc

def calc_buffered_voronoi_cell(vc, p, safety_distance):
    bvc = []
    for i in range(len(vc)):
        n, d = vc[i]
        n_norm = n / np.linalg.norm(n)
        bvc.append((n_norm, d + safety_distance * np.dot(n_norm, p)))
    return bvc

def calculate_collision_free_regions(bvc, uncertainty):
    # calculate uncertainty-aware collision-free regions
    cfr = []
    for i in range(len(bvc)):
        cfr.append(calc_uncertainty_aware_region(bvc[i], uncertainty))
    
    # check for overlaps
    for i in range(len(cfr)):
        for j in range(i+1, len(cfr)):
            if check_overlap(cfr[i], cfr[j]):
                # adjust regions to eliminate overlap
                cfr[i], cfr[j] = adjust_regions(cfr[i], cfr[j])
    
    return cfr

def calc_uncertainty_aware_region(bvc, uncertainty):
    uar = []
    for i in range(len(bvc)):
        n, d = bvc[i]
        n_norm = n / np.linalg
