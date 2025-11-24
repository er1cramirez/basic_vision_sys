#!/usr/bin/env python3
"""
Script para verificar la transformación de coordenadas de Gazebo a ArduPilot
Gazebo: X adelante, Y izquierda, Z arriba
ArduPilot (después de rotación de 90° en Z):
    X_ardupilot = Y_gazebo
    Y_ardupilot = -X_gazebo
    Z_ardupilot = Z_gazebo
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        super().__init__((0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def do_3d_projection(self, renderer=None):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, self.axes.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        return np.min(zs)

def gazebo_to_ardupilot(x_gz, y_gz, z_gz):
    """Aplica la transformación de Gazebo a ArduPilot"""
    x_ap = y_gz
    y_ap = -x_gz
    z_ap = z_gz
    return x_ap, y_ap, z_ap

def plot_coordinate_systems():
    """Visualiza los sistemas de coordenadas de Gazebo y ArduPilot"""
    fig = plt.figure(figsize=(15, 6))
    
    # Sistema de Gazebo
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.set_title('Sistema de Coordenadas Gazebo\nX: adelante, Y: izquierda, Z: arriba', fontsize=12)
    
    # Ejes de Gazebo
    arrow_prop_dict = dict(mutation_scale=20, arrowstyle='->', shrinkA=0, shrinkB=0, lw=2)
    
    # X axis (rojo) - adelante
    a = Arrow3D([0, 1], [0, 0], [0, 0], **arrow_prop_dict, color='r')
    ax1.add_artist(a)
    ax1.text(1.1, 0, 0, 'X (adelante)', color='r', fontsize=10)
    
    # Y axis (verde) - izquierda
    a = Arrow3D([0, 0], [0, 1], [0, 0], **arrow_prop_dict, color='g')
    ax1.add_artist(a)
    ax1.text(0, 1.1, 0, 'Y (izquierda)', color='g', fontsize=10)
    
    # Z axis (azul) - arriba
    a = Arrow3D([0, 0], [0, 0], [0, 1], **arrow_prop_dict, color='b')
    ax1.add_artist(a)
    ax1.text(0, 0, 1.1, 'Z (arriba)', color='b', fontsize=10)
    
    ax1.set_xlim([-0.5, 1.5])
    ax1.set_ylim([-0.5, 1.5])
    ax1.set_zlim([-0.5, 1.5])
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    
    # Sistema de ArduPilot
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.set_title('Sistema de Coordenadas ArduPilot\n(Rotación 90° en Z)\nX: Y_gz, Y: -X_gz, Z: Z_gz', fontsize=12)
    
    # Ejes de ArduPilot (después de transformación)
    # X_ap = Y_gz (lo que era Y en Gazebo ahora es X en ArduPilot)
    a = Arrow3D([0, 0], [0, 1], [0, 0], **arrow_prop_dict, color='r')
    ax2.add_artist(a)
    ax2.text(0, 1.1, 0, 'X (era Y_gz)', color='r', fontsize=10)
    
    # Y_ap = -X_gz (lo que era X en Gazebo ahora es -Y en ArduPilot)
    a = Arrow3D([0, -1], [0, 0], [0, 0], **arrow_prop_dict, color='g')
    ax2.add_artist(a)
    ax2.text(-1.1, 0, 0, 'Y (era -X_gz)', color='g', fontsize=10)
    
    # Z_ap = Z_gz (Z se mantiene igual)
    a = Arrow3D([0, 0], [0, 0], [0, 1], **arrow_prop_dict, color='b')
    ax2.add_artist(a)
    ax2.text(0, 0, 1.1, 'Z (igual)', color='b', fontsize=10)
    
    ax2.set_xlim([-1.5, 0.5])
    ax2.set_ylim([-0.5, 1.5])
    ax2.set_zlim([-0.5, 1.5])
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    
    plt.tight_layout()
    plt.savefig('/home/eric/droneSim_ws/src/basic_vision_sys/coordinate_transformation.png', dpi=150)
    print("Diagrama guardado en: coordinate_transformation.png")
    plt.show()

def test_transformation():
    """Prueba varios puntos de ejemplo"""
    print("\n=== Verificación de Transformación ===\n")
    print("Gazebo -> ArduPilot:")
    print("X_ap = Y_gz")
    print("Y_ap = -X_gz")
    print("Z_ap = Z_gz\n")
    
    test_points = [
        (1, 0, 0, "Adelante en Gazebo"),
        (0, 1, 0, "Izquierda en Gazebo"),
        (0, 0, 1, "Arriba en Gazebo"),
        (1, 1, 0, "Diagonal en plano XY"),
        (2, 3, 5, "Punto arbitrario"),
    ]
    
    print(f"{'Descripción':<25} | {'Gazebo (X, Y, Z)':<20} | {'ArduPilot (X, Y, Z)':<20}")
    print("-" * 70)
    
    for x_gz, y_gz, z_gz, desc in test_points:
        x_ap, y_ap, z_ap = gazebo_to_ardupilot(x_gz, y_gz, z_gz)
        print(f"{desc:<25} | ({x_gz:>5.1f}, {y_gz:>5.1f}, {z_gz:>5.1f})  | ({x_ap:>5.1f}, {y_ap:>5.1f}, {z_ap:>5.1f})")
    
    print("\n=== Ejemplo de movimiento del dron ===")
    print("Si el dron se mueve 1m adelante en Gazebo (X+):")
    print("  Gazebo: (1.0, 0.0, 0.0)")
    x_ap, y_ap, z_ap = gazebo_to_ardupilot(1.0, 0.0, 0.0)
    print(f"  ArduPilot: ({x_ap}, {y_ap}, {z_ap})")
    print("  En ArduPilot esto aparece como movimiento en Y negativo\n")
    
    print("Si el dron se mueve 1m a la izquierda en Gazebo (Y+):")
    print("  Gazebo: (0.0, 1.0, 0.0)")
    x_ap, y_ap, z_ap = gazebo_to_ardupilot(0.0, 1.0, 0.0)
    print(f"  ArduPilot: ({x_ap}, {y_ap}, {z_ap})")
    print("  En ArduPilot esto aparece como movimiento en X positivo\n")

if __name__ == "__main__":
    test_transformation()
    
    try:
        plot_coordinate_systems()
    except Exception as e:
        print(f"\nNo se pudo crear el gráfico (probablemente no hay display): {e}")
        print("Pero la transformación numérica es correcta como se muestra arriba.")
