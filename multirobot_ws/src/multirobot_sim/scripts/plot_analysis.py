#!/usr/bin/env python3
"""
Script per analisi post-processing dei dati di controllo del robot.
Genera grafici professionali per pubblicazioni/report.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import argparse
import os


def load_data(filename):
    """Carica i dati dal file .npz"""
    data = np.load(filename)
    return {key: data[key] for key in data.files}


def plot_trajectory_comparison(data, ax):
    """Grafico traiettoria attuale vs desiderata"""
    ax.plot(data['x'], data['y'], 'b-', linewidth=2, label='Actual trajectory')
    
    if len(data['x_des']) > 0:
        ax.plot(data['x_des'], data['y_des'], 'r--', linewidth=2, label='Desired trajectory')
    
    # Punto iniziale e finale
    ax.plot(data['x'][0], data['y'][0], 'go', markersize=10, label='Start')
    ax.plot(data['x'][-1], data['y'][-1], 'rs', markersize=10, label='End')
    
    ax.set_xlabel('X Position [m]', fontsize=12)
    ax.set_ylabel('Y Position [m]', fontsize=12)
    ax.set_title('Robot Trajectory', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=10)
    ax.axis('equal')


def plot_position_error(data, ax):
    """Grafico errore di posizione nel tempo"""
    ax.plot(data['time'], data['error_pos'], 'r-', linewidth=1.5)
    ax.fill_between(data['time'], 0, data['error_pos'], alpha=0.3, color='red')
    
    # Statistiche
    mean_error = np.mean(data['error_pos'])
    max_error = np.max(data['error_pos'])
    ax.axhline(y=mean_error, color='orange', linestyle='--', 
               label=f'Mean: {mean_error:.3f} m')
    
    ax.set_xlabel('Time [s]', fontsize=12)
    ax.set_ylabel('Position Error [m]', fontsize=12)
    ax.set_title('Position Tracking Error', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=10)
    
    # Annotazione max error
    max_idx = np.argmax(data['error_pos'])
    ax.annotate(f'Max: {max_error:.3f} m', 
                xy=(data['time'][max_idx], max_error),
                xytext=(10, 10), textcoords='offset points',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
                arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))


def plot_orientation_error(data, ax):
    """Grafico errore di orientamento"""
    error_deg = np.degrees(data['error_theta'])
    ax.plot(data['time'], error_deg, 'g-', linewidth=1.5)
    ax.fill_between(data['time'], 0, error_deg, alpha=0.3, color='green')
    
    mean_error = np.mean(np.abs(error_deg))
    ax.axhline(y=mean_error, color='orange', linestyle='--', 
               label=f'Mean |error|: {mean_error:.2f}°')
    ax.axhline(y=-mean_error, color='orange', linestyle='--')
    
    ax.set_xlabel('Time [s]', fontsize=12)
    ax.set_ylabel('Orientation Error [deg]', fontsize=12)
    ax.set_title('Orientation Tracking Error', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=10)


def plot_velocity_commands(data, ax):
    """Grafico comandi di velocità"""
    time_v = data['time'][:len(data['v_cmd'])]
    time_w = data['time'][:len(data['w_cmd'])]
    
    ax2 = ax.twinx()
    
    l1 = ax.plot(time_v, data['v_cmd'], 'b-', linewidth=1.5, label='Linear velocity')
    l2 = ax2.plot(time_w, data['w_cmd'], 'r-', linewidth=1.5, label='Angular velocity')
    
    ax.set_xlabel('Time [s]', fontsize=12)
    ax.set_ylabel('Linear Velocity [m/s]', fontsize=12, color='b')
    ax2.set_ylabel('Angular Velocity [rad/s]', fontsize=12, color='r')
    ax.set_title('Control Commands', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.tick_params(axis='y', labelcolor='b')
    ax2.tick_params(axis='y', labelcolor='r')
    
    # Legenda combinata
    lns = l1 + l2
    labs = [l.get_label() for l in lns]
    ax.legend(lns, labs, loc='best', fontsize=10)


def plot_obstacle_distance(data, ax):
    """Grafico distanza da ostacoli"""
    if len(data['min_obstacle_dist']) == 0:
        ax.text(0.5, 0.5, 'No obstacle data', ha='center', va='center', 
                transform=ax.transAxes, fontsize=12)
        return
    
    time_obs = data['time'][:len(data['min_obstacle_dist'])]
    ax.plot(time_obs, data['min_obstacle_dist'], 'r-', linewidth=1.5)
    
    # Soglie di sicurezza
    ax.axhline(y=0.5, color='orange', linestyle='--', linewidth=2, 
               label='Safe threshold')
    ax.axhline(y=1.2, color='yellow', linestyle='--', linewidth=2, 
               label='Avoidance threshold')
    
    # Evidenzia zone critiche
    critical = np.array(data['min_obstacle_dist']) < 0.5
    if np.any(critical):
        ax.fill_between(time_obs, 0, 4, where=critical[:len(time_obs)], 
                        alpha=0.2, color='red', label='Critical zone')
    
    ax.set_xlabel('Time [s]', fontsize=12)
    ax.set_ylabel('Minimum Distance [m]', fontsize=12)
    ax.set_title('Obstacle Detection', fontsize=14, fontweight='bold')
    ax.set_ylim([0, min(4, np.max(data['min_obstacle_dist']) * 1.1)])
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=10)


def plot_error_statistics(data, ax):
    """Box plot delle statistiche degli errori"""
    errors = {
        'Position\nError [m]': data['error_pos'],
        'Orientation\nError [deg]': np.abs(np.degrees(data['error_theta']))
    }
    
    ax.boxplot(errors.values(), labels=errors.keys(), showmeans=True)
    ax.set_ylabel('Error Magnitude', fontsize=12)
    ax.set_title('Error Statistics', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    
    # Aggiungi valori numerici
    for i, (label, vals) in enumerate(errors.items(), 1):
        median = np.median(vals)
        mean = np.mean(vals)
        ax.text(i, median, f'Med: {median:.3f}', ha='center', 
                bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.5))


def generate_full_report(data, output_file='robot_analysis.png'):
    """Genera un report completo con tutti i grafici"""
    fig = plt.figure(figsize=(16, 10))
    gs = GridSpec(3, 3, figure=fig, hspace=0.3, wspace=0.3)
    
    # Traiettoria (grande, sinistra)
    ax1 = fig.add_subplot(gs[:2, 0])
    plot_trajectory_comparison(data, ax1)
    
    # Errori
    ax2 = fig.add_subplot(gs[0, 1])
    plot_position_error(data, ax2)
    
    ax3 = fig.add_subplot(gs[0, 2])
    plot_orientation_error(data, ax3)
    
    # Comandi velocità
    ax4 = fig.add_subplot(gs[1, 1:])
    plot_velocity_commands(data, ax4)
    
    # Ostacoli (se disponibili)
    ax5 = fig.add_subplot(gs[2, 0])
    plot_obstacle_distance(data, ax5)
    
    # Statistiche
    ax6 = fig.add_subplot(gs[2, 1:])
    plot_error_statistics(data, ax6)
    
    # Titolo principale
    fig.suptitle('Robot Control Performance Analysis', 
                 fontsize=16, fontweight='bold', y=0.995)
    
    # Salva
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Report saved to: {output_file}")
    plt.show()


def print_statistics(data):
    """Stampa statistiche numeriche"""
    print("\n" + "="*60)
    print("PERFORMANCE STATISTICS")
    print("="*60)
    
    print(f"\nTrajectory Duration: {data['time'][-1]:.2f} seconds")
    print(f"Total Distance Traveled: {np.sum(np.sqrt(np.diff(data['x'])**2 + np.diff(data['y'])**2)):.2f} m")
    
    print(f"\nPosition Error:")
    print(f"  Mean: {np.mean(data['error_pos']):.4f} m")
    print(f"  Std:  {np.std(data['error_pos']):.4f} m")
    print(f"  Max:  {np.max(data['error_pos']):.4f} m")
    print(f"  RMS:  {np.sqrt(np.mean(data['error_pos']**2)):.4f} m")
    
    print(f"\nOrientation Error:")
    error_deg = np.degrees(data['error_theta'])
    print(f"  Mean: {np.mean(np.abs(error_deg)):.2f}°")
    print(f"  Std:  {np.std(error_deg):.2f}°")
    print(f"  Max:  {np.max(np.abs(error_deg)):.2f}°")
    
    if len(data['v_cmd']) > 0:
        print(f"\nLinear Velocity:")
        print(f"  Mean: {np.mean(np.abs(data['v_cmd'])):.3f} m/s")
        print(f"  Max:  {np.max(np.abs(data['v_cmd'])):.3f} m/s")
    
    if len(data['w_cmd']) > 0:
        print(f"\nAngular Velocity:")
        print(f"  Mean: {np.mean(np.abs(data['w_cmd'])):.3f} rad/s")
        print(f"  Max:  {np.max(np.abs(data['w_cmd'])):.3f} rad/s")
    
    if len(data['min_obstacle_dist']) > 0:
        print(f"\nObstacle Avoidance:")
        print(f"  Min distance: {np.min(data['min_obstacle_dist']):.3f} m")
        critical = np.sum(np.array(data['min_obstacle_dist']) < 0.5)
        print(f"  Critical situations: {critical}")
    
    print("="*60 + "\n")


def main():
    parser = argparse.ArgumentParser(description='Analyze robot control performance')
    parser.add_argument('log_file', help='Path to .npz log file')
    parser.add_argument('-o', '--output', default='robot_analysis.png',
                       help='Output plot filename')
    parser.add_argument('--no-plot', action='store_true',
                       help='Only print statistics, no plots')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.log_file):
        print(f"Error: File {args.log_file} not found!")
        return
    
    print(f"Loading data from {args.log_file}...")
    data = load_data(args.log_file)
    
    print_statistics(data)
    
    if not args.no_plot:
        generate_full_report(data, args.output)


if __name__ == '__main__':
    main()
