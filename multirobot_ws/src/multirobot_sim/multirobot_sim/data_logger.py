#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
from tf_transformations import euler_from_quaternion
from datetime import datetime


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        
        # Parametri
        self.declare_parameter('robot_name', '')
        self.declare_parameter('log_file', f'robot_log_{datetime.now().strftime("%Y%m%d_%H%M%S")}.npz')
        self.declare_parameter('plot_live', True)
        
        self.robot_name = self.get_parameter('robot_name').value
        self.log_file = self.get_parameter('log_file').value
        self.plot_live = self.get_parameter('plot_live').value
        
        # Prefisso per i topic
        prefix = f'/{self.robot_name}' if self.robot_name else ''
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, f'{prefix}/odom', self.odom_callback, 10)
        self.cmd_sub = self.create_subscription(
            Twist, f'{prefix}/cmd_vel', self.cmd_callback, 10)
        
        # Subscriber per sensori
        if self.robot_name:
            # Follower: usa ultrasonic sensor
            self.ultrasonic_sub = self.create_subscription(
                LaserScan, f'{prefix}/ultrasonic_sensor', self.ultrasonic_callback, 10)
        else:
            # Leader: usa laser scan
            self.laser_sub = self.create_subscription(
                LaserScan, '/scan', self.laser_callback, 10)
        
        # Solo per il leader: subscriber alla traiettoria desiderata
        if not self.robot_name:
            self.traj_sub = self.create_subscription(
                Pose2D, '/desired_trajectory', self.traj_callback, 10)
        
        # Buffer dei dati
        self.data = {
            'time': [],
            'x': [], 'y': [], 'theta': [],
            'x_des': [], 'y_des': [], 'theta_des': [],
            'v_cmd': [], 'w_cmd': [],
            'error_pos': [], 'error_theta': [],
            'min_obstacle_dist': []
        }
        
        self.start_time = self.get_clock().now()
        
        # Timer per salvare dati periodicamente
        self.save_timer = self.create_timer(1.0, self.save_data)
        
        # Setup plot live se abilitato
        if self.plot_live:
            self.setup_live_plot()
        
        robot_type = "follower" if self.robot_name else "leader"
        self.get_logger().info(f'Data Logger started for {self.robot_name or "leader"} ({robot_type})')

    def odom_callback(self, msg):
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        self.data['time'].append(t)
        self.data['x'].append(x)
        self.data['y'].append(y)
        self.data['theta'].append(theta)
        
        # Calcola errore se abbiamo setpoint
        if self.data['x_des']:
            dx = self.data['x_des'][-1] - x
            dy = self.data['y_des'][-1] - y
            error_pos = math.sqrt(dx*dx + dy*dy)
            error_theta = self.data['theta_des'][-1] - theta
            # NORMALIZZAZIONE DELL'ERRORE ANGOLARE
            error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))
        else:
            error_pos = 0.0
            error_theta = 0.0
        
        self.data['error_pos'].append(error_pos)
        self.data['error_theta'].append(error_theta)

    def traj_callback(self, msg):
        self.data['x_des'].append(msg.x)
        self.data['y_des'].append(msg.y)
        self.data['theta_des'].append(msg.theta)

    def cmd_callback(self, msg):
        self.data['v_cmd'].append(msg.linear.x)
        self.data['w_cmd'].append(msg.angular.z)

    def laser_callback(self, msg):
        """Callback per il laser scan del leader"""
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        min_dist = min(valid_ranges) if valid_ranges else float('inf')
        # Salva con timestamp
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.data['min_obstacle_dist'].append((t, min_dist))

    def ultrasonic_callback(self, msg):
        """Callback per il sensore ultrasonico dei follower"""
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        min_dist = min(valid_ranges) if valid_ranges else float('inf')
        # Salva con timestamp
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.data['min_obstacle_dist'].append((t, min_dist))

    def save_data(self):
        """Salva i dati su file"""
        if not self.data['time']:
            return
        
        # Processa i dati degli ostacoli (che hanno timestamp)
        processed_obstacle_data = []
        if self.data['min_obstacle_dist']:
            # Estrai tempi e distanze
            obs_times = [item[0] for item in self.data['min_obstacle_dist']]
            obs_dists = [item[1] for item in self.data['min_obstacle_dist']]
            
            # Interpola sui tempi principali
            if len(obs_times) > 1:
                processed_obstacle_data = np.interp(self.data['time'], obs_times, obs_dists).tolist()
            elif len(obs_times) == 1:
                # Se c'è solo un valore, replicalo
                processed_obstacle_data = [obs_dists[0]] * len(self.data['time'])
        
        # Padding dei dati per avere stessa lunghezza
        max_len = len(self.data['time'])
        
        # Sostituisci i dati degli ostacoli con quelli interpolati
        temp_obstacle_data = self.data['min_obstacle_dist']
        self.data['min_obstacle_dist'] = processed_obstacle_data
        
        for key in self.data:
            if len(self.data[key]) < max_len:
                # Padding con l'ultimo valore o zero
                pad_value = self.data[key][-1] if self.data[key] else 0.0
                self.data[key].extend([pad_value] * (max_len - len(self.data[key])))
        
        np.savez(self.log_file, **{k: np.array(v) for k, v in self.data.items()})
        
        # Ripristina il formato originale per continuare a loggare
        self.data['min_obstacle_dist'] = temp_obstacle_data
        
        self.get_logger().info(f'Data saved to {self.log_file} ({max_len} samples)')

    def setup_live_plot(self):
        """Setup per plotting in tempo reale"""
        plt.ion()
        self.fig, self.axs = plt.subplots(3, 2, figsize=(12, 10))
        robot_label = self.robot_name if self.robot_name else "Leader"
        self.fig.suptitle(f'Real-time Data - {robot_label}')
        
        self.lines = {}
        
        # Traiettoria XY
        self.axs[0, 0].set_xlabel('X [m]')
        self.axs[0, 0].set_ylabel('Y [m]')
        self.axs[0, 0].set_title('Trajectory')
        self.axs[0, 0].grid(True)
        self.lines['traj_actual'], = self.axs[0, 0].plot([], [], 'b-', label='Actual')
        if not self.robot_name:  # Solo per il leader
            self.lines['traj_desired'], = self.axs[0, 0].plot([], [], 'r--', label='Desired')
        self.axs[0, 0].legend()
        
        # Errore di posizione
        self.axs[0, 1].set_xlabel('Time [s]')
        self.axs[0, 1].set_ylabel('Position Error [m]')
        self.axs[0, 1].set_title('Position Error')
        self.axs[0, 1].grid(True)
        self.lines['error_pos'], = self.axs[0, 1].plot([], [], 'r-')
        
        # Errore di orientamento (sin e cos per evitare discontinuità)
        self.axs[1, 0].set_xlabel('Time [s]')
        self.axs[1, 0].set_ylabel('Normalized Error')
        self.axs[1, 0].set_title('Orientation Error (sin/cos)')
        self.axs[1, 0].grid(True)
        self.lines['error_sin'], = self.axs[1, 0].plot([], [], 'r-', label='sin(θ_err)', linewidth=1.5)
        self.lines['error_cos'], = self.axs[1, 0].plot([], [], 'b-', label='cos(θ_err)', linewidth=1.5)
        self.axs[1, 0].axhline(y=0, color='k', linestyle='--', alpha=0.3)
        self.axs[1, 0].set_ylim([-1.1, 1.1])
        self.axs[1, 0].legend()
        
        # Velocità lineare
        self.axs[1, 1].set_xlabel('Time [s]')
        self.axs[1, 1].set_ylabel('Linear Velocity [m/s]')
        self.axs[1, 1].set_title('Linear Velocity Command')
        self.axs[1, 1].grid(True)
        self.lines['v_cmd'], = self.axs[1, 1].plot([], [], 'b-')
        
        # Velocità angolare
        self.axs[2, 0].set_xlabel('Time [s]')
        self.axs[2, 0].set_ylabel('Angular Velocity [rad/s]')
        self.axs[2, 0].set_title('Angular Velocity Command')
        self.axs[2, 0].grid(True)
        self.lines['w_cmd'], = self.axs[2, 0].plot([], [], 'm-')
        
        # Distanza ostacolo (per TUTTI i robot, leader e follower)
        self.axs[2, 1].set_xlabel('Time [s]')
        self.axs[2, 1].set_ylabel('Distance [m]')
        sensor_type = "Ultrasonic" if self.robot_name else "Laser"
        self.axs[2, 1].set_title(f'Min Obstacle Distance ({sensor_type})')
        self.axs[2, 1].grid(True)
        self.lines['obstacle'], = self.axs[2, 1].plot([], [], 'r-', linewidth=1.5, label='Distance')
        
        # Solo soglia di sicurezza (diversa per follower e leader)
        if self.robot_name:
            # Follower con ultrasonic sensor
            safe_threshold = 0.3
        else:
            # Leader con laser
            safe_threshold = 0.5
        
        self.axs[2, 1].axhline(y=safe_threshold, color='red', linestyle='--', 
                               linewidth=2, label=f'Safety ({safe_threshold}m)')
        self.axs[2, 1].legend()
        self.axs[2, 1].set_ylim([0, 4])  # Limite fisso per stabilità
        
        plt.tight_layout()
        
        # Timer per aggiornare il plot
        self.plot_timer = self.create_timer(0.5, self.update_plot)

    def update_plot(self):
        """Aggiorna i grafici in tempo reale"""
        if not self.data['time']:
            return
        
        try:
            # Traiettoria
            self.lines['traj_actual'].set_data(self.data['x'], self.data['y'])
            if not self.robot_name and self.data['x_des']:  # Solo per il leader
                self.lines['traj_desired'].set_data(self.data['x_des'], self.data['y_des'])
            self.axs[0, 0].relim()
            self.axs[0, 0].autoscale_view()
            
            # Errori
            self.lines['error_pos'].set_data(self.data['time'], self.data['error_pos'])
            self.axs[0, 1].relim()
            self.axs[0, 1].autoscale_view()
            
            # Errore di orientamento (sin e cos)
            error_theta_array = np.array(self.data['error_theta'])
            sin_error = np.sin(error_theta_array)
            cos_error = np.cos(error_theta_array)
            
            self.lines['error_sin'].set_data(self.data['time'], sin_error)
            self.lines['error_cos'].set_data(self.data['time'], cos_error)
            # Limiti fissi, non serve relim
            self.axs[1, 0].set_xlim([0, max(self.data['time'][-1], 1)])
            
            # Comandi velocità
            if self.data['v_cmd']:
                # Allinea le lunghezze
                min_len = min(len(self.data['time']), len(self.data['v_cmd']))
                self.lines['v_cmd'].set_data(self.data['time'][:min_len], self.data['v_cmd'][:min_len])
                self.axs[1, 1].relim()
                self.axs[1, 1].autoscale_view()
            
            if self.data['w_cmd']:
                min_len = min(len(self.data['time']), len(self.data['w_cmd']))
                self.lines['w_cmd'].set_data(self.data['time'][:min_len], self.data['w_cmd'][:min_len])
                self.axs[2, 0].relim()
                self.axs[2, 0].autoscale_view()
            
            # Ostacoli (per TUTTI i robot) - interpolazione per continuità
            if self.data['min_obstacle_dist']:
                # Estrai tempi e distanze dai dati (formato tupla)
                if isinstance(self.data['min_obstacle_dist'][0], tuple):
                    obs_times = [item[0] for item in self.data['min_obstacle_dist']]
                    obs_dists = [item[1] for item in self.data['min_obstacle_dist']]
                    
                    # Interpola linearmente sui tempi di campionamento principali
                    if len(obs_times) > 1:
                        interpolated_dists = np.interp(self.data['time'], obs_times, obs_dists)
                        self.lines['obstacle'].set_data(self.data['time'], interpolated_dists)
                    else:
                        # Solo un punto, mostra costante
                        self.lines['obstacle'].set_data(self.data['time'], 
                                                       [obs_dists[0]] * len(self.data['time']))
                else:
                    # Formato vecchio (solo distanze)
                    self.lines['obstacle'].set_data(self.data['time'][:len(self.data['min_obstacle_dist'])], 
                                                   self.data['min_obstacle_dist'])
                
                # Limiti fissi per stabilità del grafico
                self.axs[2, 1].set_xlim([0, max(self.data['time'][-1], 1)])
            
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            
        except Exception as e:
            self.get_logger().error(f'Plot update error: {e}')


def main():
    rclpy.init()
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_data()
        if node.plot_live:
            plt.ioff()
            plt.show()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
