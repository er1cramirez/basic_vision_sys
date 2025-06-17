#!/usr/bin/env python3
import socket
import struct
import json
import threading
import time
import numpy as np
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objs as go
from collections import deque
import pandas as pd
import argparse
import logging
import sys
import os

# Maximum number of data points to keep in history
MAX_POINTS = 500

# Message types - MUST MATCH C++ ENUM VALUES
TELEMETRY_CONTROL = 1
TELEMETRY_VISION = 2
TELEMETRY_SYSTEM = 3
TELEMETRY_COMMAND = 4

# Set up logger
logger = logging.getLogger("drone_visualizer")

class TelemetryReceiver:
    def __init__(self, port=14559, buffer_size=4096, quiet=False):
        self.port = port
        self.buffer_size = buffer_size
        self.running = False
        self.socket = None
        self.receive_thread = None
        self.quiet = quiet
        
        # Callbacks for different message types
        self.callbacks = {
            TELEMETRY_CONTROL: [],
            TELEMETRY_VISION: [],
            TELEMETRY_SYSTEM: [],
            TELEMETRY_COMMAND: []
        }
        
        # Statistics
        self.messages_received = 0
        self.bytes_received = 0
        self.last_sequence_nums = {
            TELEMETRY_CONTROL: 0,
            TELEMETRY_VISION: 0,
            TELEMETRY_SYSTEM: 0,
            TELEMETRY_COMMAND: 0
        }
        self.dropped_messages = {
            TELEMETRY_CONTROL: 0,
            TELEMETRY_VISION: 0,
            TELEMETRY_SYSTEM: 0,
            TELEMETRY_COMMAND: 0
        }
    
    def log(self, message, level="info"):
        """Custom logging function that respects quiet mode"""
        if self.quiet:
            return
            
        if level == "info":
            logger.info(message)
        elif level == "error":
            logger.error(message)
        elif level == "debug":
            logger.debug(message)
        elif level == "warning":
            logger.warning(message)
    
    def start(self):
        if self.running:
            return
        
        # Create UDP socket
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind(('0.0.0.0', self.port))
            
            self.running = True
            self.receive_thread = threading.Thread(target=self._receive_loop)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            
            self.log(f"Telemetry receiver started on port {self.port}")
        except Exception as e:
            self.log(f"Error starting telemetry receiver: {e}", "error")
    
    def stop(self):
        if not self.running:
            return
        
        self.running = False
        if self.socket:
            self.socket.close()
        
        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)
        
        self.log("Telemetry receiver stopped")
        self.log(f"Messages received: {self.messages_received}")
        self.log(f"Bytes received: {self.bytes_received}")
        self.log(f"Dropped messages: {self.dropped_messages}")
    
    def register_callback(self, message_type, callback):
        """Register a callback for a specific message type"""
        if message_type in self.callbacks:
            self.callbacks[message_type].append(callback)
            return True
        return False
    
    def _receive_loop(self):
        """Main receive loop"""
        while self.running:
            try:
                # Receive data
                data, addr = self.socket.recvfrom(self.buffer_size)
                self.bytes_received += len(data)
                
                # Process the packet
                if len(data) < 20:  # Minimum header size: 4 + 8 + 4 + 4 = 20 bytes
                    self.log(f"Received packet too small: {len(data)} bytes", "warning")
                    continue
                
                # Parse header
                message_type = struct.unpack('!I', data[0:4])[0]
                timestamp = struct.unpack('!Q', data[4:12])[0]
                sequence_num = struct.unpack('!I', data[12:16])[0]
                payload_size = struct.unpack('!I', data[16:20])[0]
                
                # Check packet integrity
                if len(data) < 20 + payload_size:
                    self.log(f"Incomplete packet: expected {20 + payload_size} bytes, got {len(data)}", "warning")
                    continue
                
                # Extract payload
                payload = data[20:20 + payload_size]
                
                # Check for dropped messages
                if message_type in self.last_sequence_nums:
                    expected_seq = (self.last_sequence_nums[message_type] + 1) & 0xFFFFFFFF
                    if sequence_num != expected_seq and self.last_sequence_nums[message_type] != 0:
                        dropped = (sequence_num - expected_seq) & 0xFFFFFFFF
                        if dropped > 0 and dropped < 10000:  # Sanity check
                            self.dropped_messages[message_type] += dropped
                            self.log(f"Dropped {dropped} messages of type {message_type}", "warning")
                    
                    self.last_sequence_nums[message_type] = sequence_num
                
                # Convert payload to string and parse JSON
                try:
                    payload_str = payload.decode('utf-8')
                    payload_data = json.loads(payload_str)
                    
                    # Debug print the first message of each type
                    if self.messages_received < 4 and not self.quiet:
                        self.log(f"Message type {message_type}, payload: {json.dumps(payload_data, indent=2)}", "debug")
                    
                    # Increment counter
                    self.messages_received += 1
                    
                    # Call callbacks
                    if message_type in self.callbacks:
                        for callback in self.callbacks[message_type]:
                            callback(message_type, timestamp, sequence_num, payload_data)
                    
                except Exception as e:
                    self.log(f"Error processing payload: {e}", "error")
                    if not self.quiet:
                        self.log(f"Raw payload: {payload}", "debug")
                
            except Exception as e:
                if self.running:
                    self.log(f"Error in receiver loop: {e}", "error")
                    time.sleep(0.1)  # Avoid tight loop on error
    
    def get_stats(self):
        """Get receiver statistics"""
        return {
            'messages_received': self.messages_received,
            'bytes_received': self.bytes_received,
            'dropped_messages': dict(self.dropped_messages)
        }


class DroneVisualizer:
    def __init__(self, telemetry_port=14550, quiet=False):
        self.quiet = quiet
        
        # Initialize data structures
        self.telemetry = TelemetryReceiver(port=telemetry_port, quiet=quiet)
        
        # Dictionary to store time series data for different fields
        self.time_series = {}
        self.labels = {}
        
        # Base fields we always want to track separately
        base_fields = ['control_u_x', 'control_u_y', 'control_u_z', 'timestamps']
        for field in base_fields:
            self.time_series[field] = deque(maxlen=MAX_POINTS)
            
        # System status
        self.status_times = deque(maxlen=MAX_POINTS)
        self.status_messages = deque(maxlen=MAX_POINTS)
        
        # Control type (for updating graph titles)
        self.control_type = "Unknown"
        
        # Register callbacks
        self.telemetry.register_callback(TELEMETRY_CONTROL, self._handle_control_telemetry)
        self.telemetry.register_callback(TELEMETRY_VISION, self._handle_vision_telemetry)
        self.telemetry.register_callback(TELEMETRY_SYSTEM, self._handle_system_telemetry)
        
        # Start telemetry receiver
        self.telemetry.start()
        
        # Initialize Dash app
        self.app = dash.Dash(__name__, suppress_callback_exceptions=True)
        self.app.title = "Drone Control Visualization"
        
        # Configure layout
        self._setup_layout()
        
        # Configure callbacks
        self._setup_callbacks()
    
    def log(self, message, level="info"):
        """Custom logging function that respects quiet mode"""
        if self.quiet:
            return
            
        if level == "info":
            logger.info(message)
        elif level == "error":
            logger.error(message)
        elif level == "debug":
            logger.debug(message)
        elif level == "warning":
            logger.warning(message)
    
    def _handle_control_telemetry(self, message_type, timestamp, sequence_num, data):
        """Process control telemetry data using both original and generic formats"""
        try:
            # Add timestamp
            current_time = time.time()
            if 'timestamps' not in self.time_series:
                self.time_series['timestamps'] = deque(maxlen=MAX_POINTS)
            self.time_series['timestamps'].append(current_time)
            
            # Store control type if available
            if 'controlType' in data:
                self.control_type = data['controlType']
            
            # Extract control outputs (these are always present)
            if 'command' in data:
                if 'control_u_x' not in self.time_series:
                    self.time_series['control_u_x'] = deque(maxlen=MAX_POINTS)
                if 'control_u_y' not in self.time_series:
                    self.time_series['control_u_y'] = deque(maxlen=MAX_POINTS)
                if 'control_u_z' not in self.time_series:
                    self.time_series['control_u_z'] = deque(maxlen=MAX_POINTS)
                
                self.time_series['control_u_x'].append(data['command']['u_x'])
                self.time_series['control_u_y'].append(data['command']['u_y'])
                self.time_series['control_u_z'].append(data['command']['u_z'])
            
            # Try to handle original format first (for backward compatibility)
            if 'performanceData' in data:
                # Position errors - either as array or individual fields
                if 'positionError' in data['performanceData']:
                    if 'error_x' not in self.time_series:
                        self.time_series['error_x'] = deque(maxlen=MAX_POINTS)
                        self.time_series['error_y'] = deque(maxlen=MAX_POINTS)
                        self.time_series['error_z'] = deque(maxlen=MAX_POINTS)
                    
                    pos_error = data['performanceData']['positionError']
                    if isinstance(pos_error, list) and len(pos_error) >= 3:
                        self.time_series['error_x'].append(pos_error[0])
                        self.time_series['error_y'].append(pos_error[1])
                        self.time_series['error_z'].append(pos_error[2])
                
                # Current and reference position for tracking
                for field in ['currentPosition', 'referencePosition']:
                    if field in data['performanceData']:
                        field_data = data['performanceData'][field]
                        if isinstance(field_data, list) and len(field_data) >= 3:
                            base_name = field.replace('Position', '')
                            if f'{base_name}_x' not in self.time_series:
                                self.time_series[f'{base_name}_x'] = deque(maxlen=MAX_POINTS)
                                self.time_series[f'{base_name}_y'] = deque(maxlen=MAX_POINTS)
                                self.time_series[f'{base_name}_z'] = deque(maxlen=MAX_POINTS)
                            
                            self.time_series[f'{base_name}_x'].append(field_data[0])
                            self.time_series[f'{base_name}_y'].append(field_data[1])
                            self.time_series[f'{base_name}_z'].append(field_data[2])
            
            # Extract generic data if available (new format)
            if 'data' in data and 'dataLabels' in data:
                # Process each data category
                for category in data['data'].keys():
                    if category in data['dataLabels']:
                        values = data['data'][category]
                        labels = data['dataLabels'][category]
                        
                        # Print for debug (only first few messages)
                        if self.telemetry.messages_received <= 5 and not self.quiet:
                            self.log(f"Processing category: {category}", "debug")
                            self.log(f"  Labels: {labels}", "debug")
                            self.log(f"  Values: {values}", "debug")
                        
                        # Store labels
                        self.labels[category] = labels
                        
                        # Create data series if needed
                        for i, label in enumerate(labels):
                            field_name = f"{category}_{i}"
                            if field_name not in self.time_series:
                                self.time_series[field_name] = deque(maxlen=MAX_POINTS)
                            
                            # Add value to time series
                            if i < len(values):
                                self.time_series[field_name].append(values[i])
        
        except Exception as e:
            self.log(f"Error processing control telemetry: {e}", "error")
            if not self.quiet:
                import traceback
                traceback.print_exc()

    def _handle_vision_telemetry(self, message_type, timestamp, sequence_num, data):
        """Process vision telemetry data"""
        try:
            # Extract pose information if available
            if 'criticalData' in data and 'multiTargetPose' in data['criticalData']:
                position = data['criticalData']['multiTargetPose']['position']
                
                # Add to time series if valid
                if isinstance(position, list) and len(position) >= 3:
                    current_time = time.time()
                    
                    # Create time series if needed
                    if 'vision_pose_times' not in self.time_series:
                        self.time_series['vision_pose_times'] = deque(maxlen=MAX_POINTS)
                        self.time_series['vision_pose_x'] = deque(maxlen=MAX_POINTS)
                        self.time_series['vision_pose_y'] = deque(maxlen=MAX_POINTS)
                        self.time_series['vision_pose_z'] = deque(maxlen=MAX_POINTS)
                    
                    self.time_series['vision_pose_times'].append(current_time)
                    self.time_series['vision_pose_x'].append(position[0])
                    self.time_series['vision_pose_y'].append(position[1])
                    self.time_series['vision_pose_z'].append(position[2])
        
        except Exception as e:
            self.log(f"Error processing vision telemetry: {e}", "error")
    
    def _handle_system_telemetry(self, message_type, timestamp, sequence_num, data):
        """Process system telemetry data"""
        try:
            if 'status' in data:
                current_time = time.time()
                self.status_times.append(current_time)
                self.status_messages.append(data['status'])
        
        except Exception as e:
            self.log(f"Error processing system telemetry: {e}", "error")
    
    def _setup_layout(self):
        """Configure the Dash app layout"""
        # Add custom CSS
        app_css = {
            'external_url': 'https://codepen.io/chriddyp/pen/bWLwgP.css'
        }
        
        # Create custom CSS as a string
        custom_css = '''
            .graph-container {
                flex: 1;
                min-width: 500px;
                margin: 10px;
                padding: 15px;
                border: 1px solid #ddd;
                border-radius: 5px;
                background-color: white;
                box-shadow: 0 2px 5px rgba(0,0,0,0.1);
            }
            body {
                font-family: 'Segoe UI', Arial, sans-serif;
                background-color: #f5f5f5;
                margin: 0;
                padding: 20px;
            }
            .status-entry {
                padding: 5px;
                margin: 5px 0;
                border-radius: 3px;
            }
            .status-entry:nth-child(odd) {
                background-color: #f9f9f9;
            }
        '''
        
        # Create assets folder if it doesn't exist
        import os
        assets_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'assets')
        os.makedirs(assets_dir, exist_ok=True)
        
        # Write custom CSS to a file
        with open(os.path.join(assets_dir, 'custom.css'), 'w') as f:
            f.write(custom_css)
            
        self.app.layout = html.Div([
            html.H1("Drone Control Visualization", 
                    style={'textAlign': 'center', 'color': '#2c3e50', 'marginBottom': 30}),
            
            # First row: Tracking and Errors 
            html.Div([
                html.Div([
                    html.H3("Reference Tracking", style={'textAlign': 'center'}),
                    dcc.Graph(id='tracking-graph'),
                ], className='graph-container'),
                
                html.Div([
                    html.H3("Error Values", style={'textAlign': 'center'}),
                    dcc.Graph(id='error-graph'),
                ], className='graph-container'),
            ], style={'display': 'flex', 'flexWrap': 'wrap'}),
            
            # Second row: Control Output and Control Derivative
            html.Div([
                html.Div([
                    html.H3("Control Output", style={'textAlign': 'center'}),
                    dcc.Graph(id='control-graph'),
                ], className='graph-container'),
                
                html.Div([
                    html.H3("Control Derivative", style={'textAlign': 'center'}),
                    dcc.Graph(id='control-derivative-graph'),
                ], className='graph-container'),
            ], style={'display': 'flex', 'flexWrap': 'wrap'}),
            
            # Third row: 3D Visualization
            html.Div([
                html.Div([
                    html.H3("Current Position", style={'textAlign': 'center'}),
                    dcc.Graph(id='position-graph'),
                ], className='graph-container'),
                
                html.Div([
                    html.H3("3D Trajectory", style={'textAlign': 'center'}),
                    dcc.Graph(id='trajectory-graph'),
                ], className='graph-container'),
            ], style={'display': 'flex', 'flexWrap': 'wrap'}),
            
            # Fourth row: System Status and Statistics
            html.Div([
                html.Div([
                    html.H3("System Status", style={'textAlign': 'center'}),
                    html.Div(id='status-display', style={
                        'border': '1px solid #ddd',
                        'borderRadius': '5px',
                        'padding': '10px',
                        'height': '150px',
                        'overflowY': 'scroll'
                    }),
                ], style={'flex': 1, 'margin': '10px', 'minWidth': '500px'}),
                
                html.Div([
                    html.H3("Statistics", style={'textAlign': 'center'}),
                    html.Div(id='stats-display', style={
                        'border': '1px solid #ddd',
                        'borderRadius': '5px',
                        'padding': '10px',
                        'height': '150px'
                    }),
                ], style={'flex': 1, 'margin': '10px', 'minWidth': '500px'}),
            ], style={'display': 'flex', 'flexWrap': 'wrap'}),
            
            # Hidden div for storing UI state
            html.Div(id='ui-state', style={'display': 'none'}),
            
            dcc.Interval(
                id='interval-component',
                interval=500,  # in milliseconds
                n_intervals=0
            )
        ])

    
    def _setup_callbacks(self):
        """Configure the Dash app callbacks"""
        
        # Update control output graph
        @self.app.callback(
            Output('control-graph', 'figure'),
            [Input('interval-component', 'n_intervals')]
        )
        def update_control_graph(n_intervals):
            fig = go.Figure()
            
            if 'timestamps' in self.time_series and len(self.time_series['timestamps']) > 0:
                times = np.array(self.time_series['timestamps'])
                base_time = times[0]
                times = times - base_time  # Relative time in seconds
                
                # Try to use control output data from new format if available
                if 'controlOutput' in self.labels:
                    category = 'controlOutput'
                    labels = self.labels[category]
                    colors = ['red', 'green', 'blue']
                    
                    for i, label in enumerate(labels):
                        field_name = f"{category}_{i}"
                        if field_name in self.time_series and len(self.time_series[field_name]) > 0:
                            color = colors[i % len(colors)]
                            fig.add_trace(go.Scatter(
                                x=times[-len(self.time_series[field_name]):], 
                                y=list(self.time_series[field_name]),
                                mode='lines', name=label,
                                line=dict(color=color, width=2)
                            ))
                else:
                    # Fallback to command data directly from the packet
                    control_fields = ['control_u_x', 'control_u_y', 'control_u_z']
                    control_labels = ['u_x', 'u_y', 'u_z']
                    control_colors = ['red', 'green', 'blue']
                    
                    for i, field in enumerate(control_fields):
                        if field in self.time_series and len(self.time_series[field]) > 0:
                            fig.add_trace(go.Scatter(
                                x=times[-len(self.time_series[field]):], 
                                y=list(self.time_series[field]),
                                mode='lines', name=control_labels[i],
                                line=dict(color=control_colors[i], width=2)
                            ))
            
            # Set a dynamic graph title if control type is known
            title = f"Control Output ({self.control_type})" if self.control_type != "Unknown" else "Control Output"
            
            fig.update_layout(
                title=title,
                xaxis=dict(title='Time (s)'),
                yaxis=dict(title='Control Value'),
                margin=dict(l=40, r=20, t=40, b=30),
                legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
                hovermode='closest',
                # Add uirevision to maintain user interaction state
                uirevision=f'control_graph_{self.control_type}'
            )
            
            return fig
        
        # Update control derivative graph
        @self.app.callback(
            Output('control-derivative-graph', 'figure'),
            [Input('interval-component', 'n_intervals')]
        )
        def update_control_derivative_graph(n_intervals):
            fig = go.Figure()
            
            if 'timestamps' in self.time_series and len(self.time_series['timestamps']) > 0:
                times = np.array(self.time_series['timestamps'])
                base_time = times[0]
                times = times - base_time  # Relative time in seconds
                
                # Try to use control derivative data from new format
                if 'controlDerivative' in self.labels:
                    category = 'controlDerivative'
                    labels = self.labels[category]
                    colors = ['red', 'green', 'blue']
                    
                    for i, label in enumerate(labels):
                        field_name = f"{category}_{i}"
                        if field_name in self.time_series and len(self.time_series[field_name]) > 0:
                            color = colors[i % len(colors)]
                            fig.add_trace(go.Scatter(
                                x=times[-len(self.time_series[field_name]):], 
                                y=list(self.time_series[field_name]),
                                mode='lines', name=label,
                                line=dict(color=color, width=2)
                            ))
                # If no controlDerivative data, check for performanceData format
                elif 'performanceData' in self.time_series and 'controlDerivative' in self.time_series['performanceData']:
                    derivatives = self.time_series['performanceData']['controlDerivative']
                    if isinstance(derivatives, list) and len(derivatives) >= 3:
                        colors = ['red', 'green', 'blue']
                        labels = ['u_dot_x', 'u_dot_y', 'u_dot_z']
                        
                        for i in range(3):
                            fig.add_trace(go.Scatter(
                                x=times, 
                                y=[d[i] for d in derivatives],
                                mode='lines', name=labels[i],
                                line=dict(color=colors[i], width=2)
                            ))
                # If we have u_dot values in the command, use those
                elif 'command' in self.time_series and all(f'u_dot_{axis}' in self.time_series['command'] for axis in ['x', 'y', 'z']):
                    colors = ['red', 'green', 'blue']
                    labels = ['u_dot_x', 'u_dot_y', 'u_dot_z']
                    
                    for i, axis in enumerate(['x', 'y', 'z']):
                        fig.add_trace(go.Scatter(
                            x=times, 
                            y=[cmd[f'u_dot_{axis}'] for cmd in self.time_series['command']],
                            mode='lines', name=labels[i],
                            line=dict(color=colors[i], width=2)
                        ))
            
            # Set a dynamic graph title if control type is known
            title = f"Control Derivative ({self.control_type})" if self.control_type != "Unknown" else "Control Derivative"
            
            fig.update_layout(
                title=title,
                xaxis=dict(title='Time (s)'),
                yaxis=dict(title='Rate of Change'),
                margin=dict(l=40, r=20, t=40, b=30),
                legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
                hovermode='closest',
                # Add uirevision to maintain user interaction state
                uirevision=f'derivative_graph_{self.control_type}'
            )
            
            return fig
        
        # Update error graph
        @self.app.callback(
            Output('error-graph', 'figure'),
            [Input('interval-component', 'n_intervals')]
        )
        def update_error_graph(n_intervals):
            fig = go.Figure()
            
            if 'timestamps' in self.time_series and len(self.time_series['timestamps']) > 0:
                times = np.array(self.time_series['timestamps'])
                base_time = times[0]
                times = times - base_time  # Relative time in seconds
                
                # Try to use error data from new format first
                if 'errors' in self.labels:
                    category = 'errors'
                    labels = self.labels[category]
                    colors = ['red', 'green', 'blue', 'purple', 'orange', 'cyan']
                    
                    for i, label in enumerate(labels):
                        field_name = f"{category}_{i}"
                        if field_name in self.time_series and len(self.time_series[field_name]) > 0:
                            color = colors[i % len(colors)]
                            fig.add_trace(go.Scatter(
                                x=times[-len(self.time_series[field_name]):], 
                                y=list(self.time_series[field_name]),
                                mode='lines', name=label,
                                line=dict(color=color, width=2)
                            ))
                # Fall back to original format
                elif 'error_x' in self.time_series:
                    error_fields = ['error_x', 'error_y', 'error_z']
                    error_labels = ['Error X', 'Error Y', 'Error Z']
                    error_colors = ['red', 'green', 'blue']
                    
                    for i, field in enumerate(error_fields):
                        if field in self.time_series and len(self.time_series[field]) > 0:
                            fig.add_trace(go.Scatter(
                                x=times[-len(self.time_series[field]):], 
                                y=list(self.time_series[field]),
                                mode='lines', name=error_labels[i],
                                line=dict(color=error_colors[i], width=2)
                            ))
            
            # Set a dynamic graph title if control type is known
            title = f"Error Values ({self.control_type})" if self.control_type != "Unknown" else "Error Values"
            
            fig.update_layout(
                title=title,
                xaxis=dict(title='Time (s)'),
                yaxis=dict(title='Error Value'),
                margin=dict(l=40, r=20, t=40, b=30),
                legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
                hovermode='closest',
                # Add uirevision to maintain user interaction state
                uirevision=f'error_graph_{self.control_type}'
            )
            
            return fig
        
        # Update tracking graph
        @self.app.callback(
            Output('tracking-graph', 'figure'),
            [Input('interval-component', 'n_intervals')]
        )
        def update_tracking_graph(n_intervals):
            fig = go.Figure()
            
            if 'timestamps' in self.time_series and len(self.time_series['timestamps']) > 0:
                times = np.array(self.time_series['timestamps'])
                base_time = times[0]
                times = times - base_time  # Relative time in seconds
                
                # Try new format first
                if 'currentState' in self.labels and 'referenceState' in self.labels:
                    current_labels = self.labels['currentState']
                    reference_labels = self.labels['referenceState']
                    
                    colors = ['red', 'green', 'blue']
                    
                    # Show only position (first 3 elements), not rotation
                    min_length = min(len(current_labels), len(reference_labels), 3)
                    
                    for i in range(min_length):
                        current_field = f"currentState_{i}"
                        reference_field = f"referenceState_{i}"
                        
                        if (current_field in self.time_series and len(self.time_series[current_field]) > 0 and
                            reference_field in self.time_series and len(self.time_series[reference_field]) > 0):
                            
                            # Get the actual label for this field
                            label = current_labels[i].replace("Position ", "")
                            color = colors[i % len(colors)]
                            
                            # Current state
                            fig.add_trace(go.Scatter(
                                x=times[-len(self.time_series[current_field]):], 
                                y=list(self.time_series[current_field]),
                                mode='lines', name=f"Current {label}",
                                line=dict(color=color, width=2)
                            ))
                            
                            # Reference state
                            fig.add_trace(go.Scatter(
                                x=times[-len(self.time_series[reference_field]):], 
                                y=list(self.time_series[reference_field]),
                                mode='lines', name=f"Reference {label}",
                                line=dict(color=color, width=2, dash='dash')
                            ))
                # Fall back to original format
                elif 'current_x' in self.time_series and 'reference_x' in self.time_series:
                    axes = ['x', 'y', 'z']
                    colors = ['red', 'green', 'blue']
                    
                    for i, axis in enumerate(axes):
                        current_field = f'current_{axis}'
                        reference_field = f'reference_{axis}'
                        
                        if (current_field in self.time_series and len(self.time_series[current_field]) > 0 and
                            reference_field in self.time_series and len(self.time_series[reference_field]) > 0):
                            
                            # Current position
                            fig.add_trace(go.Scatter(
                                x=times[-len(self.time_series[current_field]):], 
                                y=list(self.time_series[current_field]),
                                mode='lines', name=f'Current {axis.upper()}',
                                line=dict(color=colors[i], width=2)
                            ))
                            
                            # Reference position
                            fig.add_trace(go.Scatter(
                                x=times[-len(self.time_series[reference_field]):], 
                                y=list(self.time_series[reference_field]),
                                mode='lines', name=f'Reference {axis.upper()}',
                                line=dict(color=colors[i], width=2, dash='dash')
                            ))
            
            # Set a dynamic graph title if control type is known
            title = f"Reference Tracking ({self.control_type})" if self.control_type != "Unknown" else "Reference Tracking"
            
            fig.update_layout(
                title=title,
                xaxis=dict(title='Time (s)'),
                yaxis=dict(title='Value'),
                margin=dict(l=40, r=20, t=40, b=30),
                legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
                hovermode='closest',
                # Add uirevision to maintain user interaction state
                uirevision=f'tracking_graph_{self.control_type}'
            )
            
            return fig
        
        # Update position graph
        @self.app.callback(
            Output('position-graph', 'figure'),
            [Input('interval-component', 'n_intervals')]
        )
        def update_position_graph(n_intervals):
            fig = go.Figure()
            
            if 'timestamps' in self.time_series and len(self.time_series['timestamps']) > 0:
                times = np.array(self.time_series['timestamps'])
                base_time = times[0]
                times = times - base_time  # Relative time in seconds
                
                # Try new format first
                if 'currentState' in self.labels:
                    labels = self.labels['currentState']
                    colors = ['red', 'green', 'blue']
                    
                    # Show only first 3 elements (position, not rotation)
                    for i in range(min(3, len(labels))):
                        field_name = f"currentState_{i}"
                        if field_name in self.time_series and len(self.time_series[field_name]) > 0:
                            label = labels[i]
                            color = colors[i % len(colors)]
                            
                            fig.add_trace(go.Scatter(
                                x=times[-len(self.time_series[field_name]):], 
                                y=list(self.time_series[field_name]),
                                mode='lines', name=label,
                                line=dict(color=color, width=2)
                            ))
                # Fall back to original format or vision data
                elif 'current_x' in self.time_series:
                    position_fields = ['current_x', 'current_y', 'current_z']
                    position_labels = ['X', 'Y', 'Z']
                    position_colors = ['red', 'green', 'blue']
                    
                    for i, field in enumerate(position_fields):
                        if field in self.time_series and len(self.time_series[field]) > 0:
                            fig.add_trace(go.Scatter(
                                x=times[-len(self.time_series[field]):], 
                                y=list(self.time_series[field]),
                                mode='lines', name=position_labels[i],
                                line=dict(color=position_colors[i], width=2)
                            ))
                # Try vision data as a last resort
                elif 'vision_pose_x' in self.time_series:
                    vision_times = np.array(self.time_series['vision_pose_times'])
                    if len(vision_times) > 0:
                        vision_times = vision_times - base_time  
                    
                    vision_fields = ['vision_pose_x', 'vision_pose_y', 'vision_pose_z']
                    vision_labels = ['Vision X', 'Vision Y', 'Vision Z']
                    vision_colors = ['red', 'green', 'blue']
                    
                    for i, field in enumerate(vision_fields):
                        if field in self.time_series and len(self.time_series[field]) > 0:
                            fig.add_trace(go.Scatter(
                                x=vision_times[-len(self.time_series[field]):],
                                y=list(self.time_series[field]),
                                mode='lines', name=vision_labels[i],
                                line=dict(color=vision_colors[i], width=2, dash='dot')
                            ))
            
            # Set a dynamic graph title if control type is known
            title = f"Current Position ({self.control_type})" if self.control_type != "Unknown" else "Current Position"
            
            fig.update_layout(
                title=title,
                xaxis=dict(title='Time (s)'),
                yaxis=dict(title='Value'),
                margin=dict(l=40, r=20, t=40, b=30),
                legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
                hovermode='closest',
                # Add uirevision to maintain user interaction state
                uirevision=f'position_graph_{self.control_type}'
            )
            
            return fig
        
        # Update 3D trajectory graph
        @self.app.callback(
            Output('trajectory-graph', 'figure'),
            [Input('interval-component', 'n_intervals')]
        )
        def update_trajectory_graph(n_intervals):
            fig = go.Figure()
            
            # Try new format first (currentState data)
            if ('currentState' in self.labels and 
                'currentState_0' in self.time_series and 
                'currentState_1' in self.time_series and 
                'currentState_2' in self.time_series):
                
                # Get position data
                x_data = list(self.time_series['currentState_0'])
                y_data = list(self.time_series['currentState_1'])
                z_data = list(self.time_series['currentState_2'])
                
                if len(x_data) > 0 and len(y_data) > 0 and len(z_data) > 0:
                    # Find minimum length to avoid index errors
                    min_len = min(len(x_data), len(y_data), len(z_data))
                    
                    # Add trajectory line
                    fig.add_trace(go.Scatter3d(
                        x=x_data[:min_len],
                        y=y_data[:min_len],
                        z=z_data[:min_len],
                        mode='lines',
                        line=dict(color='blue', width=4),
                        name='Trajectory'
                    ))
                    
                    # Add current position marker
                    fig.add_trace(go.Scatter3d(
                        x=[x_data[-1]],
                        y=[y_data[-1]],
                        z=[z_data[-1]],
                        mode='markers',
                        marker=dict(size=8, color='red'),
                        name='Current Position'
                    ))
                    
                    # Add reference position if available
                    if ('referenceState' in self.labels and 
                        'referenceState_0' in self.time_series and 
                        'referenceState_1' in self.time_series and 
                        'referenceState_2' in self.time_series):
                        
                        ref_x = self.time_series['referenceState_0'][-1]
                        ref_y = self.time_series['referenceState_1'][-1]
                        ref_z = self.time_series['referenceState_2'][-1]
                        
                        fig.add_trace(go.Scatter3d(
                            x=[ref_x],
                            y=[ref_y],
                            z=[ref_z],
                            mode='markers',
                            marker=dict(size=8, color='green', symbol='diamond'),
                            name='Reference Position'
                        ))
            
            # Fall back to original format
            elif ('current_x' in self.time_series and 
                  'current_y' in self.time_series and 
                  'current_z' in self.time_series):
                
                # Get position data
                x_data = list(self.time_series['current_x'])
                y_data = list(self.time_series['current_y'])
                z_data = list(self.time_series['current_z'])
                
                if len(x_data) > 0 and len(y_data) > 0 and len(z_data) > 0:
                    # Find minimum length to avoid index errors
                    min_len = min(len(x_data), len(y_data), len(z_data))
                    
                    # Add trajectory line
                    fig.add_trace(go.Scatter3d(
                        x=x_data[:min_len],
                        y=y_data[:min_len],
                        z=z_data[:min_len],
                        mode='lines',
                        line=dict(color='blue', width=4),
                        name='Trajectory'
                    ))
                    
                    # Add current position marker
                    fig.add_trace(go.Scatter3d(
                        x=[x_data[-1]],
                        y=[y_data[-1]],
                        z=[z_data[-1]],
                        mode='markers',
                        marker=dict(size=8, color='red'),
                        name='Current Position'
                    ))
                    
                    # Add reference position if available
                    if ('reference_x' in self.time_series and 
                        'reference_y' in self.time_series and 
                        'reference_z' in self.time_series):
                        
                        ref_x = self.time_series['reference_x'][-1]
                        ref_y = self.time_series['reference_y'][-1]
                        ref_z = self.time_series['reference_z'][-1]
                        
                        fig.add_trace(go.Scatter3d(
                            x=[ref_x],
                            y=[ref_y],
                            z=[ref_z],
                            mode='markers',
                            marker=dict(size=8, color='green', symbol='diamond'),
                            name='Reference Position'
                        ))
            
            # Set title with control type if known
            title = f"3D Trajectory ({self.control_type})" if self.control_type != "Unknown" else "3D Trajectory"
            
            fig.update_layout(
                title=title,
                scene=dict(
                    xaxis_title='X',
                    yaxis_title='Y',
                    zaxis_title='Z',
                    aspectmode='cube'
                ),
                margin=dict(l=0, r=0, b=0, t=40),
                legend=dict(yanchor="top", y=0.99, xanchor="left", x=0.01),
                # Add uirevision to maintain user interaction state
                uirevision=f'trajectory_graph_{self.control_type}'
            )
            
            return fig
        
        # Update status display
        @self.app.callback(
            Output('status-display', 'children'),
            [Input('interval-component', 'n_intervals')]
        )
        def update_status_display(n_intervals):
            status_elements = []
            
            for i in range(len(self.status_times) - 1, -1, -1):
                timestamp = time.strftime('%H:%M:%S', time.localtime(self.status_times[i]))
                message = self.status_messages[i]
                
                status_elements.append(html.Div(
                    f"[{timestamp}] {message}",
                    className='status-entry'
                ))
            
            return status_elements
        
        # Update statistics display
        @self.app.callback(
            Output('stats-display', 'children'),
            [Input('interval-component', 'n_intervals')]
        )
        def update_stats_display(n_intervals):
            stats = self.telemetry.get_stats()
            
            # Calculate data point counts for available data types
            control_count = len(self.time_series.get('control_u_x', []))
            
            # Try to get error data count from both formats
            error_count = 0
            if 'errors_0' in self.time_series:
                error_count = len(self.time_series['errors_0'])
            elif 'error_x' in self.time_series:
                error_count = len(self.time_series['error_x'])
            
            # Try to get position data count from both formats
            position_count = 0
            if 'currentState_0' in self.time_series:
                position_count = len(self.time_series['currentState_0'])
            elif 'current_x' in self.time_series:
                position_count = len(self.time_series['current_x'])
            
            return [
                html.Div([
                    html.Strong("Control Type: "),
                    f"{self.control_type}"
                ]),
                html.Div([
                    html.Strong("Messages Received: "),
                    f"{stats['messages_received']}"
                ]),
                html.Div([
                    html.Strong("Total Data Received: "),
                    f"{stats['bytes_received'] / 1024:.2f} KB"
                ]),
                html.Div([
                    html.Strong("Dropped Messages: "),
                    ", ".join([f"{k}: {v}" for k, v in stats['dropped_messages'].items()])
                ]),
                html.Div([
                    html.Strong("Data Points: "),
                    f"Control: {control_count}, "
                    f"Error: {error_count}, "
                    f"Position: {position_count}"
                ])
            ]
    
    def run(self, debug=False, port=8050):
        """Run the Dash application"""
        try:
            self.log(f"Starting visualization server on port {port}")
            self.app.run(debug=debug, port=port, dev_tools_silence_routes_logging=True)
        finally:
            self.telemetry.stop()
    
    def stop(self):
        """Stop the telemetry receiver"""
        self.telemetry.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Drone Control Visualization")
    parser.add_argument("--telemetry-port", type=int, default=14559,
                        help="UDP port for telemetry data (default: 14559)")
    parser.add_argument("--web-port", type=int, default=8050,
                        help="Port for web interface (default: 8050)")
    parser.add_argument("--debug", action="store_true",
                        help="Enable debug mode for Dash")
    parser.add_argument("--quiet", action="store_true",
                        help="Suppress console output")
    
    args = parser.parse_args()
    
    # Configure logging based on quiet mode
    if args.quiet:
        # Completely suppress logging
        logging.basicConfig(level=logging.CRITICAL)
        # Redirect stdout and stderr
        sys.stdout = open(os.devnull, 'w')
        sys.stderr = open(os.devnull, 'w')
    else:
        # Regular logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
    
    visualizer = DroneVisualizer(telemetry_port=args.telemetry_port, quiet=args.quiet)
    try:
        visualizer.run(debug=args.debug, port=args.web_port)
    except KeyboardInterrupt:
        if not args.quiet:
            print("Shutting down...")
    finally:
        visualizer.stop()