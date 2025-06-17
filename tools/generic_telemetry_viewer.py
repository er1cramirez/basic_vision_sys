#!/usr/bin/env python3
"""
Generic Telemetry Visualizer
Inspired by WPILib's SmartDashboard - displays any telemetry data published by the C++ system
"""

import socket
import struct
import json
import threading
import time
import logging
import argparse
import sys
import os
from collections import deque, defaultdict
from datetime import datetime

import dash
from dash import dcc, html, Input, Output, State
import plotly.graph_objs as go
import plotly.express as px
import numpy as np

# Message types - MUST MATCH C++ ENUM VALUES
TELEMETRY_CONTROL = 1
TELEMETRY_VISION = 2
TELEMETRY_SYSTEM = 3
TELEMETRY_COMMAND = 4
TELEMETRY_GENERIC = 5

# Maximum number of data points to keep in history
MAX_POINTS = 1000

class GenericTelemetryReceiver:
    def __init__(self, port=14559, buffer_size=8192, quiet=False):
        self.port = port
        self.buffer_size = buffer_size
        self.running = False
        self.socket = None
        self.receive_thread = None
        self.quiet = quiet
        
        # Generic data storage organized by category and entry name
        self.telemetry_data = defaultdict(lambda: defaultdict(lambda: {
            'values': deque(maxlen=MAX_POINTS),
            'timestamps': deque(maxlen=MAX_POINTS),
            'latest_value': None,
            'unit': '',
            'category': 'default',
            'data_type': 'unknown'
        }))
        
        # Lock for thread-safe access
        self.data_lock = threading.Lock()
        
        # Statistics
        self.messages_received = 0
        self.bytes_received = 0
        self.last_update_time = time.time()
        
        # Event log
        self.events = deque(maxlen=100)
        
    def log(self, message, level="info"):
        """Custom logging function that respects quiet mode"""
        if self.quiet:
            return
            
        timestamp = datetime.now().strftime("%H:%M:%S")
        if level == "info":
            print(f"[{timestamp}] INFO: {message}")
        elif level == "error":
            print(f"[{timestamp}] ERROR: {message}")
        elif level == "warning":
            print(f"[{timestamp}] WARNING: {message}")
    
    def start(self):
        if self.running:
            return
        
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind(('0.0.0.0', self.port))
            
            self.running = True
            self.receive_thread = threading.Thread(target=self._receive_loop)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            
            self.log(f"Generic telemetry receiver started on port {self.port}")
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
        self.log(f"Total messages received: {self.messages_received}")
    
    def _receive_loop(self):
        """Main receive loop"""
        while self.running:
            try:
                data, addr = self.socket.recvfrom(self.buffer_size)
                self.bytes_received += len(data)
                
                if len(data) < 20:
                    continue
                
                # Parse header
                message_type = struct.unpack('!I', data[0:4])[0]
                timestamp = struct.unpack('!Q', data[4:12])[0]
                sequence_num = struct.unpack('!I', data[12:16])[0]
                payload_size = struct.unpack('!I', data[16:20])[0]
                
                if len(data) < 20 + payload_size:
                    continue
                
                payload = data[20:20 + payload_size]
                
                try:
                    payload_str = payload.decode('utf-8')
                    payload_json = json.loads(payload_str)
                    
                    self.messages_received += 1
                    self.last_update_time = time.time()
                    
                    # Process the message based on type
                    if message_type == TELEMETRY_SYSTEM:
                        self._process_telemetry_message(payload_json)
                    elif message_type in [TELEMETRY_CONTROL, TELEMETRY_VISION]:
                        self._process_legacy_message(message_type, payload_json)
                        
                except Exception as e:
                    if not self.quiet:
                        self.log(f"Error parsing payload: {e}", "error")
                
            except Exception as e:
                if self.running:
                    self.log(f"Error in receive loop: {e}", "error")
    
    def _process_telemetry_message(self, data):
        """Process generic telemetry message"""
        try:
            if 'entries' in data:
                current_time = time.time()
                
                with self.data_lock:
                    for entry_name, entry_data in data['entries'].items():
                        if 'latestValue' in entry_data:
                            category = entry_data.get('category', 'default')
                            unit = entry_data.get('unit', '')
                            
                            # Store the data
                            entry_store = self.telemetry_data[category][entry_name]
                            entry_store['latest_value'] = entry_data['latestValue']
                            entry_store['unit'] = unit
                            entry_store['category'] = category
                            
                            # Add to time series
                            entry_store['timestamps'].append(current_time)
                            
                            # Handle different value types
                            if isinstance(entry_data['latestValue'], (int, float)):
                                entry_store['values'].append(entry_data['latestValue'])
                                entry_store['data_type'] = 'scalar'
                            elif isinstance(entry_data['latestValue'], list):
                                entry_store['values'].append(entry_data['latestValue'])
                                entry_store['data_type'] = 'vector'
                            elif isinstance(entry_data['latestValue'], str):
                                entry_store['values'].append(entry_data['latestValue'])
                                entry_store['data_type'] = 'string'
                                # For string data, also add to events
                                self.events.append({
                                    'time': current_time,
                                    'category': category,
                                    'name': entry_name,
                                    'value': entry_data['latestValue']
                                })
                            else:
                                entry_store['values'].append(str(entry_data['latestValue']))
                                entry_store['data_type'] = 'other'
                                
        except Exception as e:
            self.log(f"Error processing telemetry message: {e}", "error")
    
    def _process_legacy_message(self, message_type, data):
        """Process legacy control/vision messages for backward compatibility"""
        try:
            current_time = time.time()
            
            with self.data_lock:
                if message_type == TELEMETRY_CONTROL and 'command' in data:
                    # Legacy control data
                    cmd = data['command']
                    for axis in ['u_x', 'u_y', 'u_z']:
                        if axis in cmd:
                            entry_store = self.telemetry_data['legacy_control'][f'control_{axis}']
                            entry_store['values'].append(cmd[axis])
                            entry_store['timestamps'].append(current_time)
                            entry_store['latest_value'] = cmd[axis]
                            entry_store['unit'] = 'control_unit'
                            entry_store['data_type'] = 'scalar'
                            
        except Exception as e:
            self.log(f"Error processing legacy message: {e}", "error")
    
    def get_categories(self):
        """Get all available categories"""
        with self.data_lock:
            return list(self.telemetry_data.keys())
    
    def get_entries_in_category(self, category):
        """Get all entries in a specific category"""
        with self.data_lock:
            if category in self.telemetry_data:
                return list(self.telemetry_data[category].keys())
            return []
    
    def get_entry_data(self, category, entry_name):
        """Get data for a specific entry"""
        with self.data_lock:
            if category in self.telemetry_data and entry_name in self.telemetry_data[category]:
                entry = self.telemetry_data[category][entry_name]
                return {
                    'values': list(entry['values']),
                    'timestamps': list(entry['timestamps']),
                    'latest_value': entry['latest_value'],
                    'unit': entry['unit'],
                    'data_type': entry['data_type']
                }
            return None
    
    def get_all_scalar_entries(self):
        """Get all scalar entries across all categories"""
        scalar_entries = {}
        with self.data_lock:
            for category, entries in self.telemetry_data.items():
                for entry_name, entry_data in entries.items():
                    if entry_data['data_type'] == 'scalar':
                        scalar_entries[f"{category}/{entry_name}"] = entry_data
        return scalar_entries
    
    def get_recent_events(self, max_count=20):
        """Get recent event entries"""
        with self.data_lock:
            return list(self.events)[-max_count:]
    
    def get_stats(self):
        """Get receiver statistics"""
        return {
            'messages_received': self.messages_received,
            'bytes_received': self.bytes_received,
            'categories_count': len(self.telemetry_data),
            'total_entries': sum(len(entries) for entries in self.telemetry_data.values()),
            'last_update': self.last_update_time
        }


class GenericTelemetryVisualizer:
    def __init__(self, telemetry_port=14559, quiet=False):
        self.quiet = quiet
        self.receiver = GenericTelemetryReceiver(port=telemetry_port, quiet=quiet)
        
        # Start receiver
        self.receiver.start()
        
        # Initialize Dash app
        self.app = dash.Dash(__name__, suppress_callback_exceptions=True)
        self.app.title = "Generic Telemetry Dashboard"
        
        # Setup layout and callbacks
        self._setup_layout()
        self._setup_callbacks()
    
    def _setup_layout(self):
        """Configure the Dash app layout"""
        self.app.layout = html.Div([
            html.H1("Generic Telemetry Dashboard", 
                    style={'textAlign': 'center', 'color': '#2c3e50', 'marginBottom': 30}),
            
            # Control panel
            html.Div([
                html.Div([
                    html.Label("Category:"),
                    dcc.Dropdown(
                        id='category-dropdown',
                        options=[],
                        value=None,
                        placeholder="Select a category"
                    )
                ], style={'width': '30%', 'display': 'inline-block', 'marginRight': '10px'}),
                
                html.Div([
                    html.Label("Entries:"),
                    dcc.Dropdown(
                        id='entries-dropdown',
                        options=[],
                        value=[],
                        multi=True,
                        placeholder="Select entries to plot"
                    )
                ], style={'width': '60%', 'display': 'inline-block'}),
            ], style={'padding': '20px', 'backgroundColor': '#f8f9fa', 'marginBottom': '20px'}),
            
            # Graphs row
            html.Div([
                html.Div([
                    html.H3("Time Series Plot", style={'textAlign': 'center'}),
                    dcc.Graph(id='time-series-graph'),
                ], style={'width': '70%', 'display': 'inline-block', 'padding': '10px'}),
                
                html.Div([
                    html.H3("Current Values", style={'textAlign': 'center'}),
                    html.Div(id='current-values-display', style={
                        'height': '400px', 'overflowY': 'scroll', 'padding': '10px',
                        'border': '1px solid #ddd', 'borderRadius': '5px'
                    }),
                ], style={'width': '30%', 'display': 'inline-block', 'padding': '10px'}),
            ]),
            
            # Second row
            html.Div([
                html.Div([
                    html.H3("All Scalar Values", style={'textAlign': 'center'}),
                    dcc.Graph(id='scalar-overview-graph'),
                ], style={'width': '50%', 'display': 'inline-block', 'padding': '10px'}),
                
                html.Div([
                    html.H3("Recent Events", style={'textAlign': 'center'}),
                    html.Div(id='events-display', style={
                        'height': '400px', 'overflowY': 'scroll', 'padding': '10px',
                        'border': '1px solid #ddd', 'borderRadius': '5px'
                    }),
                ], style={'width': '50%', 'display': 'inline-block', 'padding': '10px'}),
            ]),
            
            # Statistics
            html.Div([
                html.H3("System Statistics", style={'textAlign': 'center'}),
                html.Div(id='stats-display', style={
                    'textAlign': 'center', 'padding': '20px',
                    'backgroundColor': '#f8f9fa', 'borderRadius': '5px'
                }),
            ], style={'margin': '20px'}),
            
            dcc.Interval(
                id='interval-component',
                interval=500,  # Update every 500ms
                n_intervals=0
            )
        ])
    
    def _setup_callbacks(self):
        """Setup Dash callbacks"""
        
        @self.app.callback(
            Output('category-dropdown', 'options'),
            [Input('interval-component', 'n_intervals')]
        )
        def update_categories(n_intervals):
            categories = self.receiver.get_categories()
            return [{'label': cat, 'value': cat} for cat in categories]
        
        @self.app.callback(
            Output('entries-dropdown', 'options'),
            [Input('category-dropdown', 'value')]
        )
        def update_entries(selected_category):
            if not selected_category:
                return []
            
            entries = self.receiver.get_entries_in_category(selected_category)
            return [{'label': entry, 'value': entry} for entry in entries]
        
        @self.app.callback(
            Output('time-series-graph', 'figure'),
            [Input('interval-component', 'n_intervals'),
             Input('category-dropdown', 'value'),
             Input('entries-dropdown', 'value')]
        )
        def update_time_series(n_intervals, category, selected_entries):
            fig = go.Figure()
            
            if category and selected_entries:
                for entry in selected_entries:
                    data = self.receiver.get_entry_data(category, entry)
                    if data and data['data_type'] == 'scalar' and data['timestamps']:
                        # Convert timestamps to relative time in seconds
                        if data['timestamps']:
                            start_time = data['timestamps'][0]
                            times = [(t - start_time) for t in data['timestamps']]
                            
                            fig.add_trace(go.Scatter(
                                x=times,
                                y=data['values'],
                                mode='lines+markers',
                                name=f"{entry} ({data['unit']})",
                                line=dict(width=2),
                                marker=dict(size=4)
                            ))
            
            fig.update_layout(
                title=f"Time Series: {category}" if category else "Select Category and Entries",
                xaxis_title="Time (seconds)",
                yaxis_title="Value",
                hovermode='x unified',
                showlegend=True
            )
            
            return fig
        
        @self.app.callback(
            Output('current-values-display', 'children'),
            [Input('interval-component', 'n_intervals')]
        )
        def update_current_values(n_intervals):
            scalars = self.receiver.get_all_scalar_entries()
            
            if not scalars:
                return html.P("No scalar data available")
            
            items = []
            for entry_path, data in scalars.items():
                value = data['latest_value']
                unit = data['unit']
                
                if isinstance(value, float):
                    value_str = f"{value:.3f}"
                else:
                    value_str = str(value)
                
                items.append(
                    html.Div([
                        html.Strong(entry_path + ": "),
                        html.Span(f"{value_str} {unit}")
                    ], style={'marginBottom': '5px', 'padding': '3px'})
                )
            
            return items
        
        @self.app.callback(
            Output('scalar-overview-graph', 'figure'),
            [Input('interval-component', 'n_intervals')]
        )
        def update_scalar_overview(n_intervals):
            scalars = self.receiver.get_all_scalar_entries()
            
            # Create a bar chart of current values
            if scalars:
                names = list(scalars.keys())
                values = [data['latest_value'] for data in scalars.values()]
                
                # Limit to most recent entries to avoid clutter
                if len(names) > 20:
                    names = names[-20:]
                    values = values[-20:]
                
                fig = go.Figure(data=[
                    go.Bar(x=names, y=values, 
                           text=[f"{v:.2f}" if isinstance(v, float) else str(v) for v in values],
                           textposition='auto')
                ])
                
                fig.update_layout(
                    title="Current Scalar Values",
                    xaxis_title="Entry",
                    yaxis_title="Value",
                    xaxis_tickangle=-45
                )
            else:
                fig = go.Figure()
                fig.update_layout(title="No scalar data available")
            
            return fig
        
        @self.app.callback(
            Output('events-display', 'children'),
            [Input('interval-component', 'n_intervals')]
        )
        def update_events(n_intervals):
            events = self.receiver.get_recent_events()
            
            if not events:
                return html.P("No events logged")
            
            items = []
            for event in reversed(events):  # Most recent first
                timestamp = datetime.fromtimestamp(event['time']).strftime("%H:%M:%S")
                items.append(
                    html.Div([
                        html.Span(f"[{timestamp}] ", style={'color': '#666', 'fontSize': '0.9em'}),
                        html.Strong(f"{event['category']}/{event['name']}: ", style={'color': '#2c3e50'}),
                        html.Span(str(event['value']))
                    ], style={'marginBottom': '5px', 'padding': '3px', 'borderBottom': '1px solid #eee'})
                )
            
            return items
        
        @self.app.callback(
            Output('stats-display', 'children'),
            [Input('interval-component', 'n_intervals')]
        )
        def update_stats(n_intervals):
            stats = self.receiver.get_stats()
            
            return html.Div([
                html.P(f"Messages Received: {stats['messages_received']}"),
                html.P(f"Bytes Received: {stats['bytes_received']:,}"),
                html.P(f"Categories: {stats['categories_count']}"),
                html.P(f"Total Entries: {stats['total_entries']}"),
                html.P(f"Last Update: {datetime.fromtimestamp(stats['last_update']).strftime('%H:%M:%S') if stats['last_update'] else 'Never'}")
            ])
    
    def run(self, debug=False, port=8050):
        """Run the Dash application"""
        try:
            if not self.quiet:
                print(f"Starting generic telemetry visualizer on port {port}")
                print("Open http://localhost:8050 in your browser")
            self.app.run(debug=debug, port=port, dev_tools_silence_routes_logging=True)
        finally:
            self.receiver.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generic Telemetry Visualizer")
    parser.add_argument("--telemetry-port", type=int, default=14559,
                        help="UDP port for telemetry data (default: 14559)")
    parser.add_argument("--web-port", type=int, default=8050,
                        help="Port for web interface (default: 8050)")
    parser.add_argument("--debug", action="store_true",
                        help="Enable debug mode for Dash")
    parser.add_argument("--quiet", action="store_true",
                        help="Suppress console output")
    
    args = parser.parse_args()
    
    visualizer = GenericTelemetryVisualizer(telemetry_port=args.telemetry_port, quiet=args.quiet)
    try:
        visualizer.run(debug=args.debug, port=args.web_port)
    except KeyboardInterrupt:
        if not args.quiet:
            print("Shutting down...")
    finally:
        visualizer.receiver.stop()
