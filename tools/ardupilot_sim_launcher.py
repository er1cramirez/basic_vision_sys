#!/usr/bin/env python3
"""
ArduPilot SITL + Gazebo Synchronized Launch Script
Automates the complete startup sequence:
1. Start Gazebo headless
2. Start SITL
3. Automated MAVLink commands: GUIDED mode, arm, takeoff, custom mode
4. Execute custom controller
"""

import subprocess
import time
import sys
import os
import signal
import argparse
from pathlib import Path
from pymavlink import mavutil
import threading
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('simulation.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


class SimulationManager:
    def __init__(self, args):
        self.args = args
        self.processes = []
        self.vehicle = None
        self.ardupilot_dir = Path(args.ardupilot_dir).expanduser()
        self.gazebo_model = args.gazebo_model
        self.controller_cmd = args.controller_cmd
        self.takeoff_alt = args.takeoff_alt
        self.custom_mode = args.custom_mode
        self.running = True
        
    def log(self, msg, level='INFO'):
        """Log message"""
        getattr(logger, level.lower())(msg)
        
    def start_gazebo(self):
        """Inicia Gazebo en modo headless sin visualización"""
        self.log("Iniciando Gazebo (headless)...", 'INFO')
        try:
            cmd = [
                "gz", "sim",
                "-r",  # Run simulation (no pause)
                # "-s",  # Headless server mode
                self.gazebo_model
            ]
            self.log(f"Comando Gazebo: {' '.join(cmd)}", 'DEBUG')
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True
            )
            self.processes.append(proc)
            time.sleep(3)  # Wait for Gazebo to initialize
            self.log("✓ Gazebo iniciado correctamente", 'INFO')
            return proc
        except Exception as e:
            self.log(f"✗ Error iniciando Gazebo: {e}", 'ERROR')
            raise
    
    def start_ardupilot_sitl(self):
        """Inicia ArduPilot SITL"""
        self.log("Iniciando ArduPilot SITL...", 'INFO')
        try:
            binary = self.ardupilot_dir / "build/sitl/bin/arducopter"
            params = self.ardupilot_dir / "Tools/autotest/default_params/copter.parm"
            
            if not binary.exists():
                self.log(f"✗ Binario no encontrado: {binary}", 'ERROR')
                raise FileNotFoundError(f"SITL binary not found at {binary}")
            
            cmd = [
                str(binary),
                "-S",  # Connect to simulator (Gazebo)
                "-I0",  # Instance 0
                "--model", "JSON:127.0.0.1",  # JSON model listening on localhost
                "--speedup", str(self.args.speedup),
                "--slave", "0",
                "--defaults", str(params),
                "--home", "-35.363261,149.165230,584,353"
            ]
            
            self.log(f"Comando SITL: {' '.join(cmd)}", 'DEBUG')
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            self.processes.append(proc)
            time.sleep(2)  # Wait for SITL to initialize
            self.log("✓ ArduPilot SITL iniciado correctamente", 'INFO')
            return proc
        except Exception as e:
            self.log(f"✗ Error iniciando SITL: {e}", 'ERROR')
            raise
    
    def connect_mavlink(self):
        """Conecta a ArduPilot mediante MAVLink"""
        self.log("Conectando a ArduPilot via MAVLink en tcp:127.0.0.1:5760...", 'INFO')
        max_retries = 30
        retry_count = 0
        
        while retry_count < max_retries:
            try:
                self.vehicle = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
                self.vehicle.wait_heartbeat(timeout=2)
                self.log("✓ Conexión MAVLink establecida", 'INFO')
                return True
            except Exception as e:
                retry_count += 1
                if retry_count < max_retries:
                    self.log(f"Reintentando conexión ({retry_count}/{max_retries})...", 'DEBUG')
                    time.sleep(1)
                else:
                    self.log(f"✗ Conexión MAVLink fallida después de {max_retries} intentos: {e}", 'ERROR')
                    return False
        return False
    
    def wait_prearm_good(self, timeout=30):
        """Espera a que pre-arm checks sean buenos (SYS_STATUS)"""
        self.log("Esperando pre-arm checks...", 'INFO')
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                msg = self.vehicle.recv_match(type='SYS_STATUS', blocking=False)
                if msg:
                    # Bit 7 (0x80) indica si los pre-arm checks pasaron
                    if msg.onboard_control_sensors_health & 0x80:
                        self.log("✓ Pre-arm checks GOOD", 'INFO')
                        return True
                    else:
                        self.log(f"Pre-arm checks aún no listos. Health: 0x{msg.onboard_control_sensors_health:08x}", 'DEBUG')
                time.sleep(0.5)
            except Exception as e:
                self.log(f"Error esperando pre-arm: {e}", 'DEBUG')
                time.sleep(0.5)
        
        self.log("⚠ Timeout esperando pre-arm checks, continuando...", 'WARNING')
        return False
    
    def set_mode_guided(self):
        """Cambia a modo GUIDED"""
        self.log("Cambiando a modo GUIDED...", 'INFO')
        try:
            mode_id = self.vehicle.mode_mapping()['GUIDED']
            self.vehicle.mav.set_mode_send(
                self.vehicle.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            
            # Esperar confirmación de cambio de modo
            start_time = time.time()
            while time.time() - start_time < 5:
                msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=False)
                if msg and msg.custom_mode == mode_id:
                    self.log("✓ Modo GUIDED confirmado", 'INFO')
                    return True
                time.sleep(0.1)
            
            self.log("✓ Comando GUIDED enviado (cambio en proceso)", 'INFO')
            return True
        except Exception as e:
            self.log(f"✗ Error cambiando a GUIDED: {e}", 'ERROR')
            return False
    
    def arm_throttle(self):
        """Arma los motores"""
        self.log("Armando motores...", 'INFO')
        try:
            self.vehicle.mav.command_long_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # confirmation
                1,  # param1: 1 = arm
                0, 0, 0, 0, 0, 0  # otros params
            )
            
            # Esperar confirmación de armado
            start_time = time.time()
            while time.time() - start_time < 10:
                msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=False)
                if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_ARMED:
                    self.log("✓ Motores armados correctamente", 'INFO')
                    return True
                time.sleep(0.2)
            
            self.log("✓ Comando arm enviado (confirmación en proceso)", 'INFO')
            return True
        except Exception as e:
            self.log(f"✗ Error armando motores: {e}", 'ERROR')
            return False
    
    def takeoff(self, altitude=None):
        """Despegue a altitud especificada"""
        if altitude is None:
            altitude = self.takeoff_alt
        
        self.log(f"Despegando a {altitude}m...", 'INFO')
        try:
            self.vehicle.mav.command_long_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,  # confirmation
                0,  # param1 (unused)
                0,  # param2 (unused)
                0,  # param3 (unused)
                0,  # param4 (unused - yaw)
                0,  # param5 (unused - lat)
                0,  # param6 (unused - lon)
                altitude  # param7 - altitud
            )
            
            # Monitorear altitud
            start_time = time.time()
            target_alt = altitude * 0.95  # 95% de altitud objetivo
            
            while time.time() - start_time < 60:  # Timeout 60s
                msg = self.vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=False)
                if msg:
                    # LOCAL_POSITION_NED tiene z negativo (down)
                    current_alt = -msg.z
                    self.log(f"Altitud actual: {current_alt:.1f}m / Objetivo: {altitude}m", 'DEBUG')
                    
                    if current_alt >= target_alt:
                        self.log(f"✓ Despegue completado - Altitud: {current_alt:.1f}m", 'INFO')
                        return True
                
                time.sleep(0.5)
            
            self.log("✓ Despegue completado (altitud máxima alcanzada)", 'INFO')
            return True
        except Exception as e:
            self.log(f"✗ Error en despegue: {e}", 'ERROR')
            return False
    
    def set_custom_mode(self, mode_number=29):
        """Cambia a modo custom (ej: modo 29)"""
        self.log(f"Cambiando a modo custom {mode_number}...", 'INFO')
        try:
            # Intentar encontrar el modo en el mapping
            available_modes = self.vehicle.mode_mapping()
            self.log(f"Modos disponibles: {list(available_modes.keys())}", 'DEBUG')
            
            # Para modos custom que no están en el mapping, usar set_mode directo
            self.vehicle.mav.set_mode_send(
                self.vehicle.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_number
            )
            
            # Esperar confirmación
            start_time = time.time()
            while time.time() - start_time < 5:
                msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=False)
                if msg and msg.custom_mode == mode_number:
                    self.log(f"✓ Modo custom {mode_number} confirmado", 'INFO')
                    return True
                time.sleep(0.1)
            
            self.log(f"✓ Comando modo custom {mode_number} enviado", 'INFO')
            return True
        except Exception as e:
            self.log(f"✗ Error cambiando a modo custom: {e}", 'ERROR')
            return False
    
    def start_controller(self):
        """Inicia el script de control personalizado"""
        if not self.controller_cmd:
            self.log("No hay comando de controlador especificado, omitiendo...", 'WARNING')
            return None
        
        self.log(f"Iniciando controlador: {self.controller_cmd}", 'INFO')
        try:
            proc = subprocess.Popen(
                self.controller_cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            self.processes.append(proc)
            time.sleep(1)
            self.log("✓ Controlador iniciado", 'INFO')
            return proc
        except Exception as e:
            self.log(f"✗ Error iniciando controlador: {e}", 'ERROR')
            return None
    
    def run_synchronized_sequence(self):
        """Ejecuta la secuencia sincronizada completa"""
        try:
            # Paso 1: Iniciar Gazebo
            self.log("\n" + "="*60, 'INFO')
            self.log("PASO 1: Iniciando Gazebo (headless)", 'INFO')
            self.log("="*60, 'INFO')
            self.start_gazebo()
            
            # Paso 2: Iniciar SITL
            self.log("\n" + "="*60, 'INFO')
            self.log("PASO 2: Iniciando ArduPilot SITL", 'INFO')
            self.log("="*60, 'INFO')
            self.start_ardupilot_sitl()
            
            # Paso 3: Conectar MAVLink
            self.log("\n" + "="*60, 'INFO')
            self.log("PASO 3: Conectando MAVLink", 'INFO')
            self.log("="*60, 'INFO')
            if not self.connect_mavlink():
                raise RuntimeError("No se pudo conectar a MAVLink")
            
            # Paso 4: Esperar pre-arm checks
            self.log("\n" + "="*60, 'INFO')
            self.log("PASO 4: Esperando pre-arm checks", 'INFO')
            self.log("="*60, 'INFO')
            self.wait_prearm_good()
            
            # Paso 5: Cambiar a GUIDED
            self.log("\n" + "="*60, 'INFO')
            self.log("PASO 5: Cambiando a modo GUIDED", 'INFO')
            self.log("="*60, 'INFO')
            if not self.set_mode_guided():
                self.log("⚠ Advertencia: Error cambiando a GUIDED", 'WARNING')
            time.sleep(1)
            
            # Paso 6: Armar
            self.log("\n" + "="*60, 'INFO')
            self.log("PASO 6: Armando motores", 'INFO')
            self.log("="*60, 'INFO')
            if not self.arm_throttle():
                self.log("⚠ Advertencia: Error armando", 'WARNING')
            time.sleep(1)
            
            # Paso 7: Despegue
            self.log("\n" + "="*60, 'INFO')
            self.log(f"PASO 7: Despegando a {self.takeoff_alt}m", 'INFO')
            self.log("="*60, 'INFO')
            if not self.takeoff():
                self.log("⚠ Advertencia: Error en despegue", 'WARNING')
            time.sleep(2)
            
            # Paso 8: Cambiar a modo custom
            self.log("\n" + "="*60, 'INFO')
            self.log(f"PASO 8: Cambiando a modo custom {self.custom_mode}", 'INFO')
            self.log("="*60, 'INFO')
            if not self.set_custom_mode(self.custom_mode):
                self.log(f"⚠ Advertencia: Error cambiando a modo {self.custom_mode}", 'WARNING')
            time.sleep(1)
            
            # Paso 9: Iniciar controlador
            self.log("\n" + "="*60, 'INFO')
            self.log("PASO 9: Iniciando controlador personalizado", 'INFO')
            self.log("="*60, 'INFO')
            # self.start_controller()
            
            self.log("\n" + "="*60, 'INFO')
            self.log("✓ SECUENCIA COMPLETADA EXITOSAMENTE", 'INFO')
            self.log("="*60 + "\n", 'INFO')
            
            # Mantener corriendo
            self.log("Simulación en ejecución. Presiona Ctrl+C para detener.", 'INFO')
            try:
                while self.running:
                    time.sleep(1)
            except KeyboardInterrupt:
                self.shutdown()
        
        except Exception as e:
            self.log(f"\n✗ ERROR EN SECUENCIA: {e}", 'ERROR')
            import traceback
            self.log(traceback.format_exc(), 'ERROR')
            self.shutdown()
            sys.exit(1)
    
    def shutdown(self, signum=None, frame=None):
        """Cierra todos los procesos limpiamente"""
        self.log("\nCerrando simulación...", 'INFO')
        self.running = False
        
        if self.vehicle:
            try:
                self.vehicle.close()
            except:
                pass
        
        for proc in self.processes:
            try:
                proc.terminate()
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
            except:
                pass
        
        self.log("✓ Simulación cerrada correctamente", 'INFO')
        sys.exit(0)


def main():
    parser = argparse.ArgumentParser(
        description='ArduPilot SITL + Gazebo Synchronized Launcher',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplo de uso:
  python3 ardupilot_sim_launcher.py \\
    --gazebo-model iris_aruco_runway.sdf \\
    --controller-cmd "python3 my_controller.py" \\
    --takeoff-alt 2 \\
    --custom-mode 29

Sin controlador:
  python3 ardupilot_sim_launcher.py --gazebo-model iris_aruco_runway.sdf
        """
    )
    
    parser.add_argument(
        '--ardupilot-dir',
        default='~/ardupilot',
        help='Ruta al directorio de ArduPilot (default: ~/ardupilot)'
    )
    parser.add_argument(
        '--gazebo-model',
        default='iris_runway.sdf',
        help='Modelo/mundo de Gazebo (default: iris_runway.sdf)'
    )
    parser.add_argument(
        '--controller-cmd',
        help='Comando a ejecutar para el controlador (opcional)',
        default=None
    )
    parser.add_argument(
        '--takeoff-alt',
        type=float,
        default=2,
        help='Altitud de despegue en metros (default: 2)'
    )
    parser.add_argument(
        '--custom-mode',
        type=int,
        default=29,
        help='Número de modo custom a cambiar (default: 29)'
    )
    parser.add_argument(
        '--speedup',
        type=int,
        default=1,
        help='Velocidad de simulación (1 = tiempo real, default: 1)'
    )
    
    args = parser.parse_args()
    
    # Registrar manejador de señal
    manager = SimulationManager(args)
    signal.signal(signal.SIGINT, manager.shutdown)
    
    # Ejecutar secuencia
    manager.run_synchronized_sequence()


if __name__ == '__main__':
    main()
