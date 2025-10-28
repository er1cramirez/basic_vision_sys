import os
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import argparse
from pathlib import Path
from datetime import datetime

# Diccionario para nombres descriptivos de archivos
FILE_TITLES = {
    # Archivos de inicialización y estado
    'INIT': 'Inicialización de Componentes',
    'STAT': 'Estado del Sistema',
    
    # Orientación y posición
    'QUAT': 'Cuaterniones y Ángulos',
    'ARUC': 'Detección ArUco(Sin transformación)',
    'POST': 'Posición del Target(Con transformación)',
    
    # Procesamiento de imágenes
    'FRAM': 'Información de Frames',
    
    # Control visual
    'CVGA': 'Ganancias de Control Visual',
    'CVDS': 'Velocidad Deseada',
    'VCPX': 'Control Velocidad Eje X',
    'VCPY': 'Control Velocidad Eje Y',
    'VCPZ': 'Control Velocidad Eje Z',
    
    # Controladores PID
    'CPUX': 'Salida Control  X',
    'CPUY': 'Salida Control  Y',
    'CPUZ': 'Salida Control  Z',
    'CPRM': 'Resumen Control ',
    
    # Control y estado
    'CTRL': 'Señales de Control',
    'EKFS': 'Estado EKF',
    
    # Otros códigos genéricos
    'PARM': 'Parámetros de Configuración',
    'GAIN': 'Ganancias de Control',
    'CONF': 'Configuración',
    'TUNE': 'Parámetros de Ajuste',
    'SENS': 'Datos de Sensores',
    'TRAJ': 'Trayectoria',
    'ERRR': 'Errores',
    'PERF': 'Métricas de Performance',
    'POSE': 'Posición y Orientación',
    # Agrega más códigos según tus necesidades
}

# Diccionario para identificar archivos que deben procesarse como texto
# Estos archivos se mostrarán en formato legible en lugar de gráficas
PARAM_FILES = {
    'INIT': True,  # Inicialización tiene texto (nombres de componentes)
    'PARM': True,  # Parámetros generales
    'GAIN': True,  # Ganancias de control (PID, etc)
    'CONF': True,  # Archivos de configuración
    'VCGA': True,  # Parámetros de ajuste/tuning
    'CVGA': True,  # Ganancias de control visual (pocos cambios, mejor como texto)
    # Agrega los códigos de tus archivos de parámetros aquí
}

class SimulationLogProcessor:
    def __init__(self, root_dir, time_offset=0.0, verbose=False):
        """
        Args:
            root_dir: Directorio raíz donde se encuentran las carpetas de simulación
            time_offset: Offset temporal en segundos a restar de los timestamps
            verbose: Activar modo debug con información detallada
        """
        self.root_dir = Path(root_dir)
        self.time_offset = time_offset
        self.verbose = verbose
        self.reports_dir =  Path('/home/eric/droneSim_ws/src/basic_vision_sys/reports')
        self.reports_dir.mkdir(exist_ok=True)
        
    def find_latest_simulation(self):
        """Encuentra la carpeta de simulación más reciente."""
        sim_folders = [f for f in self.root_dir.iterdir() 
                      if f.is_dir() and f.name.startswith('uav_control_')]
        
        if not sim_folders:
            raise ValueError("No se encontraron carpetas de simulación")
        
        # Ordenar por fecha de modificación (más reciente primero)
        latest = max(sim_folders, key=lambda x: x.stat().st_mtime)
        return latest
    
    def find_simulation_by_name(self, sim_name):
        """Busca una carpeta de simulación específica."""
        sim_path = self.root_dir / sim_name
        if not sim_path.exists():
            raise ValueError(f"No se encontró la simulación: {sim_name}")
        return sim_path
    
    def process_parm_file(self, parm_path, output_dir, custom_title=None):
        """Convierte archivos de parámetros CSV a formato de texto legible."""
        try:
            df = pd.read_csv(parm_path)
            
            if len(df) == 0:
                print(f"  ⚠ Archivo vacío: {parm_path.name}")
                return None
            
            code = self.get_file_code(parm_path.name)
            title = custom_title or FILE_TITLES.get(code, 'Parámetros')
            output_name = f'{code.lower()}_{parm_path.stem}.txt'
            output_file = output_dir / output_name
            
            with open(output_file, 'w', encoding='utf-8') as f:
                f.write("=" * 60 + "\n")
                f.write(f"{title.upper()}\n")
                f.write("=" * 60 + "\n")
                f.write(f"Archivo: {parm_path.name}\n")
                f.write("=" * 60 + "\n\n")
                
                # Convertir timestamp a segundos si existe
                if 'TimeUS' in df.columns:
                    df['TimeUS'] = (df['TimeUS'] - df['TimeUS'].iloc[0]) / 1_000_000.0 - self.time_offset
                
                for idx, row in df.iterrows():
                    if 'TimeUS' in df.columns:
                        f.write(f"[t = {row['TimeUS']:.3f}s]\n")
                        f.write("-" * 40 + "\n")
                    
                    for col in df.columns:
                        if col != 'TimeUS':
                            value = row[col]
                            # Formatear números de manera más legible
                            if pd.api.types.is_numeric_dtype(df[col]) and not pd.isna(value):
                                f.write(f"  {col:25s}: {value:.6f}\n")
                            else:
                                f.write(f"  {col:25s}: {value}\n")
                    f.write("\n")
            
            print(f"  ✓ Archivo de parámetros generado: {output_file.name}")
            return output_file
            
        except Exception as e:
            print(f"  ✗ Error procesando {parm_path.name}: {e}")
            return None
    
    def load_and_process_csv(self, csv_path):
        """Carga un CSV y procesa los timestamps."""
        try:
            # Forzar que pandas lea todas las columnas numéricas correctamente
            df = pd.read_csv(csv_path)
            
            if self.verbose:
                print(f"    [DEBUG] Leyendo {csv_path.name}")
                print(f"    [DEBUG] Forma del DataFrame: {df.shape}")
                print(f"    [DEBUG] Columnas: {list(df.columns)}")
                print(f"    [DEBUG] Tipos de datos: {df.dtypes.to_dict()}")
            
            if len(df) == 0:
                print(f"    ⚠ Archivo vacío: {csv_path.name}")
                return None
            
            # Convertir timestamp a segundos y aplicar offset
            time_col = df.columns[0]
            if 'Time' in time_col or time_col == 'TimeUS':
                # Asegurar que el timestamp es numérico
                df[time_col] = pd.to_numeric(df[time_col], errors='coerce')
                first_val = df[time_col].iloc[0]
                df[time_col] = (df[time_col] - first_val) / 1_000_000.0 - self.time_offset
                
                if self.verbose:
                    print(f"    [DEBUG] Timestamp convertido - Rango: [{df[time_col].min():.6f}, {df[time_col].max():.6f}]")
            
            # Convertir todas las demás columnas a numérico donde sea posible
            for col in df.columns[1:]:
                if df[col].dtype == 'object':
                    df[col] = pd.to_numeric(df[col], errors='ignore')
            
            if self.verbose:
                print(f"    [DEBUG] Primeras filas:")
                print(df.head())
            
            return df
        except Exception as e:
            print(f"    ⚠ Error leyendo {csv_path.name}: {e}")
            import traceback
            if self.verbose:
                traceback.print_exc()
            return None
    
    def get_file_code(self, filename):
        """Extrae el código de 4 letras del nombre del archivo."""
        name = Path(filename).stem
        # Buscar código de 4 letras mayúsculas
        for i in range(len(name) - 3):
            code = name[i:i+4]
            if code.isupper() and code.isalpha():
                return code
        return name[:4].upper()
    
    def is_param_file(self, filename):
        """Verifica si un archivo debe procesarse como parámetros."""
        code = self.get_file_code(filename)
        return PARAM_FILES.get(code, False)
    
    def _load_param_summaries(self, output_dir):
        """Carga los archivos de parámetros generados para mostrar resumen."""
        param_summaries = {}
        
        # Buscar archivos .txt de parámetros
        txt_files = list(output_dir.glob('*.txt'))
        
        for txt_file in txt_files:
            try:
                with open(txt_file, 'r', encoding='utf-8') as f:
                    lines = f.readlines()
                
                # Extraer título
                title = None
                for line in lines:
                    if line.strip() and '=' not in line and line.strip().isupper():
                        title = line.strip()
                        break
                
                if not title:
                    title = txt_file.stem
                
                # Extraer último conjunto de parámetros
                params = {}
                in_last_section = False
                last_time = None
                
                for i, line in enumerate(lines):
                    # Detectar nueva sección de tiempo
                    if '[t = ' in line:
                        last_time = line.strip()
                        in_last_section = True
                        params = {}  # Reiniciar para quedarnos solo con los últimos
                    elif in_last_section and ':' in line and '=' not in line:
                        # Extraer parámetro: valor
                        parts = line.split(':', 1)
                        if len(parts) == 2:
                            param_name = parts[0].strip()
                            param_value = parts[1].strip()
                            if param_name and param_name != 'Archivo':
                                params[param_name] = param_value
                
                if params:
                    param_summaries[title] = params
                    
            except Exception as e:
                if self.verbose:
                    print(f"  [DEBUG] Error leyendo parámetros de {txt_file.name}: {e}")
        
        return param_summaries
    
    def has_plottable_data(self, csv_path):
        """Verifica si un CSV tiene datos numéricos que se puedan graficar."""
        try:
            df = pd.read_csv(csv_path, nrows=5)  # Leer solo primeras filas para verificar
            
            if len(df) < 2:
                return False
            
            # Verificar columnas después del timestamp
            data_cols = df.columns[1:]
            if len(data_cols) == 0:
                return False
            
            # Contar cuántas columnas son numéricas
            numeric_cols = 0
            for col in data_cols:
                if pd.api.types.is_numeric_dtype(df[col]):
                    numeric_cols += 1
            
            # Si al menos una columna es numérica, es graficable
            return numeric_cols > 0
            
        except Exception as e:
            print(f"    ⚠ Error verificando {csv_path.name}: {e}")
            return False
    
    def create_single_graph(self, df, title, filename):
        """Crea una gráfica individual para un CSV."""
        if self.verbose:
            print(f"    [DEBUG] Creando gráfica para {filename}")
        
        time_col = df.columns[0]
        data_cols = df.columns[1:]
        
        # Filtrar solo columnas numéricas
        numeric_cols = [col for col in data_cols if pd.api.types.is_numeric_dtype(df[col])]
        
        if self.verbose:
            print(f"    [DEBUG] Columnas numéricas encontradas: {numeric_cols}")
        
        if not numeric_cols:
            print(f"    ⚠ No hay columnas numéricas en {filename}")
            return None
        
        # Limpiar DataFrame eliminando filas con NaN en la columna de tiempo
        df_clean = df.dropna(subset=[time_col])
        
        if self.verbose:
            print(f"    [DEBUG] Filas después de limpiar: {len(df_clean)}")
        
        if len(df_clean) == 0:
            print(f"    ⚠ No hay datos válidos después de limpiar en {filename}")
            return None
        
        fig = go.Figure()
        
        traces_added = 0
        for col in numeric_cols:
            # Usar solo filas donde esta columna no es NaN
            df_col = df_clean.dropna(subset=[col])
            
            if self.verbose:
                print(f"    [DEBUG] Columna {col}: {len(df_col)} puntos, rango Y: [{df_col[col].min():.6f}, {df_col[col].max():.6f}]")
            
            if len(df_col) > 0:
                fig.add_trace(go.Scatter(
                    x=df_col[time_col],
                    y=df_col[col],
                    mode='lines',
                    name=col,
                    hovertemplate='<b>%{fullData.name}</b><br>' +
                                 'Tiempo: %{x:.3f}s<br>' +
                                 'Valor: %{y:.6f}<extra></extra>'
                ))
                traces_added += 1
        
        if self.verbose:
            print(f"    [DEBUG] Trazas agregadas: {traces_added}")
            print(f"    [DEBUG] Número de trazas en figura: {len(fig.data)}")
        
        if traces_added == 0:
            print(f"    ⚠ No se agregaron trazas para {filename}")
            return None
        
        fig.update_layout(
            title=dict(text=title, x=0.5, xanchor='center'),
            xaxis_title='Tiempo (s)',
            yaxis_title='Valor',
            hovermode='x unified',
            template='plotly_white',
            legend=dict(
                orientation="v",
                yanchor="top",
                y=1,
                xanchor="left",
                x=1.01
            ),
            margin=dict(l=60, r=150, t=80, b=60)
        )
        
        if self.verbose:
            print(f"    [DEBUG] Figura creada exitosamente")
        
        return fig
    
    def create_dashboard(self, csv_files, sim_name, output_dir):
        """Crea un dashboard HTML único con todas las gráficas."""
        graphs = []
        
        # Crear subdirectorio para gráficas individuales
        graphs_dir = output_dir / 'graficas'
        graphs_dir.mkdir(exist_ok=True)
        
        if self.verbose:
            print(f"\n[DEBUG] Generando gráficas individuales en: {graphs_dir}")
        
        # Generar TODAS las gráficas individuales primero
        for csv_file in csv_files:
            df = self.load_and_process_csv(csv_file)
            if df is None or len(df) < 2:
                if self.verbose:
                    print(f"  [DEBUG] Saltando {csv_file.name} - sin datos suficientes")
                continue
            
            code = self.get_file_code(csv_file.name)
            title = FILE_TITLES.get(code, csv_file.stem)
            
            fig = self.create_single_graph(df, title, csv_file.name)
            if fig is not None:
                # Guardar gráfica individual
                graph_file = graphs_dir / f"{code}_{csv_file.stem}.html"
                fig.write_html(graph_file)
                
                # Guardar info para el dashboard
                graphs.append({
                    'title': title,
                    'code': code,
                    'filename': graph_file.name,
                    'path': graph_file.relative_to(output_dir)
                })
                
                print(f"  ✓ Gráfica individual creada: {graph_file.name}")
                
                if self.verbose:
                    print(f"    [DEBUG] Ruta relativa: {graph_file.relative_to(output_dir)}")
            else:
                print(f"  ⚠ No se pudo crear gráfica para: {csv_file.name}")
        
        if not graphs:
            print("  ⚠ No se generaron gráficas")
            return None
        
        if self.verbose:
            print(f"\n[DEBUG] Total de gráficas generadas: {len(graphs)}")
        
        # Leer archivos de parámetros para mostrar al inicio
        param_data = self._load_param_summaries(output_dir)
        
        if self.verbose:
            print(f"[DEBUG] Parámetros encontrados: {len(param_data)} categorías")
        
        # Crear archivo HTML con dashboard
        output_file = output_dir / f"dashboard_{sim_name}.html"
        
        if self.verbose:
            print(f"[DEBUG] Generando dashboard en: {output_file}")
        
        with open(output_file, 'w', encoding='utf-8') as f:
            # Escribir header HTML
            f.write(f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Reporte - {sim_name}</title>
    <style>
        body {{
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f5f5f5;
        }}
        .header {{
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 30px;
            border-radius: 10px;
            margin-bottom: 30px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }}
        .header h1 {{
            margin: 0 0 10px 0;
        }}
        .info {{
            opacity: 0.9;
            font-size: 14px;
        }}
        .params-section {{
            background: white;
            padding: 20px;
            margin-bottom: 30px;
            border-radius: 10px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }}
        .param-card {{
            background: #f8f9fa;
            padding: 15px;
            margin-bottom: 15px;
            border-radius: 8px;
            border-left: 4px solid #667eea;
        }}
        .param-card h3 {{
            margin: 0 0 10px 0;
            color: #333;
        }}
        .param-grid {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minwidth(300px, 1fr));
            gap: 10px;
        }}
        .param-item {{
            padding: 8px;
            background: white;
            border-radius: 4px;
            font-family: 'Courier New', monospace;
            font-size: 13px;
        }}
        .param-label {{
            color: #666;
            display: inline-block;
            min-width: 120px;
        }}
        .param-value {{
            color: #000;
            font-weight: bold;
        }}
        .graph-container {{
            background: white;
            padding: 20px;
            margin-bottom: 30px;
            border-radius: 10px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            position: relative;
        }}
        .graph-header {{
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 10px;
        }}
        .graph-header h2 {{
            margin: 0;
        }}
        .graph-actions {{
            display: flex;
            gap: 10px;
        }}
        .action-btn {{
            background: #667eea;
            color: white;
            border: none;
            padding: 8px 16px;
            border-radius: 5px;
            cursor: pointer;
            font-size: 14px;
            text-decoration: none;
            transition: background 0.3s;
        }}
        .action-btn:hover {{
            background: #5568d3;
        }}
        .graph-iframe {{
            width: 100%;
            height: 550px;
            border: none;
            border-radius: 5px;
        }}
        .fullscreen-overlay {{
            display: none;
            position: fixed;
            top: 0;
            left: 0;
            width: 100vw;
            height: 100vh;
            background: rgba(0,0,0,0.95);
            z-index: 9999;
            padding: 20px;
        }}
        .fullscreen-overlay.active {{
            display: flex;
            flex-direction: column;
        }}
        .fullscreen-iframe {{
            flex: 1;
            width: 100%;
            border: none;
        }}
        .close-fullscreen {{
            position: absolute;
            top: 20px;
            right: 20px;
            background: white;
            color: black;
            border: none;
            padding: 10px 20px;
            border-radius: 5px;
            cursor: pointer;
            font-size: 16px;
            z-index: 10000;
        }}
        .footer {{
            text-align: center;
            color: #666;
            margin-top: 40px;
            padding: 20px;
        }}
    </style>
</head>
<body>
    <div class="header">
        <h1>📊 Reporte de Simulación</h1>
        <div class="info">
            <strong>Simulación:</strong> {sim_name}<br>
            <strong>Fecha de generación:</strong> {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}<br>
            <strong>Gráficas generadas:</strong> {len(graphs)}<br>
            <strong>Offset temporal aplicado:</strong> {self.time_offset:.3f}s
        </div>
    </div>
""")
            
            # Agregar sección de parámetros si existen
            if param_data:
                if self.verbose:
                    print(f"  [DEBUG] Escribiendo {len(param_data)} secciones de parámetros")
                
                f.write("""
    <div class="params-section">
        <h2>⚙️ Parámetros de Configuración</h2>
""")
                for title, params in param_data.items():
                    f.write(f"""
        <div class="param-card">
            <h3>{title}</h3>
            <div class="param-grid">
""")
                    for label, value in params.items():
                        f.write(f"""
                <div class="param-item">
                    <span class="param-label">{label}:</span>
                    <span class="param-value">{value}</span>
                </div>
""")
                    f.write("""
            </div>
        </div>
""")
                f.write("""
    </div>
""")
            
            # Agregar overlay para pantalla completa
            f.write("""
    <div id="fullscreen-overlay" class="fullscreen-overlay">
        <button onclick="closeFullscreen()" class="close-fullscreen">✕ Cerrar</button>
        <iframe id="fullscreen-iframe" class="fullscreen-iframe"></iframe>
    </div>
""")
            
            # Agregar cada gráfica como iframe
            for i, graph_info in enumerate(graphs):
                if self.verbose:
                    print(f"  [DEBUG] Agregando iframe {i+1}/{len(graphs)}: {graph_info['title']}")
                
                f.write(f"""
    <div class="graph-container">
        <div class="graph-header">
            <h2>{graph_info['title']}</h2>
            <div class="graph-actions">
                <button class="action-btn" onclick="openFullscreen('{graph_info['path']}')">
                    🔍 Pantalla completa
                </button>
                <a href="{graph_info['path']}" target="_blank" class="action-btn">
                    🔗 Abrir en nueva pestaña
                </a>
            </div>
        </div>
        <iframe src="{graph_info['path']}" class="graph-iframe"></iframe>
    </div>
""")
            
            # Agregar scripts de JavaScript
            f.write("""
    <script>
        function openFullscreen(graphPath) {
            const overlay = document.getElementById('fullscreen-overlay');
            const iframe = document.getElementById('fullscreen-iframe');
            
            iframe.src = graphPath;
            overlay.classList.add('active');
        }
        
        function closeFullscreen() {
            const overlay = document.getElementById('fullscreen-overlay');
            const iframe = document.getElementById('fullscreen-iframe');
            
            overlay.classList.remove('active');
            iframe.src = '';
        }
        
        // Cerrar con ESC
        document.addEventListener('keydown', function(e) {
            if (e.key === 'Escape') {
                closeFullscreen();
            }
        });
        
        // Log para debug
        console.log('Dashboard cargado con', document.querySelectorAll('.graph-iframe').length, 'gráficas');
    </script>
""")
            
            # Footer
            f.write("""
    <div class="footer">
        Generado automáticamente por process_simulation_logs.py
    </div>
</body>
</html>
""")
        
        if self.verbose:
            print(f"  [DEBUG] Dashboard HTML escrito: {output_file.stat().st_size} bytes")
        
        print(f"\n✓ Dashboard generado: {output_file}")
        print(f"✓ Gráficas individuales en: {graphs_dir}")
        return output_file
    
    def create_individual_graphs(self, csv_files, sim_name, output_dir):
        """Crea archivos HTML individuales para cada gráfica."""
        graphs_dir = output_dir / 'graficas'
        graphs_dir.mkdir(exist_ok=True)
        
        generated_files = []
        
        for csv_file in csv_files:
            df = self.load_and_process_csv(csv_file)
            if df is None or len(df) < 2:
                continue
            
            code = self.get_file_code(csv_file.name)
            title = FILE_TITLES.get(code, csv_file.stem)
            
            fig = self.create_single_graph(df, title, csv_file.name)
            
            if fig is not None:
                output_file = graphs_dir / f"{code}_{csv_file.stem}.html"
                fig.write_html(output_file)
                generated_files.append(output_file)
                print(f"  ✓ Gráfica generada: {output_file.name}")
                
                if self.verbose:
                    print(f"    [DEBUG] Archivo guardado: {output_file}")
            else:
                print(f"  ⚠ No se pudo crear gráfica para: {csv_file.name}")
        
        if self.verbose:
            print(f"\n[DEBUG] Total de archivos individuales generados: {len(generated_files)}")
        
        return generated_files
    
    def process_simulation(self, sim_path, create_dashboard=True):
        """Procesa todos los archivos de una simulación."""
        print(f"\n{'='*60}")
        print(f"Procesando simulación: {sim_path.name}")
        print(f"{'='*60}\n")
        
        # Crear directorio de salida
        output_dir = self.reports_dir / sim_path.name
        output_dir.mkdir(exist_ok=True)
        
        # Buscar archivos CSV
        csv_files = list(sim_path.glob('*.csv'))
        
        if not csv_files:
            print("⚠ No se encontraron archivos CSV")
            return
        
        print(f"Archivos encontrados: {len(csv_files)}\n")
        
        # Clasificar archivos en parámetros y datos
        parm_files = []
        data_files = []
        
        print("Clasificando archivos...")
        for csv_file in csv_files:
            code = self.get_file_code(csv_file.name)
            
            # Primero verificar si está explícitamente marcado como parámetro
            if self.is_param_file(csv_file.name):
                parm_files.append(csv_file)
                print(f"  → {csv_file.name} (Parámetros - por configuración)")
            # Si no, verificar si tiene datos graficables
            elif not self.has_plottable_data(csv_file):
                parm_files.append(csv_file)
                print(f"  → {csv_file.name} (Parámetros - sin datos numéricos)")
            else:
                data_files.append(csv_file)
                print(f"  → {csv_file.name} (Gráfica)")
        
        print()
        
        if parm_files:
            print(f"Procesando {len(parm_files)} archivo(s) de parámetros...")
            for parm_file in parm_files:
                code = self.get_file_code(parm_file.name)
                title = FILE_TITLES.get(code, parm_file.stem)
                self.process_parm_file(parm_file, output_dir, title)
        
        # Procesar archivos de datos
        if data_files:
            print("\nGenerando gráficas...")
            if create_dashboard:
                self.create_dashboard(data_files, sim_path.name, output_dir)
            else:
                self.create_individual_graphs(data_files, sim_path.name, output_dir)
        
        print(f"\n✓ Procesamiento completado")
        print(f"📁 Resultados guardados en: {output_dir}")


def main():
    parser = argparse.ArgumentParser(
        description='Procesa logs de simulación y genera visualizaciones',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos de uso:
  # Procesar última simulación
  python process_simulation_logs.py
  
  # Procesar simulación específica con modo debug
  python process_simulation_logs.py -s uav_control_20251007_161006_20251007_161006 -v
  
  # Aplicar offset temporal de 2.5 segundos
  python process_simulation_logs.py --offset 2.5
  
  # Generar gráficas individuales en lugar de dashboard
  python process_simulation_logs.py --individual
  
  # Modo debug completo
  python process_simulation_logs.py --verbose

Configuración:
  - FILE_TITLES: Define títulos descriptivos para códigos de 4 letras
  - PARAM_FILES: Define qué archivos se procesan como texto (ej: GAIN, CONF)
  
  Los archivos marcados como parámetros se convierten a texto legible,
  el resto se visualiza como gráficas interactivas.
        """
    )
    
    parser.add_argument('-s', '--simulation', type=str,
                       help='Nombre de la simulación a procesar (por defecto: última)')
    parser.add_argument('-r', '--root', type=str, default='/home/eric/droneSim_ws/src/basic_vision_sys/build/simLogs',
                       help='Directorio raíz donde se encuentran las simulaciones (por defecto: directorio actual)')
    parser.add_argument('-o', '--offset', type=float, default=0.0,
                       help='Offset temporal en segundos a restar de los timestamps (por defecto: 0.0)')
    parser.add_argument('--individual', action='store_true',
                       help='Crear archivos HTML individuales en lugar de un dashboard único')
    parser.add_argument('-v', '--verbose', action='store_true',
                       help='Activar modo debug con información detallada')
    
    args = parser.parse_args()
    
    try:
        processor = SimulationLogProcessor(args.root, args.offset, args.verbose)
        
        if args.simulation:
            sim_path = processor.find_simulation_by_name(args.simulation)
        else:
            sim_path = processor.find_latest_simulation()
        
        processor.process_simulation(sim_path, create_dashboard=not args.individual)
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())