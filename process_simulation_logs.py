import os
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import argparse
from pathlib import Path
from datetime import datetime

# Lista ordenada de archivos a procesar con configuración
# Formato: (código, título, es_parámetro)
# - código: Código de 4 letras del archivo (ej: 'QUAT', 'GAIN')
# - título: Título descriptivo para mostrar
# - es_parámetro: True = mostrar como texto, False = graficar
FILES_TO_PROCESS = [
    # Parámetros y configuración (mostrar como texto)
    ('VCGA', 'Ganancias Control de Velocidad', True),
    ('CVGA', 'Parametros del campo de velocidad', True),

    ('EKFS', 'Estado EKF', False),
    ('GTPR', 'Posición Relativa(Ground Truth)', False),
    ('GTVR', 'Velocidad Relativa(Ground Truth)', False),
    ('CVDS', 'Velocidad Deseada (Campo de velocida)', False),
    ('VCPX', 'Control Velocidad Eje X', False),
    ('CPUX', 'Salida Control X', False),
    ('VCPY', 'Control Velocidad Eje Y', False),
    ('CPUY', 'Salida Control Y', False),
    ('VCPZ', 'Control Velocidad Eje Z', False),
    ('CPUZ', 'Salida Control Z', False),
    # Datos para graficar (en orden deseado)
    ('QUAT', 'Cuaterniones y Ángulos', False),
    ('ARUC', 'Detección ArUco (Sin transformación)', False),
    ('POST', 'Posición del Target (Con transformación)', False),
]

# Crear diccionarios auxiliares para compatibilidad
FILE_TITLES = {code: title for code, title, _ in FILES_TO_PROCESS}
PARAM_FILES = {code: is_param for code, _, is_param in FILES_TO_PROCESS}
FILE_ORDER = {code: idx for idx, (code, _, _) in enumerate(FILES_TO_PROCESS)}

# ============================================================================
# GRÁFICAS DE COMPARACIÓN
# ============================================================================
# Permite comparar datos de diferentes archivos en una sola gráfica.
# Ideal para evaluar rendimiento comparando estimaciones vs ground truth.
#
# Estructura de cada entrada:
# {
#     'title': 'Título de la gráfica',
#     'comparisons': [
#         {
#             'file1': 'CÓDIGO',      # Código de 4 letras del primer archivo
#             'field1': 'campo',      # Campo/columna del primer archivo
#             'file2': 'CÓDIGO',      # Código de 4 letras del segundo archivo
#             'field2': 'campo',      # Campo/columna del segundo archivo
#             'label': 'Etiqueta'     # Etiqueta descriptiva para esta comparación
#         },
#         # ... más comparaciones
#     ],
#     'show_error': True/False,       # Si True, muestra el error en subplot separado
#     'error_label': 'Etiqueta Error' # Etiqueta para el eje Y del subplot de error
# }
#
# Las gráficas se generan automáticamente y se agregan al dashboard al final.
# ============================================================================
COMPARISON_GRAPHS = [
    {
        'title': 'Comparación Posición: EKF vs Ground Truth',
        'comparisons': [
            {'file1': 'EKFS', 'field1': 'PosX', 'file2': 'GTPR', 'field2': 'Pr_x', 'label': 'Posición X'},
            {'file1': 'EKFS', 'field1': 'PosY', 'file2': 'GTPR', 'field2': 'Pr_y', 'label': 'Posición Y'},
            {'file1': 'EKFS', 'field1': 'PosZ', 'file2': 'GTPR', 'field2': 'Pr_z', 'label': 'Posición Z'},
        ],
        'show_error': True,  # Mostrar errores en subplot separado
        'error_label': 'Error Posición [m]'
    },
    {
        'title': 'Comparación Velocidad: EKF vs Ground Truth',
        'comparisons': [
            {'file1': 'EKFS', 'field1': 'VelX', 'file2': 'GTVR', 'field2': 'Vr_x', 'label': 'Velocidad X'},
            {'file1': 'EKFS', 'field1': 'VelY', 'file2': 'GTVR', 'field2': 'Vr_y', 'label': 'Velocidad Y'},
            {'file1': 'EKFS', 'field1': 'VelZ', 'file2': 'GTVR', 'field2': 'Vr_z', 'label': 'Velocidad Z'},
        ],
        'show_error': True,
        'error_label': 'Error Velocidad [m/s]'
    },
]

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
    
    def create_comparison_graph(self, comparison_config, available_files, output_dir):
        """
        Crea una gráfica de comparación entre múltiples archivos.
        
        Args:
            comparison_config: Diccionario con la configuración de la comparación
            available_files: Diccionario {código: Path} de archivos disponibles
            output_dir: Directorio donde guardar la gráfica
            
        Returns:
            Diccionario con info de la gráfica o None si falla
        """
        title = comparison_config['title']
        comparisons = comparison_config['comparisons']
        show_error = comparison_config.get('show_error', False)
        error_label = comparison_config.get('error_label', 'Error')
        
        if self.verbose:
            print(f"    [DEBUG] Creando gráfica de comparación: {title}")
        
        # Verificar que todos los archivos necesarios existen
        required_files = set()
        for comp in comparisons:
            required_files.add(comp['file1'])
            required_files.add(comp['file2'])
        
        missing_files = required_files - set(available_files.keys())
        if missing_files:
            print(f"    ⚠ Archivos faltantes para '{title}': {missing_files}")
            return None
        
        # Cargar todos los DataFrames necesarios
        dataframes = {}
        for file_code in required_files:
            df = self.load_and_process_csv(available_files[file_code])
            if df is None:
                print(f"    ⚠ No se pudo cargar {file_code}")
                return None
            dataframes[file_code] = df
        
        # Crear subplots: uno para las señales, otro para errores (si se solicita)
        n_rows = 2 if show_error else 1
        subplot_titles = [title]
        if show_error:
            subplot_titles.append(error_label)
        
        fig = make_subplots(
            rows=n_rows, cols=1,
            subplot_titles=subplot_titles,
            vertical_spacing=0.12,
            row_heights=[0.6, 0.4] if show_error else [1.0]
        )
        
        # Procesar cada comparación
        errors_data = []
        for comp in comparisons:
            file1_code = comp['file1']
            field1 = comp['field1']
            file2_code = comp['file2']
            field2 = comp['field2']
            label = comp['label']
            
            df1 = dataframes[file1_code]
            df2 = dataframes[file2_code]
            
            # Verificar que los campos existen
            if field1 not in df1.columns:
                print(f"    ⚠ Campo '{field1}' no encontrado en {file1_code}")
                continue
            if field2 not in df2.columns:
                print(f"    ⚠ Campo '{field2}' no encontrado en {file2_code}")
                continue
            
            time_col1 = df1.columns[0]
            time_col2 = df2.columns[0]
            
            # Agregar trazas de las dos señales al primer subplot
            fig.add_trace(go.Scatter(
                x=df1[time_col1],
                y=df1[field1],
                mode='lines',
                name=f'{label} ({file1_code})',
                line=dict(width=2),
                hovertemplate=f'<b>{label} ({file1_code})</b><br>' +
                             'Tiempo: %{x:.3f}s<br>' +
                             'Valor: %{y:.6f}<extra></extra>'
            ), row=1, col=1)
            
            fig.add_trace(go.Scatter(
                x=df2[time_col2],
                y=df2[field2],
                mode='lines',
                name=f'{label} (GT)',
                line=dict(width=2, dash='dash'),
                hovertemplate=f'<b>{label} (Ground Truth)</b><br>' +
                             'Tiempo: %{x:.3f}s<br>' +
                             'Valor: %{y:.6f}<extra></extra>'
            ), row=1, col=1)
            
            # Calcular error si se solicita
            if show_error:
                # Interpolar df2 a los tiempos de df1 para calcular error punto a punto
                df2_interp = df2.set_index(time_col2)[field2].reindex(
                    df1[time_col1], 
                    method='nearest', 
                    limit=1,
                    tolerance=0.1  # Tolerancia de 0.1 segundos
                ).reset_index()
                df2_interp.columns = [time_col1, field2]
                
                # Calcular error
                error = df1[field1] - df2_interp[field2]
                
                errors_data.append({
                    'time': df1[time_col1],
                    'error': error,
                    'label': label
                })
        
        # Agregar errores al segundo subplot si se solicita
        if show_error and errors_data:
            for err_data in errors_data:
                fig.add_trace(go.Scatter(
                    x=err_data['time'],
                    y=err_data['error'],
                    mode='lines',
                    name=f"Error {err_data['label']}",
                    hovertemplate=f"<b>Error {err_data['label']}</b><br>" +
                                 'Tiempo: %{x:.3f}s<br>' +
                                 'Error: %{y:.6f}<extra></extra>'
                ), row=2, col=1)
            
            # Agregar línea de cero en el subplot de error
            if errors_data:
                time_min = min(err['time'].min() for err in errors_data)
                time_max = max(err['time'].max() for err in errors_data)
                fig.add_trace(go.Scatter(
                    x=[time_min, time_max],
                    y=[0, 0],
                    mode='lines',
                    name='Referencia (0)',
                    line=dict(color='gray', width=1, dash='dot'),
                    showlegend=False
                ), row=2, col=1)
        
        # Actualizar layout
        fig.update_xaxes(title_text="Tiempo (s)", row=n_rows, col=1)
        fig.update_yaxes(title_text="Valor", row=1, col=1)
        if show_error:
            fig.update_yaxes(title_text=error_label, row=2, col=1)
        
        fig.update_layout(
            height=800 if show_error else 550,
            hovermode='x unified',
            template='plotly_white',
            legend=dict(
                orientation="v",
                yanchor="top",
                y=1,
                xanchor="left",
                x=1.01
            ),
            margin=dict(l=60, r=180, t=100, b=60)
        )
        
        if self.verbose:
            print(f"    [DEBUG] Gráfica de comparación creada exitosamente")
        
        return fig
    
    def create_dashboard(self, csv_files, sim_name, output_dir):
        """Crea un dashboard HTML único con todas las gráficas en el orden especificado."""
        graphs = []
        
        # Crear subdirectorio para gráficas individuales
        graphs_dir = output_dir / 'graficas'
        graphs_dir.mkdir(exist_ok=True)
        
        if self.verbose:
            print(f"\n[DEBUG] Generando gráficas individuales en: {graphs_dir}")
        
        # Generar gráficas individuales en el orden proporcionado (ya viene ordenado de process_simulation)
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
                
                # Guardar info para el dashboard (mantener orden de FILES_TO_PROCESS)
                graphs.append({
                    'title': title,
                    'code': code,
                    'filename': graph_file.name,
                    'path': graph_file.relative_to(output_dir),
                    'order': FILE_ORDER.get(code, 999)  # Usar orden definido
                })
                
                print(f"  ✓ Gráfica individual creada: {graph_file.name}")
                
                if self.verbose:
                    print(f"    [DEBUG] Ruta relativa: {graph_file.relative_to(output_dir)}")
                    print(f"    [DEBUG] Orden: {FILE_ORDER.get(code, 999)}")
            else:
                print(f"  ⚠ No se pudo crear gráfica para: {csv_file.name}")
        
        # Generar gráficas de comparación
        if COMPARISON_GRAPHS:
            print(f"\nGenerando {len(COMPARISON_GRAPHS)} gráfica(s) de comparación...")
            
            # Crear diccionario de archivos disponibles por código
            available_files = {}
            for csv_file in csv_files:
                code = self.get_file_code(csv_file.name)
                available_files[code] = csv_file
            
            for comp_idx, comp_config in enumerate(COMPARISON_GRAPHS):
                fig = self.create_comparison_graph(comp_config, available_files, output_dir)
                
                if fig is not None:
                    # Guardar gráfica de comparación
                    comp_title = comp_config['title']
                    safe_title = comp_title.replace(' ', '_').replace(':', '').replace('/', '_')
                    graph_file = graphs_dir / f"COMP_{comp_idx+1}_{safe_title}.html"
                    fig.write_html(graph_file)
                    
                    # Agregar al dashboard al final (después de gráficas individuales)
                    graphs.append({
                        'title': comp_title,
                        'code': f'COMP{comp_idx+1}',
                        'filename': graph_file.name,
                        'path': graph_file.relative_to(output_dir),
                        'order': 1000 + comp_idx  # Ordenar al final
                    })
                    
                    print(f"  ✓ Gráfica de comparación creada: {graph_file.name}")
                    
                    if self.verbose:
                        print(f"    [DEBUG] Ruta relativa: {graph_file.relative_to(output_dir)}")
        
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
        """Procesa todos los archivos de una simulación según FILES_TO_PROCESS."""
        print(f"\n{'='*60}")
        print(f"Procesando simulación: {sim_path.name}")
        print(f"{'='*60}\n")
        
        # Crear directorio de salida
        output_dir = self.reports_dir / sim_path.name
        output_dir.mkdir(exist_ok=True)
        
        # Buscar todos los archivos CSV disponibles
        all_csv_files = list(sim_path.glob('*.csv'))
        
        if not all_csv_files:
            print("⚠ No se encontraron archivos CSV")
            return
        
        print(f"Archivos CSV encontrados: {len(all_csv_files)}\n")
        
        # Crear un diccionario de archivos disponibles por código
        available_files = {}
        for csv_file in all_csv_files:
            code = self.get_file_code(csv_file.name)
            if code not in available_files:
                available_files[code] = []
            available_files[code].append(csv_file)
        
        if self.verbose:
            print(f"[DEBUG] Códigos encontrados: {list(available_files.keys())}\n")
        
        # Procesar archivos según el orden definido en FILES_TO_PROCESS
        parm_files = []
        data_files = []
        
        print("Procesando archivos según configuración...")
        for code, title, is_param in FILES_TO_PROCESS:
            if code not in available_files:
                if self.verbose:
                    print(f"  ⊗ {code} - {title} (No encontrado)")
                continue
            
            # Tomar el primer archivo que coincida con este código
            csv_file = available_files[code][0]
            
            if is_param:
                parm_files.append((csv_file, title))
                print(f"  → {csv_file.name} - {title} (Parámetros)")
            else:
                data_files.append((csv_file, title))
                print(f"  → {csv_file.name} - {title} (Gráfica)")
        
        print()
        
        # Procesar archivos de parámetros
        if parm_files:
            print(f"Procesando {len(parm_files)} archivo(s) de parámetros...")
            for parm_file, title in parm_files:
                result = self.process_parm_file(parm_file, output_dir, title)
                if result is None and self.verbose:
                    print(f"    [DEBUG] No se pudo procesar: {parm_file.name}")
        
        # Procesar archivos de datos
        if data_files:
            print("\nGenerando gráficas...")
            # Extraer solo los archivos (sin títulos) manteniendo el orden
            ordered_csv_files = [csv_file for csv_file, _ in data_files]
            
            if create_dashboard:
                self.create_dashboard(ordered_csv_files, sim_path.name, output_dir)
            else:
                self.create_individual_graphs(ordered_csv_files, sim_path.name, output_dir)
        
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
  - FILES_TO_PROCESS: Lista ordenada que define qué archivos procesar
    * Formato: (código, título, es_parámetro)
    * Solo se procesarán archivos definidos en esta lista
    * El orden en el dashboard respeta el orden de esta lista
    * es_parámetro=True: archivo se muestra como texto
    * es_parámetro=False: archivo se grafica
  
  Para agregar nuevos archivos, edita FILES_TO_PROCESS al inicio del script.
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